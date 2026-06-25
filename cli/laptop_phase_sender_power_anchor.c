/*
 * TO Compile:
 *   gcc -std=gnu99 -O3 -Wall -Wextra laptop_phase_sender_power_anchor.c -o sender -lm
 * Windows/MinGW:
 *   gcc -std=gnu99 -O3 -Wall -Wextra laptop_phase_sender_power_anchor.c -o sender.exe -lm -lws2_32
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#ifdef _WIN32
#include <windows.h>
#include <conio.h>
#else
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/select.h>
#endif

#ifdef _WIN32
#undef _WIN32_WINNT
#define _WIN32_WINNT 0x0600
#include <winsock2.h>
#include <ws2tcpip.h>
#ifndef INET_PTON
WINSOCK_API_LINKAGE INT WSAAPI inet_pton(INT Family, PCSTR pszAddrString, PVOID pAddrBuf);
#endif
typedef SOCKET socket_t;
#define close_socket closesocket
#else
#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
typedef int socket_t;
#define INVALID_SOCKET (-1)
#define SOCKET_ERROR (-1)
#define close_socket close
#endif

#include "phase_protocol.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288
#endif

#define NUM_BOARDS 4
#define CHANNELS_PER_BOARD 50
#define NUM_EMITTERS 200
#define MAX_PARTICLES 10
#define MOVE_INC_MM 1.0
#define DEFAULT_BOARD_DISTANCE_MM 135.0
#define DEFAULT_WAVELENGTH_MM 8.5740
#define DEFAULT_SEND_DELAY_MS 4
#define HALO_RADIUS_STEP_MM 0.25
#define HALO_WEIGHT_STEP 0.05
#define MAX_HALO_RADIUS_MM 3.0
#define MAX_HALO_WEIGHT 0.35

/* Original hardware coordinate table in 0.1 mm units. */
static const int16_t xCols[5] = {450, 350, 250, 150, 50};
static const int16_t zRows[10] = {-450, -350, -250, -150, -50, 50, 150, 250, 350, 450};

enum BoardIndex {
    BOARD_LEFT_TOP = 0,
    BOARD_RIGHT_TOP = 1,
    BOARD_LEFT_BOTTOM = 2,
    BOARD_RIGHT_BOTTOM = 3
};

typedef struct { double x, y, z; } Vec3;

typedef struct {
    Vec3 p[MAX_PARTICLES];     /* mm, centre-origin hardware coordinates */
    int count;
    int active;
} ParticleState;

typedef struct {
    Vec3 p_m[NUM_EMITTERS];
    int is_bottom[NUM_EMITTERS];
} EmitterGeometry;

static EmitterGeometry g_geom;
static double g_wavelength_mm = DEFAULT_WAVELENGTH_MM;
static unsigned int g_send_delay_ms = DEFAULT_SEND_DELAY_MS;
static double g_halo_radius_mm = 0.0;
static double g_halo_weight = 0.0;
static int g_bottom_pi_enabled = 1;
static int g_active_only_debug = 0;

static void delay_ms(unsigned int ms)
{
#ifdef _WIN32
    Sleep(ms);
#else
    usleep(ms * 1000);
#endif
}

static double getTransducerX0p1mm(enum BoardIndex board, int channel)
{
    int col = channel / 10;
    int16_t x0 = xCols[col];
    switch(board) {
        case BOARD_LEFT_TOP:     return x0;
        case BOARD_RIGHT_TOP:    return -x0;
        case BOARD_LEFT_BOTTOM:  return x0;
        case BOARD_RIGHT_BOTTOM: return -x0;
        default: return 0.0;
    }
}

static double getTransducerZ0p1mm(int channel)
{
    return zRows[channel % 10];
}

static int boardIsBottom(enum BoardIndex board)
{
    return board == BOARD_LEFT_BOTTOM || board == BOARD_RIGHT_BOTTOM;
}

static void buildEmitterGeometry(double boardDistanceMm)
{
    double half_y_mm = 0.5 * boardDistanceMm;
    for(int board = 0; board < NUM_BOARDS; ++board) {
        for(int ch = 0; ch < CHANNELS_PER_BOARD; ++ch) {
            int idx = board * CHANNELS_PER_BOARD + ch;
            g_geom.p_m[idx].x = getTransducerX0p1mm((enum BoardIndex)board, ch) * 0.0001;
            g_geom.p_m[idx].y = (boardIsBottom((enum BoardIndex)board) ? -half_y_mm : half_y_mm) / 1000.0;
            g_geom.p_m[idx].z = getTransducerZ0p1mm(ch) * 0.0001;
            g_geom.is_bottom[idx] = boardIsBottom((enum BoardIndex)board);
        }
    }
}

static uint16_t normalizePhaseInt(int32_t phase)
{
    phase %= (int32_t)HOLO_PHASE_MAX;
    if(phase < 0) phase += HOLO_PHASE_MAX;
    return (uint16_t)phase;
}

static uint16_t phaseFromAngle(double angle)
{
    int32_t tick = (int32_t)llround(angle * ((double)HOLO_PHASE_MAX) / (2.0 * M_PI));
    return normalizePhaseInt(tick);
}

static Vec3 add3(Vec3 a, Vec3 b)
{
    Vec3 r = {a.x + b.x, a.y + b.y, a.z + b.z};
    return r;
}

static double dist_m(Vec3 a_m, Vec3 b_mm)
{
    double bx = b_mm.x / 1000.0;
    double by = b_mm.y / 1000.0;
    double bz = b_mm.z / 1000.0;
    double dx = bx - a_m.x;
    double dy = by - a_m.y;
    double dz = bz - a_m.z;
    return sqrt(dx*dx + dy*dy + dz*dz);
}

static void addNodeContribution(int emitter, Vec3 target_mm, double weight, double k, double *re, double *im)
{
    double r = dist_m(g_geom.p_m[emitter], target_mm);
    double angle = -k * r;
    if(g_bottom_pi_enabled && g_geom.is_bottom[emitter]) angle += M_PI;
    *re += weight * cos(angle);
    *im += weight * sin(angle);
}

/*
 * New power-anchor phase calculation.
 *
 * For one particle this reduces to the old strong geometric pressure-node trap.
 * For multiple particles it does NOT time-multiplex.  Each transducer phase is
 * the angle of the complex sum of every requested node contribution, so all
 * particles remain present in every transmitted frame.  Optional halo terms make
 * a slightly wider catch basin without adding more time frames.
 */
static uint16_t calculatePhasePowerAnchor(const ParticleState *ps, int emitter)
{
    const double wavelength_m = g_wavelength_mm / 1000.0;
    const double k = 2.0 * M_PI / wavelength_m;
    double re = 0.0, im = 0.0;

    int start = 0, stop = ps->count;
    if(g_active_only_debug) {
        start = ps->active;
        stop = ps->active + 1;
    }

    for(int j = start; j < stop; ++j) {
        if(j < 0 || j >= ps->count) continue;
        Vec3 p = ps->p[j];
        addNodeContribution(emitter, p, 1.0, k, &re, &im);

        if(g_halo_radius_mm > 1e-9 && g_halo_weight > 1e-9) {
            double r = g_halo_radius_mm;
            const Vec3 off[4] = {
                { r, 0.0, 0.0}, {-r, 0.0, 0.0},
                {0.0, 0.0,  r}, {0.0, 0.0, -r}
            };
            for(int q = 0; q < 4; ++q)
                addNodeContribution(emitter, add3(p, off[q]), g_halo_weight, k, &re, &im);
        }
    }

    if(fabs(re) + fabs(im) < 1e-18) {
        /* Very rare cancellation fallback: hold the active particle. */
        Vec3 p = ps->p[ps->active >= 0 ? ps->active : 0];
        addNodeContribution(emitter, p, 1.0, k, &re, &im);
    }
    return phaseFromAngle(atan2(im, re));
}

static void fillPhaseFrame(HoloPhaseFrame *frame, uint16_t frameID, const ParticleState *ps)
{
    memset(frame, 0, sizeof(*frame));
    frame->magic = HOLO_PHASE_MAGIC;
    frame->version = HOLO_PHASE_VERSION;
    frame->frame_id = frameID;
    frame->phase_count = HOLO_PHASE_COUNT;
    frame->phase_max = HOLO_PHASE_MAX;

    for(int i = 0; i < NUM_EMITTERS; ++i)
        frame->phases[i] = calculatePhasePowerAnchor(ps, i);

    frame->crc32 = holo_phase_frame_crc(frame);
}

static void initParticles(ParticleState *ps)
{
    memset(ps, 0, sizeof(*ps));
    ps->count = 1;
    ps->active = 0;
    ps->p[0] = (Vec3){0.0, 0.0, 0.0};
}

static void printParticles(const ParticleState *ps)
{
    printf("particles: count=%d active=%d%s wavelength=%.3f mm halo=%.2f mm x %.2f bottom_pi=%s delay=%u ms\n",
           ps->count, ps->active + 1, g_active_only_debug ? " ACTIVE-ONLY-DEBUG" : "",
           g_wavelength_mm, g_halo_radius_mm, g_halo_weight,
           g_bottom_pi_enabled ? "on" : "off", g_send_delay_ms);
    for(int i = 0; i < ps->count; ++i) {
        printf("  %c%d: x=%7.2f mm  y=%7.2f mm  z=%7.2f mm\n",
               i == ps->active ? '*' : ' ', i + 1, ps->p[i].x, ps->p[i].y, ps->p[i].z);
    }
}

static void printHelp(void)
{
    printf("commands:\n");
    printf("  x/s    active particle X -/+ 1 mm\n");
    printf("  c/d    active particle Y -/+ 1 mm\n");
    printf("  z/a    active particle Z -/+ 1 mm\n");
    printf("  n/b    next / previous particle\n");
    printf("  1..9,0 select particle 1..10\n");
    printf("  g      add particle at centre and select it\n");
    printf("  r      remove active particle\n");
    printf("  h      home active particle to centre\n");
    printf("  H      reset to one centre particle\n");
    printf("  p      print particles/status\n");
    printf("  w/e    wider/narrower halo. Keep halo at 0 first for max strength\n");
    printf("  +/-    wavelength + / - 0.05 mm\n");
    printf("  P      toggle bottom-board pi phase flip\n");
    printf("  D      active-only debug hold toggle\n");
    printf("  [/]    slower/faster repeat send delay\n");
    printf("  ?      help\n");
    printf("  q      quit\n");
}

static int handleKey(int ch, ParticleState *ps)
{
    if(ch == EOF || ch == 0 || ch == '\n' || ch == '\r') return 0;
    int dirty = 0;
    Vec3 *p = &ps->p[ps->active];

    switch(ch) {
        case 'q': return -1;
        case '?': printHelp(); break;
        case 'p': printParticles(ps); break;

        case 'x': p->x -= MOVE_INC_MM; dirty = 1; break;
        case 's': p->x += MOVE_INC_MM; dirty = 1; break;
        case 'c': p->y -= MOVE_INC_MM; dirty = 1; break;
        case 'd': p->y += MOVE_INC_MM; dirty = 1; break;
        case 'z': p->z -= MOVE_INC_MM; dirty = 1; break;
        case 'a': p->z += MOVE_INC_MM; dirty = 1; break;

        case 'n': ps->active = (ps->active + 1) % ps->count; printParticles(ps); break;
        case 'b': ps->active = (ps->active + ps->count - 1) % ps->count; printParticles(ps); break;

        case '1': case '2': case '3': case '4': case '5':
        case '6': case '7': case '8': case '9': {
            int idx = ch - '1';
            if(idx < ps->count) { ps->active = idx; printParticles(ps); }
            break;
        }
        case '0':
            if(ps->count >= 10) { ps->active = 9; printParticles(ps); }
            break;

        case 'g':
            if(ps->count < MAX_PARTICLES) {
                ps->p[ps->count] = (Vec3){0.0, 0.0, 0.0};
                ps->active = ps->count;
                ps->count++;
                dirty = 1;
                printf("added particle %d at centre and selected it\n", ps->active + 1);
            } else {
                printf("already at max %d particles\n", MAX_PARTICLES);
            }
            break;

        case 'r':
            if(ps->count > 1) {
                for(int i = ps->active; i < ps->count - 1; ++i) ps->p[i] = ps->p[i+1];
                ps->count--;
                if(ps->active >= ps->count) ps->active = ps->count - 1;
                dirty = 1;
            } else {
                ps->p[0] = (Vec3){0.0, 0.0, 0.0};
                dirty = 1;
            }
            break;

        case 'h': *p = (Vec3){0.0, 0.0, 0.0}; dirty = 1; break;
        case 'H': initParticles(ps); dirty = 1; break;

        case 'w':
            g_halo_radius_mm += HALO_RADIUS_STEP_MM;
            if(g_halo_radius_mm > MAX_HALO_RADIUS_MM) g_halo_radius_mm = MAX_HALO_RADIUS_MM;
            g_halo_weight += HALO_WEIGHT_STEP;
            if(g_halo_weight > MAX_HALO_WEIGHT) g_halo_weight = MAX_HALO_WEIGHT;
            dirty = 1;
            printf("halo now %.2f mm x %.2f\n", g_halo_radius_mm, g_halo_weight);
            break;
        case 'e':
            g_halo_radius_mm -= HALO_RADIUS_STEP_MM;
            if(g_halo_radius_mm < 0.0) g_halo_radius_mm = 0.0;
            g_halo_weight -= HALO_WEIGHT_STEP;
            if(g_halo_weight < 0.0) g_halo_weight = 0.0;
            dirty = 1;
            printf("halo now %.2f mm x %.2f\n", g_halo_radius_mm, g_halo_weight);
            break;

        case '+': case '=':
            g_wavelength_mm += 0.05;
            dirty = 1;
            printf("wavelength %.3f mm\n", g_wavelength_mm);
            break;
        case '-': case '_':
            g_wavelength_mm -= 0.05;
            if(g_wavelength_mm < 7.0) g_wavelength_mm = 7.0;
            dirty = 1;
            printf("wavelength %.3f mm\n", g_wavelength_mm);
            break;

        case 'P':
            g_bottom_pi_enabled = !g_bottom_pi_enabled;
            dirty = 1;
            printf("bottom pi %s\n", g_bottom_pi_enabled ? "on" : "off");
            break;
        case 'D':
            g_active_only_debug = !g_active_only_debug;
            dirty = 1;
            printf("active-only debug %s\n", g_active_only_debug ? "ON" : "off");
            break;

        case '[':
            if(g_send_delay_ms < 50) g_send_delay_ms++;
            printf("delay %u ms\n", g_send_delay_ms);
            break;
        case ']':
            if(g_send_delay_ms > 1) g_send_delay_ms--;
            printf("delay %u ms\n", g_send_delay_ms);
            break;
        default:
            break;
    }

    if(dirty) printParticles(ps);
    return 0;
}

static int sendAll(socket_t sock, const void *buf, size_t len)
{
    const uint8_t *p = (const uint8_t *)buf;
    size_t off = 0;
    while(off < len) {
#ifdef _WIN32
        int n = send(sock, (const char *)p + off, (int)(len - off), 0);
#else
        ssize_t n = send(sock, p + off, len - off, 0);
#endif
        if(n <= 0) return -1;
        off += (size_t)n;
    }
    return 0;
}

static socket_t connectToPi(const char *host, uint16_t port)
{
    socket_t sock = socket(AF_INET, SOCK_STREAM, 0);
    if(sock == INVALID_SOCKET) return INVALID_SOCKET;

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);

    if(inet_pton(AF_INET, host, &addr.sin_addr) != 1) {
        struct hostent *he = gethostbyname(host);
        if(!he) {
            close_socket(sock);
            return INVALID_SOCKET;
        }
        memcpy(&addr.sin_addr, he->h_addr_list[0], he->h_length);
    }

    if(connect(sock, (struct sockaddr *)&addr, sizeof(addr)) == SOCKET_ERROR) {
        close_socket(sock);
        return INVALID_SOCKET;
    }
    return sock;
}

#ifndef _WIN32
static struct termios g_old_termios;
static int g_have_termios = 0;

static void restoreTerminal(void)
{
    if(g_have_termios) tcsetattr(STDIN_FILENO, TCSANOW, &g_old_termios);
}

static void setupTerminal(void)
{
    if(tcgetattr(STDIN_FILENO, &g_old_termios) == 0) {
        struct termios raw = g_old_termios;
        raw.c_lflag &= (tcflag_t)~(ICANON | ECHO);
        raw.c_cc[VMIN] = 0;
        raw.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSANOW, &raw);
        g_have_termios = 1;
        atexit(restoreTerminal);
    }
}
#endif

static int readKeyNonBlocking(void)
{
#ifdef _WIN32
    if(_kbhit()) return _getch();
    return 0;
#else
    fd_set set;
    struct timeval tv;
    FD_ZERO(&set);
    FD_SET(STDIN_FILENO, &set);
    tv.tv_sec = 0;
    tv.tv_usec = 0;
    if(select(STDIN_FILENO + 1, &set, NULL, NULL, &tv) > 0) {
        unsigned char c;
        if(read(STDIN_FILENO, &c, 1) == 1) return (int)c;
    }
    return 0;
#endif
}

static int selfTest(void)
{
    buildEmitterGeometry(DEFAULT_BOARD_DISTANCE_MM);
    ParticleState ps;
    initParticles(&ps);
    HoloPhaseFrame fr;
    fillPhaseFrame(&fr, 123, &ps);
    if(fr.phase_count != HOLO_PHASE_COUNT || fr.phase_max != HOLO_PHASE_MAX) return 1;
    for(int i = 0; i < NUM_EMITTERS; ++i) if(fr.phases[i] >= HOLO_PHASE_MAX) return 2;

    ps.p[0] = (Vec3){-10.0, 0.0, 0.0};
    ps.p[1] = (Vec3){0.0, 0.0, 0.0};
    ps.count = 2;
    ps.active = 1;
    fillPhaseFrame(&fr, 124, &ps);
    for(int i = 0; i < NUM_EMITTERS; ++i) if(fr.phases[i] >= HOLO_PHASE_MAX) return 3;

    ps.count = 10;
    for(int i = 0; i < 10; ++i) ps.p[i] = (Vec3){(double)((i%5)-2)*8.0, 0.0, (double)((i/5)-1)*8.0};
    g_halo_radius_mm = 1.0;
    g_halo_weight = 0.15;
    fillPhaseFrame(&fr, 125, &ps);
    int different = 0;
    for(int i = 1; i < NUM_EMITTERS; ++i) if(fr.phases[i] != fr.phases[0]) { different = 1; break; }
    if(!different) return 4;
    printf("SELF TEST PASSED\n");
    return 0;
}

int main(int argc, char **argv)
{
    if(argc >= 2 && strcmp(argv[1], "--self-test") == 0) return selfTest();

    if(argc < 2) {
        printf("usage: %s <pi-ip-or-host> [port] [board-distance-mm] [wavelength-mm]\n", argv[0]);
        printf("       %s --self-test\n", argv[0]);
        return 1;
    }

    const char *host = argv[1];
    uint16_t port = argc >= 3 ? (uint16_t)atoi(argv[2]) : HOLO_PHASE_TCP_PORT;
    double boardDistanceMm = argc >= 4 ? atof(argv[3]) : DEFAULT_BOARD_DISTANCE_MM;
    if(argc >= 5) g_wavelength_mm = atof(argv[4]);
    if(!(g_wavelength_mm > 0.0)) g_wavelength_mm = DEFAULT_WAVELENGTH_MM;

    buildEmitterGeometry(boardDistanceMm);

#ifdef _WIN32
    WSADATA wsa;
    if(WSAStartup(MAKEWORD(2, 2), &wsa) != 0) return 1;
#else
    setupTerminal();
#endif

    socket_t sock = connectToPi(host, port);
    if(sock == INVALID_SOCKET) {
        printf("could not connect to %s:%u\n", host, port);
#ifdef _WIN32
        WSACleanup();
#endif
        return 1;
    }

    ParticleState ps;
    initParticles(&ps);
    HoloPhaseFrame packet;
    uint16_t frameID = 0;

    printf("Power-anchor sender connected to %s:%u\n", host, port);
    printf("This version uses one full-duty multi-particle phase frame; no per-particle strobing.\n");
    printHelp();
    printParticles(&ps);

    while(1) {
        fillPhaseFrame(&packet, frameID++, &ps);
        if(sendAll(sock, &packet, sizeof(packet)) < 0) {
            printf("send failed\n");
            break;
        }

        int ch;
        while((ch = readKeyNonBlocking()) != 0) {
            if(handleKey(ch, &ps) < 0) goto done;
        }
        delay_ms(g_send_delay_ms);
    }

done:
    close_socket(sock);
#ifdef _WIN32
    WSACleanup();
#endif
    return 0;
}
