/*
 * laptop_phase_sender_hirayama_shell_single.c
 *
 * Single-particle static shell/twin-trap sender using a Hirayama/MATD-style
 * analytic acoustic trap structure.
 *
 * Why this exists:
 *   Your hardware consistently levitates with the original two-sided standing
 *   wave signature, but the point-node trap is shaky.  The previous dynamic
 *   "solid lock" stuttered because it time-multiplexed the trap.  This file
 *   instead sends ONE STATIC phase frame per target position: no ring cycling,
 *   no time-multiplexed shell, no multi-particle logic.
 *
 * Trap model:
 *   Hirayama et al.'s MATD computes levitation twin traps analytically at the
 *   hardware level by combining a focus term with a levitation phase signature.
 *   In this sender, the hardware-working standing-wave node is retained as the
 *   levitation signature, then four simultaneous high-pressure focus lobes are
 *   placed around the EPS bead in the horizontal X/Z plane.  Opposite lobes use
 *   opposite phase so the bead sits in a low-pressure centre surrounded by a
 *   static pressure shell/tweezer cage.
 *
 * Particle size:
 *   PARTICLE_RADIUS_MM is a radius, not diameter.  For a 3.5 mm bead use 1.75.
 *   You may also pass the radius as the fourth runtime argument.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <complex.h>
#include <time.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

static void delay_ms(unsigned int ms)
{
#ifdef _WIN32
    Sleep(ms);
#else
    usleep(ms * 1000);
#endif
}

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
#define M_PI 3.14159265358979323846
#endif

#define NUM_BOARDS 4
#define CHANNELS_PER_BOARD 50
#define NUM_TRANSDUCERS (NUM_BOARDS * CHANNELS_PER_BOARD)
#define SCALE_0P1MM 10.0
#define MOVE_INC_MM 1.0

#define DEFAULT_BOARD_DISTANCE_MM 135.0
#define DEFAULT_WAVELENGTH_MM 8.574

/* RADIUS in mm. 3 mm bead -> 1.50, 3.5 mm bead -> 1.75, 4 mm bead -> 2.00. */
#define PARTICLE_RADIUS_MM 1.75

/* Static shell/twin-trap constants. These are intentionally fixed, not controls. */
#define HIRAYAMA_NODE_WEIGHT 1.0
#define HIRAYAMA_SHELL_WEIGHT 1.5
#define HIRAYAMA_SHELL_EXTRA_MM 0.50
#define FIELD_EPS_MM 0.20
#define VALIDATION_STEP_MM 0.25
#define GORKOV_BETA_PROXY 0.010

/*
 * Board/frame order is exactly the same as the working sender / FPGA direct frame.
 */
enum BoardIndex {
    BOARD_LEFT_TOP = 0,
    BOARD_RIGHT_TOP = 1,
    BOARD_LEFT_BOTTOM = 2,
    BOARD_RIGHT_BOTTOM = 3
};

static const int16_t xCols[5] = {450, 350, 250, 150, 50};
static const int16_t zRows[10] = {-450, -350, -250, -150, -50, 50, 150, 250, 350, 450};

typedef struct { double x, y, z; } Vec3;

typedef struct {
    double x_mm, y_mm, z_mm;
    double boardDistanceMm;
    double wavelengthMm;
    double particleRadiusMm;
} TrapState;

typedef struct {
    double U0;
    double hxx, hyy, hzz;
    double shellMeanI;
    double shellMinI;
    double centreI;
    double gmag;
    double offsetProxyMm;
    int stable;
} ValidationMetrics;

static double getTransducerX(enum BoardIndex board, int channel)
{
    int col = channel / 10;
    int16_t x0 = xCols[col];
    switch(board) {
        case BOARD_LEFT_TOP:     return x0 / SCALE_0P1MM;
        case BOARD_RIGHT_TOP:    return -x0 / SCALE_0P1MM;
        case BOARD_LEFT_BOTTOM:  return x0 / SCALE_0P1MM;
        case BOARD_RIGHT_BOTTOM: return -x0 / SCALE_0P1MM;
        default: return 0.0;
    }
}

static double getTransducerZ(int channel)
{
    return zRows[channel % 10] / SCALE_0P1MM;
}

static int boardIsBottom(enum BoardIndex board)
{
    return board == BOARD_LEFT_BOTTOM || board == BOARD_RIGHT_BOTTOM;
}

static Vec3 add3(Vec3 a, Vec3 b)
{
    Vec3 r = {a.x + b.x, a.y + b.y, a.z + b.z};
    return r;
}

static Vec3 scale3(Vec3 a, double s)
{
    Vec3 r = {a.x * s, a.y * s, a.z * s};
    return r;
}

static double wrapAngle(double a)
{
    while(a < 0.0) a += 2.0 * M_PI;
    while(a >= 2.0 * M_PI) a -= 2.0 * M_PI;
    return a;
}

static uint16_t normalizePhaseInt(int32_t phase)
{
    phase %= (int32_t)HOLO_PHASE_MAX;
    if(phase < 0) phase += HOLO_PHASE_MAX;
    return (uint16_t)phase;
}

static uint16_t angleToPhase(double angle)
{
    int tick = (int)llround(wrapAngle(angle) * (double)HOLO_PHASE_MAX / (2.0 * M_PI));
    return normalizePhaseInt(tick);
}

static double phaseToAngle(uint16_t phase)
{
    return 2.0 * M_PI * (double)phase / (double)HOLO_PHASE_MAX;
}

static Vec3 emitterPosition(enum BoardIndex board, int ch, double halfHeightMm)
{
    Vec3 e;
    e.x = getTransducerX(board, ch);
    e.y = boardIsBottom(board) ? -halfHeightMm : halfHeightMm;
    e.z = getTransducerZ(ch);
    return e;
}

static double dist3(Vec3 a, Vec3 b)
{
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    double dz = a.z - b.z;
    return sqrt(dx*dx + dy*dy + dz*dz);
}

/* MATD/Hirayama focus phase: theta_t = -k |p - p_t| + phi_p. */
static double focusAngle(Vec3 focus, Vec3 emitter, double wavelengthMm, double phi)
{
    double r = dist3(focus, emitter);
    return -2.0 * M_PI * r / wavelengthMm + phi;
}

static double levitationSignature(enum BoardIndex board)
{
    /* Equivalent to adding pi to the top array up to global phase; this version
       keeps the hardware-proven convention from your working sender. */
    return boardIsBottom(board) ? M_PI : 0.0;
}

static double shellRadiusMm(const TrapState *s)
{
    /* Put the focus shell just outside the bead surface, but never smaller than
       the experimentally sensible high-pressure lobe radius for a 40 kHz trap. */
    double r = s->particleRadiusMm + HIRAYAMA_SHELL_EXTRA_MM;
    double minR = 0.25 * s->wavelengthMm;
    if(r < minR) r = minR;
    return r;
}

/*
 * Static Hirayama-style shell/twin trap.
 *
 * Component A: the proven two-sided standing-wave node at the bead centre.
 * Component B: four simultaneous high-pressure focus lobes around the bead in
 *              X/Z. Opposite lobes are pi-shifted; this creates a pressure shell
 *              without time cycling the particle through several traps.
 */
static uint16_t calculateHirayamaShellPhase(double tx, double ty, double tz,
    enum BoardIndex board, int ch, const TrapState *s)
{
    double halfHeight = 0.5 * s->boardDistanceMm;
    Vec3 centre = {tx, ty, tz};
    Vec3 e = emitterPosition(board, ch, halfHeight);

    double complex acc = 0.0 + 0.0 * I;

    /* Centre levitation signature: this is the legacy working node. */
    double thetaNode = focusAngle(centre, e, s->wavelengthMm, levitationSignature(board));
    acc += HIRAYAMA_NODE_WEIGHT * cexp(I * thetaNode);

    /* Static shell foci. These are high-pressure focus points, not additional
       time-multiplexed traps. They add lateral/torsional stiffness around a
       3-4 mm EPS bead. */
    double sr = shellRadiusMm(s);
    const Vec3 dirs[4] = {
        { 1.0, 0.0,  0.0},
        {-1.0, 0.0,  0.0},
        { 0.0, 0.0,  1.0},
        { 0.0, 0.0, -1.0}
    };
    const double phi[4] = {0.0, M_PI, 0.0, M_PI};
    for(int i = 0; i < 4; ++i) {
        Vec3 q = add3(centre, scale3(dirs[i], sr));
        double thetaFocus = focusAngle(q, e, s->wavelengthMm, phi[i]);
        acc += HIRAYAMA_SHELL_WEIGHT * cexp(I * thetaFocus);
    }

    return angleToPhase(carg(acc));
}

/* Original legacy phase law, used only in self-test comparison. */
static uint16_t calculateLegacyPhase(double tx, double ty, double tz,
    enum BoardIndex board, int ch, double halfHeightMm, double wavelengthMm)
{
    Vec3 t = {tx, ty, tz};
    Vec3 e = emitterPosition(board, ch, halfHeightMm);
    double angle = focusAngle(t, e, wavelengthMm, levitationSignature(board));
    return angleToPhase(angle);
}

static void clearFrameHeader(HoloPhaseFrame *frame, uint16_t frameID)
{
    memset(frame, 0, sizeof(*frame));
    frame->magic = HOLO_PHASE_MAGIC;
    frame->version = HOLO_PHASE_VERSION;
    frame->frame_id = frameID;
    frame->phase_count = HOLO_PHASE_COUNT;
    frame->phase_max = HOLO_PHASE_MAX;
}

static void fillPhaseFrame(HoloPhaseFrame *frame, uint16_t frameID, const TrapState *s)
{
    clearFrameHeader(frame, frameID);
    for(int board = 0; board < NUM_BOARDS; ++board) {
        for(int ch = 0; ch < CHANNELS_PER_BOARD; ++ch) {
            int idx = board * CHANNELS_PER_BOARD + ch;
            frame->phases[idx] = calculateHirayamaShellPhase(
                s->x_mm, s->y_mm, s->z_mm,
                (enum BoardIndex)board, ch, s
            );
        }
    }
    frame->crc32 = holo_phase_frame_crc(frame);
}

static void fillLegacyFrame(HoloPhaseFrame *frame, uint16_t frameID, const TrapState *s)
{
    clearFrameHeader(frame, frameID);
    double halfHeight = 0.5 * s->boardDistanceMm;
    for(int board = 0; board < NUM_BOARDS; ++board) {
        for(int ch = 0; ch < CHANNELS_PER_BOARD; ++ch) {
            int idx = board * CHANNELS_PER_BOARD + ch;
            frame->phases[idx] = calculateLegacyPhase(
                s->x_mm, s->y_mm, s->z_mm,
                (enum BoardIndex)board, ch, halfHeight, s->wavelengthMm
            );
        }
    }
    frame->crc32 = holo_phase_frame_crc(frame);
}

static double complex pressureAtPoint(const HoloPhaseFrame *frame, const TrapState *s, Vec3 p)
{
    double halfHeight = 0.5 * s->boardDistanceMm;
    double k = 2.0 * M_PI / s->wavelengthMm;
    double complex pressure = 0.0 + 0.0 * I;

    for(int board = 0; board < NUM_BOARDS; ++board) {
        for(int ch = 0; ch < CHANNELS_PER_BOARD; ++ch) {
            int idx = board * CHANNELS_PER_BOARD + ch;
            Vec3 e = emitterPosition((enum BoardIndex)board, ch, halfHeight);
            double r = dist3(p, e);
            double drive = phaseToAngle(frame->phases[idx]);
            pressure += cexp(I * (drive + k * r)) / (r + FIELD_EPS_MM);
        }
    }
    return pressure;
}

static void pressureAndGradientAtPoint(const HoloPhaseFrame *frame, const TrapState *s, Vec3 p,
    double complex *outP, Vec3 *outGRe, Vec3 *outGIm)
{
    double halfHeight = 0.5 * s->boardDistanceMm;
    double k = 2.0 * M_PI / s->wavelengthMm;
    double complex P = 0.0 + 0.0 * I;
    double complex gx = 0.0 + 0.0 * I;
    double complex gy = 0.0 + 0.0 * I;
    double complex gz = 0.0 + 0.0 * I;

    for(int board = 0; board < NUM_BOARDS; ++board) {
        for(int ch = 0; ch < CHANNELS_PER_BOARD; ++ch) {
            int idx = board * CHANNELS_PER_BOARD + ch;
            Vec3 e = emitterPosition((enum BoardIndex)board, ch, halfHeight);
            double dx = p.x - e.x;
            double dy = p.y - e.y;
            double dz = p.z - e.z;
            double r = sqrt(dx*dx + dy*dy + dz*dz);
            double denom = r + FIELD_EPS_MM;
            double phase = phaseToAngle(frame->phases[idx]) + k * r;
            double complex h = cexp(I * phase) / denom;
            P += h;

            double rSafe = r > FIELD_EPS_MM ? r : FIELD_EPS_MM;
            double complex radial = h * (I * k - 1.0 / denom) / rSafe;
            gx += radial * dx;
            gy += radial * dy;
            gz += radial * dz;
        }
    }

    *outP = P;
    outGRe->x = creal(gx); outGRe->y = creal(gy); outGRe->z = creal(gz);
    outGIm->x = cimag(gx); outGIm->y = cimag(gy); outGIm->z = cimag(gz);
}

static double pressureIntensityAtPoint(const HoloPhaseFrame *frame, const TrapState *s, Vec3 p)
{
    double complex p0 = pressureAtPoint(frame, s, p);
    return creal(p0) * creal(p0) + cimag(p0) * cimag(p0);
}

static double gorkovProxyAtPoint(const HoloPhaseFrame *frame, const TrapState *s, Vec3 p)
{
    double complex P;
    Vec3 gr, gi;
    pressureAndGradientAtPoint(frame, s, p, &P, &gr, &gi);
    double p2 = creal(P) * creal(P) + cimag(P) * cimag(P);
    double g2 = gr.x*gr.x + gr.y*gr.y + gr.z*gr.z + gi.x*gi.x + gi.y*gi.y + gi.z*gi.z;
    return p2 - GORKOV_BETA_PROXY * g2;
}

static ValidationMetrics validateTrapModel(const HoloPhaseFrame *frame, const TrapState *s)
{
    ValidationMetrics m;
    memset(&m, 0, sizeof(m));
    Vec3 c = {s->x_mm, s->y_mm, s->z_mm};
    double h = VALIDATION_STEP_MM;

    m.centreI = pressureIntensityAtPoint(frame, s, c);
    m.U0 = gorkovProxyAtPoint(frame, s, c);

    Vec3 px = {c.x + h, c.y, c.z};
    Vec3 mx = {c.x - h, c.y, c.z};
    Vec3 py = {c.x, c.y + h, c.z};
    Vec3 my = {c.x, c.y - h, c.z};
    Vec3 pz = {c.x, c.y, c.z + h};
    Vec3 mz = {c.x, c.y, c.z - h};

    double UxP = gorkovProxyAtPoint(frame, s, px);
    double UxM = gorkovProxyAtPoint(frame, s, mx);
    double UyP = gorkovProxyAtPoint(frame, s, py);
    double UyM = gorkovProxyAtPoint(frame, s, my);
    double UzP = gorkovProxyAtPoint(frame, s, pz);
    double UzM = gorkovProxyAtPoint(frame, s, mz);

    m.hxx = (UxP - 2.0 * m.U0 + UxM) / (h*h);
    m.hyy = (UyP - 2.0 * m.U0 + UyM) / (h*h);
    m.hzz = (UzP - 2.0 * m.U0 + UzM) / (h*h);

    double gx = (UxP - UxM) / (2.0 * h);
    double gy = (UyP - UyM) / (2.0 * h);
    double gz = (UzP - UzM) / (2.0 * h);
    m.gmag = sqrt(gx*gx + gy*gy + gz*gz);
    double ox = fabs(m.hxx) > 1e-12 ? gx / m.hxx : 1e300;
    double oy = fabs(m.hyy) > 1e-12 ? gy / m.hyy : 1e300;
    double oz = fabs(m.hzz) > 1e-12 ? gz / m.hzz : 1e300;
    m.offsetProxyMm = sqrt(ox*ox + oy*oy + oz*oz);

    /* finite-size pressure shell around the bead radius */
    const Vec3 dirs[14] = {
        { 1, 0, 0}, {-1, 0, 0}, {0,  1, 0}, {0, -1, 0}, {0, 0,  1}, {0, 0, -1},
        { 0.5773502692,  0.5773502692,  0.5773502692},
        { 0.5773502692,  0.5773502692, -0.5773502692},
        { 0.5773502692, -0.5773502692,  0.5773502692},
        { 0.5773502692, -0.5773502692, -0.5773502692},
        {-0.5773502692,  0.5773502692,  0.5773502692},
        {-0.5773502692,  0.5773502692, -0.5773502692},
        {-0.5773502692, -0.5773502692,  0.5773502692},
        {-0.5773502692, -0.5773502692, -0.5773502692}
    };
    m.shellMinI = 1e300;
    m.shellMeanI = 0.0;
    for(int i = 0; i < 14; ++i) {
        Vec3 q = add3(c, scale3(dirs[i], s->particleRadiusMm));
        double Iq = pressureIntensityAtPoint(frame, s, q);
        if(Iq < m.shellMinI) m.shellMinI = Iq;
        m.shellMeanI += Iq;
    }
    m.shellMeanI /= 14.0;

    m.stable = (m.hxx > 0.0 && m.hyy > 0.0 && m.hzz > 0.0 &&
                m.shellMeanI > m.centreI &&
                m.offsetProxyMm <= 0.40 * s->particleRadiusMm);
    return m;
}

static void printMetrics(const char *label, const ValidationMetrics *m)
{
    printf("%s stable=%d U0=%.6e Hdiag=(%.6e, %.6e, %.6e) |gradU|=%.3e offsetProxy=%.3fmm centreI=%.6e shellMeanI=%.6e shellMinI=%.6e\n",
        label, m->stable, m->U0, m->hxx, m->hyy, m->hzz, m->gmag, m->offsetProxyMm,
        m->centreI, m->shellMeanI, m->shellMinI);
}

static int selfTest(void)
{
    TrapState s;
    s.x_mm = 0.0;
    s.y_mm = 0.0;
    s.z_mm = 0.0;
    s.boardDistanceMm = DEFAULT_BOARD_DISTANCE_MM;
    s.wavelengthMm = DEFAULT_WAVELENGTH_MM;
    s.particleRadiusMm = PARTICLE_RADIUS_MM;

    HoloPhaseFrame shell, legacy;
    fillPhaseFrame(&shell, 123, &s);
    fillLegacyFrame(&legacy, 124, &s);

    if(shell.magic != HOLO_PHASE_MAGIC || shell.version != HOLO_PHASE_VERSION ||
       shell.phase_count != HOLO_PHASE_COUNT || shell.phase_max != HOLO_PHASE_MAX ||
       shell.frame_id != 123) {
        printf("SELF TEST FAILED: header mismatch\n");
        return 1;
    }
    if(shell.crc32 != holo_phase_frame_crc(&shell)) {
        printf("SELF TEST FAILED: crc mismatch\n");
        return 1;
    }
    for(uint32_t i = 0; i < (uint32_t)HOLO_PHASE_COUNT; ++i) {
        if(shell.phases[i] >= HOLO_PHASE_MAX) {
            printf("SELF TEST FAILED: phase out of range at %u\n", i);
            return 1;
        }
    }

    ValidationMetrics ms = validateTrapModel(&shell, &s);
    ValidationMetrics ml = validateTrapModel(&legacy, &s);
    printMetrics("Hirayama shell:", &ms);
    printMetrics("legacy node:   ", &ml);

    if(!ms.stable) {
        printf("SELF TEST FAILED: Hirayama shell proxy not stable\n");
        return 1;
    }
    if(ms.hxx < 10.0 * ml.hxx || ms.hzz < 10.0 * ml.hzz) {
        printf("SELF TEST FAILED: shell did not increase lateral stiffness enough\n");
        return 1;
    }
    if(ms.hyy <= 0.0) {
        printf("SELF TEST FAILED: vertical stiffness not positive\n");
        return 1;
    }

    const double radiusTests[] = {1.50, 1.75, 2.00};
    for(size_t ri = 0; ri < sizeof(radiusTests)/sizeof(radiusTests[0]); ++ri) {
        s.particleRadiusMm = radiusTests[ri];
        fillPhaseFrame(&shell, (uint16_t)(150 + ri), &s);
        ValidationMetrics mr = validateTrapModel(&shell, &s);
        printMetrics("radius check:", &mr);
        if(!mr.stable) {
            printf("SELF TEST FAILED: radius %.2f mm failed\n", radiusTests[ri]);
            return 1;
        }
    }
    s.particleRadiusMm = PARTICLE_RADIUS_MM;

    /* Validate moved positions used by manual control. */
    const Vec3 tests[] = {{5,0,0}, {-5,0,0}, {0,5,0}, {0,0,5}, {8,3,-4}};
    for(size_t i = 0; i < sizeof(tests)/sizeof(tests[0]); ++i) {
        s.x_mm = tests[i].x;
        s.y_mm = tests[i].y;
        s.z_mm = tests[i].z;
        fillPhaseFrame(&shell, (uint16_t)(200 + i), &s);
        ValidationMetrics m = validateTrapModel(&shell, &s);
        if(!m.stable) {
            printf("SELF TEST FAILED: moved shell trap %zu failed\n", i);
            printMetrics("moved:", &m);
            return 1;
        }
    }

    printf("SELF TEST PASSED\n");
    return 0;
}

static int benchmark(void)
{
    TrapState s = {0.0, 0.0, 0.0, DEFAULT_BOARD_DISTANCE_MM, DEFAULT_WAVELENGTH_MM, PARTICLE_RADIUS_MM};
    HoloPhaseFrame frame;
    const int n = 10000;
    clock_t start = clock();
    for(int i = 0; i < n; ++i) {
        s.x_mm = 8.0 * sin(0.003 * i);
        s.z_mm = 8.0 * cos(0.003 * i);
        fillPhaseFrame(&frame, (uint16_t)i, &s);
    }
    clock_t end = clock();
    double seconds = (double)(end - start) / (double)CLOCKS_PER_SEC;
    printf("BENCH frames=%d total=%.6f s avg=%.6f ms fps=%.1f\n",
        n, seconds, 1000.0 * seconds / (double)n, (double)n / seconds);
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

static void printHelp(void)
{
    printf("commands:\n");
    printf("  h      home at 0,0,0 and send\n");
    printf("  z/a    decrease/increase Z by %.1f mm\n", MOVE_INC_MM);
    printf("  x/s    decrease/increase X by %.1f mm\n", MOVE_INC_MM);
    printf("  c/d    decrease/increase Y by %.1f mm\n", MOVE_INC_MM);
    printf("  p      print current position and validation proxy\n");
    printf("  o      2 mm circle command\n");
    printf("  q      quit\n");
}

static void sendCurrent(socket_t sock, HoloPhaseFrame *frame, uint16_t *frameID, const TrapState *s)
{
    fillPhaseFrame(frame, (*frameID)++, s);
    if(sendAll(sock, frame, sizeof(*frame)) < 0) {
        printf("send failed\n");
        return;
    }
    printf("sent Hirayama-shell frame %u at %.1f, %.1f, %.1f mm | radius %.2f mm | shell %.2f mm\n",
        frame->frame_id, s->x_mm, s->y_mm, s->z_mm, s->particleRadiusMm, shellRadiusMm(s));
}

static void handleCircleCommand(socket_t sock, HoloPhaseFrame *frame, uint16_t *frameID, TrapState *s)
{
    const double radius = 2.0;
    double baseX = s->x_mm;
    double baseY = s->y_mm;
    for(int i = 0; i < 300; ++i) {
        double theta = 2.0 * M_PI * i / 30.0;
        s->x_mm = baseX + radius * cos(theta);
        s->y_mm = baseY + radius * sin(theta);
        sendCurrent(sock, frame, frameID, s);
        delay_ms(25);
    }
    s->x_mm = baseX;
    s->y_mm = baseY;
}

int main(int argc, char **argv)
{
    if(argc >= 2 && strcmp(argv[1], "--self-test") == 0) return selfTest();
    if(argc >= 2 && strcmp(argv[1], "--bench") == 0) return benchmark();

    if(argc < 2) {
        printf("usage: %s <pi-ip-or-host> [port] [board-distance-mm] [particle-radius-mm]\n", argv[0]);
        printf("       %s --self-test\n", argv[0]);
        printf("       %s --bench\n", argv[0]);
        return 1;
    }

    const char *host = argv[1];
    uint16_t port = argc >= 3 ? (uint16_t)atoi(argv[2]) : HOLO_PHASE_TCP_PORT;

    TrapState s;
    s.x_mm = 0.0;
    s.y_mm = 0.0;
    s.z_mm = 0.0;
    s.boardDistanceMm = argc >= 4 ? atof(argv[3]) : DEFAULT_BOARD_DISTANCE_MM;
    s.wavelengthMm = DEFAULT_WAVELENGTH_MM;
    s.particleRadiusMm = argc >= 5 ? atof(argv[4]) : PARTICLE_RADIUS_MM;
    if(s.particleRadiusMm <= 0.1) s.particleRadiusMm = PARTICLE_RADIUS_MM;

#ifdef _WIN32
    WSADATA wsa;
    if(WSAStartup(MAKEWORD(2, 2), &wsa) != 0) return 1;
#endif

    socket_t sock = connectToPi(host, port);
    if(sock == INVALID_SOCKET) {
        printf("could not connect to %s:%u\n", host, port);
        return 1;
    }

    uint16_t frameID = 0;
    HoloPhaseFrame frame;
    printHelp();

    while(1) {
        sendCurrent(sock, &frame, &frameID, &s);

        int ch = getchar();
        if(ch == EOF || ch == 'q') break;

        switch(ch) {
            case 'h': s.x_mm = 0.0; s.y_mm = 0.0; s.z_mm = 0.0; break;
            case 'z': s.z_mm -= MOVE_INC_MM; break;
            case 'a': s.z_mm += MOVE_INC_MM; break;
            case 'x': s.x_mm -= MOVE_INC_MM; break;
            case 's': s.x_mm += MOVE_INC_MM; break;
            case 'c': s.y_mm -= MOVE_INC_MM; break;
            case 'd': s.y_mm += MOVE_INC_MM; break;
            case 'p': {
                fillPhaseFrame(&frame, frameID, &s);
                ValidationMetrics m = validateTrapModel(&frame, &s);
                printf("position %.1f, %.1f, %.1f mm | radius %.2f mm | lambda %.3f mm | board %.1f mm | shell %.2f mm\n",
                    s.x_mm, s.y_mm, s.z_mm, s.particleRadiusMm, s.wavelengthMm, s.boardDistanceMm, shellRadiusMm(&s));
                printMetrics("local shell proxy:", &m);
                break;
            }
            case 'o': handleCircleCommand(sock, &frame, &frameID, &s); break;
            case '\n': case '\r': break;
            default: printHelp(); break;
        }

        while(ch != '\n' && ch != '\r' && ch != EOF) ch = getchar();
    }

    close_socket(sock);
#ifdef _WIN32
    WSACleanup();
#endif
    return 0;
}
