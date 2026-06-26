/*
 * laptop_phase_sender_hirayama_twin_3p5mm_default.c
 *
 * A fixed-default, single-particle Ryuji-Hirayama/MATD-style twin trap sender
 * for your actual experiment: one 3.5 mm Styrofoam/EPS bead at the centre.
 *
 * The important experimental correction is:
 *   - actual bead diameter:             3.50 mm
 *   - actual bead radius:               1.75 mm
 *   - curvature/twin-core radius used by NEW_TRAP_CURVATURE_BOOST: 0.50 mm
 *
 * The 0.50 mm value is deliberately NOT the bead radius.  It is the small
 * local curvature boost radius that your hardware experiment showed levitates
 * the real 3.5 mm bead.  The failed 5 mm curvature setup used a much wider
 * 2.50 mm radius and diluted the centre trap too much.
 *
 * Hirayama/MATD note:
 *   The twin trap is a single levitated bead position plus the levitation
 *   signature, not two independent particles.  Therefore this sender always
 *   gives the solver exactly ONE trap centre.  The acoustic_solver's
 *   NEW_TRAP_CURVATURE_BOOST method is fixed as the hardware-tested way to
 *   apply the MATD/twin-style curvature around that centre.
 *
 * Build in your cli/New folder:
 *   gcc -O2 -std=c99 -Wall -Wextra acoustic_solver.c laptop_phase_sender_hirayama_twin_3p5mm_default-1.c -lws2_32 -lm -o laptop_phase_sender.exe
 *
 * Run with all defaults:
 *   laptop_phase_sender.exe <pi-ip>
 *
 * Optional only:
 *   laptop_phase_sender.exe 169.254.181.37 5656 135
 *
 * There are intentionally no bead-size / curvature-size runtime arguments in
 * the normal sender path, so it cannot accidentally start as 1 mm or 5 mm.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#ifdef _WIN32
#undef _WIN32_WINNT
#define _WIN32_WINNT 0x0600
#include <winsock2.h>
#include <ws2tcpip.h>
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

#include "../phase_protocol.h"
#include "acoustic_solver.h"

#define DEFAULT_BOARD_DISTANCE_MM       135.0
#define DEFAULT_BEAD_DIAMETER_MM        3.50
#define DEFAULT_BEAD_RADIUS_MM          (0.5 * DEFAULT_BEAD_DIAMETER_MM)

/*
 * Ryuji/MATD-style twin core radius for this solver/hardware.
 * This is intentionally fixed to the empirically levitating 0.50 mm value.
 */
#define HIRAYAMA_TWIN_CORE_RADIUS_MM    0.50

#define MOVE_STEP_MM                    1.0
#define HIRAYAMA_TWIN_TRAP_COUNT        1

static int finite_positive(double v)
{
    return isfinite(v) && v > 0.0;
}

static socket_t connect_to_pi(const char *host, uint16_t port)
{
    socket_t sock = socket(AF_INET, SOCK_STREAM, 0);
    if(sock == INVALID_SOCKET) return INVALID_SOCKET;

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);

#ifdef _WIN32
    /* Older MinGW often does not expose/link inet_pton correctly. */
    addr.sin_addr.s_addr = inet_addr(host);
    if(addr.sin_addr.s_addr == INADDR_NONE) {
        struct hostent *he = gethostbyname(host);
        if(!he) {
            close_socket(sock);
            return INVALID_SOCKET;
        }
        memcpy(&addr.sin_addr, he->h_addr_list[0], he->h_length);
    }
#else
    if(inet_pton(AF_INET, host, &addr.sin_addr) != 1) {
        struct hostent *he = gethostbyname(host);
        if(!he) {
            close_socket(sock);
            return INVALID_SOCKET;
        }
        memcpy(&addr.sin_addr, he->h_addr_list[0], he->h_length);
    }
#endif

    if(connect(sock, (struct sockaddr *)&addr, sizeof(addr)) == SOCKET_ERROR) {
        close_socket(sock);
        return INVALID_SOCKET;
    }

    return sock;
}

static int send_all(socket_t sock, const void *buf, size_t len)
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

static int fill_frame(HoloPhaseFrame *frame, uint16_t id, const uint16_t phases[NEW_TRANSDUCERS])
{
    if(NEW_TRANSDUCERS != HOLO_PHASE_COUNT) {
        printf("configuration error: NEW_TRANSDUCERS=%d but HOLO_PHASE_COUNT=%d\n",
               (int)NEW_TRANSDUCERS, (int)HOLO_PHASE_COUNT);
        return -1;
    }

    memset(frame, 0, sizeof(*frame));
    frame->magic = HOLO_PHASE_MAGIC;
    frame->version = HOLO_PHASE_VERSION;
    frame->frame_id = id;
    frame->phase_count = HOLO_PHASE_COUNT;
    frame->phase_max = HOLO_PHASE_MAX;

    for(int i = 0; i < NEW_TRANSDUCERS; ++i) {
        if(phases[i] >= HOLO_PHASE_MAX) {
            printf("configuration error: phase[%d]=%u outside 0..%u\n",
                   i, phases[i], (unsigned)(HOLO_PHASE_MAX - 1));
            return -1;
        }
        frame->phases[i] = phases[i];
    }

    frame->crc32 = holo_phase_frame_crc(frame);
    return 0;
}

static void configure_hirayama_twin(NewSolverConfig *cfg, double board_distance_mm)
{
    new_solver_default_config(cfg);

    if(!finite_positive(board_distance_mm)) {
        board_distance_mm = DEFAULT_BOARD_DISTANCE_MM;
    }

    cfg->board_distance_mm = board_distance_mm;
    cfg->method = NEW_TRAP_CURVATURE_BOOST;
    cfg->cage_radius_mm = HIRAYAMA_TWIN_CORE_RADIUS_MM;
}

/*
 * This is the important part: Hirayama/MATD twin code for this solver path.
 * It creates ONE physical bead trap.  The "twin" is the levitation signature
 * generated by the solver/array phase law around that focus, not a second bead.
 */
static int make_hirayama_twin_frame(const NewSolverConfig *cfg,
                                    NewVec3 centre,
                                    uint16_t frame_id,
                                    uint16_t phases[NEW_TRANSDUCERS],
                                    HoloPhaseFrame *frame)
{
    NewVec3 twin_trap[HIRAYAMA_TWIN_TRAP_COUNT];
    twin_trap[0] = centre;

    new_solver_make_phases(cfg, twin_trap, HIRAYAMA_TWIN_TRAP_COUNT, phases);
    return fill_frame(frame, frame_id, phases);
}

static double safe_score(const NewSolverConfig *cfg,
                         const uint16_t phases[NEW_TRANSDUCERS],
                         NewVec3 centre,
                         double radius_mm)
{
    if(!finite_positive(radius_mm)) radius_mm = HIRAYAMA_TWIN_CORE_RADIUS_MM;
    double score = new_solver_trap_score(cfg, phases, centre, radius_mm);
    if(!isfinite(score)) return -1.0;
    return score;
}

static void print_defaults(void)
{
    printf("defaults: method=Ryuji-Hirayama/MATD twin via %s\n",
           new_solver_method_name(NEW_TRAP_CURVATURE_BOOST));
    printf("          actual bead=%.2f mm diameter / %.2f mm radius\n",
           DEFAULT_BEAD_DIAMETER_MM, DEFAULT_BEAD_RADIUS_MM);
    printf("          twin/core curvature radius=%.2f mm, effective diameter=%.2f mm\n",
           HIRAYAMA_TWIN_CORE_RADIUS_MM, 2.0 * HIRAYAMA_TWIN_CORE_RADIUS_MM);
    printf("          normal run accepts only: <pi-ip> [port] [board-distance-mm]\n");
}

static void print_status(const NewSolverConfig *cfg,
                         const uint16_t phases[NEW_TRANSDUCERS],
                         NewVec3 centre)
{
    double core_score = safe_score(cfg, phases, centre, HIRAYAMA_TWIN_CORE_RADIUS_MM);
    double bead_score = safe_score(cfg, phases, centre, DEFAULT_BEAD_RADIUS_MM);

    printf("method=%s centre=(%.1f %.1f %.1f) board=%.1f mm | ",
           new_solver_method_name(cfg->method),
           centre.x, centre.y, centre.z,
           cfg->board_distance_mm);
    printf("actual bead=%.2f mm diameter / %.2f mm radius | ",
           DEFAULT_BEAD_DIAMETER_MM, DEFAULT_BEAD_RADIUS_MM);
    printf("twin/core radius=%.2f mm | core_score=%.6g bead_radius_score=%.6g\n",
           cfg->cage_radius_mm,
           core_score,
           bead_score);
}

static int self_test(double board_distance_mm)
{
    NewSolverConfig cfg;
    configure_hirayama_twin(&cfg, board_distance_mm);

    if(cfg.method != NEW_TRAP_CURVATURE_BOOST) {
        printf("SELF TEST FAILED: method is not curvature boost\n");
        return 1;
    }
    if(fabs(cfg.cage_radius_mm - HIRAYAMA_TWIN_CORE_RADIUS_MM) > 1e-9) {
        printf("SELF TEST FAILED: twin/core radius changed from %.2f mm\n",
               HIRAYAMA_TWIN_CORE_RADIUS_MM);
        return 1;
    }

    NewVec3 centre = {0.0, 0.0, 0.0};
    uint16_t phases[NEW_TRANSDUCERS];
    HoloPhaseFrame frame;

    if(make_hirayama_twin_frame(&cfg, centre, 123, phases, &frame) != 0) {
        printf("SELF TEST FAILED: could not make frame\n");
        return 1;
    }

    if(frame.magic != HOLO_PHASE_MAGIC ||
       frame.version != HOLO_PHASE_VERSION ||
       frame.frame_id != 123 ||
       frame.phase_count != HOLO_PHASE_COUNT ||
       frame.phase_max != HOLO_PHASE_MAX) {
        printf("SELF TEST FAILED: bad frame header\n");
        return 1;
    }

    if(frame.crc32 != holo_phase_frame_crc(&frame)) {
        printf("SELF TEST FAILED: CRC mismatch\n");
        return 1;
    }

    for(int i = 0; i < NEW_TRANSDUCERS; ++i) {
        if(frame.phases[i] >= HOLO_PHASE_MAX) {
            printf("SELF TEST FAILED: phase out of range at %d: %u\n", i, frame.phases[i]);
            return 1;
        }
    }

    double core_score = safe_score(&cfg, phases, centre, HIRAYAMA_TWIN_CORE_RADIUS_MM);
    double bead_score = safe_score(&cfg, phases, centre, DEFAULT_BEAD_RADIUS_MM);
    if(core_score < 0.0 || bead_score < 0.0) {
        printf("SELF TEST FAILED: invalid trap score\n");
        return 1;
    }

    printf("SELF TEST PASSED\n");
    print_defaults();
    print_status(&cfg, phases, centre);
    return 0;
}

static int compare_defaults(double board_distance_mm)
{
    NewVec3 centre = {0.0, 0.0, 0.0};
    uint16_t phases[NEW_TRANSDUCERS];
    HoloPhaseFrame frame;

    printf("curvature comparison for actual 3.5 mm bead at centre\n");
    printf("board_distance_mm=%.1f\n", board_distance_mm);
    printf("radius_mm, effective_diameter_mm, core_score, bead_radius_score, note\n");

    const double radii[] = {0.35, 0.40, 0.45, 0.50, 0.55, 0.60, 0.75, 1.00, 1.75, 2.50};
    const int count = (int)(sizeof(radii) / sizeof(radii[0]));

    for(int i = 0; i < count; ++i) {
        NewSolverConfig cfg;
        configure_hirayama_twin(&cfg, board_distance_mm);
        cfg.cage_radius_mm = radii[i];

        if(make_hirayama_twin_frame(&cfg, centre, (uint16_t)i, phases, &frame) != 0) {
            return 1;
        }

        printf("%.2f, %.2f, %.8g, %.8g, %s\n",
               radii[i],
               2.0 * radii[i],
               safe_score(&cfg, phases, centre, radii[i]),
               safe_score(&cfg, phases, centre, DEFAULT_BEAD_RADIUS_MM),
               fabs(radii[i] - HIRAYAMA_TWIN_CORE_RADIUS_MM) < 1e-9 ? "DEFAULT / empirically levitated" : "debug only");
    }

    return 0;
}

static void print_help(void)
{
    printf("commands:\n");
    printf("  h      home centre to 0,0,0\n");
    printf("  x/s    move centre X -/+ %.1f mm\n", MOVE_STEP_MM);
    printf("  c/d    move centre Y -/+ %.1f mm\n", MOVE_STEP_MM);
    printf("  z/a    move centre Z -/+ %.1f mm\n", MOVE_STEP_MM);
    printf("  p      print settings/status\n");
    printf("  r      resend same trap\n");
    printf("  q      quit\n");
}

int main(int argc, char **argv)
{
    if(argc >= 2 && strcmp(argv[1], "--self-test") == 0) {
        double board_distance_mm = argc >= 3 ? atof(argv[2]) : DEFAULT_BOARD_DISTANCE_MM;
        return self_test(board_distance_mm);
    }

    if(argc >= 2 && strcmp(argv[1], "--compare") == 0) {
        double board_distance_mm = argc >= 3 ? atof(argv[2]) : DEFAULT_BOARD_DISTANCE_MM;
        return compare_defaults(board_distance_mm);
    }

    if(argc < 2) {
        printf("usage: %s <pi-ip-or-host> [port] [board-distance-mm]\n", argv[0]);
        printf("       %s --self-test [board-distance-mm]\n", argv[0]);
        printf("       %s --compare [board-distance-mm]\n", argv[0]);
        print_defaults();
        return 1;
    }

    const char *host = argv[1];
    uint16_t port = argc >= 3 ? (uint16_t)atoi(argv[2]) : HOLO_PHASE_TCP_PORT;
    double board_distance_mm = argc >= 4 ? atof(argv[3]) : DEFAULT_BOARD_DISTANCE_MM;
    if(!finite_positive(board_distance_mm)) board_distance_mm = DEFAULT_BOARD_DISTANCE_MM;

    if(argc >= 5) {
        printf("warning: extra arguments ignored. Bead/curvature are fixed defaults in this version.\n");
        print_defaults();
    }

    NewSolverConfig cfg;
    configure_hirayama_twin(&cfg, board_distance_mm);

#ifdef _WIN32
    WSADATA wsa;
    if(WSAStartup(MAKEWORD(2, 2), &wsa) != 0) return 1;
#endif

    socket_t sock = connect_to_pi(host, port);
    if(sock == INVALID_SOCKET) {
        printf("could not connect to %s:%u\n", host, port);
#ifdef _WIN32
        WSACleanup();
#endif
        return 1;
    }

    NewVec3 centre = {0.0, 0.0, 0.0};
    uint16_t frame_id = 0;
    uint16_t phases[NEW_TRANSDUCERS];
    HoloPhaseFrame frame;

    print_help();
    print_defaults();

    for(;;) {
        if(make_hirayama_twin_frame(&cfg, centre, frame_id++, phases, &frame) != 0) {
            printf("frame generation failed\n");
            break;
        }

        if(send_all(sock, &frame, sizeof(frame)) < 0) {
            printf("send failed\n");
            break;
        }

        printf("sent frame %u | ", frame.frame_id);
        print_status(&cfg, phases, centre);

        int ch = getchar();
        if(ch == EOF || ch == 'q') break;

        switch(ch) {
            case 'h': centre.x = centre.y = centre.z = 0.0; break;
            case 'x': centre.x -= MOVE_STEP_MM; break;
            case 's': centre.x += MOVE_STEP_MM; break;
            case 'c': centre.y -= MOVE_STEP_MM; break;
            case 'd': centre.y += MOVE_STEP_MM; break;
            case 'z': centre.z -= MOVE_STEP_MM; break;
            case 'a': centre.z += MOVE_STEP_MM; break;
            case 'p': print_help(); print_defaults(); print_status(&cfg, phases, centre); break;
            case 'r': break;
            case '\n': case '\r': break;
            default: print_help(); break;
        }

        while(ch != '\n' && ch != '\r' && ch != EOF) ch = getchar();
    }

    close_socket(sock);
#ifdef _WIN32
    WSACleanup();
#endif
    return 0;
}
