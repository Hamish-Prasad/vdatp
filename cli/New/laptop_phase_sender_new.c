/*
 * laptop_phase_sender_curvature_center_3p5mm_Aplus.c
 *
 * One stationary/controllable centre trap, tuned for an actual 3.5 mm
 * Styrofoam/EPS bead.  This deliberately keeps the empirically successful
 * small curvature-boost radius instead of matching the boost radius to bead
 * radius.  In this solver, cfg.cage_radius_mm is the local curvature sampling
 * radius used by NEW_TRAP_CURVATURE_BOOST, not the physical bead radius.
 *
 * Your experiment showed:
 *   - bead diameter printed as 1.0 mm -> cage radius 0.50 mm -> levitates 3.5 mm bead
 *   - bead/curvature around 5.0 mm -> cage radius 2.50 mm -> does not levitate
 *
 * Therefore the correct default for a 3.5 mm physical bead is:
 *   - actual bead diameter: 3.5 mm
 *   - actual bead radius:   1.75 mm
 *   - curvature radius:     0.50 mm  (empirically verified working core)
 *
 * Build in your cli/New folder, using your real solver:
 *   gcc -O2 -std=c99 -Wall -Wextra acoustic_solver.c laptop_phase_sender.c -lws2_32 -lm -o laptop_phase_sender.exe
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

#define DEFAULT_BOARD_DISTANCE_MM      135.0

/* Actual physical bead. */
#define DEFAULT_BEAD_DIAMETER_MM       3.5
#define DEFAULT_BEAD_RADIUS_MM         (0.5 * DEFAULT_BEAD_DIAMETER_MM)

/*
 * Important: this is NOT bead radius.  This is the curvature-sampling radius
 * inside NEW_TRAP_CURVATURE_BOOST.  Your hardware experiment says the small
 * 0.50 mm value levitates the 3.5 mm bead, while 2.50 mm does not.
 */
#define DEFAULT_CURVATURE_RADIUS_MM    0.50
#define MIN_CURVATURE_RADIUS_MM        0.20
#define MAX_CURVATURE_RADIUS_MM        2.50
#define FINE_CURVATURE_STEP_MM         0.05
#define COARSE_CURVATURE_STEP_MM       0.25

#define MOVE_STEP_MM                   1.0

static double clamp_double(double v, double lo, double hi)
{
    if(v < lo) return lo;
    if(v > hi) return hi;
    return v;
}

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

static void configure_curvature_boost(NewSolverConfig *cfg,
                                      double board_distance_mm,
                                      double curvature_radius_mm)
{
    new_solver_default_config(cfg);
    cfg->board_distance_mm = board_distance_mm;
    cfg->method = NEW_TRAP_CURVATURE_BOOST;
    cfg->cage_radius_mm = clamp_double(curvature_radius_mm,
                                       MIN_CURVATURE_RADIUS_MM,
                                       MAX_CURVATURE_RADIUS_MM);
}

static int make_center_frame(const NewSolverConfig *cfg,
                             NewVec3 centre,
                             uint16_t frame_id,
                             uint16_t phases[NEW_TRANSDUCERS],
                             HoloPhaseFrame *frame)
{
    NewVec3 traps[1];
    traps[0] = centre;
    new_solver_make_phases(cfg, traps, 1, phases);
    return fill_frame(frame, frame_id, phases);
}

static double finite_score(const NewSolverConfig *cfg,
                           const uint16_t phases[NEW_TRANSDUCERS],
                           NewVec3 centre,
                           double sample_radius_mm)
{
    double r = sample_radius_mm;
    if(!finite_positive(r)) r = DEFAULT_CURVATURE_RADIUS_MM;
    r = clamp_double(r, 0.20, 4.00);

    double score = new_solver_trap_score(cfg, phases, centre, r);
    if(!isfinite(score)) return -1.0;
    return score;
}

static void print_status(const NewSolverConfig *cfg,
                         const uint16_t phases[NEW_TRANSDUCERS],
                         NewVec3 centre,
                         double bead_diameter_mm)
{
    double bead_radius_mm = 0.5 * bead_diameter_mm;
    double core_score = finite_score(cfg, phases, centre, cfg->cage_radius_mm);
    double bead_surface_score = finite_score(cfg, phases, centre, bead_radius_mm);

    printf("method=%s centre=(%.1f %.1f %.1f) board=%.1f mm | ",
           new_solver_method_name(cfg->method),
           centre.x, centre.y, centre.z,
           cfg->board_distance_mm);
    printf("actual bead=%.2f mm diameter / %.2f mm radius | ",
           bead_diameter_mm, bead_radius_mm);
    printf("curvature radius=%.2f mm / effective curvature diameter=%.2f mm | ",
           cfg->cage_radius_mm, 2.0 * cfg->cage_radius_mm);
    printf("core_score=%.6g bead_surface_score=%.6g\n",
           core_score, bead_surface_score);
}

static int self_test(double board_distance_mm,
                     double bead_diameter_mm,
                     double curvature_radius_mm)
{
    if(!finite_positive(board_distance_mm)) board_distance_mm = DEFAULT_BOARD_DISTANCE_MM;
    if(!finite_positive(bead_diameter_mm)) bead_diameter_mm = DEFAULT_BEAD_DIAMETER_MM;
    if(!finite_positive(curvature_radius_mm)) curvature_radius_mm = DEFAULT_CURVATURE_RADIUS_MM;

    NewSolverConfig cfg;
    configure_curvature_boost(&cfg, board_distance_mm, curvature_radius_mm);

    NewVec3 centre = {0.0, 0.0, 0.0};
    uint16_t phases[NEW_TRANSDUCERS];
    HoloPhaseFrame frame;

    if(make_center_frame(&cfg, centre, 123, phases, &frame) != 0) {
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

    double score_core = finite_score(&cfg, phases, centre, cfg.cage_radius_mm);
    double score_bead = finite_score(&cfg, phases, centre, 0.5 * bead_diameter_mm);
    if(!isfinite(score_core) || score_core < 0.0 || !isfinite(score_bead) || score_bead < 0.0) {
        printf("SELF TEST FAILED: invalid score\n");
        return 1;
    }

    printf("SELF TEST PASSED\n");
    print_status(&cfg, phases, centre, bead_diameter_mm);
    return 0;
}

static int sweep(double board_distance_mm, double bead_diameter_mm)
{
    if(!finite_positive(board_distance_mm)) board_distance_mm = DEFAULT_BOARD_DISTANCE_MM;
    if(!finite_positive(bead_diameter_mm)) bead_diameter_mm = DEFAULT_BEAD_DIAMETER_MM;

    printf("sweep for actual bead %.2f mm diameter, board %.1f mm\n",
           bead_diameter_mm, board_distance_mm);
    printf("radius_mm, effective_diameter_mm, core_score, bead_surface_score\n");

    double best_r = DEFAULT_CURVATURE_RADIUS_MM;
    double best_score = -1.0;

    for(double r = 0.20; r <= 2.501; r += 0.05) {
        NewSolverConfig cfg;
        configure_curvature_boost(&cfg, board_distance_mm, r);
        NewVec3 centre = {0.0, 0.0, 0.0};
        uint16_t phases[NEW_TRANSDUCERS];
        HoloPhaseFrame frame;
        if(make_center_frame(&cfg, centre, 1, phases, &frame) != 0) return 1;

        double core = finite_score(&cfg, phases, centre, cfg.cage_radius_mm);
        double bead = finite_score(&cfg, phases, centre, 0.5 * bead_diameter_mm);
        printf("%.2f, %.2f, %.8g, %.8g\n", r, 2.0 * r, core, bead);

        /* Hardware observation favours core strength near the centre. */
        if(core > best_score) {
            best_score = core;
            best_r = r;
        }
    }

    printf("best by core_score: curvature radius %.2f mm, effective diameter %.2f mm, score %.8g\n",
           best_r, 2.0 * best_r, best_score);
    printf("recommended hardware starting point remains %.2f mm radius because that matches the levitating 1 mm setup but with actual bead labelled as 3.5 mm.\n",
           DEFAULT_CURVATURE_RADIUS_MM);
    return 0;
}

static void print_help(void)
{
    printf("commands:\n");
    printf("  h      home centre to 0,0,0\n");
    printf("  x/s    move centre X -/+ %.1f mm\n", MOVE_STEP_MM);
    printf("  c/d    move centre Y -/+ %.1f mm\n", MOVE_STEP_MM);
    printf("  z/a    move centre Z -/+ %.1f mm\n", MOVE_STEP_MM);
    printf("  +/-    curvature radius -/+ %.2f mm\n", FINE_CURVATURE_STEP_MM);
    printf("  [/]    curvature radius -/+ %.2f mm\n", COARSE_CURVATURE_STEP_MM);
    printf("  r      reset curvature radius to %.2f mm\n", DEFAULT_CURVATURE_RADIUS_MM);
    printf("  p      print settings\n");
    printf("  q      quit\n");
}

int main(int argc, char **argv)
{
    if(argc >= 2 && strcmp(argv[1], "--self-test") == 0) {
        double board_distance_mm = argc >= 3 ? atof(argv[2]) : DEFAULT_BOARD_DISTANCE_MM;
        double bead_diameter_mm = argc >= 4 ? atof(argv[3]) : DEFAULT_BEAD_DIAMETER_MM;
        double curvature_radius_mm = argc >= 5 ? atof(argv[4]) : DEFAULT_CURVATURE_RADIUS_MM;
        return self_test(board_distance_mm, bead_diameter_mm, curvature_radius_mm);
    }

    if(argc >= 2 && strcmp(argv[1], "--sweep") == 0) {
        double board_distance_mm = argc >= 3 ? atof(argv[2]) : DEFAULT_BOARD_DISTANCE_MM;
        double bead_diameter_mm = argc >= 4 ? atof(argv[3]) : DEFAULT_BEAD_DIAMETER_MM;
        return sweep(board_distance_mm, bead_diameter_mm);
    }

    if(argc < 2) {
        printf("usage: %s <pi-ip-or-host> [port] [board-distance-mm] [bead-diameter-mm] [curvature-radius-mm]\n", argv[0]);
        printf("       %s --self-test [board-distance-mm] [bead-diameter-mm] [curvature-radius-mm]\n", argv[0]);
        printf("       %s --sweep [board-distance-mm] [bead-diameter-mm]\n", argv[0]);
        printf("defaults: board=%.1f mm, actual bead=%.1f mm diameter, curvature radius=%.2f mm\n",
               DEFAULT_BOARD_DISTANCE_MM,
               DEFAULT_BEAD_DIAMETER_MM,
               DEFAULT_CURVATURE_RADIUS_MM);
        return 1;
    }

    const char *host = argv[1];
    uint16_t port = argc >= 3 ? (uint16_t)atoi(argv[2]) : HOLO_PHASE_TCP_PORT;
    double board_distance_mm = argc >= 4 ? atof(argv[3]) : DEFAULT_BOARD_DISTANCE_MM;
    double bead_diameter_mm = argc >= 5 ? atof(argv[4]) : DEFAULT_BEAD_DIAMETER_MM;
    double curvature_radius_mm = argc >= 6 ? atof(argv[5]) : DEFAULT_CURVATURE_RADIUS_MM;

    if(!finite_positive(board_distance_mm)) board_distance_mm = DEFAULT_BOARD_DISTANCE_MM;
    if(!finite_positive(bead_diameter_mm)) bead_diameter_mm = DEFAULT_BEAD_DIAMETER_MM;
    if(!finite_positive(curvature_radius_mm)) curvature_radius_mm = DEFAULT_CURVATURE_RADIUS_MM;

    NewSolverConfig cfg;
    configure_curvature_boost(&cfg, board_distance_mm, curvature_radius_mm);

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
    printf("A+ centre setup: actual bead=%.2f mm diameter / %.2f mm radius, curvature radius=%.2f mm.\n",
           bead_diameter_mm,
           0.5 * bead_diameter_mm,
           cfg.cage_radius_mm);
    printf("Note: curvature radius is the solver's local boost distance, not the bead radius.\n");

    for(;;) {
        if(make_center_frame(&cfg, centre, frame_id++, phases, &frame) != 0) {
            printf("frame generation failed\n");
            break;
        }

        if(send_all(sock, &frame, sizeof(frame)) < 0) {
            printf("send failed\n");
            break;
        }

        printf("sent frame %u | ", frame.frame_id);
        print_status(&cfg, phases, centre, bead_diameter_mm);

        int ch = getchar();
        if(ch == EOF || ch == 'q') break;

        switch(ch) {
            case 'h': centre.x = centre.y = centre.z = 0.0; break;
            case '+': cfg.cage_radius_mm = clamp_double(cfg.cage_radius_mm + FINE_CURVATURE_STEP_MM, MIN_CURVATURE_RADIUS_MM, MAX_CURVATURE_RADIUS_MM); break;
            case '-': cfg.cage_radius_mm = clamp_double(cfg.cage_radius_mm - FINE_CURVATURE_STEP_MM, MIN_CURVATURE_RADIUS_MM, MAX_CURVATURE_RADIUS_MM); break;
            case ']': cfg.cage_radius_mm = clamp_double(cfg.cage_radius_mm + COARSE_CURVATURE_STEP_MM, MIN_CURVATURE_RADIUS_MM, MAX_CURVATURE_RADIUS_MM); break;
            case '[': cfg.cage_radius_mm = clamp_double(cfg.cage_radius_mm - COARSE_CURVATURE_STEP_MM, MIN_CURVATURE_RADIUS_MM, MAX_CURVATURE_RADIUS_MM); break;
            case 'r': cfg.cage_radius_mm = DEFAULT_CURVATURE_RADIUS_MM; break;
            case 'x': centre.x -= MOVE_STEP_MM; break;
            case 's': centre.x += MOVE_STEP_MM; break;
            case 'c': centre.y -= MOVE_STEP_MM; break;
            case 'd': centre.y += MOVE_STEP_MM; break;
            case 'z': centre.z -= MOVE_STEP_MM; break;
            case 'a': centre.z += MOVE_STEP_MM; break;
            case 'p': print_help(); print_status(&cfg, phases, centre, bead_diameter_mm); break;
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
