/*
 * Windows build:
 *   gcc -O2 -std=c11 -Wall -Wextra laptop_phase_sender_vector.c -I.. -o laptop_phase_sender_vector.exe -lm -lws2_32
 *
 *              .\laptop_phase_sender_vector.exe 169.254.85.139 two --port 5656
 *          current ip - 169.254.85.139
 *          169.254.85.139
 */

#ifdef _WIN32
#define _CRT_SECURE_NO_WARNINGS
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#endif

#include <errno.h>
#include <math.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifdef _WIN32
#ifndef _WIN32_WINNT
#define _WIN32_WINNT 0x0600
#endif
#ifndef WINVER
#define WINVER 0x0600
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <conio.h>
typedef SOCKET socket_t;
#define close_socket closesocket
#else
#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>
typedef int socket_t;
#define INVALID_SOCKET (-1)
#define SOCKET_ERROR (-1)
#define close_socket close
#endif

#include "../phase_protocol.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define VECTOR_TRANSDUCERS 200
#define VECTOR_CHANNELS_PER_BOARD 50
#define VECTOR_DEFAULT_BOARD_MM 135.0
#define VECTOR_DEFAULT_LOOKAHEAD_MM 1.0
#define VECTOR_DEFAULT_ENSEMBLE_MS 9.0
#define VECTOR_DEFAULT_MOVE_STEP_MM 0.5
#define VECTOR_FREQUENCY_HZ 40000.0
#define VECTOR_SOUND_SPEED_M_S 343.0
#define VECTOR_LATERAL_DWELL_GAIN 6.0
#define VECTOR_BASE_DWELL 1.0

typedef struct Vec3 {
    double x, y, z;
} Vec3;

typedef struct TrackingState {
    int count;
    Vec3 bead[2];
    Vec3 goal[2];
} TrackingState;

static const double x_cols_mm[5] = {45.0, 35.0, 25.0, 15.0, 5.0};
static const double z_rows_mm[10] = {-45.0, -35.0, -25.0, -15.0, -5.0,
                                      5.0,  15.0,  25.0,  35.0, 45.0};
static volatile sig_atomic_t keep_running = 1;

static void stop_handler(int ignored)
{
    (void)ignored;
    keep_running = 0;
}

static uint64_t wall_time_us(void)
{
#ifdef _WIN32
    return (uint64_t)GetTickCount() * 1000u;
#else
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000u + (uint64_t)ts.tv_nsec / 1000u;
#endif
}

static void sleep_us(uint64_t microseconds)
{
#ifdef _WIN32
    DWORD ms = (DWORD)((microseconds + 999u) / 1000u);
    if(ms < 1u) ms = 1u;
    Sleep(ms);
#else
    struct timespec ts;
    ts.tv_sec = (time_t)(microseconds / 1000000u);
    ts.tv_nsec = (long)((microseconds % 1000000u) * 1000u);
    while(nanosleep(&ts, &ts) != 0 && errno == EINTR) { }
#endif
}

static double distance_sq(Vec3 a, Vec3 b)
{
    double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
    return dx*dx + dy*dy + dz*dz;
}

static int finite_vec(Vec3 p)
{
    return isfinite(p.x) && isfinite(p.y) && isfinite(p.z);
}

static int inside_workspace(Vec3 p, double board_mm)
{
    double y_limit = 0.5 * board_mm - 7.0;
    return finite_vec(p) && fabs(p.x) <= 40.0 && fabs(p.y) <= y_limit && fabs(p.z) <= 40.0;
}

static int load_tracking_state(const char *path, int expected_count,
                               double board_mm, TrackingState *out)
{
    FILE *file = fopen(path, "r");
    char mode[16];
    TrackingState next;
    int fields;
    if(!file) return -1;
    memset(&next, 0, sizeof(next));
    if(fscanf(file, "%15s", mode) != 1) {
        fclose(file);
        return -1;
    }

    if(expected_count == 1 && strcmp(mode, "one") == 0) {
        next.count = 1;
        fields = fscanf(file, "%lf %lf %lf %lf %lf %lf",
            &next.bead[0].x, &next.bead[0].y, &next.bead[0].z,
            &next.goal[0].x, &next.goal[0].y, &next.goal[0].z);
        if(fields != 6) { fclose(file); return -1; }
    } else if(expected_count == 2 && strcmp(mode, "two") == 0) {
        next.count = 2;
        fields = fscanf(file, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
            &next.bead[0].x, &next.bead[0].y, &next.bead[0].z,
            &next.bead[1].x, &next.bead[1].y, &next.bead[1].z,
            &next.goal[0].x, &next.goal[0].y, &next.goal[0].z,
            &next.goal[1].x, &next.goal[1].y, &next.goal[1].z);
        if(fields != 12) { fclose(file); return -1; }
    } else {
        fclose(file);
        return -1;
    }
    fclose(file);

    for(int i = 0; i < next.count; i++) {
        if(!inside_workspace(next.bead[i], board_mm) ||
           !inside_workspace(next.goal[i], board_mm)) return -1;
    }
    *out = next;
    return 0;
}

static void assign_goals(const TrackingState *state, int assignment[2])
{
    assignment[0] = 0;
    assignment[1] = state->count == 2 ? 1 : 0;
    if(state->count == 2) {
        double direct = distance_sq(state->bead[0], state->goal[0]) +
                        distance_sq(state->bead[1], state->goal[1]);
        double crossed = distance_sq(state->bead[0], state->goal[1]) +
                         distance_sq(state->bead[1], state->goal[0]);
        if(crossed < direct) {
            assignment[0] = 1;
            assignment[1] = 0;
        }
    }
}

static Vec3 make_waypoint(Vec3 bead, Vec3 goal, double lookahead_mm,
                          Vec3 *direction)
{
    Vec3 delta = {goal.x - bead.x, goal.y - bead.y, goal.z - bead.z};
    double distance = sqrt(delta.x*delta.x + delta.y*delta.y + delta.z*delta.z);
    double step;
    if(distance < 1e-9) {
        *direction = (Vec3){0.0, 0.0, 0.0};
        return goal;
    }
    direction->x = delta.x / distance;
    direction->y = delta.y / distance;
    direction->z = delta.z / distance;
    step = distance < lookahead_mm ? distance : lookahead_mm;
    return (Vec3){bead.x + step*direction->x,
                  bead.y + step*direction->y,
                  bead.z + step*direction->z};
}

static void dwell_weights(Vec3 direction, double weight[3])
{
    weight[0] = VECTOR_LATERAL_DWELL_GAIN * fabs(direction.x) + VECTOR_BASE_DWELL;
    weight[1] = fabs(direction.y) + VECTOR_BASE_DWELL;
    weight[2] = VECTOR_LATERAL_DWELL_GAIN * fabs(direction.z) + VECTOR_BASE_DWELL;
}

static void manual_dwell_weights(double weight[3])
{
    weight[0] = 4.0;
    weight[1] = 1.0;
    weight[2] = 4.0;
}

static Vec3 manual_home(int count, int particle)
{
    if(count == 1) return (Vec3){0.0, 0.0, 0.0};
    return (Vec3){particle == 0 ? 8.0 : -8.0, 0.0, 0.0};
}

static void init_manual_state(TrackingState *state, int count)
{
    memset(state, 0, sizeof(*state));
    state->count = count;
    for(int i = 0; i < count; i++) {
        state->goal[i] = manual_home(count, i);
        state->bead[i] = state->goal[i];
    }
}

static int read_key_nonblocking(void)
{
#ifdef _WIN32
    return _kbhit() ? _getch() : -1;
#else
    fd_set set;
    struct timeval timeout = {0, 0};
    FD_ZERO(&set);
    FD_SET(STDIN_FILENO, &set);
    if(select(STDIN_FILENO + 1, &set, NULL, NULL, &timeout) > 0) {
        unsigned char key;
        return read(STDIN_FILENO, &key, 1) == 1 ? (int)key : -1;
    }
    return -1;
#endif
}

static void print_manual_help(void)
{
    printf("manual keys: 1/2 select particle | x/s X-/+ | c/d Y-/+ | z/a Z-/+\n");
    printf("             h home selected | 0 home all | p print | q quit\n");
}

static void print_manual_state(const TrackingState *state, int selected)
{
    for(int i = 0; i < state->count; i++)
        printf("%c particle %d trap=(%.2f %.2f %.2f) mm\n",
               i == selected ? '>' : ' ', i + 1,
               state->goal[i].x, state->goal[i].y, state->goal[i].z);
}

static void poll_manual_commands(TrackingState *state, int *selected,
                                 double move_step_mm, double board_mm)
{
    int key;
    while((key = read_key_nonblocking()) >= 0) {
        Vec3 next = state->goal[*selected];
        if(key == 'q' || key == 'Q') { keep_running = 0; return; }
        if(key == '1') { *selected = 0; continue; }
        if(key == '2' && state->count == 2) { *selected = 1; continue; }
        if(key == 'x') next.x -= move_step_mm;
        else if(key == 's') next.x += move_step_mm;
        else if(key == 'c') next.y -= move_step_mm;
        else if(key == 'd') next.y += move_step_mm;
        else if(key == 'z') next.z -= move_step_mm;
        else if(key == 'a') next.z += move_step_mm;
        else if(key == 'h') next = manual_home(state->count, *selected);
        else if(key == '0') {
            for(int i = 0; i < state->count; i++) state->goal[i] = manual_home(state->count, i);
            print_manual_state(state, *selected);
            continue;
        } else if(key == 'p') {
            print_manual_state(state, *selected);
            continue;
        } else if(key == '\r' || key == '\n') continue;
        else { print_manual_help(); continue; }

        if(inside_workspace(next, board_mm)) {
            state->goal[*selected] = next;
            state->bead[*selected] = next;
            print_manual_state(state, *selected);
        } else {
            printf("movement rejected: outside validated workspace\n");
        }
    }
}

static Vec3 emitter_position(int index, double board_mm)
{
    int board = index / VECTOR_CHANNELS_PER_BOARD;
    int channel = index % VECTOR_CHANNELS_PER_BOARD;
    double x0 = x_cols_mm[channel / 10];
    int lower = board == 2 || board == 3;
    return (Vec3){(board == 0 || board == 2) ? x0 : -x0,
                  lower ? -0.5*board_mm : 0.5*board_mm,
                  z_rows_mm[channel % 10]};
}

static uint16_t quantize_phase(double phase_rad)
{
    long ticks = lround(phase_rad * (double)HOLO_PHASE_MAX / (2.0*M_PI));
    ticks %= (long)HOLO_PHASE_MAX;
    if(ticks < 0) ticks += HOLO_PHASE_MAX;
    return (uint16_t)ticks;
}

static void fill_axis_frame(HoloPhaseFrame *frame, uint16_t frame_id,
                            Vec3 waypoint, int axis, double board_mm)
{
    const double k_per_mm = 2.0*M_PI*VECTOR_FREQUENCY_HZ /
                            (VECTOR_SOUND_SPEED_M_S*1000.0);
    memset(frame, 0, sizeof(*frame));
    frame->magic = HOLO_PHASE_MAGIC;
    frame->version = HOLO_PHASE_VERSION;
    frame->frame_id = frame_id;
    frame->phase_count = HOLO_PHASE_COUNT;
    frame->phase_max = HOLO_PHASE_MAX;

    for(int index = 0; index < VECTOR_TRANSDUCERS; index++) {
        Vec3 emitter = emitter_position(index, board_mm);
        double dx = waypoint.x - emitter.x;
        double dy = waypoint.y - emitter.y;
        double dz = waypoint.z - emitter.z;
        double radius = sqrt(dx*dx + dy*dy + dz*dz);
        double emitter_axis = axis == 0 ? emitter.x : (axis == 1 ? emitter.y : emitter.z);
        double waypoint_axis = axis == 0 ? waypoint.x : (axis == 1 ? waypoint.y : waypoint.z);
        double split_phase = emitter_axis >= waypoint_axis ? 0.0 : M_PI;
        frame->phases[index] = quantize_phase(-k_per_mm*radius + split_phase);
    }
    frame->crc32 = holo_phase_frame_crc(frame);
}

static int send_all(socket_t socket_fd, const void *data, size_t length)
{
    const uint8_t *bytes = (const uint8_t *)data;
    size_t sent = 0;
    while(sent < length) {
#ifdef _WIN32
        int count = send(socket_fd, (const char *)bytes + sent,
                         (int)(length - sent), 0);
#else
        ssize_t count = send(socket_fd, bytes + sent, length - sent, 0);
#endif
        if(count <= 0) return -1;
        sent += (size_t)count;
    }
    return 0;
}

static socket_t connect_to_pi(const char *host, uint16_t port)
{
    socket_t socket_fd = socket(AF_INET, SOCK_STREAM, 0);
    struct sockaddr_in address;
    if(socket_fd == INVALID_SOCKET) return INVALID_SOCKET;
    memset(&address, 0, sizeof(address));
    address.sin_family = AF_INET;
    address.sin_port = htons(port);
#ifdef _WIN32
    address.sin_addr.s_addr = inet_addr(host);
    if(address.sin_addr.s_addr == INADDR_NONE) {
#else
    if(inet_pton(AF_INET, host, &address.sin_addr) != 1) {
#endif
        struct hostent *entry = gethostbyname(host);
        if(!entry) { close_socket(socket_fd); return INVALID_SOCKET; }
        memcpy(&address.sin_addr, entry->h_addr_list[0], entry->h_length);
    }
    if(connect(socket_fd, (struct sockaddr *)&address, sizeof(address)) == SOCKET_ERROR) {
        close_socket(socket_fd);
        return INVALID_SOCKET;
    }
    return socket_fd;
}

static int parse_mode(const char *text)
{
    if(strcmp(text, "one") == 0) return 1;
    if(strcmp(text, "two") == 0) return 2;
    return 0;
}

static void print_state(const TrackingState *state, const int assignment[2],
                        double lookahead_mm)
{
    for(int i = 0; i < state->count; i++) {
        Vec3 direction;
        Vec3 waypoint = make_waypoint(state->bead[i], state->goal[assignment[i]],
                                      lookahead_mm, &direction);
        double weight[3];
        dwell_weights(direction, weight);
        printf("particle %d bead=(%.2f %.2f %.2f) goal=%d waypoint=(%.2f %.2f %.2f) "
               "dwell=(%.2f %.2f %.2f)\n", i + 1,
               state->bead[i].x, state->bead[i].y, state->bead[i].z,
               assignment[i] + 1, waypoint.x, waypoint.y, waypoint.z,
               weight[0], weight[1], weight[2]);
    }
}

static int self_test(int count, const char *state_path, double board_mm,
                     double lookahead_mm)
{
    TrackingState state;
    int assignment[2];
    if(load_tracking_state(state_path, count, board_mm, &state) != 0) {
        fprintf(stderr, "invalid state file: %s\n", state_path);
        return 1;
    }
    assign_goals(&state, assignment);
    print_state(&state, assignment, lookahead_mm);
    for(int particle = 0; particle < count; particle++) {
        Vec3 direction;
        Vec3 waypoint = make_waypoint(state.bead[particle], state.goal[assignment[particle]],
                                      lookahead_mm, &direction);
        for(int axis = 0; axis < 3; axis++) {
            HoloPhaseFrame frame;
            fill_axis_frame(&frame, (uint16_t)(particle*3 + axis), waypoint, axis, board_mm);
            if(frame.crc32 != holo_phase_frame_crc(&frame)) return 1;
            for(int channel = 0; channel < VECTOR_TRANSDUCERS; channel++)
                if(frame.phases[channel] >= HOLO_PHASE_MAX) return 1;
            printf("particle %d axis %c ticks[0..3]=%u,%u,%u,%u crc=%08x\n",
                   particle + 1, "xyz"[axis], frame.phases[0], frame.phases[1],
                   frame.phases[2], frame.phases[3], (unsigned)frame.crc32);
        }
    }
    printf("SELF TEST PASSED\n");
    return 0;
}

static void usage(const char *program)
{
    printf("usage:\n");
    printf("  %s <pi-host> <one|two> [options]\n", program);
    printf("  %s --self-test <one|two> <state-file> [board-mm] [lookahead-mm]\n", program);
    printf("options:\n");
    printf("  --port N          Pi bridge TCP port (default %u)\n", (unsigned)HOLO_PHASE_TCP_PORT);
    printf("  --state FILE      camera/tracker state file; omit for keyboard movement\n");
    printf("  --board-mm N      board separation (default %.1f)\n", VECTOR_DEFAULT_BOARD_MM);
    printf("  --ensemble-ms N   X/Y/Z cycle per particle (default %.1f)\n", VECTOR_DEFAULT_ENSEMBLE_MS);
    printf("  --lookahead-mm N  tracked-policy look-ahead (default %.1f)\n", VECTOR_DEFAULT_LOOKAHEAD_MM);
    printf("  --step-mm N       keyboard movement step (default %.1f)\n", VECTOR_DEFAULT_MOVE_STEP_MM);
    printf("state one: one bx by bz gx gy gz\n");
    printf("state two: two b1x b1y b1z b2x b2y b2z g1x g1y g1z g2x g2y g2z\n");
}

int main(int argc, char **argv)
{
    int count;
    double board_mm, ensemble_ms, lookahead_mm, move_step_mm;
    if(argc >= 2 && strcmp(argv[1], "--self-test") == 0) {
        if(argc < 4 || !(count = parse_mode(argv[2]))) { usage(argv[0]); return 1; }
        board_mm = argc >= 5 ? atof(argv[4]) : VECTOR_DEFAULT_BOARD_MM;
        lookahead_mm = argc >= 6 ? atof(argv[5]) : VECTOR_DEFAULT_LOOKAHEAD_MM;
        if(board_mm < 80.0 || board_mm > 250.0 ||
           lookahead_mm < 0.2 || lookahead_mm > 2.0) return 1;
        return self_test(count, argv[3], board_mm, lookahead_mm);
    }
    if(argc < 3 || !(count = parse_mode(argv[2]))) { usage(argv[0]); return 1; }

    const char *host = argv[1];
    const char *state_path = NULL;
    uint16_t port = HOLO_PHASE_TCP_PORT;
    board_mm = VECTOR_DEFAULT_BOARD_MM;
    ensemble_ms = VECTOR_DEFAULT_ENSEMBLE_MS;
    lookahead_mm = VECTOR_DEFAULT_LOOKAHEAD_MM;
    move_step_mm = VECTOR_DEFAULT_MOVE_STEP_MM;
    for(int arg = 3; arg < argc; arg++) {
        if(strcmp(argv[arg], "--port") == 0 && arg + 1 < argc)
            port = (uint16_t)atoi(argv[++arg]);
        else if(strcmp(argv[arg], "--state") == 0 && arg + 1 < argc)
            state_path = argv[++arg];
        else if(strcmp(argv[arg], "--board-mm") == 0 && arg + 1 < argc)
            board_mm = atof(argv[++arg]);
        else if(strcmp(argv[arg], "--ensemble-ms") == 0 && arg + 1 < argc)
            ensemble_ms = atof(argv[++arg]);
        else if(strcmp(argv[arg], "--lookahead-mm") == 0 && arg + 1 < argc)
            lookahead_mm = atof(argv[++arg]);
        else if(strcmp(argv[arg], "--step-mm") == 0 && arg + 1 < argc)
            move_step_mm = atof(argv[++arg]);
        else {
            fprintf(stderr, "unknown or incomplete option: %s\n", argv[arg]);
            usage(argv[0]);
            return 1;
        }
    }
    if(port == 0 || board_mm < 80.0 || board_mm > 250.0 ||
       ensemble_ms < 3.0 || ensemble_ms > 100.0 ||
       lookahead_mm < 0.2 || lookahead_mm > 2.0 ||
       move_step_mm < 0.05 || move_step_mm > 2.0) {
        fprintf(stderr, "invalid port, board distance, timing, lookahead, or movement step\n");
        return 1;
    }

    TrackingState state;
    if(state_path && load_tracking_state(state_path, count, board_mm, &state) != 0) {
        fprintf(stderr, "invalid initial state file: %s\n", state_path);
        return 1;
    }
    if(!state_path) init_manual_state(&state, count);

#ifdef _WIN32
    WSADATA winsock;
    if(WSAStartup(MAKEWORD(2, 2), &winsock) != 0) return 1;
#endif
    socket_t socket_fd = connect_to_pi(host, port);
    if(socket_fd == INVALID_SOCKET) {
        fprintf(stderr, "could not connect to %s:%u\n", host, (unsigned)port);
#ifdef _WIN32
        WSACleanup();
#endif
        return 1;
    }

    signal(SIGINT, stop_handler);
    signal(SIGTERM, stop_handler);
    uint16_t frame_id = 0;
    uint64_t last_log = 0;
    unsigned long invalid_reads = 0;
    int selected = 0;
    printf("connected to %s:%u: mode=%s control=%s ensemble=%.2f ms/particle\n",
           host, (unsigned)port, count == 1 ? "one" : "two",
           state_path ? "tracked-state" : "keyboard", ensemble_ms);
    if(!state_path) {
        print_manual_help();
        print_manual_state(&state, selected);
    }

    while(keep_running) {
        TrackingState updated;
        int assignment[2];
        if(state_path) {
            if(load_tracking_state(state_path, count, board_mm, &updated) == 0)
                state = updated;
            else
                invalid_reads++;
            assign_goals(&state, assignment);
        } else {
            poll_manual_commands(&state, &selected, move_step_mm, board_mm);
            assignment[0] = 0;
            assignment[1] = 1;
        }

        for(int particle = 0; particle < count && keep_running; particle++) {
            Vec3 direction;
            Vec3 waypoint;
            double weight[3], weight_sum;
            if(state_path) {
                waypoint = make_waypoint(state.bead[particle], state.goal[assignment[particle]],
                                         lookahead_mm, &direction);
                dwell_weights(direction, weight);
            } else {
                waypoint = state.goal[particle];
                direction = (Vec3){0.0, 0.0, 0.0};
                manual_dwell_weights(weight);
            }
            weight_sum = weight[0] + weight[1] + weight[2];
            for(int axis = 0; axis < 3 && keep_running; axis++) {
                HoloPhaseFrame frame;
                uint64_t dwell_us = (uint64_t)llround(ensemble_ms * 1000.0 * weight[axis] / weight_sum);
                fill_axis_frame(&frame, frame_id++, waypoint, axis, board_mm);
                if(send_all(socket_fd, &frame, sizeof(frame)) != 0) {
                    fprintf(stderr, "phase send failed\n");
                    keep_running = 0;
                    break;
                }
                sleep_us(dwell_us);
                if(!state_path) poll_manual_commands(&state, &selected, move_step_mm, board_mm);
            }
        }

        uint64_t now = wall_time_us();
        if(now - last_log >= 1000000u) {
            if(state_path) print_state(&state, assignment, lookahead_mm);
            else print_manual_state(&state, selected);
            printf("frames=%u invalid_state_reads=%lu\n", (unsigned)frame_id, invalid_reads);
            last_log = now;
        }
    }

    close_socket(socket_fd);
#ifdef _WIN32
    WSACleanup();
#endif
    printf("sender stopped\n");
    return 0;
}
