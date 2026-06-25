
/*
 * laptop_phase_sender_morse_traplets.c
 *
 * Optimised C sender using the solo asynchronous Morse traplet method from
 * standalone_solo_async_morse_traplets_random_hardened.py, while preserving the
 * same HoloPhaseFrame network output as the original laptop_phase_sender.c.
 *
 * What is ported from the Python method:
 *   - one-bead-at-a-time asynchronous traplet schedule
 *   - private x/y/z nodal-gradient trap frames for every particle
 *   - nearby-pair barrier frames with sampled line/tube wall constraints
 *   - min-norm complex solve: G = J J^H, alpha = solve(G+lambda I, y),
 *     u = J^H alpha, then phase-only projection angle(u)
 *   - same default temporal/barrier weights that affect the phase solve
 *
 * What is necessarily adapted for your FPGA protocol:
 *   - the Python method can model independent tone amplitudes/powers; the
 *     current phase_protocol.h packet carries one phase value per transducer.
 *     Positive power balancing scales amplitudes only and therefore does not
 *     change phase angles, so this sender keeps the same phase outputs and
 *     cycles the trap/barrier channels as fast stroboscopic frames.
 *
 * Compile:
 *   gcc -std=gnu99 -O3 laptop_phase_sender.c -o laptop_phase_sender -lm
 * Windows/MinGW:
 *   gcc -std=gnu99 -O3 laptop_phase_sender.c -o laptop_phase_sender.exe -lm -lws2_32
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
#include <conio.h>
#else
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/select.h>
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
#define M_PI 3.14159265358979323846264338327950288
#endif

#define NUM_BOARDS 4
#define CHANNELS_PER_BOARD 50
#define NUM_EMITTERS 200
#define SCALE_0P1MM 10.0
#define MOVE_INC_MM 1.0

#define MAX_PARTICLES 10
#define MAX_BARRIER_PAIRS 30
#define MAX_BARRIER_FRAMES 30
#define MAX_EDGES_PER_BARRIER_FRAME 10
#define BARRIER_SAMPLES_PER_PAIR 9
#define MAX_CONSTRAINT_ROWS 360
#define MAX_SCHEDULE_FRAMES (MAX_PARTICLES * 3 + MAX_BARRIER_FRAMES)

/* Python defaults ported from the final script. */
#define DEFAULT_BASE_WAVELENGTH_MM 8.5740       /* Hardware default. Pass 20.0 as argv[4] to match the Python visual model. */
#define TEMPORAL_AXIS_TONE_SPACING 0.018
#define TEMPORAL_BARRIER_TONE_OFFSET 0.045
#define TEMPORAL_BARRIER_FRAME_WEIGHT 0.55
#define MAX_FRAME_PEAK_POWER_GAIN 20.0
#define OWNER_GRADIENT 1.0
#define TEMPORAL_ACTIVE_PRESSURE_NODE_WEIGHT 5.0  /* max(4.5, 5.0) in Python solo mode */
#define TEMPORAL_CROSS_GRADIENT_WEIGHT 1.1       /* max(1.0, 1.1) in Python solo mode */
#define TEMPORAL_INACTIVE_PRESSURE_WEIGHT 0.035  /* min(0.05, 0.035) in Python solo mode */
#define TEMPORAL_INACTIVE_GRADIENT_WEIGHT 0.010  /* min(0.015, 0.010) in Python solo mode */
#define BARRIER_RADIUS_WAVELENGTHS 1.15
#define MAX_BARRIERS_PER_PARTICLE 3
#define BARRIER_PRESSURE 1.0
#define BARRIER_MIDPOINT_PRESSURE_WEIGHT 3.4
#define BARRIER_MIDPOINT_GRADIENT_WEIGHT 1.0
#define BARRIER_TARGET_GUARD_PRESSURE_WEIGHT 0.85
#define BARRIER_TARGET_GUARD_GRADIENT_WEIGHT 0.10
#define BARRIER_TUBE_RADIUS_WAVELENGTHS 0.22
#define SOLVE_REGULARIZATION 1e-10
#define ROW_EPS 1e-300
#define GEOM_EPS_M 0.0005263157894736842 /* Python dx/2 for 0.1m/96 grid. */

static const int16_t xCols[5] = {450, 350, 250, 150, 50};
static const int16_t zRows[10] = {-450, -350, -250, -150, -50, 50, 150, 250, 350, 450};

/* ------------------------------------------------------------------------- */
/* Basic geometry and protocol helpers                                        */
/* ------------------------------------------------------------------------- */

enum BoardIndex {
    BOARD_LEFT_TOP = 0,
    BOARD_RIGHT_TOP = 1,
    BOARD_LEFT_BOTTOM = 2,
    BOARD_RIGHT_BOTTOM = 3
};

typedef struct { double x, y, z; } Vec3;

typedef struct {
    int i, j;
    double d;
    Vec3 mid;
} BarrierPair;

typedef struct {
    int count;
    BarrierPair edges[MAX_EDGES_PER_BARRIER_FRAME];
} BarrierFrame;

typedef enum {
    FRAME_TRAP_AXIS = 0,
    FRAME_BARRIER = 1
} ScheduleKind;

typedef struct {
    ScheduleKind kind;
    int owner;
    int axis;
    int edge_count;
    uint16_t phases[NUM_EMITTERS];
    char label[96];
} ScheduleFrame;

typedef struct {
    Vec3 p[MAX_PARTICLES];
    int count;
    int active;
} ParticleState;

typedef struct {
    ScheduleFrame frame[MAX_SCHEDULE_FRAMES];
    int count;
    int current;
} PhaseSchedule;

static Vec3 g_emitters[NUM_EMITTERS];
static double g_base_wavelength_m = DEFAULT_BASE_WAVELENGTH_MM / 1000.0;
static double g_half_height_m = 0.0675;
static double g_frame_delay_ms = 3.0;

/* Solver work buffers are static to avoid stack churn and malloc/free per solve. */
static double complex g_J[MAX_CONSTRAINT_ROWS][NUM_EMITTERS];
static double complex g_y[MAX_CONSTRAINT_ROWS];
static double complex g_G[MAX_CONSTRAINT_ROWS][MAX_CONSTRAINT_ROWS];
static double complex g_alpha[MAX_CONSTRAINT_ROWS];
static double complex g_drive[NUM_EMITTERS];

static double getTransducerX(enum BoardIndex board, int channel)
{
    int col = channel / 10;
    int16_t x0 = xCols[col];
    switch(board) {
        case BOARD_LEFT_TOP:     return x0;
        case BOARD_RIGHT_TOP:    return -x0;
        case BOARD_LEFT_BOTTOM:  return x0;
        case BOARD_RIGHT_BOTTOM: return -x0;
        default: return 0;
    }
}

static double getTransducerZ(int channel)
{
    return zRows[channel % 10];
}

static int boardIsBottom(enum BoardIndex board)
{
    return board == BOARD_LEFT_BOTTOM || board == BOARD_RIGHT_BOTTOM;
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

static double dot3(Vec3 a, Vec3 b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
static Vec3 add3(Vec3 a, Vec3 b) { Vec3 r = {a.x+b.x, a.y+b.y, a.z+b.z}; return r; }
static Vec3 sub3(Vec3 a, Vec3 b) { Vec3 r = {a.x-b.x, a.y-b.y, a.z-b.z}; return r; }
static Vec3 scale3(Vec3 a, double s) { Vec3 r = {a.x*s, a.y*s, a.z*s}; return r; }
static double norm3(Vec3 a) { return sqrt(dot3(a,a)); }
static Vec3 cross3(Vec3 a, Vec3 b)
{
    Vec3 r = {a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x};
    return r;
}
static Vec3 normalize3(Vec3 a)
{
    double n = norm3(a);
    if(n <= 1e-15) { Vec3 r = {1.0, 0.0, 0.0}; return r; }
    return scale3(a, 1.0 / n);
}

static void buildEmitterPositions(double boardDistanceMm)
{
    g_half_height_m = (boardDistanceMm * 0.5) / 1000.0;
    for(int board = 0; board < NUM_BOARDS; ++board) {
        for(int ch = 0; ch < CHANNELS_PER_BOARD; ++ch) {
            int idx = board * CHANNELS_PER_BOARD + ch;
            g_emitters[idx].x = getTransducerX((enum BoardIndex)board, ch) * 0.0001; /* 0.1 mm -> m */
            g_emitters[idx].y = boardIsBottom((enum BoardIndex)board) ? -g_half_height_m : g_half_height_m;
            g_emitters[idx].z = getTransducerZ(ch) * 0.0001;
        }
    }
}

/* ------------------------------------------------------------------------- */
/* Port of Python pressure_and_gradient_rows_at_point + normalized_row         */
/* ------------------------------------------------------------------------- */

static void pressureAndGradientRowsAtPoint(
    Vec3 point,
    double k,
    double complex h[NUM_EMITTERS],
    double complex hx[NUM_EMITTERS],
    double complex hy[NUM_EMITTERS],
    double complex hz[NUM_EMITTERS])
{
    const double eps = GEOM_EPS_M;
    for(int e = 0; e < NUM_EMITTERS; ++e) {
        Vec3 d = sub3(point, g_emitters[e]);
        double r = norm3(d);
        double r_dir = r > eps ? r : eps;
        double denom = r + eps;
        double phase = k * r;
        double complex base = cexp(I * phase) / denom;
        double complex radial = (I * k - 1.0 / denom) / r_dir;
        h[e] = base;
        hx[e] = base * radial * d.x;
        hy[e] = base * radial * d.y;
        hz[e] = base * radial * d.z;
    }
}

static int appendNormalizedRow(int row, const double complex src[NUM_EMITTERS], double complex target, double weight)
{
    if(row >= MAX_CONSTRAINT_ROWS) return row;
    double sum = 0.0;
    for(int e = 0; e < NUM_EMITTERS; ++e) {
        double a = cabs(src[e]);
        sum += a * a;
    }
    double n = sqrt(sum);
    if(!(n > 0.0) || !isfinite(n)) n = 1.0;
    double s = weight / n;
    for(int e = 0; e < NUM_EMITTERS; ++e) g_J[row][e] = src[e] * s;
    g_y[row] = target * s;
    return row + 1;
}

static int buildTrapAxisSystem(const ParticleState *ps, int owner, int axis, double k)
{
    double complex h[NUM_EMITTERS], hx[NUM_EMITTERS], hy[NUM_EMITTERS], hz[NUM_EMITTERS];
    const double complex zero = 0.0 + 0.0 * I;
    int row = 0;
    for(int j = 0; j < ps->count; ++j) {
        pressureAndGradientRowsAtPoint(ps->p[j], k, h, hx, hy, hz);
        const double complex *gr[3] = {hx, hy, hz};
        if(j == owner) {
            row = appendNormalizedRow(row, h, zero, TEMPORAL_ACTIVE_PRESSURE_NODE_WEIGHT);
            for(int a = 0; a < 3; ++a) {
                if(a == axis) row = appendNormalizedRow(row, gr[a], OWNER_GRADIENT + 0.0 * I, 1.0);
                else          row = appendNormalizedRow(row, gr[a], zero, TEMPORAL_CROSS_GRADIENT_WEIGHT);
            }
        } else {
            row = appendNormalizedRow(row, h, zero, TEMPORAL_INACTIVE_PRESSURE_WEIGHT);
            for(int a = 0; a < 3; ++a)
                row = appendNormalizedRow(row, gr[a], zero, TEMPORAL_INACTIVE_GRADIENT_WEIGHT);
        }
    }
    return row;
}

static void pairFrame(Vec3 pi, Vec3 pj, Vec3 *e, Vec3 *n1, Vec3 *n2)
{
    *e = normalize3(sub3(pj, pi));
    Vec3 ref = {0.0, 0.0, 1.0};
    if(fabs(dot3(*e, ref)) > 0.92) { ref.x = 1.0; ref.y = 0.0; ref.z = 0.0; }
    *n1 = normalize3(cross3(*e, ref));
    *n2 = normalize3(cross3(*e, *n1));
}

static int appendBarrierSampleConstraints(int row, Vec3 q, double k, double pressure_scale, double pressure_weight, double gradient_weight)
{
    double complex h[NUM_EMITTERS], hx[NUM_EMITTERS], hy[NUM_EMITTERS], hz[NUM_EMITTERS];
    const double complex zero = 0.0 + 0.0 * I;
    pressureAndGradientRowsAtPoint(q, k, h, hx, hy, hz);
    row = appendNormalizedRow(row, h, pressure_scale + 0.0 * I, pressure_weight);
    row = appendNormalizedRow(row, hx, zero, gradient_weight);
    row = appendNormalizedRow(row, hy, zero, gradient_weight);
    row = appendNormalizedRow(row, hz, zero, gradient_weight);
    return row;
}

static int buildBarrierFrameSystem(const ParticleState *ps, const BarrierFrame *bf, double k)
{
    double complex h[NUM_EMITTERS], hx[NUM_EMITTERS], hy[NUM_EMITTERS], hz[NUM_EMITTERS];
    const double complex zero = 0.0 + 0.0 * I;
    int row = 0;
    int endpoint[MAX_PARTICLES] = {0};

    for(int e = 0; e < bf->count; ++e) {
        endpoint[bf->edges[e].i] = 1;
        endpoint[bf->edges[e].j] = 1;
    }

    /* Particle-centre guards from build_multibarrier_frame_system. */
    for(int q = 0; q < ps->count; ++q) {
        pressureAndGradientRowsAtPoint(ps->p[q], k, h, hx, hy, hz);
        double weight = BARRIER_TARGET_GUARD_PRESSURE_WEIGHT * (endpoint[q] ? 1.25 : 0.55);
        row = appendNormalizedRow(row, h, zero, weight);
        row = appendNormalizedRow(row, hx, zero, BARRIER_TARGET_GUARD_GRADIENT_WEIGHT);
        row = appendNormalizedRow(row, hy, zero, BARRIER_TARGET_GUARD_GRADIENT_WEIGHT);
        row = appendNormalizedRow(row, hz, zero, BARRIER_TARGET_GUARD_GRADIENT_WEIGHT);
    }

    static const double fractions[5] = {0.30, 0.40, 0.50, 0.60, 0.70};
    double tube_radius = BARRIER_TUBE_RADIUS_WAVELENGTHS * g_base_wavelength_m;

    for(int eidx = 0; eidx < bf->count; ++eidx) {
        int i = bf->edges[eidx].i;
        int j = bf->edges[eidx].j;
        Vec3 pi = ps->p[i], pj = ps->p[j];
        Vec3 dir, n1, n2;
        pairFrame(pi, pj, &dir, &n1, &n2);
        (void)dir;

        for(int f = 0; f < 5; ++f) {
            double t = fractions[f];
            Vec3 q = add3(scale3(pi, 1.0 - t), scale3(pj, t));
            row = appendBarrierSampleConstraints(
                row, q, k,
                BARRIER_PRESSURE,
                BARRIER_MIDPOINT_PRESSURE_WEIGHT,
                BARRIER_MIDPOINT_GRADIENT_WEIGHT * 0.55);
        }

        Vec3 mid = scale3(add3(pi, pj), 0.5);
        Vec3 offsets[4] = { n1, scale3(n1, -1.0), n2, scale3(n2, -1.0) };
        for(int m = 0; m < 4; ++m) {
            Vec3 q = add3(mid, scale3(offsets[m], tube_radius));
            row = appendBarrierSampleConstraints(
                row, q, k,
                BARRIER_PRESSURE * 0.78,
                BARRIER_MIDPOINT_PRESSURE_WEIGHT * 0.65,
                BARRIER_MIDPOINT_GRADIENT_WEIGHT * 0.55);
        }
    }
    return row;
}

/* ------------------------------------------------------------------------- */
/* Complex Hermitian-ish linear solve. Mirrors Python solve_min_norm_phase.    */
/* ------------------------------------------------------------------------- */

static int solveComplexSystemInPlace(double complex A[MAX_CONSTRAINT_ROWS][MAX_CONSTRAINT_ROWS],
                                     double complex b[MAX_CONSTRAINT_ROWS], int n)
{
    for(int k = 0; k < n; ++k) {
        int piv = k;
        double best = cabs(A[k][k]);
        for(int r = k + 1; r < n; ++r) {
            double v = cabs(A[r][k]);
            if(v > best) { best = v; piv = r; }
        }
        if(best < 1e-24 || !isfinite(best)) return -1;
        if(piv != k) {
            for(int c = k; c < n; ++c) {
                double complex tmp = A[k][c]; A[k][c] = A[piv][c]; A[piv][c] = tmp;
            }
            double complex tb = b[k]; b[k] = b[piv]; b[piv] = tb;
        }
        double complex akk = A[k][k];
        for(int r = k + 1; r < n; ++r) {
            double complex f = A[r][k] / akk;
            A[r][k] = 0.0 + 0.0 * I;
            for(int c = k + 1; c < n; ++c) A[r][c] -= f * A[k][c];
            b[r] -= f * b[k];
        }
    }

    for(int i = n - 1; i >= 0; --i) {
        double complex s = b[i];
        for(int c = i + 1; c < n; ++c) s -= A[i][c] * b[c];
        b[i] = s / A[i][i];
    }
    return 0;
}

static int solveMinNormPhase(int rows, uint16_t out_phases[NUM_EMITTERS])
{
    if(rows <= 0 || rows > MAX_CONSTRAINT_ROWS) return -1;

    double trace = 0.0;
    for(int a = 0; a < rows; ++a) {
        for(int b = 0; b < rows; ++b) {
            double complex s = 0.0 + 0.0 * I;
            for(int e = 0; e < NUM_EMITTERS; ++e) s += g_J[a][e] * conj(g_J[b][e]);
            g_G[a][b] = s;
        }
        trace += creal(g_G[a][a]);
    }

    double lam = SOLVE_REGULARIZATION * trace / (double)(rows > 0 ? rows : 1);
    if(!(lam > 0.0) || !isfinite(lam)) lam = SOLVE_REGULARIZATION;
    for(int a = 0; a < rows; ++a) {
        g_G[a][a] += lam;
        g_alpha[a] = g_y[a];
    }

    if(solveComplexSystemInPlace(g_G, g_alpha, rows) != 0) return -1;

    for(int e = 0; e < NUM_EMITTERS; ++e) {
        double complex u = 0.0 + 0.0 * I;
        for(int a = 0; a < rows; ++a) u += conj(g_J[a][e]) * g_alpha[a];
        if(cabs(u) < ROW_EPS || !isfinite(creal(u)) || !isfinite(cimag(u))) {
            g_drive[e] = 1.0 + 0.0 * I;
        } else {
            g_drive[e] = cexp(I * carg(u));
        }
        out_phases[e] = phaseFromAngle(carg(g_drive[e]));
    }
    return 0;
}

/* ------------------------------------------------------------------------- */
/* Barrier selection / edge colouring from the Python final method            */
/* ------------------------------------------------------------------------- */

static int selectBarrierPairs(const ParticleState *ps, BarrierPair pairs[MAX_BARRIER_PAIRS])
{
    typedef struct { double d; int i; int j; } Cand;
    Cand cands[MAX_PARTICLES * MAX_PARTICLES];
    int nc = 0;
    double radius = BARRIER_RADIUS_WAVELENGTHS * g_base_wavelength_m;
    for(int i = 0; i < ps->count; ++i) {
        for(int j = i + 1; j < ps->count; ++j) {
            double d = norm3(sub3(ps->p[i], ps->p[j]));
            if(d <= radius && nc < (int)(sizeof(cands)/sizeof(cands[0]))) {
                cands[nc].d = d; cands[nc].i = i; cands[nc].j = j; ++nc;
            }
        }
    }
    for(int a = 0; a < nc; ++a) {
        for(int b = a + 1; b < nc; ++b) {
            if(cands[b].d < cands[a].d) { Cand t = cands[a]; cands[a] = cands[b]; cands[b] = t; }
        }
    }

    int counts[MAX_PARTICLES] = {0};
    int np = 0;
    for(int c = 0; c < nc && np < MAX_BARRIER_PAIRS; ++c) {
        int i = cands[c].i, j = cands[c].j;
        if(counts[i] < MAX_BARRIERS_PER_PARTICLE && counts[j] < MAX_BARRIERS_PER_PARTICLE) {
            pairs[np].i = i; pairs[np].j = j; pairs[np].d = cands[c].d;
            pairs[np].mid = scale3(add3(ps->p[i], ps->p[j]), 0.5);
            counts[i]++; counts[j]++; np++;
        }
    }
    return np;
}

static int edgeConflictsWithFrame(const BarrierFrame *bf, int i, int j)
{
    for(int e = 0; e < bf->count; ++e) {
        int a = bf->edges[e].i, b = bf->edges[e].j;
        if(a == i || a == j || b == i || b == j) return 1;
    }
    return 0;
}

static int greedyEdgeColors(const BarrierPair pairs[MAX_BARRIER_PAIRS], int pair_count, BarrierFrame frames[MAX_BARRIER_FRAMES])
{
    int nf = 0;
    for(int p = 0; p < pair_count; ++p) {
        int placed = 0;
        for(int f = 0; f < nf; ++f) {
            if(frames[f].count < MAX_EDGES_PER_BARRIER_FRAME && !edgeConflictsWithFrame(&frames[f], pairs[p].i, pairs[p].j)) {
                frames[f].edges[frames[f].count++] = pairs[p];
                placed = 1;
                break;
            }
        }
        if(!placed && nf < MAX_BARRIER_FRAMES) {
            frames[nf].count = 0;
            frames[nf].edges[frames[nf].count++] = pairs[p];
            nf++;
        }
    }
    return nf;
}

/* ------------------------------------------------------------------------- */
/* New calculatePhase: returns a phase from the generated Morse-traplet frame. */
/* ------------------------------------------------------------------------- */

static uint16_t calculatePhase(const PhaseSchedule *sched, int scheduleFrame, enum BoardIndex board, int ch)
{
    if(!sched || sched->count <= 0) return 0;
    int f = scheduleFrame % sched->count;
    if(f < 0) f += sched->count;
    int idx = board * CHANNELS_PER_BOARD + ch;
    return sched->frame[f].phases[idx];
}

static int appendTrapAxisFrame(PhaseSchedule *sched, const ParticleState *ps, int owner, int axis)
{
    if(sched->count >= MAX_SCHEDULE_FRAMES) return -1;
    double factors[3] = {1.0 - TEMPORAL_AXIS_TONE_SPACING, 1.0, 1.0 + TEMPORAL_AXIS_TONE_SPACING};
    double k = 2.0 * M_PI / (g_base_wavelength_m * factors[axis]);
    int rows = buildTrapAxisSystem(ps, owner, axis, k);
    ScheduleFrame *fr = &sched->frame[sched->count];
    memset(fr, 0, sizeof(*fr));
    fr->kind = FRAME_TRAP_AXIS;
    fr->owner = owner;
    fr->axis = axis;
    snprintf(fr->label, sizeof(fr->label), "T%d axis %c", owner, "xyz"[axis]);
    if(solveMinNormPhase(rows, fr->phases) != 0) {
        fprintf(stderr, "solve failed for %s with %d rows\n", fr->label, rows);
        return -1;
    }
    sched->count++;
    return 0;
}

static int appendBarrierFrame(PhaseSchedule *sched, const ParticleState *ps, const BarrierFrame *bf, int index)
{
    if(sched->count >= MAX_SCHEDULE_FRAMES) return -1;
    double k = 2.0 * M_PI / (g_base_wavelength_m * (1.0 + TEMPORAL_BARRIER_TONE_OFFSET));
    int rows = buildBarrierFrameSystem(ps, bf, k);
    ScheduleFrame *fr = &sched->frame[sched->count];
    memset(fr, 0, sizeof(*fr));
    fr->kind = FRAME_BARRIER;
    fr->owner = -1;
    fr->axis = -1;
    fr->edge_count = bf->count;
    snprintf(fr->label, sizeof(fr->label), "B%d %d pair(s)", index, bf->count);
    if(solveMinNormPhase(rows, fr->phases) != 0) {
        fprintf(stderr, "solve failed for %s with %d rows\n", fr->label, rows);
        return -1;
    }
    sched->count++;
    return 0;
}

static int rebuildSchedule(PhaseSchedule *sched, const ParticleState *ps)
{
    sched->count = 0;
    sched->current = 0;

    BarrierPair pairs[MAX_BARRIER_PAIRS];
    BarrierFrame bframes[MAX_BARRIER_FRAMES];
    memset(pairs, 0, sizeof(pairs));
    memset(bframes, 0, sizeof(bframes));

    int pair_count = selectBarrierPairs(ps, pairs);
    int bframe_count = greedyEdgeColors(pairs, pair_count, bframes);

    printf("rebuilding Morse traplet schedule: particles=%d, barrier_pairs=%d, barrier_frames=%d, wavelength=%.4f mm\n",
           ps->count, pair_count, bframe_count, g_base_wavelength_m * 1000.0);

    for(int owner = 0; owner < ps->count; ++owner) {
        for(int axis = 0; axis < 3; ++axis) {
            if(appendTrapAxisFrame(sched, ps, owner, axis) != 0) return -1;
        }
    }
    for(int b = 0; b < bframe_count; ++b) {
        if(appendBarrierFrame(sched, ps, &bframes[b], b) != 0) return -1;
    }

    printf("schedule ready: %d phase frames\n", sched->count);
    return sched->count > 0 ? 0 : -1;
}

static void fillPhaseFrame(HoloPhaseFrame *frame, uint16_t frameID, const PhaseSchedule *sched, int slot)
{
    memset(frame, 0, sizeof(*frame));
    frame->magic = HOLO_PHASE_MAGIC;
    frame->version = HOLO_PHASE_VERSION;
    frame->frame_id = frameID;
    frame->phase_count = HOLO_PHASE_COUNT;
    frame->phase_max = HOLO_PHASE_MAX;

    for(int board = 0; board < NUM_BOARDS; ++board)
        for(int ch = 0; ch < CHANNELS_PER_BOARD; ++ch)
            frame->phases[board * CHANNELS_PER_BOARD + ch] = calculatePhase(sched, slot, (enum BoardIndex)board, ch);

    frame->crc32 = holo_phase_frame_crc(frame);
}

/* ------------------------------------------------------------------------- */
/* Socket helpers                                                             */
/* ------------------------------------------------------------------------- */

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
        if(!he) { close_socket(sock); return INVALID_SOCKET; }
        memcpy(&addr.sin_addr, he->h_addr_list[0], he->h_length);
    }

    if(connect(sock, (struct sockaddr *)&addr, sizeof(addr)) == SOCKET_ERROR) {
        close_socket(sock);
        return INVALID_SOCKET;
    }
    return sock;
}

/* ------------------------------------------------------------------------- */
/* Non-blocking console controls                                              */
/* ------------------------------------------------------------------------- */

#ifndef _WIN32
static struct termios g_old_termios;
static int g_termios_enabled = 0;
static void restoreTerminal(void)
{
    if(g_termios_enabled) tcsetattr(STDIN_FILENO, TCSANOW, &g_old_termios);
}
static void enableRawTerminal(void)
{
    struct termios t;
    if(tcgetattr(STDIN_FILENO, &g_old_termios) == 0) {
        t = g_old_termios;
        t.c_lflag &= ~(ICANON | ECHO);
        t.c_cc[VMIN] = 0;
        t.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSANOW, &t);
        g_termios_enabled = 1;
        atexit(restoreTerminal);
    }
}
#endif

static int pollKey(void)
{
#ifdef _WIN32
    if(_kbhit()) return _getch();
    return -1;
#else
    unsigned char ch;
    fd_set rfds;
    struct timeval tv;
    FD_ZERO(&rfds);
    FD_SET(STDIN_FILENO, &rfds);
    tv.tv_sec = 0; tv.tv_usec = 0;
    if(select(STDIN_FILENO + 1, &rfds, NULL, NULL, &tv) > 0) {
        if(read(STDIN_FILENO, &ch, 1) == 1) return (int)ch;
    }
    return -1;
#endif
}

/* ------------------------------------------------------------------------- */
/* Interactive particle controls                                              */
/* ------------------------------------------------------------------------- */

static void printHelp(void)
{
    printf("commands:\n");
    printf("  x/s    active particle X -/+ %.1f mm\n", MOVE_INC_MM);
    printf("  c/d    active particle Y -/+ %.1f mm\n", MOVE_INC_MM);
    printf("  z/a    active particle Z -/+ %.1f mm\n", MOVE_INC_MM);
    printf("  n/b    select next / previous particle\n");
    printf("  1..9   select particle 1..9, 0 selects particle 10\n");
    printf("  g      add new particle at centre, select it, up to %d\n", MAX_PARTICLES);
    printf("  r      remove active particle\n");
    printf("  h/H    home active / create the 10-particle final lattice\n");
    printf("  p      print particles and current schedule frame\n");
    printf("  [/]    slow down / speed up stroboscopic frame delay\n");
    printf("  ?      help\n");
    printf("  q      quit\n");
}

static void printParticles(const ParticleState *ps, const PhaseSchedule *sched)
{
    printf("particles: count=%d active=%d schedule=%d frames delay=%.2f ms\n", ps->count, ps->active + 1, sched->count, g_frame_delay_ms);
    for(int i = 0; i < ps->count; ++i) {
        printf("  %c%2d: %+7.2f %+7.2f %+7.2f mm\n", i == ps->active ? '*' : ' ', i + 1,
               ps->p[i].x * 1000.0, ps->p[i].y * 1000.0, ps->p[i].z * 1000.0);
    }
}

static void homeActive(ParticleState *ps)
{
    if(ps->active < 0 || ps->active >= ps->count) return;
    ps->p[ps->active].x = ps->p[ps->active].y = ps->p[ps->active].z = 0.0;
}

static void makeFinalLattice10(ParticleState *ps)
{
    /* Exact centred-mm version of the Python generated default lattice:
       [0.02,0.03,0.05] etc in a 0.1m cube -> subtract [0.05,0.05,0.05]. */
    static const double mm[10][3] = {
        {-30.0, -20.0, 0.0}, {-10.0, -20.0, 0.0}, { 10.0, -20.0, 0.0}, { 30.0, -20.0, 0.0},
        {-30.0,   0.0, 0.0}, {-10.0,   0.0, 0.0}, { 10.0,   0.0, 0.0}, { 30.0,   0.0, 0.0},
        {-30.0,  20.0, 0.0}, {-10.0,  20.0, 0.0}
    };
    ps->count = MAX_PARTICLES;
    ps->active = 0;
    for(int i = 0; i < MAX_PARTICLES; ++i) {
        ps->p[i].x = mm[i][0] / 1000.0;
        ps->p[i].y = mm[i][1] / 1000.0;
        ps->p[i].z = mm[i][2] / 1000.0;
    }
}

static int processKey(int ch, ParticleState *ps, PhaseSchedule *sched)
{
    int dirty = 0;
    if(ch < 0) return 0;
    switch(ch) {
        case 'q': return -1;
        case '?': printHelp(); break;
        case 'p': printParticles(ps, sched); break;
        case 'n': if(ps->count > 0) ps->active = (ps->active + 1) % ps->count; printParticles(ps, sched); break;
        case 'b': if(ps->count > 0) ps->active = (ps->active + ps->count - 1) % ps->count; printParticles(ps, sched); break;
        case '1': case '2': case '3': case '4': case '5': case '6': case '7': case '8': case '9': {
            int idx = ch - '1'; if(idx < ps->count) ps->active = idx; printParticles(ps, sched); break;
        }
        case '0': if(ps->count >= 10) ps->active = 9; printParticles(ps, sched); break;
        case 'x': ps->p[ps->active].x -= MOVE_INC_MM / 1000.0; dirty = 1; break;
        case 's': ps->p[ps->active].x += MOVE_INC_MM / 1000.0; dirty = 1; break;
        case 'c': ps->p[ps->active].y -= MOVE_INC_MM / 1000.0; dirty = 1; break;
        case 'd': ps->p[ps->active].y += MOVE_INC_MM / 1000.0; dirty = 1; break;
        case 'z': ps->p[ps->active].z -= MOVE_INC_MM / 1000.0; dirty = 1; break;
        case 'a': ps->p[ps->active].z += MOVE_INC_MM / 1000.0; dirty = 1; break;
        case 'h': homeActive(ps); dirty = 1; break;
        case 'H': makeFinalLattice10(ps); dirty = 1; break;
        case 'g':
            if(ps->count < MAX_PARTICLES) {
                /* User workflow: introduce every new bead at the acoustic centre,
                   then manually move it out of the way before adding the next one. */
                Vec3 centre = {0.0, 0.0, 0.0};
                ps->p[ps->count] = centre;
                ps->active = ps->count;
                ps->count++;
                dirty = 1;
            } else printf("already at max particle count %d\n", MAX_PARTICLES);
            break;
        case 'r':
            if(ps->count > 1) {
                for(int i = ps->active; i < ps->count - 1; ++i) ps->p[i] = ps->p[i + 1];
                ps->count--;
                if(ps->active >= ps->count) ps->active = ps->count - 1;
                dirty = 1;
            }
            break;
        case '[': g_frame_delay_ms += 1.0; if(g_frame_delay_ms > 50.0) g_frame_delay_ms = 50.0; printf("delay %.2f ms\n", g_frame_delay_ms); break;
        case ']': g_frame_delay_ms -= 1.0; if(g_frame_delay_ms < 1.0) g_frame_delay_ms = 1.0; printf("delay %.2f ms\n", g_frame_delay_ms); break;
        case '\n': case '\r': break;
        default: break;
    }
    if(dirty) {
        printf("active %d now at %.2f %.2f %.2f mm\n", ps->active + 1,
               ps->p[ps->active].x * 1000.0, ps->p[ps->active].y * 1000.0, ps->p[ps->active].z * 1000.0);
        if(rebuildSchedule(sched, ps) != 0) fprintf(stderr, "warning: schedule rebuild failed; keeping old/empty schedule\n");
    }
    return 0;
}

int main(int argc, char **argv)
{
    if(argc < 2) {
        printf("usage: %s <pi-ip-or-host> [port] [board-distance-mm] [synthesis-wavelength-mm]\n", argv[0]);
        printf("  default synthesis wavelength is %.4f mm for the real 40 kHz hardware geometry.\n", DEFAULT_BASE_WAVELENGTH_MM);
        printf("  pass 20.0 as the 4th argument to reproduce the Python visualisation's wavelength.\n");
        return 1;
    }

    const char *host = argv[1];
    uint16_t port = argc >= 3 ? (uint16_t)atoi(argv[2]) : HOLO_PHASE_TCP_PORT;
    double boardDistanceMm = argc >= 4 ? atof(argv[3]) : 135.0;
    if(argc >= 5) g_base_wavelength_m = atof(argv[4]) / 1000.0;
    if(!(g_base_wavelength_m > 0.0)) g_base_wavelength_m = DEFAULT_BASE_WAVELENGTH_MM / 1000.0;

#ifdef _WIN32
    WSADATA wsa;
    if(WSAStartup(MAKEWORD(2, 2), &wsa) != 0) return 1;
#else
    enableRawTerminal();
#endif

    buildEmitterPositions(boardDistanceMm);

    ParticleState ps;
    memset(&ps, 0, sizeof(ps));
    /* Always begin with exactly one controllable bead at the acoustic centre.
       Additional beads added with 'g' also start at this centre. */
    ps.count = 1;
    ps.active = 0;
    ps.p[0].x = ps.p[0].y = ps.p[0].z = 0.0;

    PhaseSchedule sched;
    memset(&sched, 0, sizeof(sched));
    if(rebuildSchedule(&sched, &ps) != 0) {
        fprintf(stderr, "initial schedule build failed\n");
        return 1;
    }

    socket_t sock = connectToPi(host, port);
    if(sock == INVALID_SOCKET) {
        printf("could not connect to %s:%u\n", host, port);
        return 1;
    }

    HoloPhaseFrame packet;
    uint16_t frameID = 0;
    unsigned long sent = 0;
    printHelp();
    printParticles(&ps, &sched);

    int quit = 0;
    while(!quit) {
        if(sched.count <= 0) {
            delay_ms(20);
        } else {
            int slot = sched.current % sched.count;
            fillPhaseFrame(&packet, frameID++, &sched, slot);
            if(sendAll(sock, &packet, sizeof(packet)) < 0) {
                printf("send failed\n");
                break;
            }
            sched.current = (sched.current + 1) % sched.count;
            sent++;
            delay_ms((unsigned int)(g_frame_delay_ms < 1.0 ? 1.0 : g_frame_delay_ms));
        }

        int ch;
        while((ch = pollKey()) >= 0) {
            int r = processKey(ch, &ps, &sched);
            if(r < 0) { quit = 1; break; }
        }
    }

    close_socket(sock);
#ifdef _WIN32
    WSACleanup();
#else
    restoreTerminal();
#endif
    printf("sent %lu stroboscopic phase frames\n", sent);
    return 0;
}