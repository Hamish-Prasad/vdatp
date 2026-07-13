#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#ifdef _WIN32
#undef _WIN32_WINNT
#define _WIN32_WINNT 0x0600
#include <conio.h>
#else
#include <sys/select.h>
#include <sys/time.h>
#include <termios.h>
#include <unistd.h>
#endif

/*
 * This live sender uses an analytic baffled-piston transducer model,
 * J0(k*r*sin(theta))*exp(i*k*d)/d, then solves a small phase-only constraint
 * problem around each particle: low pressure at the centre and alternating
 * guard samples around it.  That creates a more stable local cage than simply
 * focusing all transducers at the particle centre.
 * gcc -std=gnu99 -O3 -Wall -Wextra laptop_file_sender_lev.c -o laptop_file_sender_lev.exe -lm -lws2_32
 * laptop_file_sender_lev.exe 169.254.251.233 5656 135 3 64 10000
 * laptop_file_sender_lev.exe 169.254.65.146 5656 135 3 64 4000
 *
 * laptop_file_sender_lev.exe 169.254.65.146 5656 135 3 128 0
 * laptop_file_sender_lev.exe 169.254.166.13 5656 135 3 128 300
 * laptop_file_sender_lev.exe 169.254.166.13 5656 135 3 32 200
 * laptop_file_sender_lev.exe 169.254.166.13 5656 135 2 64 390 0 xz
 * laptop_file_sender_lev.exe 169.254.166.13 5656 135 2 64 390 0 xz
 * laptop_file_sender_lev.exe 169.254.114.104 5656 135 3 64 5000
 */

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#ifndef INET_PTON
WINSOCK_API_LINKAGE INT WSAAPI inet_pton(INT Family, PCSTR pszAddrString, PVOID pAddrBuf);
#endif
typedef SOCKET socket_t;
#define close_socket closesocket
#else
#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
typedef int socket_t;
#define INVALID_SOCKET (-1)
#define SOCKET_ERROR (-1)
#define close_socket close
#endif

#include "phase_protocol.h"

#define NUM_BOARDS 4
#define CHANNELS_PER_BOARD 50
#define MAX_PARTICLES 9
#define TOTAL_CHANNELS (NUM_BOARDS * CHANNELS_PER_BOARD)
#define DEFAULT_BOARD_DISTANCE_MM 135.0
#define DEFAULT_RADIUS_MM 3.0
#define DEFAULT_FRAMES_PER_CIRCLE 64
#define DEFAULT_DELAY_US 5000
#define DEFAULT_RAMP_FRAMES 64
#define DEFAULT_RAMP_DELAY_MS 6
#define DEFAULT_SPEED_RAMP_FRAMES 128
#define DEFAULT_FAST_RAMP_SECONDS 10.0
#define FAST_RAMP_START_DELAY_US 12000
#define MOVE_INC_MM 0.5
#define TRANSDUCER_RADIUS_MM 5.0
#define TRAP_GUARD_OFFSET_MM 1.6
#define TRAP_SOLVER_ITERATIONS 72
#define TRAP_SOLVER_STEP 0.045
#define SOUND_SPEED_MM_S 343000.0
#define FREQUENCY_HZ 40000.0
#define PI 3.14159265358979323846
#define TWO_PI (2.0 * PI)
#define STABILITY_ACCEL_WARN_M_S2 120.0

enum BoardIndex {
	BOARD_LEFT_TOP = 0,
	BOARD_RIGHT_TOP = 1,
	BOARD_LEFT_BOTTOM = 2,
	BOARD_RIGHT_BOTTOM = 3
};

typedef enum CirclePlane {
	CIRCLE_PLANE_XZ = 0,
	CIRCLE_PLANE_XY,
	CIRCLE_PLANE_YZ
} CirclePlane;

typedef struct ParticleTarget {
	double x;
	double y;
	double z;
	int enabled;
} ParticleTarget;

typedef struct CircleCenters {
	double y[MAX_PARTICLES];
	double z[MAX_PARTICLES];
} CircleCenters;

typedef struct TrapConstraint {
	double x;
	double y;
	double z;
	double targetReal;
	double targetImag;
	double weight;
} TrapConstraint;

typedef struct LightState {
	uint8_t red;
	uint8_t green;
	uint8_t blue;
	uint8_t brightness;
	int enabled;
	int blink;
	int preset;
	int redEnabled;
	int greenEnabled;
	int blueEnabled;
} LightState;

typedef struct LightPreset {
	const char *name;
	uint8_t red;
	uint8_t green;
	uint8_t blue;
} LightPreset;

static const int16_t xCols[5] = {450, 350, 250, 150, 50};
static const int16_t zRows[10] = {-450, -350, -250, -150, -50, 50, 150, 250, 350, 450};
static const LightPreset photoLightPresets[9] = {
	{"warm white", 255, 214, 170},
	{"ice blue", 115, 190, 255},
	{"deep violet", 165, 90, 255},
	{"magenta", 255, 75, 210},
	{"amber", 255, 155, 35},
	{"cyan", 45, 235, 255},
	{"emerald", 70, 255, 135},
	{"rose", 255, 105, 125},
	{"studio white", 255, 255, 255}
};

static int send_particles(socket_t sock, HoloPhaseFrame *frame, uint16_t *frameID,
	const ParticleTarget *particles, double boardDistanceMm);
static int send_light_control(socket_t sock, uint16_t *frameID, const LightState *light);
static int handle_light_key(socket_t sock, uint16_t *frameID, LightState *light, int ch);
static void make_phase_frame_temporally_continuous(HoloPhaseFrame *frame);

static void delay_ms(unsigned int ms)
{
#ifdef _WIN32
	Sleep(ms);
#else
	usleep(ms * 1000u);
#endif
}

static void delay_us(unsigned int us)
{
	if(us == 0) return;
#ifdef _WIN32
	static LARGE_INTEGER freq;
	LARGE_INTEGER start;
	LARGE_INTEGER now;
	if(freq.QuadPart == 0) QueryPerformanceFrequency(&freq);
	QueryPerformanceCounter(&start);
	long long waitTicks = (long long)((double)us * (double)freq.QuadPart / 1000000.0);
	do {
		QueryPerformanceCounter(&now);
	} while(now.QuadPart - start.QuadPart < waitTicks);
#else
	usleep(us);
#endif
}

static double now_seconds(void)
{
#ifdef _WIN32
	static LARGE_INTEGER freq;
	LARGE_INTEGER tick;
	if(freq.QuadPart == 0) QueryPerformanceFrequency(&freq);
	QueryPerformanceCounter(&tick);
	return (double)tick.QuadPart / (double)freq.QuadPart;
#else
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return (double)tv.tv_sec + (double)tv.tv_usec * 1.0e-6;
#endif
}

static int key_pressed(void)
{
#ifdef _WIN32
	return _kbhit();
#else
	struct timeval tv;
	fd_set readfds;
	FD_ZERO(&readfds);
	FD_SET(STDIN_FILENO, &readfds);
	tv.tv_sec = 0;
	tv.tv_usec = 0;
	return select(STDIN_FILENO + 1, &readfds, NULL, NULL, &tv) > 0;
#endif
}

static int read_key(void)
{
#ifdef _WIN32
	return _getch();
#else
	unsigned char ch;
	if(read(STDIN_FILENO, &ch, 1) == 1) return (int)ch;
	return EOF;
#endif
}

#ifndef _WIN32
static int terminal_raw_mode(int enable)
{
	static struct termios oldt;
	static int active = 0;
	struct termios newt;

	if(enable && !active) {
		if(tcgetattr(STDIN_FILENO, &oldt) != 0) return -1;
		newt = oldt;
		newt.c_lflag &= (tcflag_t)~(ICANON | ECHO);
		newt.c_cc[VMIN] = 0;
		newt.c_cc[VTIME] = 0;
		if(tcsetattr(STDIN_FILENO, TCSANOW, &newt) != 0) return -1;
		active = 1;
	} else if(!enable && active) {
		tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
		active = 0;
	}
	return 0;
}
#else
static int terminal_raw_mode(int enable)
{
	(void)enable;
	return 0;
}
#endif

static double get_transducer_x(enum BoardIndex board, int channel)
{
	int col = channel / 10;
	switch(board) {
		case BOARD_LEFT_TOP:     return (double)xCols[col] * 0.1;
		case BOARD_RIGHT_TOP:    return (double)-xCols[4 - col] * 0.1;
		case BOARD_LEFT_BOTTOM:  return (double)xCols[4 - col] * 0.1;
		case BOARD_RIGHT_BOTTOM: return (double)-xCols[col] * 0.1;
		default: return 0.0;
	}
}

static double get_transducer_z(int channel)
{
	return (double)zRows[channel % 10] * 0.1;
}

static int board_is_bottom(enum BoardIndex board)
{
	return board == BOARD_LEFT_BOTTOM || board == BOARD_RIGHT_BOTTOM;
}

static uint16_t phase_radians_to_ticks(double phase)
{
	int32_t ticks;
	phase = fmod(phase, TWO_PI);
	if(phase < 0.0) phase += TWO_PI;
	ticks = (int32_t)floor(phase * (double)HOLO_PHASE_MAX / TWO_PI + 0.5);
	ticks %= (int32_t)HOLO_PHASE_MAX;
	if(ticks < 0) ticks += (int32_t)HOLO_PHASE_MAX;
	return (uint16_t)ticks;
}

static double bessel_j0_series(double x)
{
	double term = 1.0;
	double sum = 1.0;
	double xx = 0.25 * x * x;
	for(int m = 1; m < 24; m++) {
		term *= -xx / ((double)m * (double)m);
		sum += term;
		if(fabs(term) < 1.0e-14) break;
	}
	return sum;
}

static void calculate_transfer_components(double tx, double ty, double tz,
	enum BoardIndex board, int channel, double boardDistanceMm,
	double *real, double *imag)
{
	const double k = TWO_PI * FREQUENCY_HZ / SOUND_SPEED_MM_S;
	double ex = get_transducer_x(board, channel);
	double ey = board_is_bottom(board) ? -0.5 * boardDistanceMm : 0.5 * boardDistanceMm;
	double ez = get_transducer_z(channel);
	double nx = 0.0;
	double ny = board_is_bottom(board) ? 1.0 : -1.0;
	double nz = 0.0;
	double dx = tx - ex;
	double dy = ty - ey;
	double dz = tz - ez;
	double distance = sqrt(dx*dx + dy*dy + dz*dz);
	double cosTheta = (dx*nx + dy*ny + dz*nz) / distance;
	double sinTheta;
	double directivity;
	double amplitude;
	double phase;

	if(cosTheta < 0.0) cosTheta = 0.0;
	if(cosTheta > 1.0) cosTheta = 1.0;
	sinTheta = sqrt(fmax(0.0, 1.0 - cosTheta*cosTheta));
	directivity = bessel_j0_series(k * TRANSDUCER_RADIUS_MM * sinTheta);
	amplitude = directivity / distance;
	if(board_is_bottom(board))
		amplitude = -amplitude;
	phase = k * distance;
	*real = amplitude * cos(phase);
	*imag = amplitude * sin(phase);
}

static double calculate_focus_phase_radians(double tx, double ty, double tz,
	enum BoardIndex board, int channel, double boardDistanceMm)
{
	const double k = TWO_PI * FREQUENCY_HZ / SOUND_SPEED_MM_S;
	double ex = get_transducer_x(board, channel);
	double ey = board_is_bottom(board) ? -0.5 * boardDistanceMm : 0.5 * boardDistanceMm;
	double ez = get_transducer_z(channel);
	double nx = 0.0;
	double ny = board_is_bottom(board) ? 1.0 : -1.0;
	double nz = 0.0;
	double dx = tx - ex;
	double dy = ty - ey;
	double dz = tz - ez;
	double distance = sqrt(dx*dx + dy*dy + dz*dz);
	double cosTheta = (dx*nx + dy*ny + dz*nz) / distance;
	double sinTheta;
	double directivity;
	double drivePhase;

	if(cosTheta < 0.0) cosTheta = 0.0;
	if(cosTheta > 1.0) cosTheta = 1.0;
	sinTheta = sqrt(fmax(0.0, 1.0 - cosTheta*cosTheta));
	directivity = bessel_j0_series(k * TRANSDUCER_RADIUS_MM * sinTheta) / distance;

	drivePhase = -k * distance;
	if(directivity < 0.0) drivePhase -= PI;
	if(board_is_bottom(board)) drivePhase += PI;
	return drivePhase;
}

static int collect_enabled_particles(const ParticleTarget *particles, int particleCount, int *enabledIdx)
{
	int enabledCount = 0;
	for(int i = 0; i < particleCount; i++) {
		if(particles[i].enabled)
			enabledIdx[enabledCount++] = i;
	}
	return enabledCount;
}

static void add_constraint(TrapConstraint *constraints, int *count,
	double x, double y, double z, double targetReal, double targetImag, double weight)
{
	constraints[*count].x = x;
	constraints[*count].y = y;
	constraints[*count].z = z;
	constraints[*count].targetReal = targetReal;
	constraints[*count].targetImag = targetImag;
	constraints[*count].weight = weight;
	(*count)++;
}

static int build_trap_constraints(const ParticleTarget *particles, const int *enabledIdx,
	int enabledCount, TrapConstraint *constraints)
{
	int count = 0;
	const double d = TRAP_GUARD_OFFSET_MM;

	for(int i = 0; i < enabledCount; i++) {
		const ParticleTarget *p = &particles[enabledIdx[i]];
		double particlePhase = (enabledCount > 1) ? TWO_PI * (double)i / (double)enabledCount : 0.0;
		double cr = cos(particlePhase);
		double ci = sin(particlePhase);

		add_constraint(constraints, &count, p->x, p->y, p->z, 0.0, 0.0, 40.0);
		add_constraint(constraints, &count, p->x + d, p->y, p->z, cr, ci, 1.0);
		add_constraint(constraints, &count, p->x - d, p->y, p->z, -cr, -ci, 1.0);
		if(enabledCount == 1) {
			add_constraint(constraints, &count, p->x, p->y + d, p->z, cr, ci, 1.15);
			add_constraint(constraints, &count, p->x, p->y - d, p->z, -cr, -ci, 1.15);
		} else {
			add_constraint(constraints, &count, p->x, p->y + d, p->z, cr, ci, 1.15);
			add_constraint(constraints, &count, p->x, p->y - d, p->z, -cr, -ci, 1.15);
		}
		add_constraint(constraints, &count, p->x, p->y, p->z + d, cr, ci, 1.0);
		add_constraint(constraints, &count, p->x, p->y, p->z - d, -cr, -ci, 1.0);
	}
	return count;
}

static void fill_guard_trap_phases(HoloPhaseFrame *frame,
	const ParticleTarget *particles, const int *enabledIdx, int enabledCount,
	double boardDistanceMm)
{
	TrapConstraint constraints[MAX_PARTICLES * 7];
	double transferReal[MAX_PARTICLES * 7][TOTAL_CHANNELS];
	double transferImag[MAX_PARTICLES * 7][TOTAL_CHANNELS];
	double driveReal[TOTAL_CHANNELS];
	double driveImag[TOTAL_CHANNELS];
	int constraintCount = build_trap_constraints(particles, enabledIdx, enabledCount, constraints);

	for(int c = 0; c < constraintCount; c++) {
		for(int board = 0; board < NUM_BOARDS; board++) {
			for(int ch = 0; ch < CHANNELS_PER_BOARD; ch++) {
				int idx = board * CHANNELS_PER_BOARD + ch;
				calculate_transfer_components(
					constraints[c].x, constraints[c].y, constraints[c].z,
					(enum BoardIndex)board, ch, boardDistanceMm,
					&transferReal[c][idx], &transferImag[c][idx]);
			}
		}
	}

	for(int idx = 0; idx < TOTAL_CHANNELS; idx++) {
		double real = 0.0;
		double imag = 0.0;
		for(int c = 0; c < constraintCount; c++) {
			double w = constraints[c].weight;
			double br = constraints[c].targetReal;
			double bi = constraints[c].targetImag;
			real += w * (transferReal[c][idx] * br + transferImag[c][idx] * bi);
			imag += w * (transferReal[c][idx] * bi - transferImag[c][idx] * br);
		}
		double mag = hypot(real, imag);
		if(mag > 0.0) {
			driveReal[idx] = real / mag;
			driveImag[idx] = imag / mag;
		} else {
			driveReal[idx] = 1.0;
			driveImag[idx] = 0.0;
		}
	}

	for(int iter = 0; iter < TRAP_SOLVER_ITERATIONS; iter++) {
		double gradReal[TOTAL_CHANNELS];
		double gradImag[TOTAL_CHANNELS];
		for(int idx = 0; idx < TOTAL_CHANNELS; idx++) {
			gradReal[idx] = 0.0;
			gradImag[idx] = 0.0;
		}

		for(int c = 0; c < constraintCount; c++) {
			double fieldReal = 0.0;
			double fieldImag = 0.0;
			double w = constraints[c].weight;
			for(int idx = 0; idx < TOTAL_CHANNELS; idx++) {
				fieldReal += transferReal[c][idx] * driveReal[idx] - transferImag[c][idx] * driveImag[idx];
				fieldImag += transferReal[c][idx] * driveImag[idx] + transferImag[c][idx] * driveReal[idx];
			}
			double errReal = w * (fieldReal - constraints[c].targetReal);
			double errImag = w * (fieldImag - constraints[c].targetImag);
			for(int idx = 0; idx < TOTAL_CHANNELS; idx++) {
				gradReal[idx] += transferReal[c][idx] * errReal + transferImag[c][idx] * errImag;
				gradImag[idx] += transferReal[c][idx] * errImag - transferImag[c][idx] * errReal;
			}
		}

		for(int idx = 0; idx < TOTAL_CHANNELS; idx++) {
			double real = driveReal[idx] - TRAP_SOLVER_STEP * gradReal[idx];
			double imag = driveImag[idx] - TRAP_SOLVER_STEP * gradImag[idx];
			double mag = hypot(real, imag);
			if(mag > 0.0) {
				driveReal[idx] = real / mag;
				driveImag[idx] = imag / mag;
			}
		}
	}

	for(int idx = 0; idx < TOTAL_CHANNELS; idx++)
		frame->phases[idx] = phase_radians_to_ticks(atan2(driveImag[idx], driveReal[idx]));
}

static void set_dual_circle_positions(ParticleTarget *particles, double radiusMm,
	const CircleCenters *centers, double theta)
{
	double centerX[MAX_PARTICLES] = {radiusMm, -radiusMm};

	/*
	 * Particle 1 moves counter-clockwise around +radius.
	 * Particle 2 moves clockwise around -radius with the same cosine phase.
	 * This keeps the traps separated by at least 2*radius in X instead of
	 * colliding at the middle once per cycle.
	 */
	particles[0].x = centerX[0] + radiusMm * cos(theta);
	particles[0].y = centers->y[0];
	particles[0].z = centers->z[0] + radiusMm * sin(theta);
	particles[1].x = centerX[1] + radiusMm * cos(theta);
	particles[1].y = centers->y[1];
	particles[1].z = centers->z[1] - radiusMm * sin(theta);
	particles[0].enabled = 1;
	particles[1].enabled = 1;
}

static double particle_distance(const ParticleTarget *a, const ParticleTarget *b)
{
	double dx = a->x - b->x;
	double dy = a->y - b->y;
	double dz = a->z - b->z;
	return sqrt(dx*dx + dy*dy + dz*dz);
}

static void make_phase_frame_temporally_continuous(HoloPhaseFrame *frame)
{
	static uint16_t previous[HOLO_PHASE_COUNT];
	static int havePrevious = 0;
	double real = 0.0;
	double imag = 0.0;

	if(frame == NULL) {
		havePrevious = 0;
		return;
	}

	if(!havePrevious) {
		memcpy(previous, frame->phases, sizeof(previous));
		havePrevious = 1;
		return;
	}

	for(size_t i = 0; i < HOLO_PHASE_COUNT; i++) {
		double prev = TWO_PI * (double)previous[i] / (double)HOLO_PHASE_MAX;
		double cur = TWO_PI * (double)frame->phases[i] / (double)HOLO_PHASE_MAX;
		real += cos(prev - cur);
		imag += sin(prev - cur);
	}

	int offset = (int)floor(atan2(imag, real) * (double)HOLO_PHASE_MAX / TWO_PI + 0.5);
	for(size_t i = 0; i < HOLO_PHASE_COUNT; i++) {
		int shifted = (int)frame->phases[i] + offset;
		shifted %= (int)HOLO_PHASE_MAX;
		if(shifted < 0) shifted += (int)HOLO_PHASE_MAX;
		frame->phases[i] = (uint16_t)shifted;
		previous[i] = frame->phases[i];
	}
}

static void reset_phase_temporal_continuity(void)
{
	make_phase_frame_temporally_continuous(NULL);
}

static const char *circle_plane_name(CirclePlane plane)
{
	switch(plane) {
		case CIRCLE_PLANE_XY: return "X-Y";
		case CIRCLE_PLANE_YZ: return "Y-Z";
		case CIRCLE_PLANE_XZ:
		default: return "X-Z";
	}
}

static int parse_circle_plane(const char *text, CirclePlane *plane)
{
	if(text == NULL || text[0] == '\0') return 0;
	if((text[0] == 'x' || text[0] == 'X') && (text[1] == 'z' || text[1] == 'Z') && text[2] == '\0') {
		*plane = CIRCLE_PLANE_XZ;
		return 1;
	}
	if((text[0] == 'x' || text[0] == 'X') && (text[1] == 'y' || text[1] == 'Y') && text[2] == '\0') {
		*plane = CIRCLE_PLANE_XY;
		return 1;
	}
	if((text[0] == 'y' || text[0] == 'Y') && (text[1] == 'z' || text[1] == 'Z') && text[2] == '\0') {
		*plane = CIRCLE_PLANE_YZ;
		return 1;
	}
	return 0;
}

static void set_single_circle_position(ParticleTarget *particle, double radiusMm,
	CirclePlane plane, double theta)
{
	double c = radiusMm * cos(theta);
	double s = radiusMm * sin(theta);

	particle->x = 0.0;
	particle->y = 0.0;
	particle->z = 0.0;
	switch(plane) {
		case CIRCLE_PLANE_XY:
			particle->x = c;
			particle->y = s;
			break;
		case CIRCLE_PLANE_YZ:
			particle->y = c;
			particle->z = s;
			break;
		case CIRCLE_PLANE_XZ:
		default:
			particle->x = c;
			particle->z = s;
			break;
	}
	particle->enabled = 1;
}

static void fill_single_focus_frame(HoloPhaseFrame *frame, uint16_t frameID,
	const ParticleTarget *particle, double boardDistanceMm)
{
	memset(frame, 0, sizeof(*frame));
	frame->magic = HOLO_PHASE_MAGIC;
	frame->version = HOLO_PHASE_VERSION;
	frame->frame_id = frameID;
	frame->phase_count = HOLO_PHASE_COUNT;
	frame->phase_max = HOLO_PHASE_MAX;
	for(int board = 0; board < NUM_BOARDS; board++) {
		for(int ch = 0; ch < CHANNELS_PER_BOARD; ch++) {
			int idx = board * CHANNELS_PER_BOARD + ch;
			frame->phases[idx] = phase_radians_to_ticks(
				calculate_focus_phase_radians(particle->x, particle->y, particle->z,
					(enum BoardIndex)board, ch, boardDistanceMm));
		}
	}
	make_phase_frame_temporally_continuous(frame);
	frame->crc32 = holo_phase_frame_crc(frame);
}

static double pressure_magnitude_at(const HoloPhaseFrame *frame,
	double x, double y, double z, double boardDistanceMm)
{
	double fieldReal = 0.0;
	double fieldImag = 0.0;

	for(int board = 0; board < NUM_BOARDS; board++) {
		for(int ch = 0; ch < CHANNELS_PER_BOARD; ch++) {
			int idx = board * CHANNELS_PER_BOARD + ch;
			double tr;
			double ti;
			double phase = TWO_PI * (double)frame->phases[idx] / (double)HOLO_PHASE_MAX;
			double dr = cos(phase);
			double di = sin(phase);
			calculate_transfer_components(x, y, z, (enum BoardIndex)board, ch,
				boardDistanceMm, &tr, &ti);
			fieldReal += tr * dr - ti * di;
			fieldImag += tr * di + ti * dr;
		}
	}
	return hypot(fieldReal, fieldImag);
}

static void measure_trap_pressure_ratio(const HoloPhaseFrame *frame,
	const ParticleTarget *particles, const int *enabledIdx, int enabledCount,
	double boardDistanceMm, double *centerAvg, double *guardAvg)
{
	TrapConstraint constraints[MAX_PARTICLES * 7];
	int constraintCount = build_trap_constraints(particles, enabledIdx, enabledCount, constraints);
	double centerSum = 0.0;
	double guardSum = 0.0;
	int centerCount = 0;
	int guardCount = 0;

	for(int c = 0; c < constraintCount; c++) {
		double mag = pressure_magnitude_at(frame, constraints[c].x, constraints[c].y,
			constraints[c].z, boardDistanceMm);
		if(constraints[c].targetReal == 0.0 && constraints[c].targetImag == 0.0) {
			centerSum += mag;
			centerCount++;
		} else {
			guardSum += mag;
			guardCount++;
		}
	}

	*centerAvg = centerCount ? centerSum / (double)centerCount : 0.0;
	*guardAvg = guardCount ? guardSum / (double)guardCount : 0.0;
}

static void fill_multi_particle_frame(HoloPhaseFrame *frame, uint16_t frameID,
	const ParticleTarget *particles, int particleCount, double boardDistanceMm)
{
	memset(frame, 0, sizeof(*frame));
	frame->magic = HOLO_PHASE_MAGIC;
	frame->version = HOLO_PHASE_VERSION;
	frame->frame_id = frameID;
	frame->phase_count = HOLO_PHASE_COUNT;
	frame->phase_max = HOLO_PHASE_MAX;

	int enabledIdx[MAX_PARTICLES];
	int enabledCount = collect_enabled_particles(particles, particleCount, enabledIdx);
	if(enabledCount > 0) {
		fill_guard_trap_phases(frame, particles, enabledIdx, enabledCount, boardDistanceMm);
	} else {
		for(int board = 0; board < NUM_BOARDS; board++) {
			for(int ch = 0; ch < CHANNELS_PER_BOARD; ch++) {
				int idx = board * CHANNELS_PER_BOARD + ch;
				frame->phases[idx] = 0;
			}
		}
	}
	make_phase_frame_temporally_continuous(frame);
	frame->crc32 = holo_phase_frame_crc(frame);
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

static socket_t connect_to_pi(const char *host, uint16_t port)
{
	socket_t sock = socket(AF_INET, SOCK_STREAM, 0);
	struct sockaddr_in addr;
	int one = 1;

	if(sock == INVALID_SOCKET) return INVALID_SOCKET;
	setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, (const char *)&one, sizeof(one));

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

static int send_ramp(socket_t sock, uint16_t *frameID, double radiusMm,
	double boardDistanceMm, CirclePlane plane, int rampFrames, unsigned rampDelayMs)
{
	HoloPhaseFrame frame;
	ParticleTarget particles[MAX_PARTICLES] = {
		{0.0, 0.0, 0.0, 1},
		{0.0, 0.0, 0.0, 0}
	};
	for(int i = 0; i <= rampFrames; i++) {
		double t = (double)i / (double)rampFrames;
		double smooth = t * t * (3.0 - 2.0 * t);
		set_single_circle_position(&particles[0], radiusMm * smooth, plane, 0.0);
		fill_single_focus_frame(&frame, (*frameID)++, &particles[0], boardDistanceMm);
		if(send_all(sock, &frame, sizeof(frame)) < 0) return -1;
		delay_ms(rampDelayMs);
	}
	return 0;
}

static int build_single_circle_frames(HoloPhaseFrame *frames, int frameCount,
	double radiusMm, double boardDistanceMm, CirclePlane plane)
{
	ParticleTarget particles[MAX_PARTICLES] = {
		{0.0, 0.0, 0.0, 1},
		{0.0, 0.0, 0.0, 0}
	};
	if(frameCount < 3) return -1;
	reset_phase_temporal_continuity();
	for(int i = 0; i < frameCount; i++) {
		double theta = TWO_PI * (double)i / (double)frameCount;
		set_single_circle_position(&particles[0], radiusMm, plane, theta);
		fill_single_focus_frame(&frames[i], (uint16_t)i, &particles[0], boardDistanceMm);
	}
	reset_phase_temporal_continuity();
	return 0;
}

static int stream_single_circle_fast(socket_t sock, HoloPhaseFrame *frames, int frameCount,
	uint16_t *frameID, unsigned delayUs, unsigned long maxFrames, LightState *light)
{
	unsigned long sent = 0;
	unsigned long reportStartFrames = 0;
	double start = now_seconds();
	double reportStart = start;
	double lastDelayUs = 0.0;

	printf("streaming precomputed fast single-particle circle: ramping for %.1f seconds, then holding max speed\n",
		DEFAULT_FAST_RAMP_SECONDS);
	if(delayUs > 0) {
		printf("target max delay %u us, target max %.1f fps, %.2f circles/s\n",
			delayUs, 1000000.0 / (double)delayUs,
			1000000.0 / ((double)delayUs * (double)frameCount));
	} else {
		printf("target max delay 0 us, holding uncapped sender/transport speed after ramp\n");
	}
	printf("press q to stop, any other key prints status\n");
	while(maxFrames == 0 || sent < maxFrames) {
		HoloPhaseFrame *frame = &frames[sent % (unsigned long)frameCount];
		unsigned currentDelayUs = delayUs;
		double elapsedTotal = now_seconds() - start;
		int ramping = elapsedTotal < DEFAULT_FAST_RAMP_SECONDS;
		if(ramping) {
			double t = elapsedTotal / DEFAULT_FAST_RAMP_SECONDS;
			double smooth = t * t * (3.0 - 2.0 * t);
			currentDelayUs = (unsigned)floor(
				(double)FAST_RAMP_START_DELAY_US +
				((double)delayUs - (double)FAST_RAMP_START_DELAY_US) * smooth + 0.5);
		}
		frame->frame_id = (*frameID)++;
		frame->crc32 = holo_phase_frame_crc(frame);
		if(send_all(sock, frame, sizeof(*frame)) < 0) return -1;
		sent++;
		lastDelayUs = (double)currentDelayUs;
		delay_us(currentDelayUs);

		if(key_pressed()) {
			int ch = read_key();
			if(ch == 'q' || ch == 'Q') break;
			if(handle_light_key(sock, frameID, light, ch) < 0)
				return -1;
			double now = now_seconds();
			double elapsed = fmax(now - reportStart, 1.0e-9);
			printf("%lu fast frames total, %.1f fps, %.2f circles/s recent, command delay %.0f us%s\n",
				sent, (double)(sent - reportStartFrames) / elapsed,
				(double)(sent - reportStartFrames) / elapsed / (double)frameCount,
				lastDelayUs, ramping ? " ramping" : " holding");
			reportStart = now;
			reportStartFrames = sent;
		}

		if(now_seconds() - reportStart >= 1.0) {
			double now = now_seconds();
			double elapsed = fmax(now - reportStart, 1.0e-9);
			printf("%lu fast frames total, %.1f fps, %.2f circles/s recent, command delay %.0f us%s\n",
				sent, (double)(sent - reportStartFrames) / elapsed,
				(double)(sent - reportStartFrames) / elapsed / (double)frameCount,
				lastDelayUs, ramping ? " ramping" : " holding");
			reportStart = now;
			reportStartFrames = sent;
		}
	}
	printf("fast circle stopped after %lu frames in %.3f s\n", sent, now_seconds() - start);
	return 0;
}

static int send_dual_circle_ramp(socket_t sock, HoloPhaseFrame *frame, uint16_t *frameID,
	ParticleTarget *particles, double radiusMm, double boardDistanceMm,
	const CircleCenters *centers,
	int rampFrames, unsigned rampDelayMs)
{
	double startX[MAX_PARTICLES] = {particles[0].x, particles[1].x};
	double startY[MAX_PARTICLES] = {particles[0].y, particles[1].y};
	double startZ[MAX_PARTICLES] = {particles[0].z, particles[1].z};
	ParticleTarget end[MAX_PARTICLES] = {{0.0, 0.0, 0.0, 1}, {0.0, 0.0, 0.0, 1}};

	particles[0].enabled = 1;
	particles[1].enabled = 1;
	set_dual_circle_positions(end, radiusMm, centers, 0.0);

	for(int i = 0; i <= rampFrames; i++) {
		double t = (double)i / (double)rampFrames;
		double smooth = t * t * (3.0 - 2.0 * t);
		for(int p = 0; p < MAX_PARTICLES; p++) {
			particles[p].x = startX[p] + (end[p].x - startX[p]) * smooth;
			particles[p].y = startY[p] + (end[p].y - startY[p]) * smooth;
			particles[p].z = startZ[p] + (end[p].z - startZ[p]) * smooth;
		}
		if(send_particles(sock, frame, frameID, particles, boardDistanceMm) < 0)
			return -1;
		delay_ms(rampDelayMs);
	}
	return 0;
}

static int stream_dual_counter_circles(socket_t sock, HoloPhaseFrame *frame, uint16_t *frameID,
	ParticleTarget *particles, int frameCount, double radiusMm, double boardDistanceMm,
	const CircleCenters *centers,
	unsigned delayUs, unsigned long maxFrames, unsigned speedRampFrames, LightState *light)
{
	unsigned long sent = 0;
	unsigned long reportStartFrames = 0;
	double theta = 0.0;
	double start = now_seconds();
	double reportStart = start;
	double nominalStep = TWO_PI / (double)frameCount;

	particles[0].enabled = 1;
	particles[1].enabled = 1;

	printf("streaming two local opposite circles with %u-frame speed ramp: press q to stop, any other key prints status\n",
		speedRampFrames);
	while(maxFrames == 0 || sent < maxFrames) {
		double speedScale = 1.0;
		if(speedRampFrames > 0 && sent < speedRampFrames) {
			double t = (double)sent / (double)speedRampFrames;
			speedScale = t * t * (3.0 - 2.0 * t);
		}
		set_dual_circle_positions(particles, radiusMm, centers, theta);

		if(send_particles(sock, frame, frameID, particles, boardDistanceMm) < 0)
			return -1;
		sent++;
		theta += nominalStep * speedScale;
		if(theta >= TWO_PI) theta = fmod(theta, TWO_PI);
		delay_us(delayUs);

		if(key_pressed()) {
			int ch = read_key();
			if(ch == 'q' || ch == 'Q') break;
			if(handle_light_key(sock, frameID, light, ch) < 0)
				return -1;
			double now = now_seconds();
			double elapsed = fmax(now - reportStart, 1.0e-9);
			printf("%lu dual frames total, %.1f fps, %.2f circles/s recent\n",
				sent, (double)(sent - reportStartFrames) / elapsed,
				(double)(sent - reportStartFrames) / elapsed / (double)frameCount);
			reportStart = now;
			reportStartFrames = sent;
		}

		if(now_seconds() - reportStart >= 1.0) {
			double now = now_seconds();
			double elapsed = fmax(now - reportStart, 1.0e-9);
			printf("%lu dual frames total, %.1f fps, %.2f circles/s recent\n",
				sent, (double)(sent - reportStartFrames) / elapsed,
				(double)(sent - reportStartFrames) / elapsed / (double)frameCount);
			reportStart = now;
			reportStartFrames = sent;
		}
	}
	printf("dual circles stopped after %lu frames in %.3f s\n", sent, now_seconds() - start);
	return 0;
}

static void print_particles(const ParticleTarget *particles, int selected)
{
	for(int i = 0; i < MAX_PARTICLES; i++) {
		printf("%c particle %d: %s at %.2f, %.2f, %.2f mm\n",
			i == selected ? '*' : ' ',
			i + 1,
			particles[i].enabled ? "on " : "off",
			particles[i].x, particles[i].y, particles[i].z);
	}
}

static int send_particles(socket_t sock, HoloPhaseFrame *frame, uint16_t *frameID,
	const ParticleTarget *particles, double boardDistanceMm)
{
	fill_multi_particle_frame(frame, (*frameID)++, particles, MAX_PARTICLES, boardDistanceMm);
	return send_all(sock, frame, sizeof(*frame));
}

static int send_light_control(socket_t sock, uint16_t *frameID, const LightState *light)
{
	HoloPhaseFrame packet;
	memset(&packet, 0, sizeof(packet));
	packet.magic = HOLO_CONTROL_MAGIC;
	packet.version = HOLO_PHASE_VERSION;
	packet.frame_id = (*frameID)++;
	packet.phase_count = 0;
	packet.phase_max = HOLO_PHASE_MAX;
	packet.phases[0] = HOLO_CONTROL_SET_LIGHT;
	packet.phases[1] = light->redEnabled ? light->red : 0;
	packet.phases[2] = light->greenEnabled ? light->green : 0;
	packet.phases[3] = light->blueEnabled ? light->blue : 0;
	packet.phases[4] = light->brightness;
	packet.phases[5] = light->blink ? 1u : 0u;
	packet.phases[6] = light->enabled ? 1u : 0u;
	packet.crc32 = holo_phase_frame_crc(&packet);
	return send_all(sock, &packet, sizeof(packet));
}

static void apply_light_preset(LightState *light, int preset)
{
	int count = (int)(sizeof(photoLightPresets) / sizeof(photoLightPresets[0]));
	while(preset < 0)
		preset += count;
	preset %= count;
	light->preset = preset;
	light->red = photoLightPresets[preset].red;
	light->green = photoLightPresets[preset].green;
	light->blue = photoLightPresets[preset].blue;
	light->enabled = 1;
}

static void print_light_state(const LightState *light)
{
	const char *presetName = "custom";
	unsigned outRed = light->redEnabled ? light->red : 0;
	unsigned outGreen = light->greenEnabled ? light->green : 0;
	unsigned outBlue = light->blueEnabled ? light->blue : 0;
	int presetCount = (int)(sizeof(photoLightPresets) / sizeof(photoLightPresets[0]));
	if(light->preset >= 0 && light->preset < presetCount &&
	   light->red == photoLightPresets[light->preset].red &&
	   light->green == photoLightPresets[light->preset].green &&
	   light->blue == photoLightPresets[light->preset].blue)
		presetName = photoLightPresets[light->preset].name;
	printf("light %s preset=%d %s rgb=%u,%u,%u out=%u,%u,%u channels=%c%c%c brightness=%u%s\n",
		light->enabled ? "on " : "off",
		light->preset + 1, presetName,
		(unsigned)light->red, (unsigned)light->green, (unsigned)light->blue,
		outRed, outGreen, outBlue,
		light->redEnabled ? 'R' : '-',
		light->greenEnabled ? 'G' : '-',
		light->blueEnabled ? 'B' : '-',
		(unsigned)light->brightness, light->blink ? " blink" : "");
}

static int handle_light_key(socket_t sock, uint16_t *frameID, LightState *light, int ch)
{
	int changed = 0;
	if(ch == 'L') {
		light->enabled = !light->enabled;
		changed = 1;
	} else if(ch == '[') {
		light->brightness = light->brightness > 16 ? (uint8_t)(light->brightness - 16) : 0;
		changed = 1;
	} else if(ch == ']') {
		light->brightness = light->brightness < 239 ? (uint8_t)(light->brightness + 16) : 255;
		changed = 1;
	} else if(ch == 'B') {
		light->blink = !light->blink;
		changed = 1;
	} else if(ch == 'r') {
		light->redEnabled = !light->redEnabled;
		changed = 1;
	} else if(ch == 'e') {
		light->greenEnabled = !light->greenEnabled;
		changed = 1;
	} else if(ch == 'b') {
		light->blueEnabled = !light->blueEnabled;
		changed = 1;
	} else if(ch == 'A') {
		light->redEnabled = 1;
		light->greenEnabled = 1;
		light->blueEnabled = 1;
		changed = 1;
	} else if(ch == 'C') {
		apply_light_preset(light, light->preset + 1);
		changed = 1;
	} else if(ch == 'V') {
		apply_light_preset(light, light->preset - 1);
		changed = 1;
	} else if(ch == 'R') {
		light->red = 255; light->green = 0; light->blue = 0; light->enabled = 1; light->preset = -1;
		light->redEnabled = 1; light->greenEnabled = 1; light->blueEnabled = 1;
		changed = 1;
	} else if(ch == 'G') {
		light->red = 0; light->green = 255; light->blue = 0; light->enabled = 1; light->preset = -1;
		light->redEnabled = 1; light->greenEnabled = 1; light->blueEnabled = 1;
		changed = 1;
	} else if(ch == 'Y') {
		light->red = 255; light->green = 160; light->blue = 0; light->enabled = 1; light->preset = -1;
		light->redEnabled = 1; light->greenEnabled = 1; light->blueEnabled = 1;
		changed = 1;
	} else if(ch == 'W') {
		light->red = 255; light->green = 255; light->blue = 255; light->enabled = 1; light->preset = -1;
		light->redEnabled = 1; light->greenEnabled = 1; light->blueEnabled = 1;
		changed = 1;
	} else if(ch == 'U') {
		light->red = 0; light->green = 0; light->blue = 255; light->enabled = 1; light->preset = -1;
		light->redEnabled = 1; light->greenEnabled = 1; light->blueEnabled = 1;
		changed = 1;
	}

	if(!changed)
		return 0;
	if(send_light_control(sock, frameID, light) < 0)
		return -1;
	print_light_state(light);
	return 1;
}

static void print_usage(const char *argv0)
{
	printf("usage: %s <pi-ip-or-host> [port] [board-mm] [radius-mm] [frames] [delay-us] [max-frames] [plane]\n", argv0);
	printf("       %s --self-test [board-mm] [radius-mm] [frames]\n", argv0);
	printf("defaults: port=%u board=%.1f radius=%.1f frames=%d delay-us=%u max-frames=0(infinite)\n",
		HOLO_PHASE_TCP_PORT, DEFAULT_BOARD_DISTANCE_MM, DEFAULT_RADIUS_MM,
		DEFAULT_FRAMES_PER_CIRCLE, DEFAULT_DELAY_US);
	printf("delay-us is the held max speed after ramp: try 10000 gentler, 3000 faster, 0 for max transport\n");
	printf("plane selects the single-particle circle plane: xz(default), xy, or yz\n");
	printf("commands after connect:\n");
	printf("  1-9    select particle; selecting a new particle enables it at centre\n");
	printf("  x/s    decrease/increase X of selected particle\n");
	printf("  c/d    decrease/increase Y of selected particle\n");
	printf("  z/a    decrease/increase Z of selected particle\n");
	printf("  h      move selected particle to centre\n");
	printf("  0      disable selected particle, except particle 1\n");
	printf("  Enter  resend current particles\n");
	printf("  p      print particle positions\n");
	printf("  g      ramp up then hold precomputed fast single-particle circle\n");
	printf("  o      two particles on local opposite X-Z circles using current Y/Z\n");
	printf("  L      toggle lights, [/]=brightness, C/V photo colors, R/G/U/Y/W quick colors, B blink\n");
	printf("  r/e/b  toggle red/green/blue LED channels, A all LED channels on\n");
	printf("  q      quit\n");
	printf("photo colors via C/V:\n");
	for(size_t i = 0; i < sizeof(photoLightPresets) / sizeof(photoLightPresets[0]); i++) {
		printf("  %u      %s rgb=%u,%u,%u\n", (unsigned)(i + 1),
			photoLightPresets[i].name,
			(unsigned)photoLightPresets[i].red,
			(unsigned)photoLightPresets[i].green,
			(unsigned)photoLightPresets[i].blue);
	}
}

static double requested_circle_rate_hz(int frameCount, unsigned delayUs)
{
	if(frameCount <= 0 || delayUs == 0)
		return 0.0;
	return 1000000.0 / ((double)frameCount * (double)delayUs);
}

static void print_circle_dynamics(double radiusMm, double circleHz)
{
	if(circleHz <= 0.0)
		return;
	double omega = TWO_PI * circleHz;
	double speedMmS = omega * radiusMm;
	double accelMS2 = radiusMm * omega * omega / 1000.0;
	printf("requested path speed %.1f mm/s, centripetal %.1f m/s^2 (%.1f g)\n",
		speedMmS, accelMS2, accelMS2 / 9.80665);
	if(accelMS2 > STABILITY_ACCEL_WARN_M_S2) {
		double gentlerRadiusMm = STABILITY_ACCEL_WARN_M_S2 * 1000.0 / (omega * omega);
		printf("stability warning: %.2f mm at %.2f circles/s asks for high trap force; %.2f mm is gentler at the same speed\n",
			radiusMm, circleHz, gentlerRadiusMm);
	}
}

static void print_transport_budget(int frameCount, unsigned delayUs)
{
	if(delayUs == 0) {
		printf("transport budget: uncapped sender; Pi/FPGA SPI speed sets the real frame rate\n");
		return;
	}

	double targetFps = 1000000.0 / (double)delayUs;
	double requiredSpiHz = targetFps * (double)(1 + HOLO_PHASE_COUNT * 2) * 8.0;
	printf("transport budget: target %.0f phase frames/s for %d-frame circles; Pi bridge needs about %.1f MHz SPI before overhead\n",
		targetFps, frameCount, requiredSpiHz / 1000000.0);
}

static int run_self_test(double boardDistanceMm, double radiusMm, int frameCount)
{
	ParticleTarget singleParticles[MAX_PARTICLES] = {
		{0.0, 0.0, 0.0, 1},
		{0.0, 0.0, 0.0, 0}
	};
	ParticleTarget particles[MAX_PARTICLES] = {
		{0.0, 0.0, 0.0, 1},
		{0.0, 0.0, 0.0, 1}
	};
	CircleCenters centers = {{0.0, 0.0}, {0.0, 0.0}};
	HoloPhaseFrame frame;
	double minSeparation = 1.0e9;
	double singleMaxStep = 0.0;
	double maxStep[MAX_PARTICLES] = {0.0, 0.0};
	double worstSingleCenterGuardRatio = 0.0;
	double worstSingleBelowAboveRatio = 0.0;
	double worstCenterGuardRatio = 0.0;
	ParticleTarget singlePrevious = {0.0, 0.0, 0.0, 1};
	ParticleTarget previous[MAX_PARTICLES] = {
		{0.0, 0.0, 0.0, 1},
		{0.0, 0.0, 0.0, 1}
	};

	if(frameCount < 3 || radiusMm < 0.0 || boardDistanceMm <= 0.0)
		return 1;

	for(int i = 0; i < frameCount; i++) {
		double theta = TWO_PI * (double)i / (double)frameCount;
		singleParticles[0].x = radiusMm * cos(theta);
		singleParticles[0].y = 0.0;
		singleParticles[0].z = radiusMm * sin(theta);
		if(i > 0) {
			double step = particle_distance(&singleParticles[0], &singlePrevious);
			if(step > singleMaxStep)
				singleMaxStep = step;
		}
		singlePrevious = singleParticles[0];
		fill_multi_particle_frame(&frame, (uint16_t)i, singleParticles, MAX_PARTICLES, boardDistanceMm);
		if(frame.magic != HOLO_PHASE_MAGIC || frame.version != HOLO_PHASE_VERSION ||
		   frame.phase_count != HOLO_PHASE_COUNT || frame.phase_max != HOLO_PHASE_MAX ||
		   frame.crc32 != holo_phase_frame_crc(&frame)) {
			printf("self-test failed: invalid single frame at index %d\n", i);
			return 1;
		}
		{
			int enabledIdx[MAX_PARTICLES];
			double centerAvg;
			double guardAvg;
			double belowMag;
			double aboveMag;
			int enabledCount = collect_enabled_particles(singleParticles, MAX_PARTICLES, enabledIdx);
			measure_trap_pressure_ratio(&frame, singleParticles, enabledIdx, enabledCount,
				boardDistanceMm, &centerAvg, &guardAvg);
			if(guardAvg > 0.0) {
				double ratio = centerAvg / guardAvg;
				if(ratio > worstSingleCenterGuardRatio)
					worstSingleCenterGuardRatio = ratio;
			}
			belowMag = pressure_magnitude_at(&frame,
				singleParticles[0].x, singleParticles[0].y - TRAP_GUARD_OFFSET_MM,
				singleParticles[0].z, boardDistanceMm);
			aboveMag = pressure_magnitude_at(&frame,
				singleParticles[0].x, singleParticles[0].y + TRAP_GUARD_OFFSET_MM,
				singleParticles[0].z, boardDistanceMm);
			if(aboveMag > 0.0) {
				double ratio = belowMag / aboveMag;
				if(ratio > worstSingleBelowAboveRatio)
					worstSingleBelowAboveRatio = ratio;
			}
		}
	}

	for(int i = 0; i < frameCount; i++) {
		double theta = TWO_PI * (double)i / (double)frameCount;
		set_dual_circle_positions(particles, radiusMm, &centers, theta);
		double sep = particle_distance(&particles[0], &particles[1]);
		if(sep < minSeparation)
			minSeparation = sep;
		if(i > 0) {
			for(int p = 0; p < MAX_PARTICLES; p++) {
				double step = particle_distance(&particles[p], &previous[p]);
				if(step > maxStep[p])
					maxStep[p] = step;
			}
		}
		previous[0] = particles[0];
		previous[1] = particles[1];
		fill_multi_particle_frame(&frame, (uint16_t)i, particles, MAX_PARTICLES, boardDistanceMm);
		if(frame.magic != HOLO_PHASE_MAGIC || frame.version != HOLO_PHASE_VERSION ||
		   frame.phase_count != HOLO_PHASE_COUNT || frame.phase_max != HOLO_PHASE_MAX ||
		   frame.crc32 != holo_phase_frame_crc(&frame)) {
			printf("self-test failed: invalid frame at index %d\n", i);
			return 1;
		}
		for(size_t ch = 0; ch < HOLO_PHASE_COUNT; ch++) {
			if(frame.phases[ch] >= HOLO_PHASE_MAX) {
				printf("self-test failed: phase %u out of range at frame %d\n", (unsigned)ch, i);
				return 1;
			}
		}
		{
			int enabledIdx[MAX_PARTICLES];
			double centerAvg;
			double guardAvg;
			int enabledCount = collect_enabled_particles(particles, MAX_PARTICLES, enabledIdx);
			measure_trap_pressure_ratio(&frame, particles, enabledIdx, enabledCount,
				boardDistanceMm, &centerAvg, &guardAvg);
			if(guardAvg > 0.0) {
				double ratio = centerAvg / guardAvg;
				if(ratio > worstCenterGuardRatio)
					worstCenterGuardRatio = ratio;
			}
		}
	}

	printf("self-test ok: %d single-circle frames, max step %.3f mm, worst center/guard pressure %.3f, worst below/above pressure %.3f\n",
		frameCount, singleMaxStep, worstSingleCenterGuardRatio, worstSingleBelowAboveRatio);
	printf("self-test ok: %d dual-circle frames, min particle separation %.3f mm, max step %.3f/%.3f mm, worst center/guard pressure %.3f\n",
		frameCount, minSeparation, maxStep[0], maxStep[1], worstCenterGuardRatio);
	if(worstSingleCenterGuardRatio >= 0.85) {
		printf("self-test failed: single trap center is not sufficiently lower than guard ring\n");
		return 1;
	}
	if(minSeparation + 1.0e-6 < 2.0 * radiusMm) {
		printf("self-test failed: particles should not approach closer than 2*radius\n");
		return 1;
	}
	if(worstCenterGuardRatio >= 0.85) {
		printf("self-test failed: trap center is not sufficiently lower than guard ring\n");
		return 1;
	}
	return 0;
}

int main(int argc, char **argv)
{
	const char *host;
	uint16_t port;
	double boardDistanceMm;
	double radiusMm;
	int frameCount;
	unsigned delayUs;
	unsigned long maxFrames;
	CirclePlane circlePlane = CIRCLE_PLANE_XZ;
	uint16_t frameID = 0;
	HoloPhaseFrame currentFrame;
	HoloPhaseFrame *singleCircleFrames = NULL;
	ParticleTarget particles[MAX_PARTICLES] = {
		{0.0, 0.0, 0.0, 1},
		{0.0, 0.0, 0.0, 0}
	};
	LightState light = {255, 214, 170, 80, 0, 0, 0, 1, 1, 1};
	int selectedParticle = 0;
	socket_t sock;

	if(argc >= 2 && strcmp(argv[1], "--self-test") == 0) {
		boardDistanceMm = argc >= 3 ? atof(argv[2]) : DEFAULT_BOARD_DISTANCE_MM;
		radiusMm = argc >= 4 ? atof(argv[3]) : DEFAULT_RADIUS_MM;
		frameCount = argc >= 5 ? atoi(argv[4]) : DEFAULT_FRAMES_PER_CIRCLE;
		return run_self_test(boardDistanceMm, radiusMm, frameCount);
	}

	if(argc < 2) {
		print_usage(argv[0]);
		return 1;
	}

	host = argv[1];
	port = argc >= 3 ? (uint16_t)atoi(argv[2]) : HOLO_PHASE_TCP_PORT;
	boardDistanceMm = argc >= 4 ? atof(argv[3]) : DEFAULT_BOARD_DISTANCE_MM;
	radiusMm = argc >= 5 ? atof(argv[4]) : DEFAULT_RADIUS_MM;
	frameCount = argc >= 6 ? atoi(argv[5]) : DEFAULT_FRAMES_PER_CIRCLE;
	delayUs = argc >= 7 ? (unsigned)strtoul(argv[6], NULL, 10) : DEFAULT_DELAY_US;
	maxFrames = argc >= 8 ? strtoul(argv[7], NULL, 10) : 0ul;
	if(argc >= 9 && !parse_circle_plane(argv[8], &circlePlane)) {
		printf("unknown circle plane '%s'\n", argv[8]);
		print_usage(argv[0]);
		return 1;
	}

	if(port == 0 || boardDistanceMm <= 0.0 || radiusMm < 0.0 || frameCount < 3) {
		print_usage(argv[0]);
		return 1;
	}

#ifdef _WIN32
	{
		WSADATA wsa;
		if(WSAStartup(MAKEWORD(2, 2), &wsa) != 0) return 1;
	}
#endif

	singleCircleFrames = (HoloPhaseFrame *)calloc((size_t)frameCount, sizeof(HoloPhaseFrame));
	if(!singleCircleFrames) {
		printf("could not allocate %d single circle frames\n", frameCount);
#ifdef _WIN32
		WSACleanup();
#endif
		return 1;
	}

	printf("precomputing %d stable single-particle circle frames...\n", frameCount);
	if(build_single_circle_frames(singleCircleFrames, frameCount, radiusMm, boardDistanceMm, circlePlane) != 0) {
		printf("could not build single circle frames\n");
		free(singleCircleFrames);
#ifdef _WIN32
		WSACleanup();
#endif
		return 1;
	}

	sock = connect_to_pi(host, port);
	if(sock == INVALID_SOCKET) {
		printf("could not connect to %s:%u\n", host, port);
		free(singleCircleFrames);
#ifdef _WIN32
		WSACleanup();
#endif
		return 1;
	}

	if(send_light_control(sock, &frameID, &light) < 0) {
		printf("initial light-off send failed\n");
		free(singleCircleFrames);
		close_socket(sock);
#ifdef _WIN32
		WSACleanup();
#endif
		return 1;
	}

	fill_multi_particle_frame(&currentFrame, frameID++, particles, MAX_PARTICLES, boardDistanceMm);

	if(terminal_raw_mode(1) != 0) {
		printf("warning: could not switch terminal to raw mode; controls may require Enter\n");
	}

	printf("AcousticLev-style two-particle sender connected to %s:%u\n", host, port);
	printf("single-particle circle is %s around centre: radius %.2f mm, %d frames, delay %u us",
		circle_plane_name(circlePlane), radiusMm, frameCount, delayUs);
	if(delayUs > 0)
		printf(" (requested %.2f circles/s before transfer overhead)\n",
			requested_circle_rate_hz(frameCount, delayUs));
	else
		printf(" (uncapped transport stress-test)\n");
	print_circle_dynamics(radiusMm, requested_circle_rate_hz(frameCount, delayUs));
	print_transport_budget(frameCount, delayUs);
	print_usage(argv[0]);
	print_particles(particles, selectedParticle);
	print_light_state(&light);

	if(send_all(sock, &currentFrame, sizeof(currentFrame)) < 0) {
		printf("initial particle send failed\n");
	} else {
		printf("particle 1 is active at centre; move it away, then press 2-9 to add more particles at centre\n");
		while(1) {
			int ch = read_key();
			int sendCurrent = 0;
			int lightResult = 0;
			if(ch == EOF) {
				delay_ms(10);
				continue;
			}
			if(ch == 'q' || ch == 'Q') break;

			if(ch >= '1' && ch <= '9') {
				selectedParticle = ch - '1';
				if(!particles[selectedParticle].enabled) {
					particles[selectedParticle].x = 0.0;
					particles[selectedParticle].y = 0.0;
					particles[selectedParticle].z = 0.0;
					particles[selectedParticle].enabled = 1;
				}
				sendCurrent = 1;
			} else if(ch == '0') {
				if(selectedParticle == 0) {
					printf("particle 1 stays enabled so there is always at least one trap\n");
				} else {
					particles[selectedParticle].enabled = 0;
					sendCurrent = 1;
				}
			} else if(ch == 'h' || ch == 'H') {
				particles[selectedParticle].x = 0.0;
				particles[selectedParticle].y = 0.0;
				particles[selectedParticle].z = 0.0;
				particles[selectedParticle].enabled = 1;
				sendCurrent = 1;
			} else if(ch == 'x') {
				particles[selectedParticle].x -= MOVE_INC_MM;
				particles[selectedParticle].enabled = 1;
				sendCurrent = 1;
			} else if(ch == 's') {
				particles[selectedParticle].x += MOVE_INC_MM;
				particles[selectedParticle].enabled = 1;
				sendCurrent = 1;
			} else if(ch == 'c') {
				particles[selectedParticle].y -= MOVE_INC_MM;
				particles[selectedParticle].enabled = 1;
				sendCurrent = 1;
			} else if(ch == 'd') {
				particles[selectedParticle].y += MOVE_INC_MM;
				particles[selectedParticle].enabled = 1;
				sendCurrent = 1;
			} else if(ch == 'z') {
				particles[selectedParticle].z -= MOVE_INC_MM;
				particles[selectedParticle].enabled = 1;
				sendCurrent = 1;
			} else if(ch == 'a') {
				particles[selectedParticle].z += MOVE_INC_MM;
				particles[selectedParticle].enabled = 1;
				sendCurrent = 1;
			} else if(ch == 'p' || ch == 'P') {
				print_particles(particles, selectedParticle);
			} else if(ch == 'o' || ch == 'O') {
				CircleCenters centers = {
					{particles[0].y, particles[1].y},
					{particles[0].z, particles[1].z}
				};
				if(send_dual_circle_ramp(sock, &currentFrame, &frameID, particles,
					   radiusMm, boardDistanceMm, &centers,
					   DEFAULT_RAMP_FRAMES, DEFAULT_RAMP_DELAY_MS) < 0) {
					printf("dual circle ramp send failed\n");
					break;
				}
				if(stream_dual_counter_circles(sock, &currentFrame, &frameID, particles,
					   frameCount, radiusMm, boardDistanceMm, &centers,
					   delayUs, maxFrames, DEFAULT_SPEED_RAMP_FRAMES, &light) < 0) {
					printf("dual circle send failed\n");
					break;
				}
				selectedParticle = 0;
				sendCurrent = 1;
			} else if(ch == 'g' || ch == 'G') {
				if(send_ramp(sock, &frameID, radiusMm, boardDistanceMm,
					   circlePlane, DEFAULT_RAMP_FRAMES, DEFAULT_RAMP_DELAY_MS) < 0) {
					printf("ramp send failed\n");
					break;
				}
				if(stream_single_circle_fast(sock, singleCircleFrames, frameCount,
					   &frameID, delayUs, maxFrames, &light) < 0) {
					printf("circle send failed\n");
					break;
				}
				sendCurrent = 1;
			} else if((lightResult = handle_light_key(sock, &frameID, &light, ch)) != 0) {
				if(lightResult < 0)
					break;
				sendCurrent = 0;
			} else if(ch == '\n' || ch == '\r') {
				sendCurrent = 1;
			} else {
				print_usage(argv[0]);
			}

			if(sendCurrent) {
				if(send_particles(sock, &currentFrame, &frameID, particles, boardDistanceMm) < 0) break;
				printf("sent frame %u\n", currentFrame.frame_id);
				print_particles(particles, selectedParticle);
			}
		}
	}

	terminal_raw_mode(0);
	free(singleCircleFrames);
	close_socket(sock);
#ifdef _WIN32
	WSACleanup();
#endif
	return 0;
}
