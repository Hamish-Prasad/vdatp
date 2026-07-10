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
 * AcousticLev uses an analytic baffled-piston transducer model,
 * J0(k*r*sin(theta))*exp(i*k*d)/d, then optimizes phase holograms using
 * pressure derivatives.  This live sender keeps the analytic phase model and
 * our opposed-array pi split, but avoids online NLopt/AD so it can stream a
 * single moving trap as quickly as the Pi/FPGA path will accept frames.
 * gcc -std=gnu99 -O3 -Wall -Wextra laptop_file_sender_lev.c -o laptop_file_sender_lev.exe -lm -lws2_32
 * laptop_file_sender_lev.exe 169.254.251.233 5656 135 3 64 10000
 * laptop_file_sender_lev.exe 169.254.65.146 5656 135 3 64 4000
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
#define MAX_PARTICLES 2
#define TOTAL_CHANNELS (NUM_BOARDS * CHANNELS_PER_BOARD)
#define MULTI_PARTICLE_ITERATIONS 12
#define DEFAULT_BOARD_DISTANCE_MM 135.0
#define DEFAULT_RADIUS_MM 3.0
#define DEFAULT_FRAMES_PER_CIRCLE 64
#define DEFAULT_DELAY_US 5000
#define DEFAULT_RAMP_FRAMES 64
#define DEFAULT_RAMP_DELAY_MS 6
#define DEFAULT_SPEED_RAMP_FRAMES 128
#define MOVE_INC_MM 0.5
#define TRANSDUCER_RADIUS_MM 5.0
#define SOUND_SPEED_MM_S 343000.0
#define FREQUENCY_HZ 40000.0
#define PI 3.14159265358979323846
#define TWO_PI (2.0 * PI)

enum BoardIndex {
	BOARD_LEFT_TOP = 0,
	BOARD_RIGHT_TOP = 1,
	BOARD_LEFT_BOTTOM = 2,
	BOARD_RIGHT_BOTTOM = 3
};

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

static const int16_t xCols[5] = {450, 350, 250, 150, 50};
static const int16_t zRows[10] = {-450, -350, -250, -150, -50, 50, 150, 250, 350, 450};

static int send_particles(socket_t sock, HoloPhaseFrame *frame, uint16_t *frameID,
	const ParticleTarget *particles, double boardDistanceMm);

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

static double calculate_acousticlev_phase_radians(double tx, double ty, double tz,
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

static uint16_t calculate_multi_particle_phase(const ParticleTarget *particles,
	int particleCount, enum BoardIndex board, int channel, double boardDistanceMm)
{
	double real = 0.0;
	double imag = 0.0;
	int enabledCount = 0;

	for(int i = 0; i < particleCount; i++) {
		if(!particles[i].enabled)
			continue;

		double phase = calculate_acousticlev_phase_radians(
			particles[i].x, particles[i].y, particles[i].z,
			board, channel, boardDistanceMm);
		real += cos(phase);
		imag += sin(phase);
		enabledCount++;
	}

	if(enabledCount == 0)
		return 0;
	return phase_radians_to_ticks(atan2(imag, real));
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

static void fill_iterative_multi_particle_phases(HoloPhaseFrame *frame,
	const ParticleTarget *particles, const int *enabledIdx, int enabledCount,
	double boardDistanceMm)
{
	double focusReal[MAX_PARTICLES][TOTAL_CHANNELS];
	double focusImag[MAX_PARTICLES][TOTAL_CHANNELS];
	double targetReal[MAX_PARTICLES];
	double targetImag[MAX_PARTICLES];
	double driveReal[TOTAL_CHANNELS];
	double driveImag[TOTAL_CHANNELS];

	for(int p = 0; p < enabledCount; p++) {
		const ParticleTarget *particle = &particles[enabledIdx[p]];
		targetReal[p] = 1.0;
		targetImag[p] = 0.0;
		for(int board = 0; board < NUM_BOARDS; board++) {
			for(int ch = 0; ch < CHANNELS_PER_BOARD; ch++) {
				int idx = board * CHANNELS_PER_BOARD + ch;
				double phase = calculate_acousticlev_phase_radians(
					particle->x, particle->y, particle->z,
					(enum BoardIndex)board, ch, boardDistanceMm);
				focusReal[p][idx] = cos(phase);
				focusImag[p][idx] = sin(phase);
			}
		}
	}

	for(int iter = 0; iter < MULTI_PARTICLE_ITERATIONS; iter++) {
		for(int idx = 0; idx < TOTAL_CHANNELS; idx++) {
			double real = 0.0;
			double imag = 0.0;
			for(int p = 0; p < enabledCount; p++) {
				real += targetReal[p] * focusReal[p][idx] - targetImag[p] * focusImag[p][idx];
				imag += targetReal[p] * focusImag[p][idx] + targetImag[p] * focusReal[p][idx];
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

		for(int p = 0; p < enabledCount; p++) {
			double pressureReal = 0.0;
			double pressureImag = 0.0;
			for(int idx = 0; idx < TOTAL_CHANNELS; idx++) {
				pressureReal += focusReal[p][idx] * driveReal[idx] + focusImag[p][idx] * driveImag[idx];
				pressureImag += focusReal[p][idx] * driveImag[idx] - focusImag[p][idx] * driveReal[idx];
			}
			double mag = hypot(pressureReal, pressureImag);
			if(mag > 0.0) {
				targetReal[p] = pressureReal / mag;
				targetImag[p] = pressureImag / mag;
			}
		}
	}

	for(int idx = 0; idx < TOTAL_CHANNELS; idx++)
		frame->phases[idx] = phase_radians_to_ticks(atan2(driveImag[idx], driveReal[idx]));
}

static void make_phase_frame_temporally_continuous(HoloPhaseFrame *frame)
{
	static uint16_t previous[HOLO_PHASE_COUNT];
	static int havePrevious = 0;
	double real = 0.0;
	double imag = 0.0;

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
	if(enabledCount > 1) {
		fill_iterative_multi_particle_phases(frame, particles, enabledIdx, enabledCount, boardDistanceMm);
	} else {
		for(int board = 0; board < NUM_BOARDS; board++) {
			for(int ch = 0; ch < CHANNELS_PER_BOARD; ch++) {
				int idx = board * CHANNELS_PER_BOARD + ch;
				frame->phases[idx] = calculate_multi_particle_phase(
					particles, particleCount, (enum BoardIndex)board, ch, boardDistanceMm);
			}
		}
	}
	make_phase_frame_temporally_continuous(frame);
	frame->crc32 = holo_phase_frame_crc(frame);
}

static void fill_phase_frame(HoloPhaseFrame *frame, uint16_t frameID,
	double x, double y, double z, double boardDistanceMm)
{
	ParticleTarget particle = {x, y, z, 1};
	fill_multi_particle_frame(frame, frameID, &particle, 1, boardDistanceMm);
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

static int build_circle_frames(HoloPhaseFrame *frames, int frameCount,
	uint16_t *nextFrameID, double radiusMm, double boardDistanceMm)
{
	if(frameCount < 3) return -1;
	for(int i = 0; i < frameCount; i++) {
		double theta = TWO_PI * (double)i / (double)frameCount;
		double x = radiusMm * cos(theta);
		double y = 0.0;
		double z = radiusMm * sin(theta);
		fill_phase_frame(&frames[i], (*nextFrameID)++, x, y, z, boardDistanceMm);
	}
	return 0;
}

static int send_ramp(socket_t sock, uint16_t *frameID, double radiusMm,
	double boardDistanceMm, int rampFrames, unsigned rampDelayMs)
{
	HoloPhaseFrame frame;
	for(int i = 0; i <= rampFrames; i++) {
		double t = (double)i / (double)rampFrames;
		double smooth = t * t * (3.0 - 2.0 * t);
		fill_phase_frame(&frame, (*frameID)++, radiusMm * smooth, 0.0, 0.0, boardDistanceMm);
		if(send_all(sock, &frame, sizeof(frame)) < 0) return -1;
		delay_ms(rampDelayMs);
	}
	return 0;
}

static int stream_circle(socket_t sock, HoloPhaseFrame *frames, int frameCount,
	uint16_t *frameID,
	unsigned delayUs, unsigned long maxFrames)
{
	unsigned long sent = 0;
	unsigned long reportStartFrames = 0;
	double start = now_seconds();
	double reportStart = start;

	printf("streaming circle: press q to stop, any other key prints status\n");
	while(maxFrames == 0 || sent < maxFrames) {
		HoloPhaseFrame *frame = &frames[sent % (unsigned long)frameCount];
		frame->frame_id = (*frameID)++;
		frame->crc32 = holo_phase_frame_crc(frame);
		if(send_all(sock, frame, sizeof(*frame)) < 0) return -1;
		sent++;
		delay_us(delayUs);

		if(key_pressed()) {
			int ch = read_key();
			if(ch == 'q' || ch == 'Q') break;
			double now = now_seconds();
			double elapsed = fmax(now - reportStart, 1.0e-9);
			printf("%lu frames total, %.1f fps, %.2f circles/s recent\n",
				sent, (double)(sent - reportStartFrames) / elapsed,
				(double)(sent - reportStartFrames) / elapsed / (double)frameCount);
			reportStart = now;
			reportStartFrames = sent;
		}

		if(now_seconds() - reportStart >= 1.0) {
			double now = now_seconds();
			double elapsed = fmax(now - reportStart, 1.0e-9);
			printf("%lu frames total, %.1f fps, %.2f circles/s recent\n",
				sent, (double)(sent - reportStartFrames) / elapsed,
				(double)(sent - reportStartFrames) / elapsed / (double)frameCount);
			reportStart = now;
			reportStartFrames = sent;
		}
	}
	printf("circle stopped after %lu frames in %.3f s\n", sent, now_seconds() - start);
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
	unsigned delayUs, unsigned long maxFrames, unsigned speedRampFrames)
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

static void print_usage(const char *argv0)
{
	printf("usage: %s <pi-ip-or-host> [port] [board-mm] [radius-mm] [frames] [delay-us] [max-frames]\n", argv0);
	printf("       %s --self-test [board-mm] [radius-mm] [frames]\n", argv0);
	printf("defaults: port=%u board=%.1f radius=%.1f frames=%d delay-us=%u max-frames=0(infinite)\n",
		HOLO_PHASE_TCP_PORT, DEFAULT_BOARD_DISTANCE_MM, DEFAULT_RADIUS_MM,
		DEFAULT_FRAMES_PER_CIRCLE, DEFAULT_DELAY_US);
	printf("delay-us controls particle speed: try 10000 gentler, 3000 faster, 0 for transport stress-test\n");
	printf("commands after connect:\n");
	printf("  1/2    select particle; selecting 2 enables it at centre\n");
	printf("  x/s    decrease/increase X of selected particle\n");
	printf("  c/d    decrease/increase Y of selected particle\n");
	printf("  z/a    decrease/increase Z of selected particle\n");
	printf("  h      move selected particle to centre\n");
	printf("  0      disable selected particle, except particle 1\n");
	printf("  Enter  resend current particles\n");
	printf("  p      print particle positions\n");
	printf("  g      legacy single-particle circle test\n");
	printf("  o      two particles on local opposite X-Z circles using current Y/Z\n");
	printf("  q      quit\n");
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
}

static int run_self_test(double boardDistanceMm, double radiusMm, int frameCount)
{
	ParticleTarget particles[MAX_PARTICLES] = {
		{0.0, 0.0, 0.0, 1},
		{0.0, 0.0, 0.0, 1}
	};
	CircleCenters centers = {{0.0, 0.0}, {0.0, 0.0}};
	HoloPhaseFrame frame;
	double minSeparation = 1.0e9;
	double maxStep[MAX_PARTICLES] = {0.0, 0.0};
	ParticleTarget previous[MAX_PARTICLES] = {
		{0.0, 0.0, 0.0, 1},
		{0.0, 0.0, 0.0, 1}
	};

	if(frameCount < 3 || radiusMm < 0.0 || boardDistanceMm <= 0.0)
		return 1;

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
	}

	printf("self-test ok: %d dual-circle frames, min particle separation %.3f mm, max step %.3f/%.3f mm\n",
		frameCount, minSeparation, maxStep[0], maxStep[1]);
	if(minSeparation + 1.0e-6 < 2.0 * radiusMm) {
		printf("self-test failed: particles should not approach closer than 2*radius\n");
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
	uint16_t frameID = 0;
	HoloPhaseFrame currentFrame;
	HoloPhaseFrame *circleFrames = NULL;
	ParticleTarget particles[MAX_PARTICLES] = {
		{0.0, 0.0, 0.0, 1},
		{0.0, 0.0, 0.0, 0}
	};
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

	sock = connect_to_pi(host, port);
	if(sock == INVALID_SOCKET) {
		printf("could not connect to %s:%u\n", host, port);
#ifdef _WIN32
		WSACleanup();
#endif
		return 1;
	}

	circleFrames = (HoloPhaseFrame *)calloc((size_t)frameCount, sizeof(HoloPhaseFrame));
	if(!circleFrames) {
		printf("could not allocate %d circle frames\n", frameCount);
		close_socket(sock);
#ifdef _WIN32
		WSACleanup();
#endif
		return 1;
	}

	fill_multi_particle_frame(&currentFrame, frameID++, particles, MAX_PARTICLES, boardDistanceMm);
	if(build_circle_frames(circleFrames, frameCount, &frameID, radiusMm, boardDistanceMm) != 0) {
		printf("could not build circle frames\n");
		free(circleFrames);
		close_socket(sock);
#ifdef _WIN32
		WSACleanup();
#endif
		return 1;
	}

	if(terminal_raw_mode(1) != 0) {
		printf("warning: could not switch terminal to raw mode; controls may require Enter\n");
	}

	printf("AcousticLev-style two-particle sender connected to %s:%u\n", host, port);
	printf("circle is X-Z around y=0: radius %.2f mm, %d frames, delay %u us",
		radiusMm, frameCount, delayUs);
	if(delayUs > 0)
		printf(" (requested %.2f circles/s before transfer overhead)\n",
			requested_circle_rate_hz(frameCount, delayUs));
	else
		printf(" (uncapped transport stress-test)\n");
	print_circle_dynamics(radiusMm, requested_circle_rate_hz(frameCount, delayUs));
	print_usage(argv[0]);
	print_particles(particles, selectedParticle);

	if(send_all(sock, &currentFrame, sizeof(currentFrame)) < 0) {
		printf("initial particle send failed\n");
	} else {
		printf("particle 1 is active at centre; move it away, then press 2 to add particle 2 at centre\n");
		while(1) {
			int ch = read_key();
			int sendCurrent = 0;
			if(ch == EOF) {
				delay_ms(10);
				continue;
			}
			if(ch == 'q' || ch == 'Q') break;

			if(ch == '1') {
				selectedParticle = 0;
				particles[0].enabled = 1;
				sendCurrent = 1;
			} else if(ch == '2') {
				selectedParticle = 1;
				if(!particles[1].enabled) {
					particles[1].x = 0.0;
					particles[1].y = 0.0;
					particles[1].z = 0.0;
					particles[1].enabled = 1;
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
					   delayUs, maxFrames, DEFAULT_SPEED_RAMP_FRAMES) < 0) {
					printf("dual circle send failed\n");
					break;
				}
				selectedParticle = 0;
				sendCurrent = 1;
			} else if(ch == 'g' || ch == 'G') {
				if(send_ramp(sock, &frameID, radiusMm, boardDistanceMm,
					   DEFAULT_RAMP_FRAMES, DEFAULT_RAMP_DELAY_MS) < 0) {
					printf("ramp send failed\n");
					break;
				}
				if(stream_circle(sock, circleFrames, frameCount, &frameID, delayUs, maxFrames) < 0) {
					printf("circle send failed\n");
					break;
				}
				sendCurrent = 1;
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
	free(circleFrames);
	close_socket(sock);
#ifdef _WIN32
	WSACleanup();
#endif
	return 0;
}
