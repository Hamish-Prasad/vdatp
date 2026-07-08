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
 * gcc -std=gnu99 -O3 -Wall -Wextra cli\laptop_file_sender_lev.c -o cli\laptop_file_sender_lev.exe -lm -lws2_32
 * .\cli\laptop_file_sender_lev.exe 169.254.3.160 5656 135 3 64 0
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
#define DEFAULT_BOARD_DISTANCE_MM 135.0
#define DEFAULT_RADIUS_MM 3.0
#define DEFAULT_FRAMES_PER_CIRCLE 64
#define DEFAULT_DELAY_US 0
#define DEFAULT_RAMP_FRAMES 64
#define DEFAULT_RAMP_DELAY_MS 6
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

static const int16_t xCols[5] = {450, 350, 250, 150, 50};
static const int16_t zRows[10] = {-450, -350, -250, -150, -50, 50, 150, 250, 350, 450};

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
	if(us < 1000u) Sleep(1);
	else Sleep((us + 999u) / 1000u);
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
	double x = (double)xCols[channel / 10] * 0.1;
	return (board == BOARD_LEFT_TOP || board == BOARD_LEFT_BOTTOM) ? x : -x;
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

static uint16_t calculate_acousticlev_phase(double tx, double ty, double tz,
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
	return phase_radians_to_ticks(drivePhase);
}

static void fill_phase_frame(HoloPhaseFrame *frame, uint16_t frameID,
	double x, double y, double z, double boardDistanceMm)
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
			frame->phases[idx] = calculate_acousticlev_phase(
				x, y, z, (enum BoardIndex)board, ch, boardDistanceMm);
		}
	}
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

		if(sent % 1000ul == 0ul) {
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

static void print_usage(const char *argv0)
{
	printf("usage: %s <pi-ip-or-host> [port] [board-mm] [radius-mm] [frames] [delay-us] [max-frames]\n", argv0);
	printf("defaults: port=%u board=%.1f radius=%.1f frames=%d delay-us=%u max-frames=0(infinite)\n",
		HOLO_PHASE_TCP_PORT, DEFAULT_BOARD_DISTANCE_MM, DEFAULT_RADIUS_MM,
		DEFAULT_FRAMES_PER_CIRCLE, DEFAULT_DELAY_US);
	printf("commands after connect:\n");
	printf("  Enter  resend origin holding frame\n");
	printf("  g      ramp to the circle and stream as fast as configured\n");
	printf("  q      quit\n");
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
	HoloPhaseFrame origin;
	HoloPhaseFrame *circleFrames = NULL;
	socket_t sock;

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

	fill_phase_frame(&origin, frameID++, 0.0, 0.0, 0.0, boardDistanceMm);
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

	printf("AcousticLev-style single-particle sender connected to %s:%u\n", host, port);
	printf("circle is X-Z around y=0: radius %.2f mm, %d frames, delay %u us\n",
		radiusMm, frameCount, delayUs);
	print_usage(argv[0]);

	if(send_all(sock, &origin, sizeof(origin)) < 0) {
		printf("origin send failed\n");
	} else {
		printf("holding origin; press g to draw the circle\n");
		while(1) {
			int ch = read_key();
			if(ch == EOF) {
				delay_ms(10);
				continue;
			}
			if(ch == 'q' || ch == 'Q') break;
			if(ch == 'g' || ch == 'G') {
				if(send_ramp(sock, &frameID, radiusMm, boardDistanceMm,
					   DEFAULT_RAMP_FRAMES, DEFAULT_RAMP_DELAY_MS) < 0) {
					printf("ramp send failed\n");
					break;
				}
				if(stream_circle(sock, circleFrames, frameCount, &frameID, delayUs, maxFrames) < 0) {
					printf("circle send failed\n");
					break;
				}
				fill_phase_frame(&origin, frameID++, 0.0, 0.0, 0.0, boardDistanceMm);
				if(send_all(sock, &origin, sizeof(origin)) < 0) break;
				printf("holding origin again; press g to repeat or q to quit\n");
			} else if(ch == '\n' || ch == '\r') {
				origin.frame_id = frameID++;
				origin.crc32 = holo_phase_frame_crc(&origin);
				if(send_all(sock, &origin, sizeof(origin)) < 0) break;
				printf("resent origin frame %u\n", origin.frame_id);
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
