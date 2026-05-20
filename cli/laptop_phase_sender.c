#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#ifdef _WIN32
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

#include "phase_protocol.h"

#define NUM_BOARDS 4
#define CHANNELS_PER_BOARD 50
#define SCALE_0P1MM 10.0
#define MOVE_INC_MM 1.0

enum BoardIndex {
	BOARD_LEFT_TOP = 0,
	BOARD_RIGHT_TOP = 1,
	BOARD_LEFT_BOTTOM = 2,
	BOARD_RIGHT_BOTTOM = 3
};

static const int16_t xLeftTop[CHANNELS_PER_BOARD] = {
	450, 450, 450, 450, 450, 450, 450, 450, 450, 450,
	350, 350, 350, 350, 350, 350, 350, 350, 350, 350,
	250, 250, 250, 250, 250, 250, 250, 250, 250, 250,
	150, 150, 150, 150, 150, 150, 150, 150, 150, 150,
	50, 50, 50, 50, 50, 50, 50, 50, 50, 50
};

static const int16_t xRightTop[CHANNELS_PER_BOARD] = {
	-50, -50, -50, -50, -50, -50, -50, -50, -50, -50,
	-150, -150, -150, -150, -150, -150, -150, -150, -150, -150,
	-250, -250, -250, -250, -250, -250, -250, -250, -250, -250,
	-350, -350, -350, -350, -350, -350, -350, -350, -350, -350,
	-450, -450, -450, -450, -450, -450, -450, -450, -450, -450
};

static const int16_t xRightBottom[CHANNELS_PER_BOARD] = {
	-450, -450, -450, -450, -450, -450, -450, -450, -450, -450,
	-350, -350, -350, -350, -350, -350, -350, -350, -350, -350,
	-250, -250, -250, -250, -250, -250, -250, -250, -250, -250,
	-150, -150, -150, -150, -150, -150, -150, -150, -150, -150,
	-50, -50, -50, -50, -50, -50, -50, -50, -50, -50
};

static const int16_t xLeftBottom[CHANNELS_PER_BOARD] = {
	50, 50, 50, 50, 50, 50, 50, 50, 50, 50,
	150, 150, 150, 150, 150, 150, 150, 150, 150, 150,
	250, 250, 250, 250, 250, 250, 250, 250, 250, 250,
	350, 350, 350, 350, 350, 350, 350, 350, 350, 350,
	450, 450, 450, 450, 450, 450, 450, 450, 450, 450
};

static const int16_t zTransducer[CHANNELS_PER_BOARD] = {
	-450, -350, -250, -150, -50, 50, 150, 250, 350, 450,
	-450, -350, -250, -150, -50, 50, 150, 250, 350, 450,
	-450, -350, -250, -150, -50, 50, 150, 250, 350, 450,
	-450, -350, -250, -150, -50, 50, 150, 250, 350, 450,
	-450, -350, -250, -150, -50, 50, 150, 250, 350, 450
};

static const int16_t *boardXTable(enum BoardIndex board)
{
	switch(board) {
		case BOARD_LEFT_TOP:
			return xLeftTop;
		case BOARD_RIGHT_TOP:
			return xRightTop;
		case BOARD_LEFT_BOTTOM:
			return xLeftBottom;
		case BOARD_RIGHT_BOTTOM:
			return xRightBottom;
		default:
			return xLeftTop;
	}
}

static int boardIsBottom(enum BoardIndex board)
{
	return board == BOARD_LEFT_BOTTOM || board == BOARD_RIGHT_BOTTOM;
}

static uint16_t normalizePhase(int32_t phase)
{
	phase %= (int32_t)HOLO_PHASE_MAX;
	if(phase < 0)
		phase += HOLO_PHASE_MAX;
	return (uint16_t)phase;
}

static uint16_t calculatePhase(double targetXmm, double targetYmm, double targetZmm,
	enum BoardIndex board, int channel, double halfHeight0p1mm)
{
	const double negWaveKDiv10 = -0.07327329;
	const double scaleConstant = 81.48733;
	const int16_t *xTable = boardXTable(board);
	double transducerY = boardIsBottom(board) ? -halfHeight0p1mm : halfHeight0p1mm;
	double dx = targetXmm * SCALE_0P1MM - xTable[channel];
	double dy = targetYmm * SCALE_0P1MM - transducerY;
	double dz = targetZmm * SCALE_0P1MM - zTransducer[channel];
	double distance0p1mm = sqrt(dx * dx + dy * dy + dz * dz);
	int32_t phase = (int32_t)(distance0p1mm * negWaveKDiv10 * scaleConstant);

	if(boardIsBottom(board))
		phase += HOLO_PHASE_MAX / 2;

	return normalizePhase(phase);
}

static void fillPhaseFrame(HoloPhaseFrame *frame, uint16_t frameID,
	double x, double y, double z, double boardDistanceMm)
{
	memset(frame, 0, sizeof(*frame));
	frame->magic = HOLO_PHASE_MAGIC;
	frame->version = HOLO_PHASE_VERSION;
	frame->frame_id = frameID;
	frame->phase_count = HOLO_PHASE_COUNT;
	frame->phase_max = HOLO_PHASE_MAX;

	double halfHeight0p1mm = boardDistanceMm * SCALE_0P1MM / 2.0;
	for(int board = 0; board < NUM_BOARDS; board++) {
		for(int ch = 0; ch < CHANNELS_PER_BOARD; ch++) {
			int idx = board * CHANNELS_PER_BOARD + ch;
			frame->phases[idx] = calculatePhase(x, y, z, (enum BoardIndex)board, ch, halfHeight0p1mm);
		}
	}

	frame->crc32 = holo_phase_frame_crc(frame);
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
		if(n <= 0)
			return -1;
		off += (size_t)n;
	}

	return 0;
}

static socket_t connectToPi(const char *host, uint16_t port)
{
	socket_t sock = socket(AF_INET, SOCK_STREAM, 0);
	if(sock == INVALID_SOCKET)
		return INVALID_SOCKET;

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
	printf("  z/a    decrease/increase Z\n");
	printf("  x/s    decrease/increase X\n");
	printf("  c/d    decrease/increase Y\n");
	printf("  p      print current position\n");
	printf("  q      quit\n");
}

int main(int argc, char **argv)
{
	if(argc < 2) {
		printf("usage: %s <pi-ip-or-host> [port] [board-distance-mm]\n", argv[0]);
		return 1;
	}

	const char *host = argv[1];
	uint16_t port = argc >= 3 ? (uint16_t)atoi(argv[2]) : HOLO_PHASE_TCP_PORT;
	double boardDistanceMm = argc >= 4 ? atof(argv[3]) : 135.0;

#ifdef _WIN32
	WSADATA wsa;
	if(WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
		printf("WSAStartup failed\n");
		return 1;
	}
#endif

	socket_t sock = connectToPi(host, port);
	if(sock == INVALID_SOCKET) {
		printf("could not connect to %s:%u\n", host, port);
		return 1;
	}

	double x = 0.0;
	double y = 0.0;
	double z = 0.0;
	uint16_t frameID = 0;
	HoloPhaseFrame frame;

	printHelp();
	while(1) {
		fillPhaseFrame(&frame, frameID++, x, y, z, boardDistanceMm);
		if(sendAll(sock, &frame, sizeof(frame)) < 0) {
			printf("send failed\n");
			break;
		}

		printf("sent frame %u at %.1f, %.1f, %.1f mm\n", frame.frame_id, x, y, z);
		int ch = getchar();
		if(ch == EOF || ch == 'q')
			break;

		switch(ch) {
			case 'h':
				x = 0.0;
				y = 0.0;
				z = 0.0;
				break;
			case 'z':
				z -= MOVE_INC_MM;
				break;
			case 'a':
				z += MOVE_INC_MM;
				break;
			case 'x':
				x -= MOVE_INC_MM;
				break;
			case 's':
				x += MOVE_INC_MM;
				break;
			case 'c':
				y -= MOVE_INC_MM;
				break;
			case 'd':
				y += MOVE_INC_MM;
				break;
			case 'p':
				printf("position %.1f, %.1f, %.1f mm\n", x, y, z);
				break;
			case '\n':
			case '\r':
				break;
			default:
				printHelp();
				break;
		}

		while(ch != '\n' && ch != '\r' && ch != EOF)
			ch = getchar();
	}

	close_socket(sock);
#ifdef _WIN32
	WSACleanup();
#endif
	return 0;
}
