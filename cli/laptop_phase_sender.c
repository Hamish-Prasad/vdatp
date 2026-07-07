#include <stdio.h>    /* printf/getchar for the small laptop command interface. */
#include <stdint.h>   /* int16_t/uint16_t/int32_t give exact-width integer storage. */
#include <stdlib.h>   /* atoi/atof convert command-line strings into numbers. */
#include <string.h>   /* memset/memcpy are used for packet and socket address setup. */
#include <math.h>     /* sqrt is used for transducer-to-focus distance. */

#ifndef _WIN32
#include <unistd.h>
#endif

/*
 * Windows and Linux/macOS expose sockets through slightly different headers and
 * close functions. This block hides those differences behind socket_t and
 * close_socket so the rest of the file can stay platform-neutral.
 */
#ifdef _WIN32
#undef _WIN32_WINNT
#define _WIN32_WINNT 0x0600
#include <winsock2.h>                         /* Windows socket API. */
#include <ws2tcpip.h>                         /* inet_pton on Windows. */
#include <windows.h>                          /* Sleep and Windows platform declarations. */
#ifndef INET_PTON
WINSOCK_API_LINKAGE INT WSAAPI inet_pton(INT Family, PCSTR pszAddrString, PVOID pAddrBuf);
#endif
typedef SOCKET socket_t;                      /* SOCKET is the native Windows socket handle type. */
#define close_socket closesocket              /* Windows closes sockets with closesocket(). */
#else
#include <arpa/inet.h>                        /* inet_pton and sockaddr helpers on Unix-like systems. */
#include <netdb.h>                            /* gethostbyname fallback for hostnames. */
#include <netinet/in.h>                       /* sockaddr_in and htons. */
#include <sys/socket.h>                       /* socket/connect/send. */
#include <unistd.h>                           /* close for Unix file descriptors. */
typedef int socket_t;                         /* Unix sockets are plain integer file descriptors. */
#define INVALID_SOCKET (-1)                   /* Match the Windows-style invalid socket name. */
#define SOCKET_ERROR (-1)                     /* Match the Windows-style socket error name. */
#define close_socket close                    /* Unix closes sockets with close(). */
#endif

static void delay_ms(unsigned int ms)
{
#ifdef _WIN32
	Sleep(ms);
#else
	usleep(ms * 1000);
#endif
}

#include "phase_protocol.h"                   /* Shared 200-phase packet format and CRC helpers. */

#define NUM_BOARDS 4                          /* The physical levitator has four FPGA/transducer boards. */
#define CHANNELS_PER_BOARD 50                 /* Each FPGA drives 50 ultrasonic transducer channels. */
#define SCALE_0P1MM 10.0                      /* FPGA geometry is stored in 0.1 mm units, so 1 mm = 10. */
#define MOVE_INC_MM 1.0                       /* Keyboard moves adjust the target point by 1 mm. */

/*
 * Global phase-frame order.
 *
 * This order matches the FPGA direct-frame selection logic in Holo.sv:
 * 0: phases 0..49,   top=0 left=1
 * 1: phases 50..99,  top=0 left=0
 * 2: phases 100..149, top=1 left=1
 * 3: phases 150..199, top=1 left=0
 */
enum BoardIndex {
	BOARD_LEFT_TOP = 0,                       /* First 50 phases: left/top board. */
	BOARD_RIGHT_TOP = 1,                      /* Next 50 phases: right/top board. */
	BOARD_LEFT_BOTTOM = 2,                    /* Next 50 phases: left/bottom board. */
	BOARD_RIGHT_BOTTOM = 3                    /* Final 50 phases: right/bottom board. */
};

/*
 * Coordinate table note:
 *
 * int16_t means "signed integer, exactly 16 bits". The FPGA stores these
 * coordinates in a 13-bit signed-ish SystemVerilog signal, and the values are
 * small enough to fit safely in int16_t on the laptop side.
 *
 * All coordinate numbers below are copied from fpga/CalcPhase.sv and are in
 * 0.1 mm units. For example, 450 means 45.0 mm and -50 means -5.0 mm.
 */

static const int16_t xCols[5] = {450, 350, 250, 150, 50};
static const int16_t zRows[10] = {-450, -350, -250, -150, -50, 50, 150, 250, 350, 450};

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

static uint16_t normalizePhase(int32_t phase)
{
	phase %= (int32_t)HOLO_PHASE_MAX;
	if(phase < 0) phase += HOLO_PHASE_MAX;
	return (uint16_t)phase;
}

static uint16_t calculatePhase(double tx, double ty, double tz, enum BoardIndex board, int ch, double halfHeight)
{
	const double K = -0.07327329;    /* NEG_WAVE_K_DIV10 */
	const double S = 81.48733;       /* SCALE_CNST */

	double x = getTransducerX(board, ch);
	double y = boardIsBottom(board) ? -halfHeight : halfHeight;
	double z = getTransducerZ(ch);

	double dx = tx * SCALE_0P1MM - x;
	double dy = ty * SCALE_0P1MM - y;
	double dz = tz * SCALE_0P1MM - z;
	double dist = sqrt(dx*dx + dy*dy + dz*dz);

	int32_t phase = (int32_t)(dist * K * S);
	if(boardIsBottom(board)) phase += HOLO_PHASE_MAX / 2;

	return normalizePhase(phase);
}

static void fillPhaseFrame(HoloPhaseFrame *frame, uint16_t frameID,
	double x, double y, double z, double boardDistanceMm)
{
	memset(frame, 0, sizeof(*frame));              /* Start from a clean packet so padding/unused bytes are zero. */
	frame->magic = HOLO_PHASE_MAGIC;               /* Mark this packet as a HOLO phase packet. */
	frame->version = HOLO_PHASE_VERSION;           /* Tell the Pi which packet format this is. */
	frame->frame_id = frameID;                     /* Store the frame counter for logging/debugging. */
	frame->phase_count = HOLO_PHASE_COUNT;         /* Store the expected 200 phase count. */
	frame->phase_max = HOLO_PHASE_MAX;             /* Store the expected 512-tick phase period. */

	double halfHeight0p1mm = boardDistanceMm * SCALE_0P1MM / 2.0; /* Convert board separation to half-height in 0.1 mm units. */
	for(int board = 0; board < NUM_BOARDS; board++) {             /* Generate phases for all four boards. */
		for(int ch = 0; ch < CHANNELS_PER_BOARD; ch++) {          /* Generate phases for each board's 50 channels. */
			int idx = board * CHANNELS_PER_BOARD + ch;            /* Convert board/channel to global 0..199 phase index. */
			frame->phases[idx] = calculatePhase(x, y, z, (enum BoardIndex)board, ch, halfHeight0p1mm); /* Write phase. */
		}
	}

	frame->crc32 = holo_phase_frame_crc(frame);     /* Fill CRC last because it checks the earlier packet bytes. */
}

static int sendAll(socket_t sock, const void *buf, size_t len)
{
	const uint8_t *p = (const uint8_t *)buf;        /* Treat the packet as raw bytes for sending. */
	size_t off = 0;                                 /* Number of bytes already sent. */

	while(off < len) {                              /* Keep going until the whole packet is sent. */
#ifdef _WIN32
		int n = send(sock, (const char *)p + off, (int)(len - off), 0); /* Windows send expects char*. */
#else
		ssize_t n = send(sock, p + off, len - off, 0);                  /* Unix send accepts void-ish byte pointers. */
#endif
		if(n <= 0)                                  /* n <= 0 means error or closed connection. */
			return -1;                              /* Tell caller the send failed. */
		off += (size_t)n;                           /* Advance by the number of bytes actually sent. */
	}

	return 0;                                       /* Success: every byte was sent. */
}

/* Load a stiffness-optimised, static two-particle hologram (200 phase ticks). */
static int loadPhaseFile(const char *path, HoloPhaseFrame *frame, uint16_t frameID)
{
	FILE *file = fopen(path, "r");
	if(!file) return -1;
	memset(frame, 0, sizeof(*frame));
	frame->magic = HOLO_PHASE_MAGIC;
	frame->version = HOLO_PHASE_VERSION;
	frame->frame_id = frameID;
	frame->phase_count = HOLO_PHASE_COUNT;
	frame->phase_max = HOLO_PHASE_MAX;
	for(int i = 0; i < HOLO_PHASE_COUNT; i++) {
		unsigned value;
		if(fscanf(file, "%u", &value) != 1 || value >= HOLO_PHASE_MAX) {
			fclose(file);
			return -1;
		}
		frame->phases[i] = (uint16_t)value;
	}
	/* Reject trailing numeric values: the file must describe exactly one frame. */
	unsigned extra;
	if(fscanf(file, "%u", &extra) == 1) { fclose(file); return -1; }
	fclose(file);
	frame->crc32 = holo_phase_frame_crc(frame);
	return 0;
}

static socket_t connectToPi(const char *host, uint16_t port)
{
	socket_t sock = socket(AF_INET, SOCK_STREAM, 0); /* Create a TCP/IPv4 socket. */
	if(sock == INVALID_SOCKET)                       /* Check socket creation failure. */
		return INVALID_SOCKET;                       /* Return failure to caller. */

	struct sockaddr_in addr;                         /* IPv4 address/port structure used by connect(). */
	memset(&addr, 0, sizeof(addr));                  /* Clear the whole address structure. */
	addr.sin_family = AF_INET;                       /* Use IPv4. */
	addr.sin_port = htons(port);                     /* Convert port from CPU byte order to network byte order. */

	if(inet_pton(AF_INET, host, &addr.sin_addr) != 1) { /* First try parsing host as a numeric IPv4 address. */
		struct hostent *he = gethostbyname(host);       /* If that fails, try resolving it as a hostname. */
		if(!he) {                                       /* Hostname lookup failed. */
			close_socket(sock);                         /* Close the socket before returning. */
			return INVALID_SOCKET;                      /* Return failure to caller. */
		}
		memcpy(&addr.sin_addr, he->h_addr_list[0], he->h_length); /* Copy resolved IP into sockaddr_in. */
	}

	if(connect(sock, (struct sockaddr *)&addr, sizeof(addr)) == SOCKET_ERROR) { /* Connect to Pi bridge. */
		close_socket(sock);                            /* Close socket on connection failure. */
		return INVALID_SOCKET;                         /* Return failure to caller. */
	}

	return sock;                                      /* Return connected socket. */
}

static void printHelp(void)
{
	printf("commands:\n");                            /* Print command list heading. */
	printf("  h      home at 0,0,0 and send\n");       /* h resets target point. */
	printf("  z/a    decrease/increase Z\n");         /* z/a move through Z. */
	printf("  x/s    decrease/increase X\n");         /* x/s move through X. */
	printf("  c/d    decrease/increase Y\n");         /* c/d move through Y. */
	printf("  p      print current position\n");       /* p prints without moving. */
	printf("  2      in --staged mode, add particle 2 at the origin\n");
	printf("  o      circle command\n");			      /* circle command. */
	printf("  q      quit\n");                         /* q exits program. */
}

static void handleCircleCommand(socket_t sock, HoloPhaseFrame *frame, uint16_t *frameID, double z, double boardDistanceMm)
{
	const double radius = 2.0;
	const double PI = 3.141592653589793;
	for(int i = 0; i < 300; i++) {
		double theta = 2.0 * PI * i / 30.0;
		double x = radius * cos(theta);
		double y = radius * sin(theta);

		fillPhaseFrame(frame, (*frameID)++, x, y, z, boardDistanceMm);
		if(sendAll(sock, frame, sizeof(*frame)) < 0) {
			printf("circle send failed\n");
			break;
		}
		printf("sent circle frame %u at %.1f, %.1f, %.1f mm\n", frame->frame_id, x, y, z);
		delay_ms(25);
	}
}

int main(int argc, char **argv)
{
	if(argc < 2) {
		printf("usage: %s <pi-ip-or-host> [port] [board-distance-mm]\n", argv[0]);
		printf("       %s <pi-ip-or-host> --two <phase-file> [port]\n", argv[0]);
		printf("       %s <pi-ip-or-host> --staged <phase-file> <first-x> <first-y> <first-z> [port]\n", argv[0]);
		return 1;
	}

	const char *host = argv[1];
	int requestedTwoParticleMode = argc >= 3 && strcmp(argv[2], "--two") == 0;
	int stagedMode = argc >= 3 && strcmp(argv[2], "--staged") == 0;
	if(requestedTwoParticleMode && argc < 4) {
		printf("--two requires a 200-value phase file\n");
		return 1;
	}
	if(stagedMode && argc < 7) {
		printf("--staged requires a phase file and the first particle's staged x y z\n");
		return 1;
	}
	int twoParticleMode = requestedTwoParticleMode;
	const char *phaseFile = (twoParticleMode || stagedMode) ? argv[3] : NULL;
	double stagedX = stagedMode ? atof(argv[4]) : 0.0;
	double stagedY = stagedMode ? atof(argv[5]) : 0.0;
	double stagedZ = stagedMode ? atof(argv[6]) : 0.0;
	uint16_t port = stagedMode ? (argc >= 8 ? (uint16_t)atoi(argv[7]) : HOLO_PHASE_TCP_PORT)
	                : twoParticleMode ? (argc >= 5 ? (uint16_t)atoi(argv[4]) : HOLO_PHASE_TCP_PORT)
	                                : (argc >= 3 ? (uint16_t)atoi(argv[2]) : HOLO_PHASE_TCP_PORT);
	double boardDistanceMm = (twoParticleMode || stagedMode) ? 135.0 : (argc >= 4 ? atof(argv[3]) : 135.0);
	if(port == 0) {
		printf("invalid TCP port\n");
		return 1;
	}

#ifdef _WIN32
	WSADATA wsa;
	if(WSAStartup(MAKEWORD(2, 2), &wsa) != 0) return 1;
#endif

	socket_t sock = connectToPi(host, port);
	if(sock == INVALID_SOCKET) {
		printf("could not connect to %s:%u\n", host, port);
		return 1;
	}

	double x = 0.0, y = 0.0, z = 0.0;
	uint16_t frameID = 0;
	HoloPhaseFrame frame;
	if(twoParticleMode) {
		if(loadPhaseFile(phaseFile, &frame, frameID++) != 0) {
			printf("invalid two-particle phase file: %s\n", phaseFile);
			close_socket(sock);
			return 1;
		}
		if(sendAll(sock, &frame, sizeof(frame)) < 0) {
			printf("two-particle frame send failed\n");
			close_socket(sock);
			return 1;
		}
		printf("two-particle hologram active (frame %u); press Enter to resend or q to quit\n", frame.frame_id);
		for(int ch; (ch = getchar()) != EOF && ch != 'q'; ) {
			if(ch == '\n' || ch == '\r') {
				frame.frame_id = frameID++;
				frame.crc32 = holo_phase_frame_crc(&frame);
				if(sendAll(sock, &frame, sizeof(frame)) < 0) break;
				printf("resent frame %u\n", frame.frame_id);
			}
		}
		close_socket(sock);
#ifdef _WIN32
		WSACleanup();
#endif
		return 0;
	}

	if(stagedMode)
		printf("staged loading: move particle 1 to %.1f, %.1f, %.1f mm, then press 2 to add particle 2 at the origin\n",
		       stagedX, stagedY, stagedZ);
	printHelp();
	while(1) {
		fillPhaseFrame(&frame, frameID++, x, y, z, boardDistanceMm);
		if(sendAll(sock, &frame, sizeof(frame)) < 0) break;

		printf("sent frame %u at %.1f, %.1f, %.1f mm\n", frame.frame_id, x, y, z);
		int ch = getchar();
		if(ch == EOF || ch == 'q') break;

		switch(ch) {
			case 'h': x = 0.0; y = 0.0; z = 0.0; break;
			case 'z': z -= MOVE_INC_MM; break;
			case 'a': z += MOVE_INC_MM; break;
			case 'x': x -= MOVE_INC_MM; break;
			case 's': x += MOVE_INC_MM; break;
			case 'c': y -= MOVE_INC_MM; break;
			case 'd': y += MOVE_INC_MM; break;
			case 'p': printf("position %.1f, %.1f, %.1f mm\n", x, y, z); break;
			case '2':
				if(!stagedMode) { printHelp(); break; }
				if(fabs(x-stagedX) > 0.05 || fabs(y-stagedY) > 0.05 || fabs(z-stagedZ) > 0.05) {
					printf("move particle 1 to %.1f, %.1f, %.1f mm before adding particle 2\n",
					       stagedX, stagedY, stagedZ);
					break;
				}
				if(loadPhaseFile(phaseFile, &frame, frameID++) != 0) {
					printf("invalid staged two-particle phase file: %s\n", phaseFile);
					break;
				}
				if(sendAll(sock, &frame, sizeof(frame)) < 0) {
					printf("two-particle activation failed\n");
					ch = EOF;
					break;
				}
				printf("two-particle field active: particle 1 at %.1f, %.1f, %.1f; add particle 2 at 0,0,0; q quits\n",
				       stagedX, stagedY, stagedZ);
				while((ch = getchar()) != EOF && ch != 'q') { }
				break;
			case 'o': handleCircleCommand(sock, &frame, &frameID, z, boardDistanceMm); break;
			case '\n': case '\r': break;
			default: printHelp(); break;
		}
		if(ch == EOF || ch == 'q') break;

		while(ch != '\n' && ch != '\r' && ch != EOF) ch = getchar();
	}

	close_socket(sock);
#ifdef _WIN32
	WSACleanup();
#endif
	return 0;
}
