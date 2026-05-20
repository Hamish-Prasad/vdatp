#include <stdio.h>    /* printf/getchar for the small laptop command interface. */
#include <stdint.h>   /* int16_t/uint16_t/int32_t give exact-width integer storage. */
#include <stdlib.h>   /* atoi/atof convert command-line strings into numbers. */
#include <string.h>   /* memset/memcpy are used for packet and socket address setup. */
#include <math.h>     /* sqrt is used for transducer-to-focus distance. */

/*
 * Windows and Linux/macOS expose sockets through slightly different headers and
 * close functions. This block hides those differences behind socket_t and
 * close_socket so the rest of the file can stay platform-neutral.
 */
#ifdef _WIN32
#include <winsock2.h>                         /* Windows socket API. */
#include <ws2tcpip.h>                         /* inet_pton on Windows. */
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

static const int16_t xLeftTop[CHANNELS_PER_BOARD] = {
	450, 450, 450, 450, 450, 450, 450, 450, 450, 450,        /* x = 45 mm column. */
	350, 350, 350, 350, 350, 350, 350, 350, 350, 350,        /* x = 35 mm column. */
	250, 250, 250, 250, 250, 250, 250, 250, 250, 250,        /* x = 25 mm column. */
	150, 150, 150, 150, 150, 150, 150, 150, 150, 150,        /* x = 15 mm column. */
	50, 50, 50, 50, 50, 50, 50, 50, 50, 50                  /* x = 5 mm column. */
};

static const int16_t xRightTop[CHANNELS_PER_BOARD] = {
	-50, -50, -50, -50, -50, -50, -50, -50, -50, -50,        /* x = -5 mm column. */
	-150, -150, -150, -150, -150, -150, -150, -150, -150, -150, /* x = -15 mm column. */
	-250, -250, -250, -250, -250, -250, -250, -250, -250, -250, /* x = -25 mm column. */
	-350, -350, -350, -350, -350, -350, -350, -350, -350, -350, /* x = -35 mm column. */
	-450, -450, -450, -450, -450, -450, -450, -450, -450, -450  /* x = -45 mm column. */
};

static const int16_t xRightBottom[CHANNELS_PER_BOARD] = {
	-450, -450, -450, -450, -450, -450, -450, -450, -450, -450, /* x = -45 mm column. */
	-350, -350, -350, -350, -350, -350, -350, -350, -350, -350, /* x = -35 mm column. */
	-250, -250, -250, -250, -250, -250, -250, -250, -250, -250, /* x = -25 mm column. */
	-150, -150, -150, -150, -150, -150, -150, -150, -150, -150, /* x = -15 mm column. */
	-50, -50, -50, -50, -50, -50, -50, -50, -50, -50         /* x = -5 mm column. */
};

static const int16_t xLeftBottom[CHANNELS_PER_BOARD] = {
	50, 50, 50, 50, 50, 50, 50, 50, 50, 50,                  /* x = 5 mm column. */
	150, 150, 150, 150, 150, 150, 150, 150, 150, 150,        /* x = 15 mm column. */
	250, 250, 250, 250, 250, 250, 250, 250, 250, 250,        /* x = 25 mm column. */
	350, 350, 350, 350, 350, 350, 350, 350, 350, 350,        /* x = 35 mm column. */
	450, 450, 450, 450, 450, 450, 450, 450, 450, 450         /* x = 45 mm column. */
};

static const int16_t zTransducer[CHANNELS_PER_BOARD] = {
	-450, -350, -250, -150, -50, 50, 150, 250, 350, 450,     /* row 1 z positions. */
	-450, -350, -250, -150, -50, 50, 150, 250, 350, 450,     /* row 2 z positions. */
	-450, -350, -250, -150, -50, 50, 150, 250, 350, 450,     /* row 3 z positions. */
	-450, -350, -250, -150, -50, 50, 150, 250, 350, 450,     /* row 4 z positions. */
	-450, -350, -250, -150, -50, 50, 150, 250, 350, 450      /* row 5 z positions. */
};

static const int16_t *boardXTable(enum BoardIndex board)
{
	switch(board) {                               /* Select the copied CalcPhase x-coordinate table. */
		case BOARD_LEFT_TOP:                     /* top=0, left=1 board. */
			return xLeftTop;                     /* Return left/top x coordinates. */
		case BOARD_RIGHT_TOP:                    /* top=0, left=0 board. */
			return xRightTop;                    /* Return right/top x coordinates. */
		case BOARD_LEFT_BOTTOM:                  /* top=1, left=1 board. */
			return xLeftBottom;                  /* Return left/bottom x coordinates. */
		case BOARD_RIGHT_BOTTOM:                 /* top=1, left=0 board. */
			return xRightBottom;                 /* Return right/bottom x coordinates. */
		default:                                 /* Defensive fallback for impossible enum values. */
			return xLeftTop;                     /* Return a valid table so callers never get NULL. */
	}
}

static int boardIsBottom(enum BoardIndex board)
{
	return board == BOARD_LEFT_BOTTOM || board == BOARD_RIGHT_BOTTOM; /* Bottom boards get negative Y and phase inversion. */
}

static uint16_t normalizePhase(int32_t phase)
{
	phase %= (int32_t)HOLO_PHASE_MAX;            /* Wrap the signed phase into one 512-tick period. */
	if(phase < 0)                                /* C modulo can leave negative remainders. */
		phase += HOLO_PHASE_MAX;                 /* Shift negative values into the 0..511 range. */
	return (uint16_t)phase;                      /* Return an unsigned 16-bit value for the protocol frame. */
}

static uint16_t calculatePhase(double targetXmm, double targetYmm, double targetZmm,
	enum BoardIndex board, int channel, double halfHeight0p1mm)
{
	const double negWaveKDiv10 = -0.07327329;    /* Float value of CalcPhase.sv NEG_WAVE_K_DIV10. */
	const double scaleConstant = 81.48733;       /* Float value of CalcPhase.sv SCALE_CNST for 512 ticks. */
	const int16_t *xTable = boardXTable(board);  /* Pick the board-specific x-coordinate table. */
	double transducerY = boardIsBottom(board) ? -halfHeight0p1mm : halfHeight0p1mm; /* Match top/bottom yDiff logic. */
	double dx = targetXmm * SCALE_0P1MM - xTable[channel];       /* X distance in 0.1 mm units. */
	double dy = targetYmm * SCALE_0P1MM - transducerY;           /* Y distance in 0.1 mm units. */
	double dz = targetZmm * SCALE_0P1MM - zTransducer[channel];  /* Z distance in 0.1 mm units. */
	double distance0p1mm = sqrt(dx * dx + dy * dy + dz * dz);    /* Euclidean distance, same geometry as CalcPhase. */
	int32_t phase = (int32_t)(distance0p1mm * negWaveKDiv10 * scaleConstant); /* Convert distance into phase ticks. */

	if(boardIsBottom(board))                       /* CalcPhase offsets bottom boards by half a cycle. */
		phase += HOLO_PHASE_MAX / 2;               /* Half of 512 ticks is 256 ticks. */

	return normalizePhase(phase);                  /* Return the wrapped 0..511 phase. */
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
	printf("  q      quit\n");                         /* q exits program. */
}

int main(int argc, char **argv)
{
	if(argc < 2) {                                    /* Require at least the Pi host/IP argument. */
		printf("usage: %s <pi-ip-or-host> [port] [board-distance-mm]\n", argv[0]); /* Show usage. */
		return 1;                                    /* Return nonzero for incorrect command line. */
	}

	const char *host = argv[1];                       /* Pi IP address or hostname. */
	uint16_t port = argc >= 3 ? (uint16_t)atoi(argv[2]) : HOLO_PHASE_TCP_PORT; /* TCP port, default 5656. */
	double boardDistanceMm = argc >= 4 ? atof(argv[3]) : 135.0; /* Board separation in mm, default old CLI value. */

#ifdef _WIN32
	WSADATA wsa;                                      /* Windows socket library startup data. */
	if(WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {       /* Windows requires this before socket calls. */
		printf("WSAStartup failed\n");                /* Explain startup failure. */
		return 1;                                    /* Stop because sockets will not work. */
	}
#endif

	socket_t sock = connectToPi(host, port);          /* Connect to the Raspberry Pi phase bridge. */
	if(sock == INVALID_SOCKET) {                      /* Check connection failure. */
		printf("could not connect to %s:%u\n", host, port); /* Tell user what target failed. */
		return 1;                                    /* Stop because there is nowhere to send frames. */
	}

	double x = 0.0;                                   /* Current target X position in millimeters. */
	double y = 0.0;                                   /* Current target Y position in millimeters. */
	double z = 0.0;                                   /* Current target Z position in millimeters. */
	uint16_t frameID = 0;                             /* 16-bit frame counter sent in packet header. */
	HoloPhaseFrame frame;                             /* Local packet buffer for one 200-phase frame. */

	printHelp();                                      /* Show controls once at startup. */
	while(1) {                                        /* Main interactive send loop. */
		fillPhaseFrame(&frame, frameID++, x, y, z, boardDistanceMm); /* Calculate and pack all 200 phases. */
		if(sendAll(sock, &frame, sizeof(frame)) < 0) { /* Send the complete packet to the Pi. */
			printf("send failed\n");                  /* Tell user the TCP send failed. */
			break;                                    /* Leave the loop and shut down. */
		}

		printf("sent frame %u at %.1f, %.1f, %.1f mm\n", frame.frame_id, x, y, z); /* Log sent frame. */
		int ch = getchar();                          /* Read the next keyboard command. */
		if(ch == EOF || ch == 'q')                   /* EOF or q means quit. */
			break;                                   /* Leave the loop. */

		switch(ch) {                                  /* Update target position based on key. */
			case 'h':                                 /* Home command. */
				x = 0.0;                              /* Reset X. */
				y = 0.0;                              /* Reset Y. */
				z = 0.0;                              /* Reset Z. */
				break;                                /* Finish command. */
			case 'z':                                 /* Move negative Z. */
				z -= MOVE_INC_MM;                     /* Decrease Z by 1 mm. */
				break;                                /* Finish command. */
			case 'a':                                 /* Move positive Z. */
				z += MOVE_INC_MM;                     /* Increase Z by 1 mm. */
				break;                                /* Finish command. */
			case 'x':                                 /* Move negative X. */
				x -= MOVE_INC_MM;                     /* Decrease X by 1 mm. */
				break;                                /* Finish command. */
			case 's':                                 /* Move positive X. */
				x += MOVE_INC_MM;                     /* Increase X by 1 mm. */
				break;                                /* Finish command. */
			case 'c':                                 /* Move negative Y. */
				y -= MOVE_INC_MM;                     /* Decrease Y by 1 mm. */
				break;                                /* Finish command. */
			case 'd':                                 /* Move positive Y. */
				y += MOVE_INC_MM;                     /* Increase Y by 1 mm. */
				break;                                /* Finish command. */
			case 'p':                                 /* Print command. */
				printf("position %.1f, %.1f, %.1f mm\n", x, y, z); /* Show current position. */
				break;                                /* Finish command. */
			case '\n':                                /* Ignore blank line on Unix terminals. */
			case '\r':                                /* Ignore blank line on Windows-style terminals. */
				break;                                /* Finish command. */
			default:                                  /* Unknown key. */
				printHelp();                          /* Remind user of available commands. */
				break;                                /* Finish command. */
		}

		while(ch != '\n' && ch != '\r' && ch != EOF)  /* Discard the rest of the typed line. */
			ch = getchar();                           /* Read until line ending. */
	}

	close_socket(sock);                               /* Close the TCP connection. */
#ifdef _WIN32
	WSACleanup();                                     /* Shut down Windows socket library. */
#endif
	return 0;                                         /* Normal program exit. */
}
