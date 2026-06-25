#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

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

static socket_t connect_to_pi(const char *host, uint16_t port)
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

static void fill_frame(HoloPhaseFrame *frame, uint16_t id, const uint16_t phases[NEW_TRANSDUCERS])
{
	memset(frame, 0, sizeof(*frame));
	frame->magic = HOLO_PHASE_MAGIC;
	frame->version = HOLO_PHASE_VERSION;
	frame->frame_id = id;
	frame->phase_count = HOLO_PHASE_COUNT;
	frame->phase_max = HOLO_PHASE_MAX;
	for(int i = 0; i < NEW_TRANSDUCERS; i++) frame->phases[i] = phases[i];
	frame->crc32 = holo_phase_frame_crc(frame);
}

static void print_help(void)
{
	printf("commands: h home, 1/2/3 select method, +/- trap spacing, x/s z/a c/d move trap set, p print, q quit\n");
}

int main(int argc, char **argv)
{
	if(argc < 2) {
		printf("usage: %s <pi-ip-or-host> [port] [board-distance-mm] [trap-count]\n", argv[0]);
		return 1;
	}
	const char *host = argv[1];
	uint16_t port = argc >= 3 ? (uint16_t)atoi(argv[2]) : HOLO_PHASE_TCP_PORT;
	NewSolverConfig cfg;
	new_solver_default_config(&cfg);
	if(argc >= 4) cfg.board_distance_mm = atof(argv[3]);
	cfg.cage_radius_mm = 1.6;
	int trap_count = argc >= 5 ? atoi(argv[4]) : 1;
	if(trap_count < 1) trap_count = 1;
	if(trap_count > 3) trap_count = 3;

#ifdef _WIN32
	WSADATA wsa;
	if(WSAStartup(MAKEWORD(2, 2), &wsa) != 0) return 1;
#endif

	socket_t sock = connect_to_pi(host, port);
	if(sock == INVALID_SOCKET) {
		printf("could not connect to %s:%u\n", host, port);
		return 1;
	}

	NewVec3 center = {0, 0, 0};
	double spacing = 12.0;
	uint16_t frame_id = 0;
	print_help();
	for(;;) {
		NewVec3 traps[NEW_MAX_TRAPS];
		for(int i = 0; i < trap_count; i++) {
			double o = ((double)i - 0.5 * (double)(trap_count - 1)) * spacing;
			traps[i] = center;
			traps[i].x += o;
		}
		uint16_t phases[NEW_TRANSDUCERS];
		new_solver_make_phases(&cfg, traps, trap_count, phases);
		HoloPhaseFrame frame;
		fill_frame(&frame, frame_id++, phases);
		if(send_all(sock, &frame, sizeof(frame)) < 0) break;
		printf("sent %s frame %u traps=%d center=(%.1f %.1f %.1f) spacing=%.1f score0=%.4g\n",
			new_solver_method_name(cfg.method), frame.frame_id, trap_count,
			center.x, center.y, center.z, spacing,
			new_solver_trap_score(&cfg, phases, traps[0], 0.8));
		int ch = getchar();
		if(ch == EOF || ch == 'q') break;
		switch(ch) {
			case 'h': center.x = center.y = center.z = 0.0; break;
			case '1': cfg.method = NEW_TRAP_REPLICATE_WGS; break;
			case '2': cfg.method = NEW_TRAP_SHADOW_NULLSPACE; cfg.cage_radius_mm = 2.6; break;
			case '3': cfg.method = NEW_TRAP_CURVATURE_BOOST; cfg.cage_radius_mm = 1.6; break;
			case '+': spacing += 1.0; break;
			case '-': if(spacing > 4.0) spacing -= 1.0; break;
			case 'x': center.x -= 1.0; break;
			case 's': center.x += 1.0; break;
			case 'c': center.y -= 1.0; break;
			case 'd': center.y += 1.0; break;
			case 'z': center.z -= 1.0; break;
			case 'a': center.z += 1.0; break;
			case 'p': print_help(); break;
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
