#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

#define DEVICE "/dev/spidev0.0"
#define BUF_SIZE 200

int main()
{
	int spi = open(DEVICE, O_RDWR);

	int sock = socket(AF_INET, SOCK_STREAM, 0);

	struct sockaddr_in addr;
	addr.sin_family = AF_INET;
	addr.sin_port = htons(12345);
	addr.sin_addr.s_addr = INADDR_ANY;

	bind(sock, (struct sockaddr*)&addr, sizeof(addr));
	listen(sock, 1);

	int client = accept(sock, NULL, NULL);

	uint8_t buffer[BUF_SIZE];

	while(1)
	{
		int len = recv(client, buffer, BUF_SIZE, 0);
		if(len > 0)
		{
			write(spi, buffer, len);
		}
	}

	return 0;
}