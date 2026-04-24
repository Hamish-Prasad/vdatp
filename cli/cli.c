#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <string.h>
#include <linux/spi/spidev.h>

#define TOTAL 200

#define MAX_PHASE 512
#define FREQ 40000.0f
#define SPEED 343000.0f

#define SPI_DEV "/dev/spidev0.0"

int spi_fd;

float tx_x[TOTAL], tx_y[TOTAL], tx_z[TOTAL];
uint16_t phase[TOTAL];
uint8_t enable[TOTAL];

float k;

// ================= SPI =================
int open_spi(const char *dev)
{
    int fd = open(dev, O_RDWR);
    if(fd < 0)
    {
        printf("Failed to open %s: %s\n", dev, strerror(errno));
        return -1;
    }

    uint8_t mode = 0;
    uint8_t bits = 8;
    uint32_t speed = 2000000;

    ioctl(fd, SPI_IOC_WR_MODE, &mode);
    ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);

    printf("Opened SPI: %s\n", dev);
    return fd;
}

void send_spi(int fd, uint8_t *data, int len)
{
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)data,
        .len = len,
        .speed_hz = 2000000,
        .bits_per_word = 8,
    };

    ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
}

// ================= GEOMETRY =================
void init()
{
    int idx = 0;

    for(int p=0;p<2;p++)
    for(int s=0;s<2;s++)
    for(int r=0;r<5;r++)
    for(int c=0;c<10;c++)
    {
        tx_x[idx] = (s==0 ? 50 : -50) + r*100;
        tx_y[idx] = (p==0 ? 42.5 : -42.5);
        tx_z[idx] = -450 + c*100;
        idx++;
    }

    k = (2*M_PI*FREQ/SPEED) * (MAX_PHASE/(2*M_PI));

    printf("Geometry initialized\n");
}

// ================= PHASE =================
void compute(float x, float y, float z)
{
    for(int i=0;i<TOTAL;i++)
    {
        float dx = x - tx_x[i];
        float dy = y - tx_y[i];
        float dz = z - tx_z[i];

        float r = sqrtf(dx*dx + dy*dy + dz*dz);

        int p = ((int)(-r * k)) % MAX_PHASE;
        if(p < 0) p += MAX_PHASE;

        // half-array phase flip (matches FPGA top/bottom logic)
        if(tx_y[i] < 0)
            p = (p + MAX_PHASE/2) % MAX_PHASE;

        phase[i] = p;

        enable[i] = (dx*dx + dz*dz < 40000);
    }
}

// ================= SEND SINGLE STREAM =================
void send()
{
    uint8_t buf[1 + TOTAL * 3];

    buf[0] = 0xA5;

    for(int i=0;i<TOTAL;i++)
    {
        buf[1 + i*3 + 0] = phase[i] & 0xFF;
        buf[1 + i*3 + 1] = phase[i] >> 8;
        buf[1 + i*3 + 2] = enable[i];
    }

    send_spi(spi_fd, buf, sizeof(buf));
}

// ================= MAIN =================
int main()
{
    printf("Starting SINGLE-STREAM FPGA controller...\n");

    spi_fd = open_spi(SPI_DEV);

    if(spi_fd < 0)
        return 1;

    init();

    float t = 0;

    while(1)
    {
        compute(0, 0, 10 * cosf(t));
        send();

        usleep(2000);
        t += 0.05;
    }

    return 0;
}