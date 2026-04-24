// FULL physics restored: 200 transducers, masking, inversion

#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

#define NUM_CHANNELS 200
#define BOARD_HALF 42.5f

#define MAX_PHASE 512
#define FREQ 40000.0f
#define SOUND_SPEED 343000.0f

#define SPI_DEV "/dev/spidev0.0"
#define SPI_SPEED 1000000

int spiFD;

float tx_x[NUM_CHANNELS];
float tx_y[NUM_CHANNELS];
float tx_z[NUM_CHANNELS];

uint16_t phase[NUM_CHANNELS];
uint8_t enable[NUM_CHANNELS];

float kConst;

// ================= SPI =================
void spi_init()
{
    uint8_t mode=0,bits=8;
    spiFD = open(SPI_DEV, O_RDWR);

    ioctl(spiFD, SPI_IOC_WR_MODE, &mode);
    ioctl(spiFD, SPI_IOC_WR_BITS_PER_WORD, &bits);
    ioctl(spiFD, SPI_IOC_WR_MAX_SPEED_HZ, &(uint32_t){SPI_SPEED});
}

void spi_send(uint8_t *data, int len)
{
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)data,
        .len = len,
        .speed_hz = SPI_SPEED,
        .bits_per_word = 8,
    };
    ioctl(spiFD, SPI_IOC_MESSAGE(1), &tr);
}

// ================= GEOMETRY =================
void init_array()
{
    int idx=0;

    for(int plate=0;plate<2;plate++) {
        float y = (plate==0) ? BOARD_HALF : -BOARD_HALF;

        for(int side=0;side<2;side++) {
            for(int r=0;r<5;r++) {
                for(int c=0;c<10;c++) {

                    float x = (side==0) ? (50 + r*100) : (-50 - r*100);
                    float z = -450 + c*100;

                    tx_x[idx]=x;
                    tx_y[idx]=y;
                    tx_z[idx]=z;
                    idx++;
                }
            }
        }
    }

    kConst = (2*M_PI*FREQ / SOUND_SPEED) * (MAX_PHASE/(2*M_PI));
}

// ================= PHASE =================
void compute(float x, float y, float z)
{
    for(int i=0;i<NUM_CHANNELS;i++)
    {
        float dx=x-tx_x[i];
        float dy=y-tx_y[i];
        float dz=z-tx_z[i];

        float r=sqrtf(dx*dx+dy*dy+dz*dz);

        float ph = -r*kConst;

        int p = ((int)ph) % MAX_PHASE;
        if(p<0) p+=MAX_PHASE;

        // TOP/BOTTOM inversion
        if(tx_y[i] < 0)
            p = (p + MAX_PHASE/2) % MAX_PHASE;

        phase[i]=p;

        // radius mask
        enable[i] = (dx*dx + dz*dz < 40000) ? 1 : 0;
    }
}

// ================= SEND =================
void send_frame()
{
    uint8_t buf[1 + NUM_CHANNELS*3];
    buf[0]=0xA5;

    for(int i=0;i<NUM_CHANNELS;i++)
    {
        buf[1+i*3+0]=phase[i]&0xFF;
        buf[1+i*3+1]=phase[i]>>8;
        buf[1+i*3+2]=enable[i];
    }

    spi_send(buf,sizeof(buf));
}

// ================= MAIN =================
int main()
{
    spi_init();
    init_array();

    float t=0;

    while(1)
    {
        float x = 0;
        float y = 0;
        float z = 10*cosf(t);

        compute(x,y,z);
        send_frame();

        usleep(2000);
        t+=0.05;
    }
}