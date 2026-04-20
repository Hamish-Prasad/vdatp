#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <termios.h>
#include <fcntl.h>

#define NUM_CHANNELS 50
#define MAX_PHASE_CNT 512.0f
#define FREQ 40000.0f
#define SPEED_OF_SOUND 343000.0f

#define CMD_SET_PHASES 11
#define INC 1.0f

// ---------- keyboard ----------
int kbhit(void)
{
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF) {
        ungetc(ch, stdin);
        return 1;
    }

    return 0;
}

int getch(void)
{
    struct termios oldattr, newattr;
    int ch;

    tcgetattr(STDIN_FILENO, &oldattr);
    newattr = oldattr;
    newattr.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newattr);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldattr);
    return ch;
}

// ---------- transducer positions ----------
float xTransducer[NUM_CHANNELS] = {
     45,45,45,45,45,45,45,45,45,45,
     35,35,35,35,35,35,35,35,35,35,
     25,25,25,25,25,25,25,25,25,25,
     15,15,15,15,15,15,15,15,15,15,
      5, 5, 5, 5, 5, 5, 5, 5, 5, 5
};

float zTransducer[NUM_CHANNELS] = {
    -45,-35,-25,-15,-5,5,15,25,35,45,
    -45,-35,-25,-15,-5,5,15,25,35,45,
    -45,-35,-25,-15,-5,5,15,25,35,45,
    -45,-35,-25,-15,-5,5,15,25,35,45,
    -45,-35,-25,-15,-5,5,15,25,35,45
};

// ---------- phase math ----------
float compute_phase(float x, float y, float z, float xt, float yt, float zt)
{
    float dx = x - xt;
    float dy = y - yt;
    float dz = z - zt;

    float r = sqrtf(dx*dx + dy*dy + dz*dz);

    float k = 2.0f * M_PI * FREQ / SPEED_OF_SOUND;
    float phase = -k * r;

    phase *= (MAX_PHASE_CNT / (2.0f * M_PI));

    int p = (int)phase % (int)MAX_PHASE_CNT;
    if (p < 0) p += (int)MAX_PHASE_CNT;

    return (float)p;
}

void compute_frame(float x, float y, float z, uint16_t *phases)
{
    for(int i = 0; i < NUM_CHANNELS; i++)
    {
        phases[i] = (uint16_t)compute_phase(
            x, y, z,
            xTransducer[i],
            0,
            zTransducer[i]
        );
    }
}

void printPos(float x, float y, float z)
{
    printf("Pos: %.1f %.1f %.1f\n", x, y, z);
}

// ---------- MAIN ----------
int main()
{
    int sock = socket(AF_INET, SOCK_STREAM, 0);

    struct sockaddr_in serv_addr;
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(12345);
    inet_pton(AF_INET, "192.168.1.100", &serv_addr.sin_addr);

    connect(sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr));

    uint16_t phases[NUM_CHANNELS];
    uint8_t buffer[1 + NUM_CHANNELS * 2];

    float x = 0, y = 0, z = 0;

    printf("Controls:\n");
    printf("a/z = Z axis\n");
    printf("s/x = X axis\n");
    printf("d/c = Y axis\n");

    while(1)
    {
        // -------- input --------
        if(kbhit())
        {
            int ch = getch();

            switch(ch)
            {
                case 'a': z += INC; break;
                case 'z': z -= INC; break;

                case 's': x += INC; break;
                case 'x': x -= INC; break;

                case 'd': y += INC; break;
                case 'c': y -= INC; break;

                case 'h':
                    x = y = z = 0;
                    break;
            }

            printPos(x, y, z);
        }

        // -------- compute --------
        compute_frame(x, y, z, phases);

        // -------- send --------
        buffer[0] = (CMD_SET_PHASES << 1);

        for(int i = 0; i < NUM_CHANNELS; i++) {
            buffer[1 + i*2] = phases[i] & 0xFF;
            buffer[1 + i*2 + 1] = phases[i] >> 8;
        }

        send(sock, buffer, sizeof(buffer), 0);

        usleep(10000); // ~100 Hz update
    }

    return 0;
}