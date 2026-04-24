// server.c
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <arpa/inet.h>

#define NUM_CHANNELS 50
#define MAX_PHASE 512
#define FREQ 40000.0f
#define SPEED_OF_SOUND 343000.0f

float tx_x[NUM_CHANNELS];
float tx_y[NUM_CHANNELS];
float tx_z[NUM_CHANNELS];

uint16_t phases[NUM_CHANNELS];

float phaseConst;

// =========================
// INIT ARRAY
// =========================
void initArray(float boardDistance)
{
    phaseConst = (FREQ * MAX_PHASE) / SPEED_OF_SOUND;

    int idx = 0;
    float half = boardDistance / 2.0f;

    for(int r=0;r<5;r++)
    {
        for(int c=0;c<10;c++)
        {
            tx_x[idx] = -45 + r*10;
            tx_z[idx] = -45 + c*10;
            tx_y[idx] = half;
            idx++;
        }
    }
}

// =========================
// PHASE COMPUTE
// =========================
void computePhases(float x, float y, float z)
{
    for(int i=0;i<NUM_CHANNELS;i++)
    {
        float dx = x - tx_x[i];
        float dy = y - tx_y[i];
        float dz = z - tx_z[i];

        float r = sqrtf(dx*dx + dy*dy + dz*dz);

        float ph = -r * phaseConst;

        int p = ((int)ph) % MAX_PHASE;
        if(p < 0) p += MAX_PHASE;

        phases[i] = p;
    }
}

// =========================
// DUMMY SEND (replace with SPI)
// =========================
void sendPhases()
{
    // TODO: connect your SPI here
}

// =========================
// COMMAND HANDLER
// =========================
void handleCommand(char *cmd)
{
    float x,y,z;

    if(sscanf(cmd, "move %f %f %f", &x,&y,&z)==3)
    {
        computePhases(x,y,z);
        sendPhases();
        printf("Moved to %.2f %.2f %.2f\n", x,y,z);
    }
    else if(strncmp(cmd,"circle",6)==0)
    {
        printf("Running circle...\n");

        float t=0;
        for(int i=0;i<200;i++)
        {
            float x = 10*cosf(t);
            float z = 10*sinf(t);

            computePhases(x,0,z);
            sendPhases();

            usleep(10000);
            t += 0.1f;
        }
    }
    else
    {
        printf("Unknown command\n");
    }
}

// =========================
// MAIN SERVER
// =========================
int main()
{
    int server_fd, client;
    struct sockaddr_in addr;

    initArray(85.0f);

    server_fd = socket(AF_INET, SOCK_STREAM, 0);

    addr.sin_family = AF_INET;
    addr.sin_port = htons(5677);
    addr.sin_addr.s_addr = INADDR_ANY;

    bind(server_fd, (struct sockaddr*)&addr, sizeof(addr));
    listen(server_fd, 1);

    printf("Server listening on port 5677...\n");

    while(1)
    {
        client = accept(server_fd, NULL, NULL);
        printf("Client connected\n");

        char buffer[256];

        while(1)
        {
            int len = read(client, buffer, sizeof(buffer)-1);
            if(len <= 0) break;

            buffer[len] = 0;

            handleCommand(buffer);
        }

        close(client);
        printf("Client disconnected\n");
    }
}