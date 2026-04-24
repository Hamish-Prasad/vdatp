#include <stdio.h>
#include <string.h>
#include <winsock2.h>

#pragma comment(lib,"ws2_32.lib")

#define PI_IP "169.254.65.21"
#define PI_PORT 1234

void print_help()
{
    printf("\nCommands:\n");
    printf("  move x y z     -> move focus point\n");
    printf("  circle         -> run circle test\n");
    printf("  ping           -> test connection\n");
    printf("  help           -> show this help\n");
    printf("  exit           -> quit client\n\n");
}

int main()
{
    WSADATA wsa;
    SOCKET s;
    struct sockaddr_in server;

    char cmd[256];
    char recvbuf[256];
    int recvlen;

    WSAStartup(MAKEWORD(2,2), &wsa);

    s = socket(AF_INET, SOCK_STREAM, 0);

    server.sin_addr.s_addr = inet_addr(PI_IP);
    server.sin_family = AF_INET;
    server.sin_port = htons(PI_PORT);

    if (connect(s, (struct sockaddr*)&server, sizeof(server)) < 0)
    {
        printf("Connection failed\n");
        return 1;
    }

    printf("Connected to Pi at %s:%d\n", PI_IP, PI_PORT);
    print_help();

    while (1)
    {
        printf("> ");
        fgets(cmd, sizeof(cmd), stdin);

        // remove newline
        cmd[strcspn(cmd, "\n")] = 0;

        if (strlen(cmd) == 0)
            continue;

        // EXIT
        if (strcmp(cmd, "exit") == 0)
        {
            printf("Closing connection...\n");
            break;
        }

        // HELP
        if (strcmp(cmd, "help") == 0)
        {
            print_help();
            continue;
        }

        // PING
        if (strcmp(cmd, "ping") == 0)
        {
            send(s, "ping\n", 5, 0);

            recvlen = recv(s, recvbuf, sizeof(recvbuf)-1, 0);
            if (recvlen > 0)
            {
                recvbuf[recvlen] = 0;
                printf("Pi: %s\n", recvbuf);
            }
            continue;
        }

        // SEND NORMAL COMMAND
        strcat(cmd, "\n");
        send(s, cmd, strlen(cmd), 0);

        // Try read response (non-blocking style simple)
        recvlen = recv(s, recvbuf, sizeof(recvbuf)-1, 0);
        if (recvlen > 0)
        {
            recvbuf[recvlen] = 0;
            printf("Pi: %s\n", recvbuf);
        }
    }

    closesocket(s);
    WSACleanup();

    return 0;
}