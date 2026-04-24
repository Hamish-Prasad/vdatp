#include <stdio.h>
#include <winsock2.h>

#pragma comment(lib,"ws2_32.lib")

int main()
{
    WSADATA wsa;
    SOCKET s;
    struct sockaddr_in server;

    WSAStartup(MAKEWORD(2,2), &wsa);

    s = socket(AF_INET, SOCK_STREAM, 0);

    server.sin_addr.s_addr = inet_addr("169.254.103.217"); // Pi IP CAN PROBS CHANGE??? idk my experience with ips is rough
    server.sin_family = AF_INET;
    server.sin_port = htons(1234); // port, idk if 1234 is safe but cool beans

    if(connect(s, (struct sockaddr *)&server, sizeof(server)) < 0)
    {
        printf("Connection failed\n");
        return 1;
    }

    printf("Connected to Pi\n");

    char cmd[256];

    while(1)
    {
        printf("> ");
        fgets(cmd, sizeof(cmd), stdin);
        send(s, cmd, strlen(cmd), 0);
    }

    closesocket(s);
    WSACleanup();
}