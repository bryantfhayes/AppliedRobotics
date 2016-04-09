#include "EdisonSocket.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <netdb.h>
#include <sys/socket.h>
#include <arpa/inet.h>

EdisonSocket::EdisonSocket(){
    fprintf(stdout, "Creating Edison Socket Object\n");
    /* create a UDP socket */
    if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("cannot create socket\n");
    }

    /* bind the socket to any valid IP address and a specific port */
    memset((char *)&myaddr, 0, sizeof(myaddr));
    myaddr.sin_family = AF_INET;
    myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    myaddr.sin_port = htons(SERVICE_PORT);

    if (bind(fd, (struct sockaddr *)&myaddr, sizeof(myaddr)) < 0) {
        perror("bind failed");
    }
}

void EdisonSocket::readLine(void){
    int sel = select(fd, NULL, NULL, NULL, NULL);
    if(sel < 0)
        return;
    fprintf(stdout, "Waiting for data\n");
    recvlen = recvfrom(fd, recvBuffer, MAX_MSG_SIZE, 0, (struct sockaddr *)&remaddr, &addrlen);
    printf("received %d bytes\n", recvlen);
    if (recvlen > 0) {
        recvBuffer[recvlen] = 0;
        printf("received message: \"%s\"\n", recvBuffer);
    }
}

void EdisonSocket::writeLine(void){

}
