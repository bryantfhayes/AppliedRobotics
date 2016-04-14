#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <netdb.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include "EdisonSocket.h"
#include "mraa.hpp"


EdisonSocket::EdisonSocket(){
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

    /* now define remaddr, the address to whom we want to send messages */
    /* For convenience, the host address is expressed as a numeric IP address */
    /* that we will convert to a binary format via inet_aton */
    memset((char *) &remaddr, 0, sizeof(remaddr));
    remaddr.sin_family = AF_INET;
    remaddr.sin_port = htons(SERVICE_PORT);
    if (inet_aton(server, &remaddr.sin_addr)==0) {
        fprintf(stderr, "inet_aton() failed\n");
        exit(1);
    }

    fprintf(stdout, "SOCKET:     RUNNING\n");
}

EdisonSocket::~EdisonSocket(){
    close(fd);
}

void EdisonSocket::readLine(void){
    while(running){
        FD_ZERO(&readset);
        FD_SET(fd, &readset);
        int result = select(fd+1, &readset, NULL, NULL, NULL);
        if(result > 0){
            if(FD_ISSET(fd, &readset)){
                //fprintf(stdout, "Waiting for data\n");
                recvlen = recvfrom(fd, recvBuffer, MAX_MSG_SIZE, 0, (struct sockaddr *)&remaddr, &addrlen);
                //printf("received %d bytes\n", recvlen);
                if (recvlen > 0) {
                    recvBuffer[recvlen] = 0;
                    //printf("received message: \"%s\"\n", recvBuffer);
                    break;
                }else if (recvlen == 0){
                    close(fd);
                    perror("Socket closed");
                }
            }
        }
    }
}

void EdisonSocket::writeLine(char* msg, int max_msg_size){
    sprintf(sendBuffer, msg);
    if (sendto(fd, sendBuffer, strlen(sendBuffer), 0, (struct sockaddr *)&remaddr, sizeof(remaddr))==-1)
        perror("sendto");
}
