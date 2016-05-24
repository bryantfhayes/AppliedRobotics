/*
* @Author: Bryant Hayes
* @Date:   2016-05-06 23:17:40
* @Last Modified by:   Bryant Hayes
* @Last Modified time: 2016-05-23 13:36:49
*/

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
    
    /*
    if (inet_aton(server, &remaddr.sin_addr)==0) {
        fprintf(stderr, "inet_aton() failed\n");
        exit(1);
    }
    */

    fprintf(stdout, "Initializing udp socket...\n");

    gameover = false;
}

EdisonSocket::~EdisonSocket(){
    close(fd);
}

void EdisonSocket::readLine(void){
    while(!gameover){
        FD_ZERO(&readset);
        FD_SET(fd, &readset);
        int result = select(fd+1, &readset, NULL, NULL, NULL);
        if(result > 0){
            if(FD_ISSET(fd, &readset)){
                //fprintf(stdout, "Waiting for data\n");
                recvlen = recvfrom(fd, recvBuffer, MAX_MSG_SIZE, 0, (struct sockaddr *)&remaddr, &addrlen);
                inet_ntop(AF_INET, &((&remaddr)->sin_addr), clientIP, INET_ADDRSTRLEN);
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

void EdisonSocket::writeLine(const char* msg){
    sprintf(sendBuffer, msg);
    if (clientIP == "") {
        fprintf(stderr, "No target IP\n");
        return;
    }
    if (inet_aton(clientIP, &remaddr.sin_addr)==0) {
        fprintf(stderr, "inet_aton() failed\n");
        return;
    }
    (&remaddr)->sin_port = htons(SERVICE_PORT);
    if (sendto(fd, sendBuffer, strlen(sendBuffer), 0, (struct sockaddr *)&remaddr, sizeof(remaddr))==-1)
        perror("sendto");
    //printf("sent msg: %s : to ip: %s on port: %d.\n", sendBuffer, clientIP, (&remaddr)->sin_port);
}
