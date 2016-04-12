//
// TEST FILE: FOR ALL IDEAS AND METHODS TO PLAY AROUND WITH.
//

#include <unistd.h>
#include <signal.h>
#include <iostream>
#include <sstream>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <netdb.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include "mraa.hpp"

#define SERVICE_PORT  21234
#define BUFSIZE       2048
#define SERIAL_MODE   0
#define WIRELESS_MODE 1

class USocket{
	private:
		int _sockNum;
		struct sockaddr_in myaddr;					/* our address */
		struct sockaddr_in remaddr;					/* remote address */
		socklen_t addrlen = sizeof(remaddr);		/* length of addresses */
		int recvlen;								/* # bytes received */
		int fd;										/* our socket */
	public:
		USocket();
		void read();
		unsigned char buf[BUFSIZE];					/* receive buffer */

};

USocket::USocket(){
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

void USocket::read(){
	recvlen = recvfrom(fd, buf, BUFSIZE, 0, (struct sockaddr *)&remaddr, &addrlen);
	printf("received %d bytes\n", recvlen);
	if (recvlen > 0) {
		buf[recvlen] = 0;
		printf("received message: \"%s\"\n", buf);
	}
}

int running = 1;

void sig_handler(int signo) {
    if (signo == SIGINT) {
        printf("Shutting down cleanly...\n");
        running = 0;
    }
}

int setupEnvironment(int argc, char* argv[], int* mode){
    // Example for reference: http://www.gnu.org/software/libc/manual/html_node/Example-of-Getopt.html#Example-of-Getopt
    int index;
    int c;

    opterr = 0;
    while ((c = getopt (argc, argv, "w")) != -1){
        switch (c){
          case 'w':
            *mode = WIRELESS_MODE;
            break;
          case '?':
            if (isprint (optopt))
              fprintf (stderr, "Unknown option `-%c'.\n", optopt);
            else
              fprintf (stderr, "Unknown option character `\\x%x'.\n", optopt);
            return 1;
          default:
            abort ();
        }
    }

    printf ("mode = %s\n", *mode ? "Wireless" : "Serial");
    return 0;
}

void setupInterrupts(){
    // Catch CTRL-C interrupt
    signal(SIGINT, sig_handler);
    fprintf(stdout, "INTERRUPTS: RUNNING\n");
}

void getCommand(USocket* com){
    fprintf(stdout, "getCommand()\n");
    com->read();
    printf("From getCommand msg was: %s\n", com->buf);
}

void setupWireless(USocket** sock){
    fprintf(stdout, "setupWireless()\n");
    *sock = new USocket();
}

int main(int argc, char* argv[]){
	// Variables
    int mode = SERIAL_MODE; //DEFAULT MODE
 	USocket* com;

	setupInterrupts();
    setupEnvironment(argc, argv, &mode);
	if(mode == WIRELESS_MODE){
        setupWireless(&com);
    }

	while(running){
 		getCommand(com);
 		usleep(1000000);
	}
	
	delete com;
}
