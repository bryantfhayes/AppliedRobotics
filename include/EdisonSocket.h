#ifndef __EDISONSOCKET_H
#define __EDISONSOCKET_H
#include "EdisonComm.h"
#include <netdb.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#define SERVICE_PORT 21224

class EdisonSocket : public EdisonComm{
    public:
        EdisonSocket();
        ~EdisonSocket();
        void readLine(void);
        void writeLine(void);
    private:
        int _sockNum;
        fd_set readset;
        struct sockaddr_in myaddr;                  /* our address */
        struct sockaddr_in remaddr;                 /* remote address */
        socklen_t addrlen = sizeof(remaddr);        /* length of addresses */
        int recvlen;                                /* # bytes received */
        int fd;                                     /* our socket */
    
};
#endif
