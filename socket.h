//#ifndef SOCKET_H
//#define SOCKET_H


//#include <sys/types.h>
//#include <netinet/in.h>
//#include <arpa/inet.h>
//#include <stdlib.h>
//#include <stdio.h>
//#include <errno.h>
//#include <string.h>
//#include <unistd.h>



//void socketinit();
//void senddata(char *sendbuf,int len);
//int revdata(char *revbuf,int len);



#ifndef SOCKET_H
#define SOCKET_H

#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>



void socketinit();
void senddata(char *sendbuf,int len);
int revdata(char *revbuf,int len);

int revanalysis(char *revbuf);
//void sendanalysis(char *sendbuf,int x,int y,int num);

//void sendanalysis(char *sendbuf,int wFlag, int xFlag, int yFlag,int x,int y,int num);
void sendanalysis(char *sendbuf,int wFlag, int xFlag, int yFlag,int x,int y);

#endif // SOCKET_H

























//#endif // SOCKET_H



