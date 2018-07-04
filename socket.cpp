

//#include "socket.h"





//struct sockaddr_in servaddr;
//struct sockaddr_in clientaddr;
//int sock;
//struct timeval timeOut;
//void socketinit()
//{
//    if((sock = socket(PF_INET, SOCK_DGRAM, 0)) < 0){
//        perror("socket");
//        exit(EXIT_FAILURE);
//    }

//    timeOut.tv_sec = 0;                 //设置5s超时
//    timeOut.tv_usec = 500;
//    clientaddr.sin_family = AF_INET;
//    clientaddr.sin_addr.s_addr = INADDR_ANY;
//    clientaddr.sin_port = htons(9999);
//    if (bind(sock, (struct sockaddr *) &clientaddr,sizeof(clientaddr)) < 0)
//                   ;
//    if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeOut, sizeof(timeOut)) < 0)
//    {
//            printf("time out setting failed\n");
//    }
//    memset(&servaddr, 0, sizeof(servaddr));
//    servaddr.sin_family = AF_INET;
//    servaddr.sin_port = htons(8080);
//    servaddr.sin_addr.s_addr = inet_addr("172.16.1.172");
//}

//void senddata(char *sendbuf,int len)
//{
//    sendto(sock, sendbuf, len, 0, (struct sockaddr *)&servaddr, sizeof(servaddr));
//}


//int revdata(char *recvbuf,int len)
//{
//    return recvfrom(sock, recvbuf, len, 0, NULL, NULL);
//}


#include "socket.h"



struct sockaddr_in servaddr;
struct sockaddr_in clientaddr;
int sock;
struct timeval timeOut;
void socketinit()
{
    if((sock = socket(PF_INET, SOCK_DGRAM, 0)) < 0){
        perror("socket");
        exit(EXIT_FAILURE);
    }

    timeOut.tv_sec = 0;                 //设置5s超时
    timeOut.tv_usec = 500;
    clientaddr.sin_family = AF_INET;
    clientaddr.sin_addr.s_addr = INADDR_ANY;
    clientaddr.sin_port = htons(9999);
    if (bind(sock, (struct sockaddr *) &clientaddr,sizeof(clientaddr)) < 0)
                   ;
    if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeOut, sizeof(timeOut)) < 0)
    {
            printf("time out setting failed\n");
    }
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(30000);
    servaddr.sin_addr.s_addr = inet_addr("21.192.104.200");
}

void senddata(char *sendbuf,int len)
{
    sendto(sock, sendbuf, len, 0, (struct sockaddr *)&servaddr, sizeof(servaddr));
}


int revdata(char *recvbuf,int len)
{
    return recvfrom(sock, recvbuf, len, 0, NULL, NULL);
}


//void sendanalysis(char *sendbuf,int x,int y,int num)
//{
//    int i;
//    char check=0;
//    sendbuf[0]=0xfe;

//    sendbuf[1]=((x&0x0000ff00)>>8);
//    sendbuf[2]=(x&0xff);

//    sendbuf[3]=((y&0x0000ff00)>>8);
//    sendbuf[4]=(y&0xff);

//    sendbuf[5]=((num&0x0000ff00)>>8);
//    sendbuf[6]=(num&0xff);

//    for(i=0;i<7;i++)
//        check+=sendbuf[i];

//    sendbuf[7]=check;
//}


//void sendanalysis(char *sendbuf,int wFlag, int xFlag, int yFlag,int x,int y,int num)
//{
//    int i;
//    char check=0;
//    sendbuf[0]=0xfe;
//    sendbuf[1]=(wFlag&0xff);
//    sendbuf[2]=(xFlag&0xff);
//    sendbuf[3]=(yFlag&0xff);
//    sendbuf[4]=((x&0x0000ff00)>>8);
//    sendbuf[5]=(x&0xff);

//    sendbuf[6]=((y&0x0000ff00)>>8);
//    sendbuf[7]=(y&0xff);

//    sendbuf[8]=((num&0x0000ff00)>>8);
//    sendbuf[9]=(num&0xff);

//    for(i=0;i<10;i++)
//        check+=sendbuf[i];

//    sendbuf[10]=check;
//}

void sendanalysis(char *sendbuf,int wFlag, int xFlag, int yFlag,int x,int y)
{
    int i;
    char check=0;
    sendbuf[0]=0xfe;
    sendbuf[1]=(wFlag&0xff);
    sendbuf[2]=(xFlag&0xff);
    sendbuf[3]=(yFlag&0xff);
    sendbuf[4]=((x&0x0000ff00)>>8);
    sendbuf[5]=(x&0xff);

    sendbuf[6]=((y&0x0000ff00)>>8);
    sendbuf[7]=(y&0xff);

//    sendbuf[8]=((num&0x0000ff00)>>8);
//    sendbuf[9]=(num&0xff);

    for(i=0;i<8;i++)
        check+=sendbuf[i];

    sendbuf[8]=check;
}

int revanalysis(char *revbuf)
{
    int i;
    char check=0;

    for(i=0;i<3;i++)
        check+=revbuf[i];

    if(revbuf[3]!=check)
        return 0;

    return revbuf[2];
}
