#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>


main()
{
  int sock;
  int n;
  struct sockaddr_in server;

  sock = socket(AF_INET, SOCK_STREAM, 0);
  server.sin_family = AF_INET;
  server.sin_port = htons(10001);
  server.sin_addr.s_addr = inet_addr("192.168.0.106");
  if(connect(sock, (struct sockaddr *)&server, sizeof(server))<0){
    printf("Connection Error\n");
  }
  char Send_Buf[]="AGO:A10000000B10000\r\n";
  char rcvbuf[256];
  memset(rcvbuf, 0, sizeof(rcvbuf));
  write(sock, Send_Buf, strlen(Send_Buf));
  close(sock);										// Close device
}
