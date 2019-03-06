#include <WINSOCK2.H>
#include <WS2tcpip.h>
#include <stdio.h>
#pragma comment(lib,"ws2_32.lib")
#include "TinySocket.h"


void sk_startup(void)
{
	static int wsa_started;
	if (!wsa_started)
	{
		WSADATA wd;
		WSAStartup(0x0202, &wd);
		wsa_started = 1;
	}
}
void sk_cleanup(void)
{
	WSACleanup();

}

static void set_blocking(int fd, int blocking)
{
	u_long sock_flags;
	sock_flags = !blocking;
	int ret = ioctlsocket(fd, FIONBIO, (u_long*)&sock_flags);
}

CTinySocket::CTinySocket(int server, int port)
{
	sk_startup();
	sockfd = -1;
	sockfd = socket(AF_INET, SOCK_DGRAM, 0);

	if (server)
	{
		DWORD tv = 30000;
		static struct sockaddr_in servaddr = { 0 };
		if (sockfd != -1)
		{

			servaddr.sin_family = AF_INET;
			servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
			servaddr.sin_port = htons(port);
			bind(sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr));
			setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv));
			set_blocking(sockfd, 0);
		}

	}

	//	//广播且非阻塞
	//	// Enable broadcast
	//	//setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, (const char*)&one, sizeof(one));

}


CTinySocket::~CTinySocket()
{
	closesocket(sockfd);

}
int	CTinySocket::Sendto(const char* destip, const int destport, char* buf, int bufsize)
{
	struct sockaddr_in socketinfo;
	int ip_valid = 0;
	socketinfo.sin_family = AF_INET;
	socketinfo.sin_addr.s_addr = inet_addr(destip);
	socketinfo.sin_port = htons(destport);

	ip_valid = socketinfo.sin_addr.s_addr != INADDR_NONE;
	if (!ip_valid)
	{
		return 0;
	}
	socketinfo.sin_family = AF_INET;
	socketinfo.sin_port = htons(destport);

	int n = sendto(sockfd, buf, bufsize, 0, (struct sockaddr *) &socketinfo, sizeof(socketinfo));//向地址发送buf数据

	return n;
}

int CTinySocket::Recvfrom(char* buf, int bufsize, char hostip[], int &hostport)
{
	struct sockaddr_in addr = { 0 };
	int struct_len = sizeof(addr);

	int n = recvfrom(sockfd, buf, bufsize, 0, (struct sockaddr *)&addr, (socklen_t *)&struct_len);//n为接受的字符数组的长度

	//en = recvfrom(sockfd, recvline, 512, 0, (struct sockaddr *) &ca, (socklen_t *)&struct_len);  //接受数据-------正确写法

	if (n > 0)
	{
		char* hip = inet_ntoa(addr.sin_addr);
		strcpy(hostip, hip);
		hostport = htons(addr.sin_port);
	}

	return n;
}
