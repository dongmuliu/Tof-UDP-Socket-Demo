class CTinySocket
{
public:
	CTinySocket(int server = 0, int port = 0);
	//CTinySocket();
	~CTinySocket();
	int	Sendto(const char* destip, const int destport, char* buf, int bufsize);
	int Recvfrom(char* buf, int bufsize, char hostip[], int &hostport);
private:
	int  sockfd;
	int  sk_master;
	int  sk_slave;
	int  my_port;
	char udp_ip[128];
};

extern "C" void sk_startup(void); //extern "C"��ʵ�ֵ���C��C++�Ļ�ϱ��
extern "C" void sk_cleanup(void);