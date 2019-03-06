#include<Windows.h>
#include<opencv2\opencv.hpp>
#include<thread>
using namespace std;
using namespace cv;
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
void thread_RecvSocketDcam()
{

}
void thread_Requestdcam()
{

}
int main()
{
	//start sockets
	sk_startup();


	std::thread th_recv(thread_RecvSocketDcam); 
	std::thread th_query(thread_Requestdcam);
	
	th_recv.detach();
	th_query.detach();



	sk_cleanup();
	return 0;
}