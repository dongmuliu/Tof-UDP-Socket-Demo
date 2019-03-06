#include<opencv2\opencv.hpp>
#include<thread>
#include<Windows.h>
#include<tchar.h>
#include<iostream>
#include <fstream>
#include<direct.h>
#include"TinySocket.h"
#include <io.h>

using namespace std;
using namespace cv;

//参数设置
#define   Img_width   (320)
#define   Img_height  (240)
#define	 MYPORT		(8978)
#define MAX_NUM_PIX	 82656	//328 * 252
#define LOW_AMPLITUDE 	32500
#define MAX_PHASE      30000.0
#define MAX_DIST_VALUE 	30000
#define maxdepth 10000
#define mindepth  1000
//相机内参
#define   FX  286.6034
#define   FY  287.3857	
#define   CX  176.2039
#define   CY  126.5788
//相机畸变
#define   K1  -0.12346
#define   K2  0.159423
string g_temprequire = { "getTemperature" };   //向相机发送温度指令
string g_strquery = { "getDistanceSorted" };   //向相机发送距离指令
string _ip = { "192.168.7.63" };              //相机的Ip地址
string save_dir = { "H:\\save\\" };                          //保存路径
int _port = 50660;                          //相机的端口号


int		drnuLut[50][252][328];
static	int		g_Exit = 0;
int				g_enableshtter = 1;
int             packetnum = 0;
int				g_TempReadEnable;
int		offsetPhase;
int		orgOffsetLSB=0;
int		gOffsetDRNU_Temp=0;
int		gTempCal_Temp=0;
int		offsetPhaseDefault=0;
int	 g_TempReadDelay = 0;
int	_status = 0;  //0 空闲状态，1= 数据更新  2 数据处理中
int _grayoffset = 0;
int k = 0;
int	_ch;
uint64	_saveimgid;
double	gStepCalMM = 0;
_TCHAR	g_myexepath[MAX_PATH];
char dir_time[50];			//用于按时间保存

ushort	realTempChip;
ushort	realTempBoard1;
ushort	realTempBoard2;
unsigned short	*	_depthdata = new unsigned short[Img_width*Img_height];
cv::Mat _matimg;
cv::Mat save_src(Img_height, Img_width, CV_16UC1, Scalar(0));
uint16_t depth[Img_height][Img_width];
CTinySocket		g_myRecvUdp(1, MYPORT); //connect to camera - need bind
CTinySocket		g_mySendUdp; //connect to server- not neet bind
//显示相机温度
void setrealtemperature(char *buf)
{
	int i;
	ushort temp = 0;
	for (i = 0; i < 4; i++){
		temp += (uchar)*(buf + i * 2) + (ushort)(*(buf + i * 2 + 1)) * 256;
	}
	realTempChip = temp / 4;
	realTempBoard1 = (uchar)*(buf + 8) + (ushort)(*(buf + 9)) * 256;
	realTempBoard2 = (uchar)*(buf + 10) + (ushort)(*(buf + 11)) * 256;

	printf("Temperature Read = %d, %d, %d\n", realTempChip, realTempBoard1, realTempBoard2);
}
int saveTemperature(char * buf)
{
	setrealtemperature(buf);
	return 0;
}

// 发送指令线程
void thread_Requestdcam()  
{

	double t1, t2;
	t1 = t2 = 0;
	while (!g_Exit)
	{
		t1 = (double)cv::getTickCount();

		double delays = (t1 - t2) / cv::getTickFrequency(); 
		int n = 0;
		if (1 == g_enableshtter || delays > 0.5)
		{
			
			  packetnum = 0;
			
			if (1 == g_TempReadEnable)
			{
				
				// read temperature from previous cam
				n = g_myRecvUdp.Sendto((char*)_ip.c_str(), _port, (char*)g_temprequire.c_str(), g_temprequire.size());
				g_TempReadEnable = 0;
			}
			else
			{
				n = g_myRecvUdp.Sendto((char*)_ip.c_str(), _port, (char*)g_strquery.c_str(), g_strquery.size());
				g_TempReadDelay++;
			}
			if (n <= 0)
			{
				printf("--------------------Send fail---------------------");
			}
			else
			{
				//LOGI("send request to " << nid + 1 << "***" << g_strquery);
			}
			g_enableshtter = 0;

			//读取深度五十次后，读取温度
			if (g_TempReadDelay > 50)
			{
				g_TempReadDelay = 0;
				g_TempReadEnable = 1;
			}
			Sleep(50);//50ms
			
			t2 = t1;
		}
		Sleep(1);
	}
}

//接收指令线程
void thread_RecvSocketDcam()
{
	const int  maxsize = 1024 * 60;
	char buf[maxsize];
	char ip[64];
	int nport = 0;
	int pkgsize = Img_width*Img_height *sizeof(unsigned short);//153600
	while (!g_Exit)
	{
		int n = g_myRecvUdp.Recvfrom(buf, maxsize, ip, nport);
		if (n > 0)
		{
			if (n == 15)
			{
				buf[n] = 0;
				if (strcmp(buf, "shutterclosed  ") == 0)
				{
					g_enableshtter = 1;
					//tempersendflag = 1;
				}
			}
			else if (n == 12) //temperature
			{

				g_enableshtter = 1;
				saveTemperature(buf);
		
			}
			else if (51202 == n && buf[0] <= 3 && buf[1] == 'x')
			{
				if (_status != 2)
				{
							
					packetnum++;
					//printf("--------------------packnum=%d\n", packetnum);
					memcpy((char*)_depthdata + 51200 * ((int)buf[0] - 1), buf + 2, 51200);
					if (buf[0] == 3)
						_status = 1;
				}
				
			}
			
		}

		Sleep(1);
	}

}

//保存数据
Mat saveprocess()
{
	
	for (int i = 0; i < Img_height; i++)
	{
		for (int j = 0; j < Img_width; j++)
		{
			//_depthdata保存了320*240数据
			
			save_src.at<ushort>(i, j) = _depthdata[j + i * Img_width];

		}
	}
	string img_Name = save_dir + to_string(k) + ".png";
	k++;
	imwrite(img_Name, save_src);
	return save_src.clone();
}
//显示图像
void showprocess(Mat src1)
{
	Mat showsrc(Img_height, Img_width, CV_8UC1, Scalar(0));
	for (int i = 0; i < 240; i++)
	{
		for (int j = 0; j < 320; j++)
		{
			if (src1.at<ushort>(i, j) < mindepth)
			{
				src1.at<ushort>(i, j) = mindepth;
			}
			if (src1.at<ushort>(i, j) > maxdepth)
			{
				src1.at<ushort>(i, j) = maxdepth;
			}
			showsrc.at<uchar>(i, j) = 255 - (src1.at<ushort>(i, j) -mindepth)*255 / (maxdepth-mindepth);
			//showsrc.at<uchar>(i, j) = src1.at<ushort>(i, j) * 25 / 3000;
			//cout << src2.at<uchar>(i, j) << endl;
		}

	}


	imshow("showsrc", showsrc);
	waitKey(1);
}
//八均值滤波
void imageAverageEightConnectivity(ushort *depthdata) {
	int pixelCounter;
	int nCols = 320;
	int nRowsPerHalf = 120;
	//int size = nCols * nRowsPerHalf * 2;
	int size = 320 * 240;
	ushort actualFrame[MAX_NUM_PIX];
	int i, j, index;
	int pixdata;
	memcpy(actualFrame, depthdata, size * sizeof(ushort));
	// up side
	// dowm side
	// left side and right side
	// normal part
	for (i = 1; i < 239; i++) {
		for (j = 1; j < 319; j++){
			index = i * 320 + j;
			pixelCounter = 0;
			pixdata = 0;
			if (actualFrame[index] < 30000) {
				pixelCounter++;
				pixdata += actualFrame[index];
			}
			if (actualFrame[index - 1]  < 30000) {   // left
				pixdata += actualFrame[index - 1];
				pixelCounter++;
			}
			if (actualFrame[index + 1]  < 30000) {   // right
				pixdata += actualFrame[index + 1];
				pixelCounter++;
			}
			if (actualFrame[index - 321]  < 30000) {   // left up
				pixdata += actualFrame[index - 321];
				pixelCounter++;
			}
			if (actualFrame[index - 320]  < 30000) {   // up
				pixdata += actualFrame[index - 320];
				pixelCounter++;
			}
			if (actualFrame[index - 319]  < 30000) {   // right up
				pixdata += actualFrame[index - 319];
				pixelCounter++;
			}
			if (actualFrame[index + 319]  < 30000) {   // left down
				pixdata += actualFrame[index - 321];
				pixelCounter++;
			}
			if (actualFrame[index + 320]  < 30000) {   // down
				pixdata += actualFrame[index - 320];
				pixelCounter++;
			}
			if (actualFrame[index + 321]  < 30000) {   // right down
				pixdata += actualFrame[index - 319];
				pixelCounter++;
			}

			if (pixelCounter < 6) {
				*(depthdata + index) = LOW_AMPLITUDE;
			}
			else {
				*(depthdata + index) = pixdata / pixelCounter;
			}
		}
	}
	//
	//end part
}
//温度校正
int calculationCorrectDRNU(ushort * img){
	int i, x, y, l;
	double dist, tempDiff = 0;
	int offset = gOffsetDRNU_Temp;
	//printf("gOffsetDRNU = %d\n", offset);  //w
	uint16_t maxDistanceMM = 150000000 / 12000;
	double stepLSB = gStepCalMM * MAX_PHASE / maxDistanceMM;
	//printf("stepLSB = %2.4f\n", stepLSB);  //w

	uint16_t *pMem = img;
	int width = 320;
	int height = 240;
	tempDiff = realTempChip - gTempCal_Temp;
	for (l = 0, y = 0; y< height; y++){
		for (x = 0; x < width; x++, l++){
			dist = pMem[l];

			if (dist < LOW_AMPLITUDE){	//correct if not saturated
				dist -= offset;

				if (dist<0)	//if phase jump
					dist += MAX_DIST_VALUE;

				i = (int)(round(dist / stepLSB));

				if (i<0) i = 0;
				else if (i >= 50) i = 49;

				dist = (double)pMem[l] - drnuLut[i][y][x];
				
				dist -= tempDiff * 3.12262;	// 0.312262 = 15.6131 / 50

				pMem[l] = (uint16_t)round(dist);

			}	//end if LOW_AMPLITUDE
		}	//end width
	}	//end height
	//printf(" pMem = %d, %d, %d\n", pMem[1300], pMem[1301], pMem[1302]);
	return 0;
}
//偏移补偿
void calculationAddOffset(ushort *img){
	int offset = 0;
	uint16_t maxDistanceCM = 0;
	int l = 0;
	//offsetPhase = MAX_PHASE / (gSpeedOfLightDiv2 / configGetModulationFrequency(deviceAddress) / 10) * value;	
	offset = offsetPhaseDefault;
	//printf("offset 1 = %d\n", offset);
	uint16_t val;
	uint16_t *pMem = img;
	int numPix = 320 * 240;

	for (l = 0; l<numPix; l++){
		val = pMem[l];
		if (val < 30000) { //if not low amplitude, not saturated and not  adc overflow
			pMem[l] = (val + offset) % MAX_DIST_VALUE;
		}
	}
}
void  calibrate(ushort *img)
{
	imageAverageEightConnectivity(img);
	offsetPhase = orgOffsetLSB;
	calculationCorrectDRNU(img);
	calculationAddOffset(img);
}
//畸变矫正
//输入： 待矫正的图片
//输出： 校正后的图片
Mat undistimg(Mat src)
{

	Mat img;

	//内参矩阵
	Mat cameraMatrix = Mat::eye(3, 3, CV_64F);		//3*3单位矩阵
	cameraMatrix.at<double>(0, 0) = FX;
	cameraMatrix.at<double>(0, 1) = 0;
	cameraMatrix.at<double>(0, 2) = CX;
	cameraMatrix.at<double>(1, 1) = FY;
	cameraMatrix.at<double>(1, 2) = CY;
	cameraMatrix.at<double>(2, 2) = 1;

	//畸变参数
	Mat distCoeffs = Mat::zeros(5, 1, CV_64F);		//5*1全0矩阵
	distCoeffs.at<double>(0, 0) = K1;
	distCoeffs.at<double>(1, 0) = K2;
	distCoeffs.at<double>(2, 0) = 0;
	distCoeffs.at<double>(3, 0) = 0;
	distCoeffs.at<double>(4, 0) = 0;

	Size imageSize = src.size();
	Mat map1, map2;
	//参数1：相机内参矩阵
	//参数2：畸变矩阵
	//参数3：可选输入，第一和第二相机坐标之间的旋转矩阵
	//参数4：校正后的3X3相机矩阵
	//参数5：无失真图像尺寸
	//参数6：map1数据类型，CV_32FC1或CV_16SC2
	//参数7、8：输出X/Y坐标重映射参数
	cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(), cameraMatrix, imageSize, CV_32FC1, map1, map2);	//计算畸变映射
	//参数1：畸变原始图像
	//参数2：输出图像
	//参数3、4：X\Y坐标重映射
	//参数5：图像的插值方式
	//参数6：边界填充方式
	cv::remap(src, img, map1, map2, INTER_LINEAR);																	//畸变矫正
	return img.clone();
}
int main()
{
	time_t t = time(0);
	strftime(dir_time, sizeof(dir_time), "%H%M%S", localtime(&t));
	//start socket
	sk_startup();
	memset(_depthdata, 0, Img_width*Img_height*sizeof(unsigned short));
	std::thread th_recv(thread_RecvSocketDcam);
	std::thread th_query(thread_Requestdcam);
	th_recv.detach();
	th_query.detach();
	while (true)
	{
		if (_status == 1)
		{
			_status = 2;
			if (packetnum >= 3)
			{
				calibrate(_depthdata);
				Mat src_1= saveprocess();
				Mat src_2 = undistimg(src_1);
				showprocess(src_2);
			}
			packetnum = 0;
			_status = 0;
		}
	}
	//exit thread
	g_Exit = 1;
	Sleep(1000);
	sk_cleanup();
	return 0;
}