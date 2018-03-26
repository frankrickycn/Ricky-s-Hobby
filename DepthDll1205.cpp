// DepthDll1205.cpp : Defines the exported functions for the DLL application.
//

#include "stdafx.h"
#include "ipp.h"
#include "DeclareExternVars.h"
#include "DeclareFuncs.h"
#include <math.h>

//////预开辟空间大小
int DataLen;
//const int a = 10;
//int sili[a] = { 0 };
//////软件输入参数的一一对应，在DataExecute中赋值
#pragma region 软件参数赋值
INT16 *channel1;
INT16 *channel2;
INT16 *channel3;
INT16 *channel4;
INT16 *channelEnv1;
INT16 *channelEnv2;
INT16 *channelEnv3;
INT16 *channelEnv4;

int Range;
int Len;
int fs;
int PowerL;
int PowerH;
int GainL;
int GainH;
int PwGnL;
int PwGnH;
int WidthL;
int WidthH;
int CountAll;
double ThredH;
int DepthpointL;
int DepthpointH;
int WidthLowPoints;
int WidthHighPoints;
int MaxVL;
int MaxVH;
int FlgDepthLimit;//深度上下限设置，值为0则没有限制，为1则有限制
int Depthplower;//深度下限点数
int Depthpupper;//深度上限点数
int FlgChannelLow;//判断低频数据取第一组还是第二组，值为0则取有TVG的通道，为1则取无TVG的通道
int FlgChannelHigh;//判断高频数据取第一组还是第二组，值为0则取有TVG的通道，为1则取无TVG的通道

#pragma endregion 

//////带通滤波器相关量声明，在InitialMemory中赋值
#pragma region 滤波器相关量
int rLowXia;
int rLowShang;
int rHighXia;
int rHighShang;
int order;///
int tapslen;///
double* taps64f;///
float* taps32f;///
INT16* filtercachearr;///
int numIters;///
int specSize;///
int bufSize;///
IppsFIRSpec_32f *pSpec;///
Ipp8u *buf;///

#pragma endregion

//////低通滤波器相关量声明，在InitialMemory中赋值
#pragma region 低通滤波器相关量
int orderLow;///
int tapslenLow;///
double* taps64fLow;///
float* taps32fLow;///
INT16* filtercachearrLow;///
int numItersLow;///
int specSizeLow;///
int bufSizeLow;///
IppsFIRSpec_32f *pSpecLow;///
Ipp8u *bufLow;///

#pragma endregion


//////包络相关量声明，在InitialMemory中赋值
#pragma region 包络相关量
float* envelpSrc;//在求包络子函数中放置float型数组
Ipp16sc *envelpDst;//在求包络子函数中放置复数结果数组
INT16 *envelmag;//在求包络子函数中放置int16型包络数组
#pragma endregion

#pragma region 发射信号仿真
//INT16 SigL[78] = { 16, 27, 11, -26, -53, -36, 21, 73, 69, 0, -82, -105, -31, 59, 100, 59, -31, -95, -81, 0, 81, 95, 31, -59, -100, -59, 31, 95, 81, 0, -81, -95, -31, 59, 100, 59, -31, -95, -81, 0, 81, 95, 31, -53, -82, -44, 21, 58, 44, 0, -36, -39, -11, 20, 30, 16, -8, -21, -16, 0, 13, 14, 4, -7, -11, -6, 3, 8, 6, 0, -5, -5, -2, 3, 4, 2, -1, -3 };
//INT16 SigL[96] = { 16, 22, 8, -17, -32, -21, 12, 40, 36, 0, -41, -51, -18, 35, 63, 39, -22, -69, -61, 0, 66, 81, 27, -54, -94, -57, 31, 99, 86, 0, -81, -95, -31, 59, 100, 59, -31, -95, -81, 0, 81, 95, 31, -59, -100, -59, 31, 95, 81, 0, -81, -95, -31,	59, 100, 59, -31, -95, -81, 0, 81, 86, 25, -44, -67, -36, 17, 47, 36, 0, -30, -32, -9, 16, 25, 13, -6, -17, -13, 0, 11, 12, 3, -6, -9, -5, 2, 6, 5, 0, -4, -4, -1, 2, 3, 2 };
INT16 SigL[84] = { 16, 24, 9, -21, -41, -27, 16, 54, 50, 0, -59, -74, -26, 52, 94, 58, -32, -105, -96, 0, 94, 109, 35, -66, -111, -64, 33, 101, 85, 0, -83, -96, -31, 58, 97, 56, -29, -89, -74, 0, 72, 84, 27, -50, -84, -49, 25, 76, 66, 0, -54, -58, -17, 29, 45, 24, -11, -32, -24, 0, 20, 21, 6, -11, -17, -9, 4, 12, 9, 0, -7, -8, -2, 4, 6, 3, -2, -4, -3, 0, 3, 3, 1, -1 };

int SigLLen;
#pragma endregion

//////高频数据处理变量声明，在InitialMemory中赋值

INT16 NoiseLevH;//高频噪声值
int SNRH;//高频信噪比
INT16 maxvalueH;//高频一次回波最大值
int maxpointH;//高频一次回波最大值所在点
INT16 AmpH;//阈值*最大值
int sNH;
int pingSpanH;
int NH;
int* depthStoreH;
short int* IntenStoreH;
int flgPingH;//初始化之后更新depthStoreH和IntenStoreH的标志位
int foreseeValueH;
int xiaxianH;
int shangxianH;
int secondH;
int thirdH;
int fourthH;
int flgH2;
int flgH3;
int flgH4;
INT16 maxvalueH2ed;
int maxpointH2ed;
INT16 maxvalueH3rd;
int maxpointH3rd;
INT16 maxvalueH4th;
int maxpointH4th;

#pragma endregion

//////低频数据变量声明，在InitialMemory中赋值
#pragma region 高频数据处理变量声明
INT16* DataLowArr;//定义一个10*DataLen长度的数组的指针
INT64* DataLowCorrCache64s;
INT32* DataLowCorrCache32s;
INT16* DataLowSumCache16s;

int pastd[10] = { 1, 1, 1, 1, 1, 1, 2, 16, 16, 16 };
int pa[20] = { 0 };
int DepthLPointArr[10] = { 0 };

INT16 NoiseLevL;//低频噪声值
int SNRL;//低频信噪比
INT16 maxvalueL;//低频一次回波最大值
int maxpointL;//低频一次回波最大值所在点
INT16 AmpL;//阈值*最大值
#pragma endregion

//////缓存变量及其他变量声明
double* arr;
int LowerLimt;
int UperLimit;
int cache1;
int cache2;
int cache3;
int cache4;
int cache5;
int cache6;
INT16 *HighCache;//用来求解高频水深的缓存数组
INT16 *LowCache;//用来求解低频水深的缓存数组
INT16 *DataLow;//低频处理子函数中的操作变量
INT16 *DataHigh;//高频处理子函数中的操作变量
int banpointsLow;//低频处理子函数中盲区点数
int banpointsHigh;//高频处理子函数中盲区点数

extern "C" __declspec(dllexport) void InitialMemory()//初始化，开辟全局变量的内存空间
{
	ippInit();
	//////预开辟空间大小，=65536*2，800kH采样率、300m深度下，单通道点数为320,000点
	DataLen = 400000;

	//////软件输入参数的一一对应，初始化
	channel1 = ippsMalloc_16s(DataLen*sizeof(INT16));
	ippsZero_16s(channel1, DataLen);
	channel2 = ippsMalloc_16s(DataLen*sizeof(INT16));
	ippsZero_16s(channel2, DataLen);
	channel3 = ippsMalloc_16s(DataLen*sizeof(INT16));
	ippsZero_16s(channel3, DataLen);
	channel4 = ippsMalloc_16s(DataLen*sizeof(INT16));
	ippsZero_16s(channel4, DataLen);
	channelEnv1 = ippsMalloc_16s(DataLen*sizeof(INT16));
	ippsZero_16s(channelEnv1, DataLen);
	channelEnv2 = ippsMalloc_16s(DataLen*sizeof(INT16));
	ippsZero_16s(channelEnv2, DataLen);
	channelEnv3 = ippsMalloc_16s(DataLen*sizeof(INT16));
	ippsZero_16s(channelEnv3, DataLen);
	channelEnv4 = ippsMalloc_16s(DataLen*sizeof(INT16));
	ippsZero_16s(channelEnv4, DataLen);
	Range = 0;
	Len = 0;
	fs = 0;
	PowerL = 0;
	PowerH = 0;
	GainL = 0;
	GainH = 0;
	PwGnL = 0;
	PwGnH = 0;
	WidthL = 0;
	WidthH = 0;
	CountAll = 0;
	ThredH = 0;
	DepthpointL = 0;
	DepthpointH = 0;
	WidthLowPoints = 0;
	WidthHighPoints = 0;
	MaxVL = 0;
	MaxVH = 0;
	FlgDepthLimit = 0;
	Depthplower = 0;
	Depthpupper = 0;
	FlgChannelLow = 0;
	FlgChannelHigh = 0;

	//////带通滤波器相关量初始化
	rLowXia = 0;
	rLowShang = 0;
	rHighXia = 0;
	rHighShang = 0;
	order = 64;//带通滤波器阶数
	tapslen = 65;//滤波器系数个数
	taps64f = ippsMalloc_64f(tapslen*sizeof(double));//带通滤波器系数数组
	ippsZero_64f(taps64f, tapslen);
	taps32f = ippsMalloc_32f(tapslen*sizeof(float));//带通滤波器系数数组
	ippsZero_32f(taps32f, tapslen);
	filtercachearr = ippsMalloc_16s(DataLen*sizeof(INT16));
	ippsZero_16s(filtercachearr, DataLen);
	numIters = 0;
	ippsFIRSRGetSize(tapslen, ipp32f, &specSize, &bufSize);
	pSpec = (IppsFIRSpec_32f*)ippsMalloc_8u(specSize);
	buf = ippsMalloc_8u(bufSize);
	ippsZero_8u(buf, bufSize);

	//////低通滤波器相关量初始化
	orderLow = 64;
	tapslenLow = 65;//滤波器系数个数
	numItersLow = 0;
	taps64fLow = ippsMalloc_64f(tapslenLow*sizeof(double));//带通滤波器系数数组
	ippsZero_64f(taps64fLow, tapslenLow);
	taps32fLow = ippsMalloc_32f(tapslenLow*sizeof(float));//带通滤波器系数数组
	ippsZero_32f(taps32fLow, tapslenLow);
	filtercachearrLow = ippsMalloc_16s(DataLen*sizeof(INT16));
	ippsZero_16s(filtercachearrLow, DataLen);
	ippsFIRSRGetSize(tapslenLow, ipp32f, &specSizeLow, &bufSizeLow);
	pSpecLow = (IppsFIRSpec_32f*)ippsMalloc_8u(specSizeLow);
	bufLow = ippsMalloc_8u(bufSizeLow);
	ippsZero_8u(bufLow, bufSizeLow);


	//////包络相关量初始化
	envelpSrc = ippsMalloc_32f(DataLen * sizeof(float));//在求包络子函数中放置float型数组
	ippsZero_32f(envelpSrc, DataLen);
	envelpDst = ippsMalloc_16sc(DataLen * sizeof(Ipp16sc));//在求包络子函数中放置复数结果数组
	ippsZero_16sc(envelpDst, DataLen);
	envelmag = ippsMalloc_16s(DataLen * sizeof(INT16));//在求包络子函数中放置float型包络数组
	ippsZero_16s(envelmag, DataLen);

	//////发射信号相关量初始化
	SigLLen = 84;//matlab200us下低频长度

	//////高频数据处理变量初始化
	NoiseLevH = 0;
	SNRH = 0;
	maxvalueH = 0;
	maxpointH = 0;
	AmpH = 0;
	sNH = 0;
	pingSpanH = 5;
	NH = 5;
	depthStoreH = ippsMalloc_32s(NH * sizeof(int));
	ippsZero_32s(depthStoreH, NH);
	IntenStoreH = ippsMalloc_16s(NH * sizeof(INT16));
	ippsZero_16s(IntenStoreH, NH);
	flgPingH = 0;
	foreseeValueH = 0;
	xiaxianH = 0;
	shangxianH = 0;
	secondH = 0;
	thirdH = 0;
	fourthH = 0;
	flgH2 = 0;
	flgH3 = 0;
	flgH4 = 0;
	maxvalueH2ed = 0;
	maxpointH2ed = 0;
	maxvalueH3rd = 0;
	maxpointH3rd = 0;
	maxvalueH4th = 0;
	maxpointH4th = 0;

	//////低频数据处理变量初始化

	DataLowArr = ippsMalloc_16s(10 * DataLen * sizeof(INT16));
	ippsZero_16s(DataLowArr, 10 * DataLen);
	//DataLowArr[10][D] = { 0 };
	//INT16(*pDataLowArr)[D] = DataLowArr;//定义一个10行的二维数组的指针
	DataLowCorrCache64s = ippsMalloc_64s(DataLen * sizeof(INT64));
	ippsZero_64s(DataLowCorrCache64s, DataLen);
	DataLowCorrCache32s = ippsMalloc_32s(DataLen * sizeof(INT32));
	ippsZero_32s(DataLowCorrCache32s, DataLen);
	DataLowSumCache16s = ippsMalloc_16s(DataLen * sizeof(INT16));
	ippsZero_16s(DataLowSumCache16s, DataLen);

	NoiseLevL = 0;
	SNRL = 0;
	maxvalueL = 0;
	maxpointL = 0;
	AmpL = 0;


	//////缓存变量及其他变量初始化
	arr = ippsMalloc_64f(20 * sizeof(double));
	ippsZero_64f(arr, 20);
	LowerLimt = 0;//水深下限
	UperLimit = 0;//水深上限
	cache1 = 0;
	cache2 = 0;
	cache3 = 0;
	cache4 = 0;
	cache5 = 0;
	cache6 = 0;
	LowCache = ippsMalloc_16s(DataLen * sizeof(INT16));//用来求解低频水深的缓存数组
	ippsZero_16s(LowCache, DataLen);
	HighCache = ippsMalloc_16s(DataLen * sizeof(INT16));//用来求解高频水深的缓存数组
	ippsZero_16s(HighCache, DataLen);
	DataLow = ippsMalloc_16s(DataLen * sizeof(INT16));
	ippsZero_16s(DataLow, DataLen);
	DataHigh = ippsMalloc_16s(DataLen * sizeof(INT16));
	ippsZero_16s(DataHigh, DataLen);
	banpointsLow = 0;
	banpointsHigh = 0;

	return;
}

extern "C" __declspec(dllexport) void FreeMemory()//释放内存空间
{
	ippsFree(taps64f);
	ippsFree(taps32f);
	ippsFree(filtercachearr);
	ippFree(pSpec);
	ippFree(buf);
	ippsFree(taps64fLow);
	ippsFree(taps32fLow);
	ippsFree(filtercachearrLow);
	ippFree(pSpecLow);
	ippFree(bufLow);

	ippsFree(envelpSrc);
	ippsFree(envelpDst);
	ippsFree(envelmag);
	ippsFree(depthStoreH);
	ippsFree(IntenStoreH);
	ippsFree(DataLowArr);
	ippsFree(DataLowCorrCache64s);
	ippsFree(DataLowCorrCache32s);
	ippsFree(DataLowSumCache16s);

	ippsFree(arr);
	ippsFree(HighCache);
	ippsFree(LowCache);
	ippsFree(DataLow);
	ippsFree(DataHigh);

	return;
}


extern "C" __declspec(dllexport) void DataExecute(INT16* dataL, INT16* dataH, int* range, int* lens, int FS, int* powerL, int* powerH,
	int* gainL, int* gainH, int* widthL, int* widthH, int* countall, double thredH, int* depthpointL, int* depthpointH,
	int* maxvL, int* maxvH, double depthlower, double depthupper, double* crr)//传入低频、高频原始回波数据，传入计数值，传出标志位，低频、高频水底所在点数
{

	///20171208--->在关闭自动参数更新的情况下，使用该代码进行参数判定
	int flg = 0;
	if (Range != *range)
		flg++;
	if (fs != FS)
		flg++;
	if (PowerL != *powerL)
		flg++;
	if (PowerH != *powerH)
		flg++;
	if (GainL != *gainL)
		flg++;
	if (GainH != *gainH)
		flg++;
	if (WidthL != *widthL)
		flg++;
	if (WidthH != *widthH)
		flg++;
	if (ThredH != thredH)
		flg++;




//////////////////
	if (flg != 0)
	{
		CountAll = 1;
		*countall = CountAll;
		ippsZero_16s(DataLowArr, 10 * DataLen);
		ippsZero_16s(DataLowSumCache16s, DataLen);
	}

	
	//传入的量赋值到相应内存中
	Range = *range;
	Len = *lens / 2;//数据长度除以2得到每个通道的数据长度
	fs = FS;
	PowerL = *powerL;
	PowerH = *powerH;
	GainL = *gainL;
	GainH = *gainH;
	WidthL = *widthL;
	WidthH = *widthH;
	CountAll = *countall;
	ThredH = thredH;
	DepthpointL = *depthpointL;
	DepthpointH = *depthpointH;
	MaxVL = *maxvL;
	MaxVH = *maxvH;
	ippsCopy_64f(crr, arr, 10);

	Depthplower = 0; //(int)depthlower * 2 * fs / c;
	Depthpupper = 0; //(int)depthupper * 2 * fs / c;

	//将传入数组分割为四通道数据
	ippsCopy_16s(dataL, channel1, Len);
	ippsCopy_16s(&dataL[Len], channel2, Len);
	ippsCopy_16s(dataH, channel3, Len);
	ippsCopy_16s(&dataH[Len], channel4, Len);

	//判断是否有深度上下限限制
	if (Depthplower >= 0 && Depthpupper > Depthplower)
		FlgDepthLimit = 1;
	else
		FlgDepthLimit = 0;

	//功率和增益拼接为一个值
	for (int sr = 0; sr < GAIN_LEN; sr++)
	{
		if (GainL <= GAINL_ARR[sr])
		{
			PwGnL = 10 * (PowerL - 1) + sr;
			break;
		}
	}
	for (int ss = 0; ss < GAIN_LEN; ss++)
	{
		if (GainH <= GAINH_ARR[ss])
		{
			PwGnH = 10 * (PowerH - 1) + ss;
			break;
		}
	}



	//双频信号的发射信号点数
	WidthLowPoints = WidthL * fs / 1000 + 1 + 3 * fs / fL; //+ 1 + 5 * fs / fL;//低频信号对应的点数
	WidthHighPoints = WidthH * fs / 1000 + 1 + 13 * fs / fH + 1;//高频信号对应的点数

	banpointsLow = 2 * WidthLowPoints;
	banpointsHigh = 2 * WidthHighPoints;


//arr[5] = banpointsLow;
//arr[6] = banpointsHigh;
//arr[7] = WidthLowPoints;
//arr[8] = WidthL;


	int BanPoints = (int)(arr[1] * 2 / c*fs * 1000);
	if (banpointsLow < BanPoints)
	{
		banpointsLow = BanPoints;
	}
	if (banpointsHigh < BanPoints)
	{
		banpointsHigh = BanPoints;
	}

	//arr[7] = BanPoints;
	//arr[8] = banpointsHigh;




	//该部分做两个工作：1.确定滤波上下限；2.求高频水深，并确认通道选择、声学数组、量程、功率、增益、脉宽等参数
	if (CountAll < 1)
	{
		arr[0] = -1;//计数值错误
		ippsCopy_64f(arr, crr, 10);
		*lens = 0;
		ippsZero_64f(arr, 10);
		*depthpointL = 0;
		*depthpointH = 0;
		return;
	}

	if (CountAll == 1)
	{
		//设定双频滤波的上下限频率值
		rLowXia = 20; rLowShang = 30;
		if (fs == 250){
			rHighXia = 40; rHighShang = 60;
		}
		else if (fs == 160){
			rHighXia = 30; rHighShang = 50;
		}
		else if (fs == 500 || fs == 800){
			rHighXia = 190; rHighShang = 210;
		}
		else{
			arr[0] = -2;//采样率错误
			ippsCopy_64f(arr, crr, 10);
			ippsZero_64f(arr, 10);
			*depthpointL = 0;
			*depthpointH = 0;
			return;
		}

		//2.确认通道选择，并求双频水深
		FlgChannelLow = 0;
		FlgChannelHigh = 0;
	}

		if ((FlgChannelLow != 0 && FlgChannelLow != 1) || (FlgChannelHigh != 0 && FlgChannelHigh != 1))
		{
			arr[0] = -3;
			ippsCopy_64f(arr, crr, 10);
			ippsZero_64f(arr, 10);
			*depthpointL = 0;
			*depthpointH = 0;
			return;
		}

		BandPassFilter(channel1, Len, (double)rLowXia / (double)fs, (double)rLowShang / (double)fs);
		Envel(channel1, channelEnv1, Len);
		BandPassFilter(channel2, Len, (double)rLowXia / (double)fs, (double)rLowShang / (double)fs);
		Envel(channel2, channelEnv2, Len);
		BandPassFilter(channel3, Len, (double)rHighXia / (double)fs, (double)rHighShang / (double)fs);
		Envel(channel3, channelEnv3, Len);
		BandPassFilter(channel4, Len, (double)rHighXia / (double)fs, (double)rHighShang / (double)fs);
		Envel(channel4, channelEnv4, Len);

		//ippsCopy_16s(channelEnv1, dataL, Len);
		//ippsCopy_16s(channelEnv2, &dataL[Len], Len);
		//ippsCopy_16s(channelEnv3, dataH, Len);
		//ippsCopy_16s(channelEnv4, &dataH[Len], Len);

		//return;





		HighGetDepthPoint();
		LowGetDepthPoint();

		//声学数组，深度值更新至上位机内存中
		ippsCopy_16s(DataLow, dataL, Len);//DataLow
		ippsZero_16s(&dataL[Len], Len);
		ippsCopy_16s(DataHigh, dataH, Len);//DataHigh
		ippsZero_16s(&dataH[Len], Len);

		//此处为依据实际数据，对不同深度的实际高低频深度点数差值取不同的值
		if (DepthpointL>=3*2*160000/c)
			DepthpointL = DepthpointL - 20;//
		else if (DepthpointL>=1*2*160000/c && DepthpointL<3*2*160000/c)
			DepthpointL = DepthpointL - 20;//
		else
			DepthpointL = DepthpointL - 20;//

		if (DepthpointL < DepthpointH)
			DepthpointL = DepthpointH;






		//更新深度点数
		*depthpointL = DepthpointL;//
		*depthpointH = DepthpointH;

		//更新截取数据长度
		double depthcut = ((*depthpointH) *c / fs / 2 / 1000);
		int depthcut32s = (((int)(depthcut * 2))/10 + 1)*10*2;
		*lens = depthcut32s*fs *1000/ ((int)c);
//////20171212
		//arr[2] = depthcut;
//////////

		//通道选择、量程、功率、增益、脉宽、最大值更新模块
		//ParasUpdate();

		//其他参数更新至上位机的内存空间中
		*range = Range;
		*countall = CountAll;

		*powerL = PowerL;
		*gainL = GainL;
		*widthL = WidthL;
		*maxvL = (int)MaxVL;//NoiseLevL;

		*powerH = PowerH;
		*gainH = GainH;
		*widthH = WidthH;
		*maxvH = (int)MaxVH;//NoiseLevH;
		ippsCopy_64f(arr, crr, 10);
		
		
		//相关的值清零
		ippsZero_16s(DataLow, Len);
		ippsZero_16s(DataHigh, Len);

		
		ippsZero_16s(channel1, Len);
		ippsZero_16s(channel2, Len);
		ippsZero_16s(channel3, Len);
		ippsZero_16s(channel4, Len);
		ippsZero_16s(channelEnv1, Len);
		ippsZero_16s(channelEnv2, Len);
		ippsZero_16s(channelEnv3, Len);
		ippsZero_16s(channelEnv4, Len);


		
		ippsZero_64f(arr, 10);

		return;

	//}




//	//CountAll > 1的情况：
//	BandPassFilter(channel1, Len, (double)rLowXia / fs, (double)rLowShang / fs);
//	Envel(channel1, channelEnv1, Len);
//	BandPassFilter(channel2, Len, (double)rLowXia / fs, (double)rLowShang / fs);
//	Envel(channel2, channelEnv2, Len);
//	BandPassFilter(channel3, Len, (double)rHighXia / fs, (double)rHighShang / fs);
//	Envel(channel3, channelEnv3, Len);
//	BandPassFilter(channel4, Len, (double)rHighXia / fs, (double)rHighShang / fs);
//	Envel(channel4, channelEnv4, Len);
//
//
///////20171212
//	
//	//若回波很弱或未检测到回波，则长度置为0，arr[0]置为-2，返回
//	ippsMaxIndx_16s(&channelEnv1[banpointsLow], Len - banpointsLow - WidthLowPoints, &maxvalueL, &maxpointL);
//	ippsMaxIndx_16s(&channelEnv3[banpointsHigh], Len - banpointsHigh - WidthHighPoints, &maxvalueH, &maxpointH);
//		
//	arr[3] = (double)maxvalueH;
//	arr[4] = (double)maxpointH + banpointsHigh;
//	arr[5] = (double)maxvalueL;
//	arr[6] = (double)maxpointL + banpointsLow;
//	ippsCopy_64f(arr, crr, 10);
//
//////////////
//	
//	if ( maxvalueH <= 200)//maxvalueL <= 200 ||
//	{
//		//*lens = 0;//1
//		//CountAll--;
//		//*countall = CountAll;
//
//		arr[0] = -3;
//		ippsCopy_64f(arr, crr, 10);
//		ippsZero_64f(arr, 10);
//		//return;
//	}
//
//	//求水深
//	HighGetDepthPoint();
//	LowGetDepthPoint();
//	
//
//	//声学数组，深度值更新至上位机内存中
//	ippsCopy_16s(DataLow, dataL, Len);
//	ippsZero_16s(&dataL[Len], Len);
//	ippsCopy_16s(DataHigh, dataH, Len);
//	ippsZero_16s(&dataH[Len], Len);
//
//
//	if (DepthpointL>=3*2*160000/c)
//		DepthpointL = DepthpointL - 20;//
//	else if (DepthpointL>=1*2*160000/c && DepthpointL<3*2*160000/c)
//		DepthpointL = DepthpointL - 20;
//	else
//		DepthpointL = DepthpointL - 20;
//
//	if (DepthpointL < DepthpointH)
//		DepthpointL = DepthpointH;
//
//	*depthpointL = DepthpointL;//
//	*depthpointH = DepthpointH;
//
//	//截取数据长度
//	double depthcut = ((*depthpointH) *c / fs / 2 / 1000);
//	int depthcut32s = (((int)(depthcut * 2))/10 + 1)*10*2;
//	*lens = depthcut32s*fs *1000/ ((int)c);
////////20171212
//	arr[2] = depthcut;
////////////
//
//	ippsZero_16s(DataLow, Len);
//	ippsZero_16s(DataHigh, Len);
//
//	//相关的值清零
//	ippsZero_16s(channel1, Len);
//	ippsZero_16s(channel2, Len);
//	ippsZero_16s(channel3, Len);
//	ippsZero_16s(channel4, Len);
//	ippsZero_16s(channelEnv1, Len);
//	ippsZero_16s(channelEnv2, Len);
//	ippsZero_16s(channelEnv3, Len);
//	ippsZero_16s(channelEnv4, Len);
//
//
//	//通道选择、量程、功率、增益、脉宽、最大值更新模块
//	//ParasUpdate();
//
//	//更新值至上位机的内存空间中
//	*range = Range;
//	*countall = CountAll;
//
//	*powerL = PowerL;
//	*gainL = GainL;
//	*widthL = WidthL;
//	*maxvL = (int)maxvalueL;
//
//	*powerH = PowerH;
//	*gainH = GainH;
//	*widthH = WidthH;
//	*maxvH = (int)maxvalueH;
//
//	
//	ippsCopy_64f(arr, crr, 10);
//	ippsZero_64f(arr, 10);
//
//	return;

}



extern "C" __declspec(dllexport) void HighGetDepthPoint()
{
	if (FlgDepthLimit == 0)
	{

		switch (FlgChannelHigh)
		{
		case 0:ippsCopy_16s(channelEnv3, DataHigh, Len); break;
		case 1:ippsCopy_16s(channelEnv4, DataHigh, Len); break;
		default:{ }//通道选取标志位错误
		}	

		//排除孤立高点的干扰
		for (int pl = 0; pl < 2; pl++)
		{
			ippsMaxIndx_16s(&DataHigh[banpointsHigh], Len - banpointsHigh - WidthHighPoints, &maxvalueH, &maxpointH);//maxpoint的值从data+banpoints开始计0
			maxpointH += banpointsHigh;

			int cc = 0;//用来计数
			for (int i = maxpointH - WidthHighPoints; i < maxpointH + WidthHighPoints; i++)
			{
				if (DataHigh[i] >(maxvalueH >> 1))
					cc = cc + 1;
			}

			if (cc < (int)(WidthHighPoints*0.3))
			{
				ippsZero_16s(&DataHigh[maxpointH - (WidthHighPoints >> 1)], 1 * WidthHighPoints);
			}
			else
			{
				break;
			}
		}

		//求噪声等级和最大值
		ippsMean_16s_Sfs(&DataHigh[(int)(0.85*Len)], (int)(0.10*Len), &NoiseLevH, 0);
		ippsMaxIndx_16s(&DataHigh[banpointsHigh + WidthHighPoints], Len - banpointsHigh -2* WidthHighPoints, &maxvalueH, &maxpointH);//maxpoint的值从data+banpoints开始计0
		maxpointH += banpointsHigh;

		//求信噪比SNR，利用该值判断水质清澈或者浑浊，分别对应不同算法，后续补充
		SNRH = (int)(maxvalueH / (NoiseLevH + 1));//此处加1是为了防止噪声为0的情况
		
		
		
		//arr[4] = SNRH;



		if (maxvalueH <= 1000 || SNRH <= 10)
			arr[0] = -4;//无回波或回波幅值很小

		if (SNRH >= 10)
		{
			//找到大于最大值50％以上的第一个点，认为是水底
			int sk;
			for (sk = banpointsHigh + WidthHighPoints; sk < Len - WidthHighPoints; sk++)
			{
				if (DataHigh[sk] >= (maxvalueH * 0.6))
				{
					ippsCopy_16s(&DataHigh[sk - WidthHighPoints], HighCache, 2 * WidthHighPoints);
					break;
				}
			}


			ippsMaxIndx_16s(HighCache, 2 * WidthHighPoints, &maxvalueH, &maxpointH);//maxpoint的值从data+banpoints开始计0
			maxpointH += sk - WidthHighPoints;
			MaxVH = (int)maxvalueH;

			//根据输入的ThredH参数确定阈值并求解深度
			AmpH = (short)(maxvalueH * 0.1);

	//20171205版本
			int j,chch = 0;
			if ((sk - 2 * WidthHighPoints) > banpointsHigh)
				chch = sk - 2 * WidthHighPoints + 1 * WidthHighPoints;//为下面预留出一个WidthHighPoints的间隔
			else
				chch = banpointsHigh + 1 * WidthHighPoints;//为下面预留出一个WidthHighPoints的间隔

			for (j = sk; j >= banpointsHigh; j--)//chch
			{
				if (DataHigh[j] <= (short)(maxvalueH * 0.1))
				{
					DepthpointH = j + 1;
					//arr[7] = -3;
					if (((j - WidthHighPoints) > banpointsHigh) && (arr[0] != -4))
					{
						for (int ks = j - WidthHighPoints; ks <= j; ks++)
						{
							if (DataHigh[ks] >= AmpH)
							{
								DepthpointH = ks + 1;

								//arr[7] = -4;
								break;
							}
						}
					}
					break;
				}
				DepthpointH = j + 1;

			}

			//数据处理
//////////////////////////////
			//ippsSet_16s(8192/2, &DataHigh[DepthpointH], WidthHighPoints/4);
			//ippsSet_16s(0, &DataHigh[DepthpointH + 1*WidthHighPoints], Len - DepthpointH - 2*WidthHighPoints);
			ippsSet_16s(0, &DataHigh[DepthpointH], Len - DepthpointH - 1*WidthHighPoints);

		}
		else
		{
			DepthpointH = 0;
		}




///////////20171228抽取
		int coef_SD = Len / 2048;
		int outputLen1 = 0;
		int phase1 = 0;
		int outputLen2 = 0;
		int phase2 = 0;
		if (coef_SD != 1)
		{
			//LowPassFilter(DataLow, Len, 0.5/(double)coef_SD);
			LowPassFilter(DataHigh, Len, 0.5/(double)coef_SD);
			//ippsSampleDown_16s(DataLow, Len, DataLow, &outputLen1, coef_SD, &phase1);
			ippsSampleDown_16s(DataHigh, Len, DataHigh, &outputLen2, coef_SD, &phase2);
		}

		ippsSet_16s(4096, &DataHigh[DepthpointH/coef_SD], WidthHighPoints/2/coef_SD);//maxvalueH
		ippsCopy_16s(&DataHigh[1], DataHigh, 2049);

//////////


		//整理数据以便软件端做处理
		ippsThreshold_LTVal_16s_I(DataHigh, 2048, AmpH, 0);

////////把值划归至1-64之间
		ippsDivC_16s_ISfs(128, DataHigh, 2048, 0);
		//ippsAddC_16s_I(1, DataHigh, 2048);
		ippsThreshold_GT_16s_I(DataHigh, 2048, 64);
		ippsThreshold_LT_16s_I(DataHigh, 2048,  0);
////////

		ippsSet_16s(64, DataHigh, banpointsHigh/coef_SD);//maxvalueH


		ippsZero_16s(HighCache, 2 * WidthHighPoints);

		return;

	}

	else if (FlgDepthLimit == 1)
	{
		//switch (FlgChannelHigh)
		//{
		//case 0:ippsCopy_16s(channelEnv3, DataHigh, Len); break;
		//case 1:ippsCopy_16s(channelEnv4, DataHigh, Len); break;
		//default:{DepthpointH = -1; return; }
		//}

		////排除孤立高点的干扰
		//ippsCopy_16s(&DataHigh[Depthplower], HighCache, Depthpupper - Depthplower);

		////求噪声等级和最大值
		//ippsMean_16s_Sfs(&DataHigh[(int)(0.6*Len)], (int)(0.05*Len), &NoiseLevH, 0);

		//ippsMaxIndx_16s(HighCache, 2 * WidthHighPoints, &maxvalueH, &maxpointH);//maxpoint的值从data+banpoints开始计0
		//maxpointH += Depthplower - WidthHighPoints;
		//MaxVH = (int)maxvalueH;

		////求信噪比SNR，利用该值判断水质清澈或者浑浊，分别对应不同算法，后续补充
		//SNRH = (int)(maxvalueH / (NoiseLevH + 1));//此处加1是为了防止噪声为0的情况

		////根据输入的ThredH参数确定阈值并求解深度
		//AmpH = (short)(maxvalueH * ThredH);//maxvalueH * ThredH
		//for (int j = 0; j < (Depthpupper - Depthplower); j++)
		//{
		//	if (HighCache[j] >= AmpH)
		//	{
		//		DepthpointH = Depthplower + j + 1;
		//		break;
		//	}
		//}




		////整理数据以便软件端做处理
		//ippsThreshold_LTVal_16s_I(DataHigh, Len, AmpH, 0);
		//ippsSet_16s(maxvalueH, DataHigh, banpointsHigh);

		//ippsZero_16s(HighCache, 2 * WidthHighPoints);

		//return;

	}

}



extern "C" __declspec(dllexport) void LowGetDepthPoint()
{
	int  remainder = (CountAll - 1) % 10;
	if (FlgDepthLimit == 0)//判断是否有上下限限制
	{
		switch (FlgChannelLow)
		{
			case 0:ippsCopy_16s(channel1, DataLow, Len); break;
			case 1:ippsCopy_16s(channel2, DataLow, Len); break;
			default:{ }//通道选取标志位错误
		}

		//求信噪比SNR，利用该值判断水质清澈或者浑浊，分别对应不同算法，后续补充
		ippsMean_16s_Sfs(&channelEnv1[(int)(0.85*Len)], (int)(0.10*Len), &NoiseLevL, 0);
		ippsMaxIndx_16s(&channelEnv1[banpointsLow], Len - banpointsLow - WidthLowPoints, &maxvalueL, &maxpointL);//maxpoint的值从data+banpoints开始计0
		maxpointL += banpointsLow;
		MaxVL = (int)maxvalueL;
		SNRL = (int)(maxvalueL / (NoiseLevL + 1));//此处加1是为了防止噪声为0的情况


		ippsCrossCorr_16s64s(SigL, SigLLen, DataLow, Len, DataLowCorrCache64s, Len, 0);//该步骤可求得所需的相关结果
		ippsDivC_64s_ISfs(65536, DataLowCorrCache64s, Len, 0);//保证1ping的结果可转化为INT16，10ping的加和仍在INT16范围内
		ippsConvert_64s32s_Sfs(DataLowCorrCache64s, DataLowCorrCache32s, Len, ippRndZero,0);
		ippsConvert_32s16s(DataLowCorrCache32s, DataLow, Len);
		Envel(DataLow, DataLow, Len);

		//对未平均的数据做处理，排除孤立高点的干扰
		for (int pl = 0; pl < 2; pl++)
		{
			ippsMaxIndx_16s(&DataLow[banpointsLow], Len - banpointsLow - WidthLowPoints, &maxvalueL, &maxpointL);//maxpoint的值从data+banpoints开始计0
			maxpointL += banpointsLow;

			int cc = 0;//用来计数
			for (int i = maxpointL - WidthLowPoints; i < maxpointL + WidthLowPoints; i++)
			{
				if (DataLow[i] >(maxvalueL >> 1))
					cc = cc + 1;
			}

			if (cc < (int)(WidthLowPoints*0.3))
			{
				ippsZero_16s(&DataLow[maxpointL - (WidthLowPoints >> 1)], 1 * WidthLowPoints);
			}
			else
			{
				break;
			}
		}

		//找到大于最大值50％以上的第一个点，认为是水底
		int sl;
		for (sl = banpointsLow; sl < Len; sl++)
		{
			if (DataLow[sl] >= (maxvalueL * 0.6))
			{
				ippsMaxIndx_16s(&DataLow[sl], 4 * WidthLowPoints, &maxvalueL, &maxpointL);
				maxpointL += sl;
				DepthLPointArr[remainder] = maxpointL;


				break;
			}
		}


////////->2017version:处理沙丘时双频有差别
		int addmethod = 1;
		switch (addmethod){
		case 0:{

				   ippsSub_16s_I(&DataLowArr[remainder*DataLen], DataLowSumCache16s, Len);
				   ippsAdd_16s_I(DataLow, DataLowSumCache16s, Len);
				   ippsCopy_16s(DataLow, &DataLowArr[remainder*DataLen], Len);
				   ippsCopy_16s(DataLowSumCache16s, DataLow, Len);
		}
			break;
		case 1:{
				   int  remainderr = (CountAll - 1) % 5;
				   ippsSub_16s_I(&DataLowArr[remainderr*DataLen], DataLowSumCache16s, Len);
				   ippsAdd_16s_I(DataLow, DataLowSumCache16s, Len);
				   ippsCopy_16s(DataLow, &DataLowArr[remainderr*DataLen], Len);
				   ippsCopy_16s(DataLowSumCache16s, DataLow, Len);

				   for (int ldk = 0; ldk < 5; ldk++)
					ippsAdd_16s_I(&DataLowArr[remainderr*DataLen], DataLow, Len);

		}
			break;
		default:{ }
		}


		//arr[0] = (double)DataLowSumCache16s[0];// DataLowArr[remainder*DataLen + 0];
		//arr[1] = (double)DataLowSumCache16s[1];


		//对平均后的数据做处理，排除孤立高点的干扰
		for (int pl = 0; pl < 2; pl++)
		{
			ippsMaxIndx_16s(&DataLow[banpointsLow], Len - banpointsLow - WidthLowPoints, &maxvalueL, &maxpointL);//maxpoint的值从data+banpoints开始计0
			maxpointL += banpointsLow;

			int cc = 0;//用来计数
			for (int i = maxpointL - WidthLowPoints; i < maxpointL + WidthLowPoints; i++)
			{
				if (DataLow[i] >(maxvalueL >> 1))
					cc = cc + 1;
			}

			if (cc < (int)(WidthLowPoints*0.3))
			{
				ippsZero_16s(&DataLow[maxpointL - (WidthLowPoints >> 1)], 1 * WidthLowPoints);
			}
			else
			{
				break;
			}
		}

		//找到大于最大值50％以上的第一个点，认为是水底
		int sk = 0;
		for (sk = banpointsLow; sk < Len; sk++)
		{
			if (DataLow[sk] >= (maxvalueL * 0.9))
			{
				ippsCopy_16s(&DataLow[sk - 2 * WidthLowPoints], LowCache, 4 * WidthLowPoints);
				ippsZero_16s(&DataLow[sk + 2 * WidthLowPoints], Len - sk - 3 * WidthLowPoints);


//////20180119->低频最大值之前的数值进行处理
				if ((sk-3*WidthLowPoints) > banpointsLow)
					//ippsZero_16s(&DataLow[sk - banpointsLow], banpointsLow);
					ippsDivC_16s_ISfs(10, &DataLow[sk - 3*WidthLowPoints], 3*WidthLowPoints, 0);
				else
					//ippsZero_16s(&DataLow[banpointsLow], sk - banpointsLow);
					ippsDivC_16s_ISfs(10, &DataLow[banpointsLow], sk - banpointsLow + 1, 0);
				//arr[9] = 100;

				break;
			}
		}
		//arr[9] = 100;


		//求噪声等级和最大值
		//ippsMean_16s_Sfs(&DataLow[(int)(0.9*Len)], (int)(0.05*Len), &NoiseLevL, 0);
		//arr[9] = NoiseLevL;
		ippsMaxIndx_16s(LowCache, 4 * WidthLowPoints, &maxvalueL, &maxpointL);//maxpoint的值从data+banpoints开始计0
		maxpointL += sk - 2*WidthLowPoints;

/////////////////20180322
		int sumcache = 0; int pSum = 0;
		if (MaxVL > 8400)
		{
			
			for (int cx = 0; cx < 4 * WidthLowPoints; cx++)
			{
				if (LowCache[cx] < maxvalueL*0.8)
					LowCache[cx] = 0;
				else
					sumcache += cx*LowCache[cx];
			}

			ippsSum_16s32s_Sfs(LowCache, 4 * WidthLowPoints, &pSum, 0);
			maxpointL = sk - 2*WidthLowPoints + sumcache / pSum;// - WidthLowPoints/4
		}




//////
		ippsCopy_32s(&pastd[9 - remainder], &pa[0], remainder + 1);
		ippsCopy_32s(&pastd[0], &pa[remainder + 1], 9 - remainder);
		int sm = remainder + 1;
		if (CountAll >= 10)
			sm = 10;
		int psum1 = 0;
		int psum = 0;
		int pck[10] = { 0 };
		ippsSum_32s_Sfs(pa, sm, &psum1, 0);
		ippsMul_32s_Sfs(DepthLPointArr, pa, pck, sm, 0);
		ippsSum_32s_Sfs(pck, sm, &psum, 0);
		
		DepthpointL = psum/psum1;
///////////


///////////20180223
		int T_l = 0;
		switch (T_l)
		{
			case 0:DepthpointL = maxpointL; break;//{ if (MaxVL < 8191) DepthpointL = maxpointL; else DepthpointL = sk; break; }
			case 1:DepthpointL = (psum + maxpointL * 8) / (psum1 + 8); break;
			default:{ }
		}


///////////20180302高频之前的回波图像清零
		if (DepthpointL < DepthpointH)
		{
			if ((DepthpointL - 2*WidthLowPoints) > banpointsLow)
				ippsZero_16s(&DataLow[DepthpointL - 2*WidthLowPoints], DepthpointH - (DepthpointL - 2*WidthLowPoints));
			else
				ippsZero_16s(&DataLow[banpointsLow], DepthpointH - banpointsLow);

		}


///////////20180322低频深度之前的回波图像清零
		if (banpointsLow < DepthpointL)
			ippsZero_16s(&DataLow[banpointsLow], DepthpointL - banpointsLow);


			
			
//////////		

		//DepthpointL = maxpointL;


		//根据输入的ThredH参数确定阈值并求解深度
		AmpL = (short)(maxvalueL * 0.5);//maxvalueH * ThredH
		//for (int j = maxpointL - sk + 2*WidthLowPoints; j > 0; j--)
		//{
		//	if (LowCache[j] == maxvalueL)
		//	{
		//		DepthpointL = sk - 2 * WidthLowPoints + j + 1;
		//		break;
		//	}
		//}


///////////20171228
		int coef_SD = Len / 2048;//除数的理论取值为2048
		int outputLen1 = 0;
		int phase1 = 0;
		int outputLen2 = 0;
		int phase2 = 0;
		if (coef_SD != 1)
		{
			LowPassFilter(DataLow, Len, 0.5/(double)coef_SD);
			//LowPassFilter(DataHigh, Len, 0.5/(double)coef_SD);
			ippsSampleDown_16s(DataLow, Len, DataLow, &outputLen1, coef_SD, &phase1);
			//ippsSampleDown_16s(DataHigh, Len, DataHigh, &outputLen2, coef_SD, &phase2);
		}


		ippsCopy_16s(&DataLow[20/coef_SD], DataLow, 2049);//数组从1开始取是由于ipp降采样方法导致的

//////////




		//整理数据以便软件端做处理
		ippsThreshold_LTVal_16s_I(DataLow, 2048, AmpL, 0);

////////把值划归至1-64之间
		ippsDivC_16s_ISfs(50, DataLow, 2048, 0);
		//ippsAddC_16s_I(1, DataLow, 2048);
		ippsThreshold_GT_16s_I(DataLow, 2048, 64);
		ippsThreshold_LT_16s_I(DataLow, 2048,  0);
////////

	ippsSet_16s(64, DataLow, banpointsLow/coef_SD);//maxvalueL

	return;
	}

	else if (FlgDepthLimit == 1)
	{
		//switch (FlgChannelLow)
		//{
		//case 0:ippsCopy_16s(channelEnv1, DataLow, Len); break;
		//case 1:ippsCopy_16s(channelEnv2, DataLow, Len); break;
		//default:{DepthpointL = -1; return; }
		//}

		////排除孤立高点的干扰
		//ippsCopy_16s(&DataLow[Depthplower], LowCache, Depthpupper - Depthplower);

		////求噪声等级和最大值
		//ippsMean_16s_Sfs(&DataLow[(int)(0.6*Len)], (int)(0.05*Len), &NoiseLevL, 0);

		//ippsMaxIndx_16s(LowCache, 2 * WidthLowPoints, &maxvalueL, &maxpointL);//maxpoint的值从data+banpoints开始计0
		//maxpointL += Depthplower;
		//MaxVL = (int)maxvalueL;

		////求信噪比SNR，利用该值判断水质清澈或者浑浊，分别对应不同算法，后续补充
		//SNRL = (int)(maxvalueL / (NoiseLevL + 1));//此处加1是为了防止噪声为0的情况

		////根据输入的ThredH参数确定阈值并求解深度
		//AmpL = (short)(maxvalueL * ThredH);//maxvalueH * ThredH
		//for (int j = maxpointL - Depthplower; j > 0; j--)
		//{
		//	if (LowCache[j] <= AmpL)
		//	{
		//		DepthpointL = Depthplower + j + 1;
		//		break;
		//	}
		//}


		////整理数据以便软件端做处理
		//ippsThreshold_LTVal_16s_I(DataLow, Len, AmpL, 0);
		//ippsSet_16s(64, DataLow, banpointsLow);//maxvalueL


		//ippsZero_16s(LowCache, 2 * WidthLowPoints);

		//return;


	}

}



extern "C" __declspec(dllexport) void BandPassFilter(INT16* data, int len, double rLowFreq, double rHighFreq)
{
	//int numIters = len + order / 2;
	numIters = len + order / 2;
	ippsCopy_16s(data, filtercachearr, len);
	//滤波系数求解
	ippsFIRGenBandpass_64f(rLowFreq, rHighFreq, taps64f, tapslen, ippWinHamming, ippTrue);
	ippsConvert_64f32f(taps64f, taps32f, tapslen);

	//滤波器获取结构大小和缓存大小
	//int specSize, bufSize;
	//ippsFIRSRGetSize(tapslen, ipp32f, &specSize, &bufSize);

	//initialize the spec structure
	//IppsFIRSpec_32f *pSpec;
	//pSpec = (IppsFIRSpec_32f*)ippsMalloc_8u(specSize);
	ippsFIRSRInit_32f(taps32f, tapslen, ippAlgFFT, pSpec);

	//apply the FIR filter
	//Ipp8u *buf;
	//buf = ippsMalloc_8u(bufSize);
	ippsFIRSR_16s(filtercachearr, filtercachearr, numIters, pSpec, NULL, NULL, buf);
	ippsCopy_16s(&filtercachearr[order / 2], data, len);

	ippsZero_16s(filtercachearr, DataLen);

	////free memory
	//ippFree(pSpec);
	//ippFree(buf);

	return;
}



extern "C" __declspec(dllexport) void Envel(INT16* data, INT16* result, int len)
{
	//hilbert transform
	IppsHilbertSpec_16s16sc* pSpece;
	ippsHilbertInitAlloc_16s16sc(&pSpece, len, ippAlgHintFast);
	ippsHilbert_16s16sc_Sfs(data, envelpDst, pSpece, 0);

	//求包络
	ippsMagnitude_16sc_Sfs(envelpDst, result, len, 0);

	//防止尾部突起
	ippsZero_16s(&result[len - 2 * WidthLowPoints], 2 * WidthLowPoints);

	//ippsFree(pSpec);
	ippsHilbertFree_16s16sc(pSpece);

	return;

}



extern "C" __declspec(dllexport) void ParasUpdate()//通道选择、量程、功率、增益、脉宽更新模块
{

	int dcache = (int)(DepthpointH * c / fs / 1000 / 2);

	//通道选择
	if (dcache >= (int)(10 * 0.8))//depthpointH/fs*c/2=10 -> 20 * fs / c * 0.8
	{
		//1.通道选择
		FlgChannelLow = 0;
		FlgChannelHigh = 0;

		//2.量程调整
		int ir;
		for (ir = 0; ir < RANGE_LEN; ir++)
		{
			if (RANGE_ARR[ir] >(int)(dcache*1.5))
			{
				Range = RANGE_ARR[ir];
				break;
			}
		}

		//3.脉宽选择

		//4.功率增益控制，目前用maxvalueL和maxvalueH控制，范围在4000-8000之间为理想范围
		if (maxvalueL > 4000 && maxvalueL < 8000)
		{

		}
		if (maxvalueL < 4000)
		{
			PwGnL += 1;
			if (PwGnL % 10 == 0)
				PwGnL += 4;
			if (PwGnL > 99)
				PwGnL = 99;
			PowerL = POWERL_ARR[PwGnL / 10];
			GainL = GAINL_ARR[PwGnL % 10];
		}
		if (maxvalueL > 8000)
		{
			PwGnL -= 1;
			if (PwGnL % 10 == 9)
				PwGnL -= 4;
			if (PwGnL < 0)
				PwGnL = 0;
			PowerL = POWERL_ARR[PwGnL / 10];
			GainL = GAINL_ARR[PwGnL % 10];
		}

		if (maxvalueH > 4000 && maxvalueH < 8000)
		{

		}
		if (maxvalueH < 4000)
		{
			PwGnH += 1;
			if (PwGnH % 10 == 0)
				PwGnH += 4;
			if (PwGnH > 99)
				PwGnH = 99;

			PowerH = POWERH_ARR[PwGnH / 10];
			GainH = GAINH_ARR[PwGnH % 10];
		}
		if (maxvalueH > 8000)
		{
			PwGnH -= 1;
			if (PwGnH % 10 == 9)
				PwGnH -= 4;
			if (PwGnH < 0)
				PwGnH = 0;
			PowerH = POWERH_ARR[PwGnH / 10];
			GainH = GAINH_ARR[PwGnH % 10];
		}

	}
	else
	{
		//1.通道选择
		FlgChannelLow = 1;
		FlgChannelHigh = 1;

		//2.量程调整
		int ir;
		for (ir = 0; ir < RANGE_LEN; ir++)
		{
			if (RANGE_ARR[ir] >(int)(dcache*1.5))
			{
				Range = RANGE_ARR[ir];
				break;
			}
		}

		//3.脉宽选择

		//4.功率增益控制，目前用maxvalueL和maxvalueH控制，范围在4000-8000之间为理想范围
		if (maxvalueL > 4000 && maxvalueL < 8000)
		{

		}
		if (maxvalueL < 4000)
		{
			PowerL += 1;
			PwGnL = 10 * PowerL;
			if (PwGnL > 90)
				PwGnL = 90;
			PowerL = POWERL_ARR[PwGnL / 10];
			GainL = GAINL_ARR[PwGnL % 10];

		}
		if (maxvalueL > 8000)
		{
			PowerL -= 1;
			PwGnL = 10 * PowerL;
			if (PwGnL < 0)
				PwGnL = 0;
			PowerL = POWERL_ARR[PwGnL / 10];
			GainL = GAINL_ARR[GainL % 10];

		}

		if (maxvalueH > 4000 && maxvalueH < 8000)
		{

		}
		if (maxvalueH < 4000)
		{
			PowerH += 1;
			PwGnH = 10 * PowerH;
			if (PwGnH > 90)
				PwGnH = 90;
			PowerH = POWERH_ARR[PwGnH / 10];
			GainH = GAINH_ARR[GainH % 10];
		}
		if (maxvalueH > 8000)
		{
			PowerH -= 1;
			PwGnH = 10 * PowerH;
			if (PwGnH < 0)
				PwGnH = 0;
			PowerH = POWERH_ARR[PwGnH / 10];
			GainH = GAINH_ARR[GainH % 10];

		}


	}

}



extern "C" __declspec(dllexport) void LowPassFilter(INT16* data, int len, double rLowFreq)
{
	numItersLow = len + orderLow / 2;
	ippsFIRGenLowpass_64f(rLowFreq, taps64fLow, tapslenLow, ippWinHamming, ippTrue);

	ippsConvert_64f32f(taps64fLow, taps32fLow, tapslenLow);

	ippsFIRSRInit_32f(taps32fLow, tapslenLow, ippAlgFFT, pSpecLow);

	ippsFIRSR_16s(data, filtercachearrLow, numItersLow, pSpecLow, NULL, NULL, bufLow);
	ippsCopy_16s(&filtercachearrLow[order / 2], data, len);

	ippsZero_16s(filtercachearrLow, DataLen);

	
}

