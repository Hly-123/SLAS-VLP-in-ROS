#ifndef imageProcess_ID_hpp_
#define imageProcessZ_ID_hpp_

//-----------------------------------��ͷ�ļ��������֡�------------------------------ 
//      ����������������������ͷ�ļ�
//----------------------------------------------------------------------------------------------  
#include <single_led/vlcCommonInclude.hpp>

//-----------------------------------�������ռ������֡�------------------------------ 
//      ����������������ʹ�õ������ռ�  
//----------------------------------------------------------------------------------------------  
using namespace cv;

//-----------------------------------��ͼ������������---------------------------
//OpenCV��threshold�Զ���ֵ������matlab�е�graythresh������Զ���ֵ������ͼ���ֵ��
double getThreshVal_Otsu_8u(const cv::Mat& _src);
//�ָ��LED�������x_min,x_max,y_min,y_max
void ls_LED(const Mat& _img, int& X_min, int& X_max, int& Y_min, int& Y_max, Mat& img_next);
//�ҳ�LED�����������x_min,x_max,y_min,y_max
void LED_contour(const Mat& _img, vector<Point> &myPoints, vector<int> &myRadius);
// ��������
void para_set(double& X, double& Y, double& D_LED, double& f, double& led_x, double& led_y, double& all_H);
//roi������
//����ȷ��LED�ƴ�С
double get_similarity(Mat&a,Mat&b);
Rect find_minrect(Mat frame,Rect selected,int thresh);
Rect ls_LED_sample(Mat img, int& X_min, int& X_max, int& Y_min, int& Y_max, cv::Mat& imgNext,int sample);
//ʵ�ֶ�ͼ���ϸ��
void thinImage(Mat &srcimage);

//ID�������׺���
/* -------------------�� LEDͼ��Ԥ���� ��----------------
���ܣ�
	LEDͼ��Ԥ������ԭLEDͼ�����ÿ�з�0���ؾ�ֵ�������Ϸ�0���أ�ʵ������ĳ����ֵ���ϣ�Ŀ�����ų�������
	ͳ��Ϊ�о��󣬲����в�ֵ3.9������
�����������ͣ�
	cv::Mat imgLED �и������LEDͼ��
	int threshold ������ֵ�������ɰ洦������ȡ��LED����״
����������ͣ�
	cv::Mat meanRowOfPxiel ��ÿ�о�ֵ��ɵ��о���float��������
		��ΪҪΪ��ƽ����Ч������Ҫ���߾��ȵ���������
------------------------------------------------------------*/
cv::Mat_<float>::Mat ImagePreProcessing(cv::Mat imgLED, int backgroundThreshold,int &backgroundCompensation);
/* -------------------�� ƽ�ƴ��� ��----------------
���ܣ�
	ͼ��ƽ�ƴ����ƶ���¶�Ĳ�����0���
// �����������ͣ�
	cv::Mat frame �����о���ת��Ϊ�о����������
	int shiftCol �е�ƽ��ֵ��+��-��
	int shiftRow ��ƽ�ƣ�+��-��
����������ͣ�
	cv::Mat row �����о���ת��Ϊ�о������λ
------------------------------------------------------------*/
cv::Mat matShift(cv::Mat frame, int shiftCol, int shiftRow);
/* -------------------�� Ѱ�Ҳ��岨�ȴ��� ��----------------
���ܣ�
	Ѱ��LED�о�ֵ�Ĳ��岨��λ������
�����������ͣ�
	cv::Mat imgRow �����о���ת��Ϊ�о���������У�ע�⣬����Ҫfloat���ͣ�
		��ΪҪΪ��ƽ����Ч������Ҫ���߾��ȵ���������
����������ͣ�
	cv::Mat NonZeroLocations ���岨�����ڵ�����
------------------------------------------------------------*/
cv::Mat LEDMeanRowCrestsTroughs(const cv::Mat_<float>::Mat imgRow, int BlurSize);
/* -------------------�� ��ֵ������ ��----------------
���ܣ�
	��ֵ������
�����������ͣ�
	cv::Mat row �����о���ת��Ϊ�о����������
	cv::Mat NonZeroLocations LED�о�ֵ�Ĳ��岨��λ������
	int backgroundThreshold ������ֵ
	int backgroundCompensation ��������ֵ
		Ϊ�˲��������ƴ���СֵΪ0����ɵĶ�ֵ����ֵƫ�ͣ����ڴ˽��е��ڡ�
		������СֵӦ��ָ��Ϊ��ֵ�����ݹ۲�������ã�һ���ָ�������Ǵ���ȥ������
		����ֵ����ImagePreProcessing�����е�backgroundThreshold��������
		С�ڻ��������δ��ȥ���������������Сֵ�����籾������backgroundThreshold
		Ϊ20������δ��ȥ���������������Сֵ��90���ϣ����ڴ˾��������ȷ��ȡ40
����������ͣ�
	cv::Mat row �����о���ת��Ϊ�о������λ
------------------------------------------------------------*/
cv::Mat LEDMeanRowThreshold(cv::Mat imgRow, cv::Mat NonZeroLocations, int backgroundThreshold, int &backgroundCompensation);
cv::Mat convertPxielRowToBit(cv::Mat row,int continuousnum);
/* -------------------�� ��Ϣ���ݻ�ȡ ��----------------
���ܣ�
	�����ʶ����ѽ���LED����λ���ֽ�ͷ����������ݽھ������磺
	cv::Mat msgDate = getMsgDate(imageLED, msgHeaderStampTest);
�����������ͣ�
	const cv::Mat imageLED ��ʶ���LED��ͼ��
	cv::Mat headerStamp �ֽ�ͷ����ע�⣬�˲���������CV_8U��ʽ��cv::Mat_<uchar>(i, j)��һά����
		����ʾ�� cv::Mat msgHeaderStampTest = (cv::Mat_<uchar>(1, 5) <<  0, 1, 0, 1, 0);
����������ͣ�
	������� CV_8U��ʽ���о���
		�������ʾ�� msgDate = [  0,   0,   1,   1,   0,   0,   1,   0,   1,   1,   1]
	�쳣��� 0���������쳣��ԭ���������⵽����Ϣͷ�����ص����colRange��ȡ��Ϣ�������
		��⵽���һ����Ϣͷ�������û�м�⵽��Ϣͷ�������vector.at��������Խ�����
		�����һ���쳣��ͨ��goto��������������Ĳ���ֱ��Խ���Ϊ�ڶ������������ڶ�����
		��ֱ�ӷ����������0����
		�쳣���ʾ�� msgDate = [  0]
--------------------------------------------------------*/
cv::Mat getMsgDate(cv::Mat imgRow, cv::Mat headerStamp,int idlength);
// cv::Mat getMsgDate(const cv::Mat imageLED) {
	// https://stackoverflow.com/questions/32737420/multiple-results-in-opencvsharp3-matchtemplate
	// ����ȡ������λ������Ϊ��ƥ�����
	// // ͼ��Ԥ������ȡͼ��ÿ�о�ֵ����ֵ�����о������
	// imageLED = ImagePreProcessing(imageLED, 20);

	// // ����ȡ��ͼ��ÿ�о�ֵ���ж�ֵ�������о������
	// cv::Mat row =  LEDMeanRowThreshold(imageLED);

	// std::cout << "imgRow = "<< imgRow <<std::endl;
	// cv::Mat ref = convertPxielRowToBitBySample(imgRow);
int subregions(cv::Mat row);
int countstripes(cv::Mat imageLED, int backgroundThreshold,int &backgroundCompensation,int Blursize);
cv::Mat MsgProcess(cv::Mat imageLED, cv::Mat headerStamp,int backgroundThreshold,int &backgroundCompensation,int Blursize,int continuousnum,int idlength);

#endif
