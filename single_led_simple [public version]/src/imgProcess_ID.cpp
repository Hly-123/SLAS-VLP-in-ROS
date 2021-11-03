//-----------------------------------��ͷ�ļ��������֡�------------------------------ 
//      ����������������������ͷ�ļ�
//----------------------------------------------------------------------------------------------  
#include<single_led/imgProcess_ID.hpp>


//-----------------------------------��ͼ��������---------------------------------


//OpenCV��threshold�Զ���ֵ������matlab�е�graythresh������Զ���ֵ������ͼ���ֵ��
double getThreshVal_Otsu_8u(const cv::Mat& _src)
{
	cv::Size size = _src.size();
	if (_src.isContinuous())
	{
		size.width *= size.height;
		size.height = 1;
	}
	const int N = 256;
	int i, j, h[N] = { 0 };
	for (i = 0; i < size.height; i++)
	{
		const uchar* src = _src.data + _src.step*i;
		for (j = 0; j <= size.width - 4; j += 4)
		{
			int v0 = src[j], v1 = src[j + 1];
			h[v0]++; h[v1]++;
			v0 = src[j + 2]; v1 = src[j + 3];
			h[v0]++; h[v1]++;
		}
		for (; j < size.width; j++)
			h[src[j]]++;
	}

	double mu = 0, scale = 1. / (size.width*size.height);
	for (i = 0; i < N; i++)
		mu += i * h[i];

	mu *= scale;
	double mu1 = 0, q1 = 0;
	double max_sigma = 0, max_val = 0;

	for (i = 0; i < N; i++)
	{
		double p_i, q2, mu2, sigma;

		p_i = h[i] * scale;
		mu1 *= q1;
		q1 += p_i;
		q2 = 1. - q1;

		if (std::min(q1, q2) < FLT_EPSILON || std::max(q1, q2) > 1. - FLT_EPSILON)
			continue;

		mu1 = (mu1 + i * p_i) / q1;
		mu2 = (mu - q1 * mu1) / q2;
		sigma = q1 * q2*(mu1 - mu2)*(mu1 - mu2);
		if (sigma > max_sigma)
		{
			max_sigma = sigma;
			max_val = i;
		}
	}

	return max_val;
}


//�ָ��LED�������x_min,x_max,y_min,y_max
void ls_LED(const Mat& _img, int& X_min, int& X_max, int& Y_min, int& Y_max, Mat& img_next)
{
	Mat temp1 = _img.clone();

	//��xmin��xmax
	int row1 = temp1.rows;//����
	int col1 = temp1.cols;//��
	int j = 0;//ע���Ǵ�0��ʼ
	while (j < col1)//j�ĳ�ֵΪ1
	{
		double sum1 = 0.0;
		for (int i = 0;i < row1;i++)//ע��û�е��ں�
		{
			uchar* data1 = temp1.ptr<uchar>(i);//ptr<uchar>(i)[j]���ʵ�i�е�j�е�����
			sum1 = data1[j] + sum1;
		}//����j�е�ÿһ�м���
		if (sum1 > -0.000001 && sum1 < 0.000001)//double���ͣ�����д==0
		{
			j++;
		}
		else
		{
			break;//�������whileѭ��
		}

	}
	X_min = j;

	while (j < col1)//j�ĳ�ֵΪX_min 
	{
		double sum1 = 0.0;
		for (int i = 0;i < row1;i++)
		{
			uchar* data1 = temp1.ptr<uchar>(i);//ptr<uchar>(i)[j]���ʵ�i�е�j�е�����
			sum1 = data1[j] + sum1;
		}//����j�е�ÿһ��XXXXXX����
		if (sum1 != 0)
		{
			j++;
		}
		else
		{
			break;//�������whileѭ��
		}
	}
	X_max = j;

	//�����и�
	Mat image_cut = temp1(Rect(X_min, 0, X_max - X_min, row1));
	Mat temp = image_cut.clone();



	//��ymin��ymax
	int row = temp.rows;//����
	int col = temp.cols;//��
	int i = 0;
	while (i < row)//i�ĳ�ֵΪ1
	{
		double sum = 0.0;
		uchar* data = temp.ptr<uchar>(i);
		for (j = 0;j < col;j++)//��ÿһ���е�ÿһ�����ؽ�����ӣ�ptr<uchar>(i)[j]���ʵ�i�е�j�е�����
		{
			sum = data[j] + sum;
		}//���ջ�õ�i�е��к�
		if (sum > -0.000001 && sum < 0.000001)
		{
			i++;
		}
		else
		{
			Y_min = i;
			break;//�������whileѭ��
		}
	}
	Y_min = i;

	while (i <= row - 16)//i�ĳ�ֵΪY_min
	{
		double sum = 0.0;
		uchar* data = temp.ptr<uchar>(i);
		for (j = 0;j < col;j++)//��ÿһ���е�ÿһ�����ؽ�����ӣ�ptr<uchar>(i)[j]���ʵ�i�е�j�е�����
		{
			sum = data[j] + sum;
		}//���ջ�õ�i�е��к�
		if (sum != 0)
		{
			i++;
		}
		else
		{
			double sum6 = 0.0;
			int iiii = i + 16;
			uchar* data = temp.ptr<uchar>(iiii);
			for (j = 0;j < col;j++)//��ÿһ���е�ÿһ�����ؽ�����ӣ�ptr<uchar>(i)[j]���ʵ�i�е�j�е�����
			{
				sum6 = data[j] + sum6;
			}//���ջ�õ�i��֮��20�У���iiii���к�
			if (sum6 > -0.000001 && sum6 < 0.000001)//�����ȻΪ0��������
			{
				Y_max = i;
				goto logo;//�������whileѭ��
			}
			else//�������ִ��
			{
				i++;
			}
		}
	}
logo:
	Y_max = i;

	//�����и�
	Mat image_cut1 = temp(Rect(0, Y_min, col, Y_max - Y_min));
	img_next = image_cut1.clone();   //clone���������µ�ͼƬ 
}


//�ҳ�LED�����������x_min,x_max,y_min,y_max
void LED_contour(const Mat& _img, vector<Point> &myPoints, vector<int> &myRadius)
{
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	findContours(_img, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, Point());//������⣬���ĸ�����˵��ֻ�������Χ��������ֻ��⵽������������ÿһ��LED�е�ϸС�����ѱ���ʴ����������һ��Բ��
	for (int i = 0; i < contours.size(); i++)
	{
		int x_max = 0, y_max = 0, x_min = 999999, y_min = 999999;
		for (int j = 0; j < contours[i].size(); j++)
		{//minmaxLoc with mask i need u
			//contours[i].size()������ǵ�i���������������ص���
			//�����ҵ�������������i�����ڵ��������ص�
			//��ѭ������i����������ѭ������ÿ��������j�����ص㣻�ҵ��������������ҡ����Ϻ������ĸ���
			if (x_max < contours[i][j].x) x_max = contours[i][j].x;//contours �Ƕ�ά������Ԫ���ǵ��ࡣ
			if (y_max < contours[i][j].y) y_max = contours[i][j].y;//ֱ��������Ϊ��λ���������������ͼ���������ʡʱ��
			if (x_min > contours[i][j].x) x_min = contours[i][j].x;
			if (y_min > contours[i][j].y) y_min = contours[i][j].y;
		}
		myPoints.push_back(Point((x_max + x_min) / 2, (y_max + y_min) / 2));//���Բ��
		myRadius.push_back((x_max - x_min + y_max - y_min) / 4);//����뾶
	}

	circle(_img, myPoints[0], myRadius[0], Scalar(0, 0, 255), 2, 8, 0);
	namedWindow("src", 0);
	imshow("src", _img);
	waitKey(100);
}

void para_set(double& Center_X, double& Center_Y, double& D_LED, double& f, double& led_x, double& led_y, double& all_H)
{
	/*---------------����1-------------------*/
	// Center_X = 575;	//685;//674;//������Ķ�׼�����·�670;//640;//429;
	// Center_Y = 499;	//495;//447;//486;//480;//334.5;
	// D_LED = 180;	//LED�ƾߵ���ʵֱ��
	// f = 2.821511*312.5;	//���ݸ߶Ƚ����Ľ���(2.6389)//3.111;//����ͼƬ��С��ͬ,���в�ͬ�ĵ�Ч����(1280*960��Ӧ2.5)
	// led_x = 3000;
	// led_y = 800;
	// all_H = 2.61;

	/*---------------����2-------------------*/
	Center_X = 586.25;//583.9167;//587.5;//583.9167;//584.75;//653;//685;//674;//������Ķ�׼�����·�670;//640;//429;	��ת90��֮�������������Ϊ691
	Center_Y = 498;//497.04167;//499.25;//497.04167;//500;//498;//495;//447;//486;//480;//334.5;	��ת90��֮�������������Ϊ528
	D_LED = 180;//LED�ƾߵ���ʵֱ��
	f = 2.58808889*312.5;//2.5585778(�߶�2.73)���ݸ߶Ƚ����Ľ���(2.6389)//3.111;//����ͼƬ��С��ͬ,���в�ͬ�ĵ�Ч����(1280*960��Ӧ2.5)
	led_x = 0;
	led_y = 0;
	//all_H = 2.67;
    all_H = 2.09;
}


//roi������
//����ȷ��LED�ƴ�С
double get_similarity(Mat&a,Mat&b)
{
    if(a.rows!=b.rows || a.cols != b.cols ||a.channels()!=b.channels())
    {
        printf("not the same size!\n");
        return 0;
    }
    int samenum=0;
    for(int i=0;i<a.rows;i++)
    {
        for(int j=0;j<a.cols;j++)
        {
            if(a.at<uchar>(i,j)==b.at<uchar>(i,j))
            {
                samenum++;
            }
        }
    }
    double area=a.cols*a.rows;
    double similarity=(double)samenum/area;
    return similarity;
}//�Ƚ�ͬ����С������ͼͬ�����ظ���ռ��

Rect find_minrect(Mat frame,Rect selected,int thresh)
{
    int cols=frame.cols;
    int rows=frame.rows;
    if(selected.x<0||selected.x>cols)
    {
        return Rect(0,0,0,0);
    }
    if(selected.y<0||selected.y>rows)
    {
        return Rect(0,0,0,0);
    }
    int x_min,x_max,y_min,y_max;

    int i=selected.x;
    while(i<selected.x+selected.width)
    {
        int sum=0;
        for(int j=selected.y;j<selected.y+selected.height;j++)
        {
            int val=frame.at<uchar>(j,i);
            if(val>100)
            sum+=val;
        }
        if(sum>thresh)
        {
            //x_min=i;
            break;
        }
        else
        {
            i++;
        }
    }
    x_min=i;

    i=selected.x+selected.width-1;
    while(i>selected.x)
    {
        int sum=0;
        for(int j=selected.y;j<selected.y+selected.height;j++)
        {
            int val=frame.at<uchar>(j,i);
            if(val>100)
            sum+=val;
        }
        if(sum>thresh)
        {
            //x_max=i;
            break;
        }
        else
        {
            i--;
        }
    }
    x_max=i;

    int j=selected.y;
    while(j<selected.y+selected.height)
    {
        int sum=0;
        for(int i=selected.x;i<selected.x+selected.width;i++)
        {
            int val=frame.at<uchar>(j,i);
            if(val>100)
            sum+=val;
        }
        if(sum>thresh)
        {
            //y_min=j;
            break;
        }
        else
        {
            j++;
        }
    }
    y_min=j;

    j=selected.y+selected.height-1;
    while(j>selected.y)
    {
        int sum=0;
        for(int i=selected.x;i<selected.x+selected.width;i++)
        {
            int val=frame.at<uchar>(j,i);
            if(val>100)
            sum+=val;
        }
        if(sum>thresh)
        {
            //y_max=j;
            break;
        }
        else
        {
            j--;
        }
    }
    y_max=j;
    
    Rect minrect=Rect(x_min,y_min,max(x_max-x_min+1,0),max(y_max-y_min+1,0));
    return minrect;
}

Rect ls_LED_sample(Mat img, int& X_min, int& X_max, int& Y_min, int& Y_max, cv::Mat& imgNext,int sample)
{
    // ��xmin��xmax
    int row1 = img.rows;// ����
    int col1 = img.cols;// ��

    int j = 0;// ע���Ǵ�0��ʼ
    while (j < col1)// j�ĳ�ֵΪ1
    {
        double sum1 = 0.0;
        for (int i = 0;i < row1;i++)// ע��û�е��ں�
        {
            uchar* data1 = img.ptr<uchar>(i);// ptr<uchar>(i)[j]���ʵ�i�е�j�е�����
            sum1 = data1[j] + sum1;
        }// ����j�е�ÿһ�м���
        if (sum1>-0.000001 && sum1< 0.000001)// double���ͣ�����д==0
        {
            j+=sample;
        }
        else
        {
            break;// �������whileѭ��
        }

    }
    X_min = j;

    while (j < col1)// j�ĳ�ֵΪX_min
    {
        double sum1 = 0.0;
        for (int i = 0;i < row1;i++)
        {
            uchar* data1 = img.ptr<uchar>(i);// ptr<uchar>(i)[j]���ʵ�i�е�j�е�����
            sum1 = data1[j] + sum1;
        }// ����j�е�ÿһ��XXXXXX����
        if (sum1 != 0)
        {
            j+=sample;
        }
        else
        {
            break;// �������whileѭ��
        }
    }
    X_max = j;

    // �����и�
    Mat temp = img(Rect(X_min, 0, X_max - X_min, row1));

    // ��ymin��ymax
    int row = temp.rows;// ����
    int col = temp.cols;// ��

    int i = 0;
    while (i < row)// i�ĳ�ֵΪ1
    {
        double sum = 0.0;
        uchar* data = temp.ptr<uchar>(i);
        for (j = 0;j < col;j++)// ��ÿһ���е�ÿһ�����ؽ�����ӣ�ptr<uchar>(i)[j]���ʵ�i�е�j�е�����
        {
            sum = data[j] + sum;
        }// ���ջ�õ�i�е��к�
        if (sum>-0.000001 && sum < 0.000001)
        {
            i+=sample;
        }
        else
        {
            break;// �������whileѭ��
        }
    }
    Y_min = i;

    while (i < row)// i�ĳ�ֵΪ1
    {
        double sum = 0.0;
        uchar* data = temp.ptr<uchar>(i);
        for (j = 0;j < col;j++)// ��ÿһ���е�ÿһ�����ؽ�����ӣ�ptr<uchar>(i)[j]���ʵ�i�е�j�е�����
        {
            sum = data[j] + sum;
        }// ���ջ�õ�i�е��к�
        if (sum!=0 )
        {
            i+=sample;
        }
        else
        {
            break;// �������whileѭ��
        }
    }
    Y_max = i;

    // �����и�
    //imgNext = temp(Rect(0, Y_min, col, Y_max - Y_min)); // clone���������µ�ͼƬ
    Rect minorrect(X_min-sample,Y_min-sample,(X_max-X_min+2*sample),(Y_max-Y_min+2*sample));
    Rect ROI=find_minrect(img,minorrect,0);
    imgNext=img(ROI);
    return ROI;
}

//ʵ�ֶ�ͼ���ϸ��
void thinImage(Mat &srcimage)//��ͨ������ֵ�����ͼ��  
{
    using namespace std;

	vector<Point> deletelist1;
	int Zhangmude[9];
	int nl = srcimage.rows;
	int nc = srcimage.cols;
	while (true)
	{
		for (int j = 1; j < (nl - 1); j++)
		{
			uchar* data_last = srcimage.ptr<uchar>(j - 1);
			uchar* data = srcimage.ptr<uchar>(j);
			uchar* data_next = srcimage.ptr<uchar>(j + 1);
			for (int i = 1; i < (nc - 1); i++)
			{
				if (data[i] == 255)
				{
					Zhangmude[0] = 1;
					if (data_last[i] == 255) Zhangmude[1] = 1;
					else  Zhangmude[1] = 0;
					if (data_last[i + 1] == 255) Zhangmude[2] = 1;
					else  Zhangmude[2] = 0;
					if (data[i + 1] == 255) Zhangmude[3] = 1;
					else  Zhangmude[3] = 0;
					if (data_next[i + 1] == 255) Zhangmude[4] = 1;
					else  Zhangmude[4] = 0;
					if (data_next[i] == 255) Zhangmude[5] = 1;
					else  Zhangmude[5] = 0;
					if (data_next[i - 1] == 255) Zhangmude[6] = 1;
					else  Zhangmude[6] = 0;
					if (data[i - 1] == 255) Zhangmude[7] = 1;
					else  Zhangmude[7] = 0;
					if (data_last[i - 1] == 255) Zhangmude[8] = 1;
					else  Zhangmude[8] = 0;
					int whitepointtotal = 0;
					for (int k = 1; k < 9; k++)
					{
						whitepointtotal = whitepointtotal + Zhangmude[k];
					}
					if ((whitepointtotal >= 2) && (whitepointtotal <= 6))
					{
						int ap = 0;
						if ((Zhangmude[1] == 0) && (Zhangmude[2] == 1)) ap++;
						if ((Zhangmude[2] == 0) && (Zhangmude[3] == 1)) ap++;
						if ((Zhangmude[3] == 0) && (Zhangmude[4] == 1)) ap++;
						if ((Zhangmude[4] == 0) && (Zhangmude[5] == 1)) ap++;
						if ((Zhangmude[5] == 0) && (Zhangmude[6] == 1)) ap++;
						if ((Zhangmude[6] == 0) && (Zhangmude[7] == 1)) ap++;
						if ((Zhangmude[7] == 0) && (Zhangmude[8] == 1)) ap++;
						if ((Zhangmude[8] == 0) && (Zhangmude[1] == 1)) ap++;
						if (ap == 1)
						{
							if ((Zhangmude[1] * Zhangmude[7] * Zhangmude[5] == 0) && (Zhangmude[3] * Zhangmude[5] * Zhangmude[7] == 0))
							{
								deletelist1.push_back(Point(i, j));
							}
						}
					}
				}
			}
		}
		if (deletelist1.size() == 0) break;
		for (size_t i = 0; i < deletelist1.size(); i++)
		{
			Point tem;
			tem = deletelist1[i];
			uchar* data = srcimage.ptr<uchar>(tem.y);
			data[tem.x] = 0;
		}
		deletelist1.clear();

		for (int j = 1; j < (nl - 1); j++)
		{
			uchar* data_last = srcimage.ptr<uchar>(j - 1);
			uchar* data = srcimage.ptr<uchar>(j);
			uchar* data_next = srcimage.ptr<uchar>(j + 1);
			for (int i = 1; i < (nc - 1); i++)
			{
				if (data[i] == 255)
				{
					Zhangmude[0] = 1;
					if (data_last[i] == 255) Zhangmude[1] = 1;
					else  Zhangmude[1] = 0;
					if (data_last[i + 1] == 255) Zhangmude[2] = 1;
					else  Zhangmude[2] = 0;
					if (data[i + 1] == 255) Zhangmude[3] = 1;
					else  Zhangmude[3] = 0;
					if (data_next[i + 1] == 255) Zhangmude[4] = 1;
					else  Zhangmude[4] = 0;
					if (data_next[i] == 255) Zhangmude[5] = 1;
					else  Zhangmude[5] = 0;
					if (data_next[i - 1] == 255) Zhangmude[6] = 1;
					else  Zhangmude[6] = 0;
					if (data[i - 1] == 255) Zhangmude[7] = 1;
					else  Zhangmude[7] = 0;
					if (data_last[i - 1] == 255) Zhangmude[8] = 1;
					else  Zhangmude[8] = 0;
					int whitepointtotal = 0;
					for (int k = 1; k < 9; k++)
					{
						whitepointtotal = whitepointtotal + Zhangmude[k];
					}
					if ((whitepointtotal >= 2) && (whitepointtotal <= 6))
					{
						int ap = 0;
						if ((Zhangmude[1] == 0) && (Zhangmude[2] == 1)) ap++;
						if ((Zhangmude[2] == 0) && (Zhangmude[3] == 1)) ap++;
						if ((Zhangmude[3] == 0) && (Zhangmude[4] == 1)) ap++;
						if ((Zhangmude[4] == 0) && (Zhangmude[5] == 1)) ap++;
						if ((Zhangmude[5] == 0) && (Zhangmude[6] == 1)) ap++;
						if ((Zhangmude[6] == 0) && (Zhangmude[7] == 1)) ap++;
						if ((Zhangmude[7] == 0) && (Zhangmude[8] == 1)) ap++;
						if ((Zhangmude[8] == 0) && (Zhangmude[1] == 1)) ap++;
						if (ap == 1)
						{
							if ((Zhangmude[1] * Zhangmude[3] * Zhangmude[5] == 0) && (Zhangmude[3] * Zhangmude[1] * Zhangmude[7] == 0))
							{
								deletelist1.push_back(Point(i, j));
							}
						}
					}
				}
			}
		}
		if (deletelist1.size() == 0) break;
		for (size_t i = 0; i < deletelist1.size(); i++)
		{
			Point tem;
			tem = deletelist1[i];
			uchar* data = srcimage.ptr<uchar>(tem.y);
			data[tem.x] = 0;
		}
		deletelist1.clear();
	}
}


//ID����ʵ�ֵ����׺���-----------------------------------------------------------------------------

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
cv::Mat_<float>::Mat ImagePreProcessing(cv::Mat imgLED, int backgroundThreshold,int &backgroundCompensation) {
    // cv::cvtColor(imgLED,imgLED,cv::COLOR_BGR2GRAY);
    // ������ģ�����ھ�ֵ���㡣
    cv::Mat maskOfimgLED;
    cv::threshold(imgLED, maskOfimgLED, backgroundThreshold, 255, cv::THRESH_BINARY);

    // ������ʾ�������뺯������
    //cv::imshow("maskOfimgLED", maskOfimgLED);

    imgLED.convertTo(imgLED, CV_32F);

    // ȡ��ֵ����ֵ�ľ�ֵ���߼���������ģ�����е���ֵΪ0����1��Ϊ1�ĵط��������image������Ԫ�صľ�ֵ��Ϊ0�ĵط���������
    cv::Mat meanRowOfPxiel(imgLED.col(0).t().size(),CV_32FC1);
    int meanOfPxielRow;  //.val[0]��ʾ��һ��ͨ���ľ�ֵ
    cv::MatIterator_<float> it, end;
    int RowOfimgLED = 0;
    vector<int> Compensation;
    for( it = meanRowOfPxiel.begin<float>(), end = meanRowOfPxiel.end<float>(); it != end; it++) {
        meanOfPxielRow = cv::mean(imgLED.row(RowOfimgLED), maskOfimgLED.row(RowOfimgLED)).val[0];
        if(meanOfPxielRow<240)
        Compensation.push_back(meanOfPxielRow);
        RowOfimgLED ++;
        //std::cout << "ֵ = "<< meanOfPxielRow <<std::endl;
        *it = meanOfPxielRow;
    }
    // std::cout << "��ֵǰ = "<< meanRowOfPxiel <<std::endl;
    // std::cout << "meanRowOfPxiel.rows = "<< meanRowOfPxiel.cols <<std::endl;

    sort(Compensation.begin(),Compensation.end());
    //cout<<"Compensation.at(Compensation.size()-1)"<<Compensation.at(Compensation.size()-1)<<endl;
    backgroundCompensation=(int)(0.5*(int)Compensation.at(Compensation.size()-4))-28;//һ������,̫��0��Сһ��
    // cout<<"backgroundCompensation"<<backgroundCompensation<<endl;
    
    // ��ֵ 
    // double chazhi=1;
    // cv::resize(meanRowOfPxiel, meanRowOfPxiel, cv::Size(meanRowOfPxiel.cols*chazhi, 1), cv::INTER_CUBIC);
    // std::cout << "��ֵ = "<< meanRowOfPxiel <<std::endl;

    // ������ʾ�������뺯������
    cv::Mat meanShow(meanRowOfPxiel.size(),CV_32FC1);
    cv::resize(meanRowOfPxiel, meanShow, cv::Size(meanRowOfPxiel.cols, 100), cv::INTER_CUBIC);
    meanShow.convertTo(meanShow, CV_8U);
    cv::imshow("meanRowOfPxiel", meanShow);
    waitKey(30);
    // cvWaitKey(0);

    return meanRowOfPxiel;
 }

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
cv::Mat matShift(cv::Mat frame, int shiftCol, int shiftRow) {
    using namespace cv;
    // std::cout << "ƽ��ǰ = "<< frame <<std::endl;
    cv::Mat out = cv::Mat::zeros(frame.size(), frame.type());
    cv::Rect source = cv::Rect(max(0,-shiftCol),max(0,-shiftRow), frame.cols-abs(shiftCol),frame.rows-abs(shiftRow));
    cv::Rect target = cv::Rect(max(0,shiftCol),max(0,shiftRow),frame.cols-abs(shiftCol),frame.rows-abs(shiftRow));
    frame(source).copyTo(out(target));
    // std::cout << "ƽ�ƺ� = "<< out <<std::endl;
    return out;
}


/* -------------------�� Ѱ�Ҳ��岨�ȴ��� ��----------------
���ܣ�
    Ѱ��LED�о�ֵ�Ĳ��岨��λ������
�����������ͣ�
    cv::Mat imgRow �����о���ת��Ϊ�о���������У�ע�⣬����Ҫfloat���ͣ�
        ��ΪҪΪ��ƽ����Ч������Ҫ���߾��ȵ���������
����������ͣ�
    cv::Mat NonZeroLocations ���岨�����ڵ�����
------------------------------------------------------------*/
cv::Mat LEDMeanRowCrestsTroughs(const cv::Mat_<float>::Mat imgRow, int BlurSize) {
    // Ѱ�����еļ���Сֵ��Ҳ���ǲ���Ͳ��ȣ�
    cv::Mat imgRowBlur=imgRow;

    // ƽ������ֵ�˲�������������������С�Ķ���
    // cv::GaussianBlur(imgRow, imgRowBlur, cv::Size(21,1), 0, 0 );
    // ����2048ͼ��BlurSizeȡ15
    //cv::blur(imgRow, imgRowBlur, cv::Size(BlurSize,1));
    // GaussianBlur(imgRow,imgRowBlur,cv::Size(BlurSize,1),0,0);//youquanzhong better than blur
    //Blursize need to be odd interger
    //std::cout << "ƽ�� = "<< imgRowBlur <<std::endl;

    // ������ʾ�������뺯������
    //cv::Mat imgRowBlurShow;
    //cv::resize(imgRowBlur, imgRowBlurShow, cv::Size(imgRowBlur.cols, 100), cv::INTER_CUBIC);
    //imgRowBlurShow.convertTo(imgRowBlurShow, CV_8U);
    //cv::imshow("imgRowBlurShow", imgRowBlurShow);

    cv::Mat imgRowRightShift = matShift(imgRowBlur, 1, 0);
    // std::cout << "���� = "<< imgRowRightShift <<std::endl;

    cv::Mat difference;
    cv::subtract(imgRowBlur, imgRowRightShift, difference);
    // std::cout << "�����ֵ = "<< difference <<std::endl;

    cv::threshold(difference, difference, 0, 255, cv::THRESH_BINARY);
    difference.convertTo(difference, CV_8U);
    // std::cout << "��ֵ����ֵ = "<< difference <<std::endl;

    cv::Mat differenceLeftShift = matShift(difference, -1, 0);
    // std::cout << "��ֵ����ֵ���� = "<< differenceLeftShift <<std::endl;

    cv::Mat CrestsTroughs;
    cv::bitwise_xor(difference, differenceLeftShift, CrestsTroughs);

    // Ϊĩβ����һ����ֵ��ǣ��Ա�����Ķ���ֵ��������߼�����Ҫ©�����һ������
    CrestsTroughs = CrestsTroughs.t();
    cv::Mat endRow = (cv::Mat_<uchar>(1, 1) << 255);
    CrestsTroughs.push_back(endRow);
    CrestsTroughs = CrestsTroughs.t();
    // std::cout << "��� = "<< CrestsTroughs <<std::endl;

    // ������ʾ�������뺯������
    //cv::Mat CrestsTroughsShow;
    //cv::resize(CrestsTroughs, CrestsTroughsShow, cv::Size(CrestsTroughs.cols, 100), cv::INTER_CUBIC);
    //cv::imshow("CrestsTroughsShow", CrestsTroughsShow);

    // Ѱ�Ҳ����ط�0Ԫ�ص�λ�ã���Ϊ���岨��
    cv::Mat NonZeroLocations;
    cv::findNonZero(CrestsTroughs, NonZeroLocations);
    //std::cout << "Non-Zero Locations = " << NonZeroLocations << std::endl;

    return NonZeroLocations;
}

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
cv::Mat LEDMeanRowThreshold(cv::Mat imgRow, cv::Mat NonZeroLocations, int backgroundThreshold, int &backgroundCompensation) 
{
    cv::Mat ROI,ROIthresh;
    std::vector<int> Thresh {};
    cv::Rect roiRange;
    // int roiThreshold, roiStart, roiEnd;
    int roiStart, roiEnd;
    double roiThreshold;
    double minVal, maxVal;
    roiStart = 0;
    for (int i = 0; i < NonZeroLocations.size().height; i++)
	{
        // �����ȡNonZeroLocations���ݿ�
        cv::Point pnt = NonZeroLocations.at<cv::Point>(i);
        roiEnd = pnt.x;
        // std::cout << "roiEnd = " << roiEnd << std::endl;

        // ���ݶ�ȡ����������ζ�ֵ����ROI����
        //ROIthresh=imgRow(Rect(roiStart, 0, (min(roiEnd+1,imgRow.cols) - roiStart), 1));
        roiRange = cv::Rect(roiStart, 0, (roiEnd - roiStart), 1);
        ROI = imgRow(roiRange);
        // std::cout << "ROI_1 = "<< ROI <<std::endl;

        // ��ȡÿ������������Сֵ
        cv::minMaxIdx(ROI, &minVal, &maxVal);
        
        for(int ii=roiStart;ii<roiEnd;ii++)
        {
           Thresh.push_back(((minVal + maxVal) / 2));
        }

        // Ϊ�˲��������ƴ���СֵΪ0����ɵĶ�ֵ����ֵƫ�ͣ����ڴ˽��е���
        // ������СֵӦ��ָ��Ϊ��ֵ�����ݹ۲�������ã�һ���ָ�������Ǵ���ȥ������
        // ����ֵ����ImagePreProcessing�����е�backgroundThreshold��������
        // С�ڻ��������δ��ȥ���������������Сֵ�����籾������backgroundThreshold
        // Ϊ20������δ��ȥ���������������Сֵ��90���ϣ����ڴ˾��������ȷ��ȡ40
        if (minVal < backgroundThreshold)
		{
            minVal = backgroundCompensation;
            if (maxVal < minVal)
			{
                maxVal = minVal;
            }
        }

        //std::cout << "minVal = "<< minVal <<std::endl;
        //std::cout << "maxVal = "<< maxVal <<std::endl;

       

        // Ϊ��Ӧ�Կ������м�û�б�ƽ�����������ֻ�����ж����伫ֵ���ڴ���ֵʱ�Ż�ִ�ж�ֵ��
        if ((maxVal - minVal) > 10) 
		{
            // roiThreshold = ((minVal + maxVal) / 2);
            roiThreshold = minVal + 0.45*(maxVal-minVal);///�ֲ���ֵ
            // std::cout << "roiThreshold = "<< roiThreshold <<std::endl;

            cv::threshold(ROI, ROI, roiThreshold, 255, cv::THRESH_BINARY);
            // std::cout << "ROI_2 = "<< ROI <<std::endl;
        }

        //thresh record
	
        // if((maxVal - minVal) > 20)
        // {
        //     for(int ii=roiStart;ii<roiEnd;ii++)
        //     {
        //         Thresh.push_back(roiThreshold);
        //     }    
        // }
        // else
        // {
        //     for(int ii=roiStart;ii<roiEnd;ii++)
        //     {
        //         Thresh.push_back(backgroundCompensation);
        //     }
        // }
        

        // // ���������ֵ��������ƻ�ԭͼ��
        // ROI.copyTo(imgRow(roiRange));

        // roiStart = roiEnd;
    }
    
    // Thresh.push_back(Thresh.at(Thresh.size()-1));
	
    //std::cout<<"Thresh="<<cv::Mat(Thresh, true).t()<<std::endl;

    // ������ѭ����û�д���Ĳ���ͳһ������Ϊû��ƽ���������ֻλ�ڿ������м䣬
    // ��˶�Ӧ������������δ����ֵ���Ƚϴ󣬴���ȡ�м�ֵ���ж�ֵ�����ɣ��������
    // ��ֵ���Ĳ��������Ѿ���0��255���������Ӱ��
    cv::threshold(imgRow, imgRow, backgroundCompensation, 255, cv::THRESH_BINARY);
    imgRow.convertTo(imgRow, CV_8U);

    // ������ʾ�������뺯������
    cv::Mat imgRowShow;
    cv::resize(imgRow, imgRowShow, cv::Size(imgRow.cols, 100), cv::INTER_CUBIC);
    cv::imshow("LEDMeanRowThreshold", imgRowShow);
    waitKey(30);
    // cvWaitKey(0);
   //std::cout << "imgRow = "<< imgRow <<std::endl;

    return imgRow;
}


cv::Mat convertPxielRowToBit(cv::Mat row,int continuousnum) {
    // row =  (cv::Mat_<uchar>(1, 18) << 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0); // ��������
    // cout << "row = "<< row <<endl;

    // ���м������ؼ���������ͬ���أ���ת�壬����001100001111ת��Ϊ2244
    std::vector<int> SamePxielCount {};
    int pxielCount = 0;
    cv::MatIterator_<uchar> start, it, end;
    for( it = row.begin<uchar>(), end = row.end<uchar>(), start = it; it != end; it++) {
        if (*start != *it) {
            // std::cout << "pxielCount = "<< pxielCount <<std::endl;
            SamePxielCount.push_back(pxielCount);
            pxielCount = 1;
            start = it;
        } else {
            pxielCount++;
        }
    }
    // �����һ���������⴦����Ϊ��������ǰ��������ж�
    pxielCount++;
    // std::cout << "pxielCount = "<< pxielCount <<std::endl;
    SamePxielCount.push_back(pxielCount);

    double bit;
    // ��SamePxielCount��Ա���������ų���С���쳣ֵ
    std::vector<int> SamePxielCountCopy = SamePxielCount;

    // ��ת�������������
    std::sort(SamePxielCountCopy.begin(), SamePxielCountCopy.end());
    int num=SamePxielCountCopy.size();
    if(num<=3)
    {
        return cv::Mat_<uchar>(1,1)<<0;
    }
    //-----------������-------num���Ķ��ĸ�
    int totalbit=SamePxielCountCopy.at(num-1);
    int totalnum=1;
    if(abs(SamePxielCountCopy.at(num-2)-SamePxielCountCopy.at(num-1))<=1){
        totalbit+=SamePxielCountCopy.at(num-2);
        totalnum++;
    }
    if(abs(SamePxielCountCopy.at(num-3)-SamePxielCountCopy.at(num-1))<=1){
        totalbit+=SamePxielCountCopy.at(num-3);
        totalnum++;
    }
    //-----------������-------num���Ķ��ĸ�

    //bit=(double)(SamePxielCountCopy.at(num-2)+SamePxielCountCopy.at(num-3))/(2*continuousnum);
    // bit=(double)(SamePxielCountCopy.at(num-1)+SamePxielCountCopy.at(num-2))/(2*continuousnum);
    bit=((double)totalbit/totalnum)/continuousnum;
    /*int num=0;
    // ��ȡת�������еĵ���С��ֵ����Ϊ��С��ǰ����ֵ�������ǲ��������ƣ�����ӹ֮����ȡ���ֵ
    for(int ii=0;ii<SamePxielCountCopy.size();ii++)
    {
        if(SamePxielCountCopy.at(ii)>20)
            num++;
        if(num==SamePxielCountCopy.size()/10+15)
        {
            bit = SamePxielCountCopy.at(ii);
            break;
        }
    }*/

    //std::cout <<"SamePxielCountCopy.size"<<SamePxielCountCopy.size()<<endl;
    // std::cout << "bit = "<< bit <<std::endl;

    // // ��ȡת�������е���Сֵ����Ϊһ���ֽ�����Ӧ������
    // bit = *std::min_element(SamePxielCount.begin(), SamePxielCount.end());
    // bit = 10;
    // std::cout << "bit = "<< bit <<std::endl;

    // ��ת��������תΪ����λ����
    std::vector<int> BitVector {};
    pxielCount = 0;
    int sameBitRange,addnum;
    double decimal;

    // ʶ��ͼ���һ�����صĸߵ͵�ƽ��ת��Ϊ����λ���ߵ�ƽ��λ1
    int pxielFlag;
    if (*row.begin<uchar>() == 255 || *row.begin<uchar>() == 1) {
        pxielFlag = 1;
    } else {
        pxielFlag = *row.begin<uchar>();  // ��ȡ��һ������
    }

    for (pxielCount = 0; pxielCount < SamePxielCount.size(); pxielCount++) {
        sameBitRange = round(static_cast<double>(SamePxielCount.at(pxielCount)) / bit);
        decimal=(static_cast<double>(SamePxielCount.at(pxielCount)) / bit)-sameBitRange;
        if(decimal>0.4&&decimal<0.6)
        {
            if(pxielCount==0)
            {
                addnum=round((static_cast<double>(SamePxielCount.at(pxielCount)) / bit)-(int)(static_cast<double>(SamePxielCount.at(pxielCount)) / bit)
            +(static_cast<double>(SamePxielCount.at(pxielCount+1)) / bit)-(int)(static_cast<double>(SamePxielCount.at(pxielCount+1)) / bit))-
            round((static_cast<double>(SamePxielCount.at(pxielCount+1)) / bit)-(int)(static_cast<double>(SamePxielCount.at(pxielCount+1)) / bit));
            }
            else if(pxielCount==SamePxielCount.size()-1)
            {
                addnum=round((static_cast<double>(SamePxielCount.at(pxielCount-1)) / bit)-(int)(static_cast<double>(SamePxielCount.at(pxielCount-1)) / bit)
            +(static_cast<double>(SamePxielCount.at(pxielCount)) / bit)-(int)(static_cast<double>(SamePxielCount.at(pxielCount)) / bit))-
            round((static_cast<double>(SamePxielCount.at(pxielCount-1)) / bit)-(int)(static_cast<double>(SamePxielCount.at(pxielCount-1)) / bit));
            }
            else
            {
                addnum=round((static_cast<double>(SamePxielCount.at(pxielCount-1)) / bit)-(int)(static_cast<double>(SamePxielCount.at(pxielCount-1)) / bit)
            +(static_cast<double>(SamePxielCount.at(pxielCount)) / bit)-(int)(static_cast<double>(SamePxielCount.at(pxielCount)) / bit)
            +(static_cast<double>(SamePxielCount.at(pxielCount+1)) / bit)-(int)(static_cast<double>(SamePxielCount.at(pxielCount+1)) / bit))-
            round((static_cast<double>(SamePxielCount.at(pxielCount-1)) / bit)-(int)(static_cast<double>(SamePxielCount.at(pxielCount-1)) / bit)
            +(static_cast<double>(SamePxielCount.at(pxielCount+1)) / bit)-(int)(static_cast<double>(SamePxielCount.at(pxielCount+1)) / bit));
            }
            sameBitRange=(int)(static_cast<double>(SamePxielCount.at(pxielCount)) / bit)+addnum;
        }
        if(sameBitRange>continuousnum)
        {
            sameBitRange=continuousnum;
        }
        for (int bitCount = 0; bitCount < sameBitRange; bitCount++) {
            BitVector.push_back(pxielFlag);
            // ��Bitĩβ����sameBitRaneg���������أ�������ֵ��pxielFlag����
        }
        pxielFlag = !pxielFlag;
        // һ��������ɺ�����ر�־ȡ������Ϊת�����������������Աָ��������λ���Ƿ���
    }

    // cout << "Bit = "<< Mat(BitVector, true).t() <<endl;
    return  cv::Mat(BitVector, true).t();  // �����ĵ�������һ��n�У����Խ���ת��
}

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
cv::Mat getMsgDate(cv::Mat imgRow, cv::Mat headerStamp,int idlength) {
// cv::Mat getMsgDate(const cv::Mat imageLED) {
    // https://stackoverflow.com/questions/32737420/multiple-results-in-opencvsharp3-matchtemplate
    // ����ȡ������λ������Ϊ��ƥ�����
    // // ͼ��Ԥ������ȡͼ��ÿ�о�ֵ����ֵ�����о������
    // imageLED = ImagePreProcessing(imageLED, 20);

    // // ����ȡ��ͼ��ÿ�о�ֵ���ж�ֵ�������о������
    // cv::Mat row =  LEDMeanRowThreshold(imageLED);

    // std::cout << "imgRow = "<< imgRow <<std::endl;
    // cv::Mat ref = convertPxielRowToBitBySample(imgRow);
    cv::Mat ref = imgRow;
    if(ref.cols==1)
    {
        return cv::Mat_<uchar>(1,6)<<0,1,1,1,1,0;
    }
    ref.convertTo(ref, CV_8U);
    // std::cout << "Bit = "<< ref <<std::endl;

    // cv::cvtColor(headerStamp,headerStamp,cv::COLOR_BGR2GRAY);
    // cv::Mat headerStamp = (cv::Mat_<uchar>(1, 3) << 0, 1, 0);

    // ��ģ��ƥ��Ѱ������λ�е���Ϣͷ
    std::vector<int> HeaderStamp {};
    cv::Mat res(ref.rows - headerStamp.rows + 1, ref.cols-headerStamp.cols + 1, CV_8U);
    cv::matchTemplate(ref, headerStamp, res, CV_TM_CCOEFF_NORMED);
    // cv::matchTemplate(ref, headerStamp, res, TM_SQDIFF);
    cv::threshold(res, res, 0.8, 1., CV_THRESH_TOZERO);
    // cout<<"res:"<<res<<endl;

    while (true) {
        double minval, maxval, threshold = 0.8;
        cv::Point minloc, maxloc;
        cv::minMaxLoc(res, &minval, &maxval, &minloc, &maxloc);

        if (maxval >= threshold) {
            HeaderStamp.push_back(maxloc.x);
            // ��ˮ����Ѿ�ʶ�𵽵�����
            cv::floodFill(res, maxloc, cv::Scalar(0), 0, cv::Scalar(.1), cv::Scalar(1.));
        } else {
            break;
        }
    }

    // ��HeaderStamp��Ա�������򣬷�����ܳ���˳�򵹹ҽ���Ӱ������ʶ��
    std::sort(HeaderStamp.begin(), HeaderStamp.end());
    //-------������---��ӽ�ȥ��һ��
    int oneheader=0;
    if(HeaderStamp.size()==1)
    {
        oneheader=1;
    }
    else
    {
        oneheader=0;
    }
    //-------������---��ӽ�ȥ��һ��

    // cout<<"Headerstamp="<<Mat(HeaderStamp,true)<<endl;

    //-----������-----�������------
    // ��������Ϣͷ֮����ȡROI���򣬼�λID��Ϣ
    int ptrHeaderStamp = 0;
    cv::Mat IDdata = Mat::zeros(1, idlength, CV_8UC1);
    cv::Mat id,idfront,idback;
    getROI:
    if(oneheader==1)
    {
        if(HeaderStamp.at(ptrHeaderStamp)>2)
        {
            idfront=ref.colRange(2,HeaderStamp.at(ptrHeaderStamp));
        }
        if(HeaderStamp.at(ptrHeaderStamp)+headerStamp.size().width<ref.cols-2)
        {
            idback=ref.colRange(HeaderStamp.at(ptrHeaderStamp) + headerStamp.size().width,ref.cols-2);
        }
        
        if(idfront.cols>=idlength)
        {
            IDdata=idfront(Rect(idfront.cols-idlength,0,idlength,1));
        }
        else if(idback.cols>=idlength)
        {
            IDdata=idback(Rect(0,0,idlength,1));
        }
        else if(idfront.cols+idback.cols<idlength)
        {
            std::cout << "��LEDͼ��ID�޷�ʶ��" << std::endl;
            IDdata= cv::Mat_<uchar>(1, 1) << 0;
        }
        else
        {
            Mat colfirst=IDdata(Rect(0,0,idback.cols,1));
            idback.copyTo(colfirst);
            Mat colsecond=IDdata(Rect(IDdata.cols-idfront.cols,0,idfront.cols,1));
            idfront.copyTo(colsecond);
        }
    }
    else
    {
        try {
            // cout<<"length:"<<headerStamp.size().width<<endl;
            id=ref.colRange(HeaderStamp.at(ptrHeaderStamp) + headerStamp.size().width,
                            HeaderStamp.at(ptrHeaderStamp + 1));
            cout<<"id="<<id<<endl;
            if(id.cols==idlength)
            IDdata=id;
            else
            {
                ptrHeaderStamp++;
                goto getROI;
            }
        } catch ( cv::Exception& e ) {  // �쳣����
            ptrHeaderStamp++;
            // const char* err_msg = e.what();
            // std::cout << "exception caught: " << err_msg << std::endl;
            std::cout << "Normal! Extract again!" << std::endl;
            goto getROI;
        } catch ( std::out_of_range& e ) {  // �쳣����
            std::cout << "cannot recongize ID" << std::endl;
            IDdata= (cv::Mat_<uchar>(1, 3) << 1,1,1);
        }
    }
    return IDdata;
    //-----������-----�������------
}



/*---------------------------��count stripes��----------------------------------------
function:
----------------------------------------------------------------------------------------*/
int subregions(cv::Mat row)
{
    std::vector<int> SamePxielCount {};
    int pxielCount = 0;
    cv::MatIterator_<uchar> start, it, end;
    for( it = row.begin<uchar>(), end = row.end<uchar>(), start = it; it != end; it++) {
        if (*start != *it) {
            // std::cout << "pxielCount = "<< pxielCount <<std::endl;
            SamePxielCount.push_back(pxielCount);
            pxielCount = 1;
            start = it;
        } else {
            pxielCount++;
        }
    }
    // �����һ���������⴦����Ϊ��������ǰ��������ж�
    pxielCount++;
    // std::cout << "pxielCount = "<< pxielCount <<std::endl;
    SamePxielCount.push_back(pxielCount);

    double bit;
    // ��SamePxielCount��Ա���������ų���С���쳣ֵ
    std::vector<int> SamePxielCountCopy = SamePxielCount;

    // ��ת�������������
    std::sort(SamePxielCountCopy.begin(), SamePxielCountCopy.end());
    int num=SamePxielCountCopy.size();
    return num;
}

int countstripes(cv::Mat imageLED, int backgroundThreshold,int &backgroundCompensation,int Blursize)
{
    imageLED = ImagePreProcessing(imageLED,backgroundThreshold,backgroundCompensation);

    // ��ȡ���岨��
    cv::Mat NonZeroLocations = LEDMeanRowCrestsTroughs(imageLED, Blursize);

    cv::Mat imgRow =  LEDMeanRowThreshold(imageLED, NonZeroLocations,backgroundThreshold,backgroundCompensation);
    int stripenum=subregions(imgRow);
    return stripenum;
}


cv::Mat MsgProcess(cv::Mat imageLED, cv::Mat headerStamp,int backgroundThreshold,int &backgroundCompensation,int Blursize,int continuousnum,int idlength) {
    // ͼ��Ԥ������ȡͼ��ÿ�о�ֵ����ֵ�����о������
    // GaussianBlur(imageLED,imageLED,Size(45,1));
    // GaussianBlur(imageLED,imageLED,cv::Size(25,1),0,0);
    imageLED = ImagePreProcessing(imageLED,backgroundThreshold,backgroundCompensation);

    // ��ȡ���岨��
    // ����2048ͼ��BlurSizeȡ15
    cv::Mat NonZeroLocations = LEDMeanRowCrestsTroughs(imageLED, Blursize);

    // ����ȡ��ͼ��ÿ�о�ֵ���ж�ֵ�������о������
    //     Ϊ�˲��������ƴ���СֵΪ0����ɵĶ�ֵ����ֵƫ�ͣ����ڴ˽��е��ڡ�
    // ������СֵӦ��ָ��Ϊ��ֵ�����ݹ۲�������ã�һ���ָ�������Ǵ���ȥ������
    // ����ֵ����ImagePreProcessing�����е�backgroundThreshold��������
    // С�ڻ��������δ��ȥ���������������Сֵ�����籾������backgroundThreshold
    // Ϊ20������δ��ȥ���������������Сֵ��90���ϣ����ڴ˾��������ȷ��ȡ40
    cv::Mat imgRow =  LEDMeanRowThreshold(imageLED, NonZeroLocations,backgroundThreshold,backgroundCompensation);

    // �Բ�������ô�ƥ����������
    // cv::Mat ref = convertPxielRowToBitBySample(imgRow);

    // ��bit��ȷ������ƥ����������
    cv::Mat ref = convertPxielRowToBit(imgRow,continuousnum);
    
    // ����ʶ�������Ϊģ��ƥ���㷨�Ĵ�ƥ��ģ��
    cv::Mat msgDate = getMsgDate(ref, headerStamp,idlength);
    // cout<<"backgroundCompensation"<<backgroundCompensation<<endl;

    return msgDate;
}


