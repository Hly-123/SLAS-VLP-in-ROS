//-----------------------------------��ͷ�ļ��������֡�------------------------------ 
//      ����������������������ͷ�ļ�
//----------------------------------------------------------------------------------------------  
#include<single_led/imgProcess.hpp>


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
	Center_X = 584.75;//586.9;//593.5;//��584.5��602.5)//685;//674;//������Ķ�׼�����·�670;//640;//429;
	Center_Y = 500;//494.8;//489;//��477~501)//495;//447;//486;//480;//334.5;
	D_LED = 180;//LED�ƾߵ���ʵֱ��
	f = 2.5585778*312.5;//2.604267//2.56978//2.61333���ݸ߶Ƚ����Ľ���(2.6389)//3.111;//����ͼƬ��С��ͬ,���в�ͬ�ĵ�Ч����(1280*960��Ӧ2.5)
	led_x = 3000;
	led_y = 800;
	all_H = 2.73;

	/*---------------����2-------------------*/
	// Center_X = 653;//685;//674;//������Ķ�׼�����·�670;//640;//429;
	// Center_Y = 498;//495;//447;//486;//480;//334.5;
	// D_LED = 103;//LED�ƾߵ���ʵֱ��
	// f = 2.596*312.5;//���ݸ߶Ƚ����Ľ���(2.6389)//3.111;//����ͼƬ��С��ͬ,���в�ͬ�ĵ�Ч����(1280*960��Ӧ2.5)
	// led_x = 0;
	// led_y = 0;
	// all_H = 1.57;
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

