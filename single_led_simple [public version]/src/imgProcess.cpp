//-----------------------------------【头文件包含部分】------------------------------ 
//      描述：包含程序所依赖的头文件
//----------------------------------------------------------------------------------------------  
#include<single_led/imgProcess.hpp>


//-----------------------------------【图像处理函数】---------------------------------


//OpenCV中threshold自动阈值，类似matlab中的graythresh。获得自动阈值，用于图像二值化
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


//分割出LED区域，求出x_min,x_max,y_min,y_max
void ls_LED(const Mat& _img, int& X_min, int& X_max, int& Y_min, int& Y_max, Mat& img_next)
{
	Mat temp1 = _img.clone();

	//求xmin与xmax
	int row1 = temp1.rows;//行数
	int col1 = temp1.cols;//列
	int j = 0;//注意是从0开始
	while (j < col1)//j的初值为1
	{
		double sum1 = 0.0;
		for (int i = 0;i < row1;i++)//注意没有等于号
		{
			uchar* data1 = temp1.ptr<uchar>(i);//ptr<uchar>(i)[j]访问第i行第j列的像素
			sum1 = data1[j] + sum1;
		}//将第j列的每一行加完
		if (sum1 > -0.000001 && sum1 < 0.000001)//double类型，不能写==0
		{
			j++;
		}
		else
		{
			break;//跳出这个while循环
		}

	}
	X_min = j;

	while (j < col1)//j的初值为X_min 
	{
		double sum1 = 0.0;
		for (int i = 0;i < row1;i++)
		{
			uchar* data1 = temp1.ptr<uchar>(i);//ptr<uchar>(i)[j]访问第i行第j列的像素
			sum1 = data1[j] + sum1;
		}//将第j列的每一行XXXXXX加完
		if (sum1 != 0)
		{
			j++;
		}
		else
		{
			break;//跳出这个while循环
		}
	}
	X_max = j;

	//进行切割
	Mat image_cut = temp1(Rect(X_min, 0, X_max - X_min, row1));
	Mat temp = image_cut.clone();



	//求ymin与ymax
	int row = temp.rows;//行数
	int col = temp.cols;//列
	int i = 0;
	while (i < row)//i的初值为1
	{
		double sum = 0.0;
		uchar* data = temp.ptr<uchar>(i);
		for (j = 0;j < col;j++)//对每一行中的每一列像素进行相加，ptr<uchar>(i)[j]访问第i行第j列的像素
		{
			sum = data[j] + sum;
		}//最终获得第i行的列和
		if (sum > -0.000001 && sum < 0.000001)
		{
			i++;
		}
		else
		{
			Y_min = i;
			break;//跳出这个while循环
		}
	}
	Y_min = i;

	while (i <= row - 16)//i的初值为Y_min
	{
		double sum = 0.0;
		uchar* data = temp.ptr<uchar>(i);
		for (j = 0;j < col;j++)//对每一行中的每一列像素进行相加，ptr<uchar>(i)[j]访问第i行第j列的像素
		{
			sum = data[j] + sum;
		}//最终获得第i行的列和
		if (sum != 0)
		{
			i++;
		}
		else
		{
			double sum6 = 0.0;
			int iiii = i + 16;
			uchar* data = temp.ptr<uchar>(iiii);
			for (j = 0;j < col;j++)//对每一行中的每一列像素进行相加，ptr<uchar>(i)[j]访问第i行第j列的像素
			{
				sum6 = data[j] + sum6;
			}//最终获得第i行之后20行，即iiii的列和
			if (sum6 > -0.000001 && sum6 < 0.000001)//如果仍然为0，才跳出
			{
				Y_max = i;
				goto logo;//跳出这个while循环
			}
			else//否则继续执行
			{
				i++;
			}
		}
	}
logo:
	Y_max = i;

	//进行切割
	Mat image_cut1 = temp(Rect(0, Y_min, col, Y_max - Y_min));
	img_next = image_cut1.clone();   //clone函数创建新的图片 
}


//找出LED的轮廓，求出x_min,x_max,y_min,y_max
void LED_contour(const Mat& _img, vector<Point> &myPoints, vector<int> &myRadius)
{
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	findContours(_img, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, Point());//轮廓检测，第四个参数说明只检测最外围轮廓，即只检测到外框（外层轮廓，每一个LED中的细小条纹已被腐蚀，即是填充成一个圆）
	for (int i = 0; i < contours.size(); i++)
	{
		int x_max = 0, y_max = 0, x_min = 999999, y_min = 999999;
		for (int j = 0; j < contours[i].size(); j++)
		{//minmaxLoc with mask i need u
			//contours[i].size()代表的是第i个轮廓上所有像素点数
			//遍历找到的所有轮廓（i个）内的所有像素点
			//外循环遍历i个轮廓，内循环遍历每个轮廓内j个像素点；找到轮廓的最左、最右、最上和最下四个点
			if (x_max < contours[i][j].x) x_max = contours[i][j].x;//contours 是二维向量，元素是点类。
			if (y_max < contours[i][j].y) y_max = contours[i][j].y;//直接以轮廓为单位遍历，相对于整张图像遍历，节省时间
			if (x_min > contours[i][j].x) x_min = contours[i][j].x;
			if (y_min > contours[i][j].y) y_min = contours[i][j].y;
		}
		myPoints.push_back(Point((x_max + x_min) / 2, (y_max + y_min) / 2));//求出圆心
		myRadius.push_back((x_max - x_min + y_max - y_min) / 4);//求出半径
	}

	circle(_img, myPoints[0], myRadius[0], Scalar(0, 0, 255), 2, 8, 0);
	namedWindow("src", 0);
	imshow("src", _img);
	waitKey(100);
}

void para_set(double& Center_X, double& Center_Y, double& D_LED, double& f, double& led_x, double& led_y, double& all_H)
{
	/*---------------场地1-------------------*/
	Center_X = 584.75;//586.9;//593.5;//（584.5～602.5)//685;//674;//相机中心对准灯正下方670;//640;//429;
	Center_Y = 500;//494.8;//489;//（477~501)//495;//447;//486;//480;//334.5;
	D_LED = 180;//LED灯具的真实直径
	f = 2.5585778*312.5;//2.604267//2.56978//2.61333依据高度矫正的焦距(2.6389)//3.111;//根据图片大小不同,会有不同的等效焦距(1280*960对应2.5)
	led_x = 3000;
	led_y = 800;
	all_H = 2.73;

	/*---------------场地2-------------------*/
	// Center_X = 653;//685;//674;//相机中心对准灯正下方670;//640;//429;
	// Center_Y = 498;//495;//447;//486;//480;//334.5;
	// D_LED = 103;//LED灯具的真实直径
	// f = 2.596*312.5;//依据高度矫正的焦距(2.6389)//3.111;//根据图片大小不同,会有不同的等效焦距(1280*960对应2.5)
	// led_x = 0;
	// led_y = 0;
	// all_H = 1.57;
}


//roi新增的
//用于确定LED灯大小
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
}//比较同样大小的两张图同样像素个数占比

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
    // 求xmin与xmax
    int row1 = img.rows;// 行数
    int col1 = img.cols;// 列

    int j = 0;// 注意是从0开始
    while (j < col1)// j的初值为1
    {
        double sum1 = 0.0;
        for (int i = 0;i < row1;i++)// 注意没有等于号
        {
            uchar* data1 = img.ptr<uchar>(i);// ptr<uchar>(i)[j]访问第i行第j列的像素
            sum1 = data1[j] + sum1;
        }// 将第j列的每一行加完
        if (sum1>-0.000001 && sum1< 0.000001)// double类型，不能写==0
        {
            j+=sample;
        }
        else
        {
            break;// 跳出这个while循环
        }

    }
    X_min = j;

    while (j < col1)// j的初值为X_min
    {
        double sum1 = 0.0;
        for (int i = 0;i < row1;i++)
        {
            uchar* data1 = img.ptr<uchar>(i);// ptr<uchar>(i)[j]访问第i行第j列的像素
            sum1 = data1[j] + sum1;
        }// 将第j列的每一行XXXXXX加完
        if (sum1 != 0)
        {
            j+=sample;
        }
        else
        {
            break;// 跳出这个while循环
        }
    }
    X_max = j;

    // 进行切割
    Mat temp = img(Rect(X_min, 0, X_max - X_min, row1));

    // 求ymin与ymax
    int row = temp.rows;// 行数
    int col = temp.cols;// 列

    int i = 0;
    while (i < row)// i的初值为1
    {
        double sum = 0.0;
        uchar* data = temp.ptr<uchar>(i);
        for (j = 0;j < col;j++)// 对每一行中的每一列像素进行相加，ptr<uchar>(i)[j]访问第i行第j列的像素
        {
            sum = data[j] + sum;
        }// 最终获得第i行的列和
        if (sum>-0.000001 && sum < 0.000001)
        {
            i+=sample;
        }
        else
        {
            break;// 跳出这个while循环
        }
    }
    Y_min = i;

    while (i < row)// i的初值为1
    {
        double sum = 0.0;
        uchar* data = temp.ptr<uchar>(i);
        for (j = 0;j < col;j++)// 对每一行中的每一列像素进行相加，ptr<uchar>(i)[j]访问第i行第j列的像素
        {
            sum = data[j] + sum;
        }// 最终获得第i行的列和
        if (sum!=0 )
        {
            i+=sample;
        }
        else
        {
            break;// 跳出这个while循环
        }
    }
    Y_max = i;

    // 进行切割
    //imgNext = temp(Rect(0, Y_min, col, Y_max - Y_min)); // clone函数创建新的图片
    Rect minorrect(X_min-sample,Y_min-sample,(X_max-X_min+2*sample),(Y_max-Y_min+2*sample));
    Rect ROI=find_minrect(img,minorrect,0);
    imgNext=img(ROI);
    return ROI;
}

//实现对图像的细化
void thinImage(Mat &srcimage)//单通道、二值化后的图像  
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

