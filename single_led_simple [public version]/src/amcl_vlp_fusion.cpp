#include <stdio.h>
#include <stdlib.h>
#include "std_msgs/String.h"
#include <sstream>
#include <math.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>

using namespace std;

const int V = 2;        //生成的粒子的均方差
const int N_particles = 100;        //生成的粒子数目
const double PI = 3.1415926 ;

//点的基本数据结构
struct Point
{
	double x_vlc;       //从VLC接收到的x坐标
	double y_vlc;       //从VLC接受到的y坐标
	double x_amcl;      //从AMCL接受到的x坐标
	double y_amcl;      //从AMCL接受到的y坐标
	double x_fused;     //融合后的x坐标
	double y_fused;     //融合后的y坐标
} data, init_1, init_2;

//粒子的基本数据结构
struct Particle
{
    double x;
    double y;
};
std::vector<struct Particle> particle;


//产生高斯分布的噪声//
double gauss_rand()
{
    static double V1, V2, S;
    static int phase = 0;
    double X;
     
    if ( phase == 0 ) 
    {
        do 
        {
            double U1 = (double)rand() / RAND_MAX;
            double U2 = (double)rand() / RAND_MAX;
             
            V1 = 2 * U1 - 1;
            V2 = 2 * U2 - 1;
            S = V1 * V1 + V2 * V2;
        } while(S >= 1 || S == 0);
         
        X = V1 * sqrt(-2 * log(S) / S);
    } else
        X = V2 * sqrt(-2 * log(S) / S);
         
    phase = 1 - phase;
 
    return X;
}

//生成 0：1 的均匀分布
double evenly_rand()
{
    srand((unsigned)time(0));
    double rnd;
    double a = 0, b = 1;

    rnd = (double) rand()/RAND_MAX*(b-a) + a;

    return rnd;
}

//重采样步骤
void resampling(std::vector<double>& normal_weight)
{\
    std::vector<Particle> resampled_particle;
    std::vector<double> rand;

    //生成一系列的0：1均匀分布的数字
    //用来做重采样
    for(int num_1 = 0; num_1 < N_particles; num_1++)
    {
        rand.push_back (evenly_rand());
    }

    for (int num_2 = 0; num_2 < N_particles; num_2++)
    {
        int a = 0;
        int accumulate_weight = 0;
        while(a < N_particles)
        {
            accumulate_weight += normal_weight[a];      //计算传送过来的比重数组的累计概率
            if (rand[num_2] < accumulate_weight)
            {
                //思路大概就是：
                //随机生成数与计算得到的累计概率做对比，如果随机生成数落入到某一个粒子对应的累计概率区间，那么对应的粒子就复制一次
                //例如： 随机生成数为0.7, 三个粒子的比重分别是0.2,0.4,0.6, 则累计概率数组为{0,0.2，0.6,1}
                //0.7落入到第三个粒子的区间中， 第三个粒子复制一次
                resampled_particle.push_back (particle[a]);
                break;
            }
            ++a;
        }
    }

    particle.assign(resampled_particle.begin(), resampled_particle.end());
}

int complet = 0;        //初始化完成标记值

double lastdata_x_vlc;
double lastdata_y_vlc;

void coordinate_exchange(int a)
{   
    if (a == 3)     //a可更改，但为最好大于1的整形
    {
        //取第一个标记点
        init_1.x_vlc = data.x_vlc;
        lastdata_x_vlc = data.x_vlc;
        init_1.y_vlc = data.y_vlc;
        lastdata_y_vlc = data.y_vlc;
        init_1.x_amcl = data.x_amcl;
        init_1.y_amcl = data.y_amcl;
        cout << "init_1.x_vlc = " << init_1.x_vlc << '\n' ;
        cout << "init_1.y_vlc = " << init_1.y_vlc << '\n' ;
    }
    
    //条件： turtlebot行驶距离超过sqrt（400）且complet=0，为了限制VLC的数据漂移，增加了限定条件为行驶距离不超过sqrt（500）
    else if (((pow(data.x_vlc-init_1.x_vlc,2) + pow(data.y_vlc-init_1.y_vlc,2)) > 400) && ((pow(data.x_vlc-init_1.x_vlc,2) + pow(data.y_vlc-init_1.y_vlc,2)) < 500) && (!complet))
    {
        //取第二个标记点
        init_2.x_vlc = data.x_vlc;
        init_2.y_vlc = data.y_vlc;
        init_2.x_amcl = data.x_amcl;
        init_2.y_amcl = data.y_amcl;
        cout << "初始化已完成"<< '\n';
        complet = 1;
    }

    cout << "dis = " << sqrt(pow(data.x_vlc-init_1.x_vlc,2) + pow(data.y_vlc-init_1.y_vlc,2)) << '\n' ;
    lastdata_x_vlc = data.x_vlc;
    lastdata_y_vlc = data.y_vlc;

    //做SLAM的坐标变换
    if (complet)
    {
        double length_init = sqrt(pow((init_2.x_amcl - init_1.x_amcl),2) + pow((init_2.y_amcl - init_1.y_amcl),2)); 
        double length = sqrt(pow((data.x_amcl - init_1.x_amcl),2) + pow((data.y_amcl - init_1.y_amcl),2));
        //double length_2 = sqrt(pow(data.x_amcl - init_2.x_amcl,2) + pow(data.y_amcl - init_2.y_amcl,2));
        //double B_x = atan((init_2.x_amcl - init_1.x_amcl)/(init_2.y_amcl - init_1.y_amcl));
        //double R_x = atan((init_2.x_vlc - init_1.x_vlc)/(init_2.y_vlc - init_1.y_vlc));
        //double B_y = atan((init_2.y_amcl - init_1.y_amcl)/(init_2.x_amcl - init_1.x_amcl));
        //double R_y = atan((init_2.y_vlc - init_1.y_vlc)/(init_2.x_vlc - init_1.x_vlc));
        //double delta = acos((pow(length_2,2)-pow(length,2)-pow(length_init,2)) / (2 * length_init * length));
        //double B_x = atan((init_2.y_vlc - init_1.y_vlc) / (init_2.x_vlc - init_1.x_vlc));
        //double B_y = atan((init_2.x_vlc - init_1.x_vlc) / (init_2.y_vlc - init_1.y_vlc));
        double B_1 = atan2((init_2.y_amcl - init_1.y_amcl) , (init_2.x_amcl - init_1.x_amcl));
        double B_2 = atan2((data.y_amcl - init_1.y_amcl) , (data.x_amcl - init_1.x_amcl));
        double B_vlc_x = atan2((init_2.y_vlc - init_1.y_vlc) , (init_2.x_vlc - init_1.x_vlc));
        double B_vlc_y = (1 / B_vlc_x);
        double delta = B_1 - B_2;

        double X = cos(B_vlc_x - delta) * length;
        //cout <<"X = " << X << '\n';
        data.x_amcl = init_1.x_vlc + X;

        double Y = sin(B_vlc_x + delta) * length;
        data.y_amcl = init_1.y_vlc + Y; 

        /*cout << "init_vlc_x = " << init_1.x_vlc << '\n';
        cout << "init_vlc_y = " << init_1.y_vlc << '\n';
        cout << "data_slam_x = " << data.x_amcl << '\n';
        cout << "data_slam_y = " << data.y_amcl << '\n';*/
    }
}


int i = 0;
void Fusion_Process()
{   
    ++i;
    std::vector<double> distance; 
    std::vector<double> weight;
    std::vector<double> normal_weight;
    Particle rand_particle;

	//----------这是slam的坐标变换------------
	//coordinate_exchange(i);
	//----------这是slam的坐标变换------------

    //生成粒子
    for (int num = 0;num < N_particles;num++)
    {
        rand_particle.x = data.x_amcl + V * gauss_rand();
        rand_particle.y = data.y_amcl + V * gauss_rand();
        particle.push_back (rand_particle);
    }

    for (int num = 0;num < N_particles;num++)
    {
        distance.push_back (sqrt(pow((particle[num].x - data.x_vlc),2) + pow((particle[num].y - data.y_vlc),2))); 
    }

    //计算各粒子比重，为了增强效果，特意取平方
    for (int num = 0;num < N_particles;num++)
    {
        weight.push_back (pow((1/sqrt(2*PI*V)) * exp((-distance[num]) / (2*V)), 2));
    }

    double sum = 0;
    for (int num = 0;num < N_particles;num++)
    {
        sum += weight[num];
    } 

    //归一化比重
    for (int num = 0;num < N_particles;num++)
    {
        normal_weight.push_back (weight[num] / sum); 
    }

    //重采样
    resampling(normal_weight);
    
    data.x_fused = 0;
    data.y_fused = 0;
    
    //计算融合结果
    for (int num = 0;num < N_particles;num++)
    {
        data.x_fused += particle[num].x * (1.0 / N_particles);
        data.y_fused += particle[num].y * (1.0 / N_particles);
    }
}


//定义一个类
class VLC_SLAM_FUSION
{
    private:    ros::NodeHandle nh;
                ros::Publisher msgPointPub;
		ros::Publisher slam_PointPub;
                ros::Subscriber sub_vlc;
                ros::Subscriber sub_amcl;

				//标志位
				bool vlc_flag;
				bool amcl_flag;

    public:     
    VLC_SLAM_FUSION()
    {
		vlc_flag = 0;
		amcl_flag = 0;
        msgPointPub = nh.advertise<geometry_msgs::PointStamped>("fused_location", 1000);
		slam_PointPub = nh.advertise<geometry_msgs::PointStamped>("received_amcl", 1000);
        sub_vlc = nh.subscribe("/location_info",1, &VLC_SLAM_FUSION::Subscribe_VLCdata,this);
        sub_amcl = nh.subscribe("/amcl_pose",1, &VLC_SLAM_FUSION::Subscribe_SLAMdata,this);
    }
    
    void Subscribe_VLCdata(const geometry_msgs::PointStamped::ConstPtr& vlc_msgs)
    {
        data.x_vlc = vlc_msgs->point.x;
        data.y_vlc = vlc_msgs->point.y;
		vlc_flag = 1;
		ROS_INFO("x_vlc=%.5f, y_vlc=%.5f\n", data.x_vlc * 100, data.y_vlc * 100);
    }

    void Subscribe_SLAMdata(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amclpose_msg)
    {

        data.x_amcl = amclpose_msg->pose.pose.position.x;
        data.y_amcl = amclpose_msg->pose.pose.position.y;

		if (vlc_flag == 1 && amcl_flag == 0)
		{
			data.x_amcl += data.x_vlc;
			data.y_amcl += data.y_vlc;
		}

		//使amcl仅在初始时刻接收vlc的值，其他时刻不接收(如此,amcl那边可以不用改动)
		amcl_flag = 1;


        Fusion_Process();
	
		std_msgs::String slam_msg;
        geometry_msgs::PointStamped slam_PointStamped;
        std::stringstream sss;

        slam_PointStamped.point.x = data.x_amcl;
        slam_PointStamped.point.y = data.y_amcl;
        sss << '\n' << "x_amcl = " << data.x_amcl*100 << '\n' << "y_amcl = " << data.y_amcl*100 <<'\n' ;
        slam_msg.data = sss.str();
        slam_PointStamped.header.stamp = ros::Time::now();
        slam_PointStamped.header.frame_id = "map";
        slam_PointPub.publish(slam_PointStamped);
        ROS_INFO("%s", slam_msg.data.c_str());
	    
	    
        std_msgs::String msg;
        geometry_msgs::PointStamped msgPointStamped;
        std::stringstream ss;

        msgPointStamped.point.x = data.x_fused;
        msgPointStamped.point.y = data.y_fused;
        ss << '\n' << "x_fused = " << data.x_fused*100 << '\n' << "y_fused = " << data.y_fused*100 <<'\n' ;
        msg.data = ss.str();
        msgPointStamped.header.stamp = ros::Time::now();
        msgPointStamped.header.frame_id = "map";
        msgPointPub.publish(msgPointStamped);
        ROS_INFO("%s", msg.data.c_str());
    }
};


int main(int argc, char** argv)
{
    ros::init(argc,argv, "fusion_vlc_slam");
    VLC_SLAM_FUSION instance;

    ros::AsyncSpinner spiner(2);
    spiner.start();
    ros::waitForShutdown();

    return 0;
}
