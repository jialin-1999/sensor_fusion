#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Int64.h>
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include <std_msgs/Int16MultiArray.h>

#include <algorithm>
#include <iostream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <fstream>
#include <stdlib.h>
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>
#include <ctime>

#define Pi 3.14159
//#define Threshold_1 0.2    //阈值1，当数据变化值超过这个值时开始改变delta_x ,变化很小时说明领航员行走平缓，则维持之前的信任度
//#define Threshold_2 20
using std::cout; 
using std::endl;
using namespace std;

unsigned long nframe;
float target1x;
float target1y;
float target2x;
float target2y;
float lastposition_x;
float lastposition_y;
float rad1;
float rad2;
float last_rad;
float  last_angleping;
float degree1;
float degree2;
float last_degree;
float last_degreeping;
float kpten = 0.175;
float kptwenty = 0.35;
/*
int delta_x = 0;     	//同方向变化量越大，delta_x越大 
float q_x = 0.35;	//表示对新读入的数据的信任度，取值范围0-1 
int old_trendx = 0;	//表示第n-2个数据到第n-1个数据的变化趋势，右移为1，左移为0 
int new_trendx = 0;	//表示第n-1个数据到第n个数据的变化趋势，右移为1，左移为0
float old_x2 = 0;		//第n-1次的x
float new_x2 = 0;
*/
static ofstream log_ronghe("/home/sucro/log_ronghe.csv");
inline float radian2degree(float radian){ return radian * 180 / Pi;}
inline float degree2radian(float angle){ return angle * Pi / 180;}
void pos1Callback(const std_msgs::Float64MultiArray& msg)
{
     target1x = msg.data[1];
     target1y = -msg.data[0];
    
}
void angleCallback(const std_msgs::Float64& msg)
{
     rad1 = msg.data;   
}

void pos2Callback(const std_msgs::Float64MultiArray& msg)
{
      target2x = msg.data[0];
      target2y = msg.data[1];
      rad2 =  degree2radian(atan2(target2y, target2x) / Pi * 180);   
}
void numCallback(const std_msgs::int16& msg)
{
    person_num = msg.data;
}
/*
void dataprocessx(float x){
	new_x2 = x;
	if(new_x2 > old_x2)   		//x方向右移
		new_trendx = 1;    		//右移trend为1
	else new_trendx = 0;		//左移trend为0
	if(new_trendx = old_trendx){    //前后两次变化趋势一样，连续两次向右移动或者向左移动
			if(abs(new_x2 - old_x2) > Threshold_1)
				delta_x += 5;            //移动的幅度超过阈值，说明移动较大，增大delta
			if(delta_x >= Threshold_2)
				q_x += 0.15;          //变化持续，说明一直在往一个方向移动，应增大对当前数据的信任值q
		}else{       //连续两次的变化是反向的
			delta_x = 0;
			q_x = 0.2;
			old_trendx = new_trendx;     //更新变化趋势
		}
	if(q_x > 0.95) 
		q_x = 0.95;     //q取值0-1之间
	new_x2 = (1-q_x) * old_x2 + q_x * new_x2;
	old_x2 = new_x2;      //更新old_x2
	return new_x2;
}
*/
  int main(int argc, char **argv)
{	
    ros::init(argc, argv, "chose_position");
    ros::NodeHandle nh;
    ros::Subscriber target1_sub = nh.subscribe("/shuangmu_position", 1, pos1Callback);
    ros::Subscriber angle = nh.subscribe("/dynamixel_mx28/command", 1, angleCallback);
    //ros::Subscriber angle = nh.subscribe("/dianji", 1, angleCallback);
    //ros::Subscriber angle = nh.subscribe("/shaungmu_angle", 1, angleCallback);
    ros::Subscriber target2_sub= nh.subscribe("/target_position", 1, pos2Callback);
    ros::subscriber person_num_sub= nh.subscribe("/kcf_ssd/person_num",1,numCallback);
    ros::Publisher  target_pub = nh.advertise<std_msgs::Float64MultiArray>("last_target", 1);
    ros::Publisher  target_angle = nh.advertise<std_msgs::Float64>("last_angle", 1);

     
     ros::Rate loop_rate(20);
      while(ros::ok()){
       ros::spinOnce();
        ++nframe;
	printf("Frame #%ld: \n", nframe);
       if ( abs(target1y-target2y) < 0.8 && abs(target1x-target2x) < 0.5)
       { 
	 lastposition_x= ( target1x + target2x )/2;
	 lastposition_y = ( target1y + target2y )/2;
       }
       else 
       {
         lastposition_x=  target2x;
	 lastposition_y = target2y;
       
       }
     
        if ( abs(rad1-rad2) < kpten )
	last_rad = (rad1 + rad2)/2;

        else if ( kpten < abs(rad1-rad2) < kptwenty )
	{
	 last_rad = 0.7*rad2 + 0.3*rad1;
	}
	else 
	  last_rad = rad2;
	
         
	//last_angleping = dataprocessx(last_angle);
	degree1 = radian2degree(rad1);
	degree2 = radian2degree(rad2);
	last_degree = radian2degree(last_rad);
	//last_degreeping = radian2degree(last_angleping);
        log_ronghe<<target1x<<",";
	log_ronghe<<target1y<<",";
	log_ronghe<<rad1<<",";
	log_ronghe<<degree1<<",";
        log_ronghe<<target2x<<",";
	log_ronghe<<target2y<<",";
	log_ronghe<<rad2<<",";
	log_ronghe<<degree2<<",";
	log_ronghe<<lastposition_x<<",";
	log_ronghe<<lastposition_y<<",";
	log_ronghe<<last_rad<<",";
	//log_ronghe<<last_angleping<<",";
	log_ronghe<<last_degree<<",";
	//log_ronghe<<last_degreeping<<",";
        log_ronghe<<endl;
	
        std_msgs::Float64MultiArray lastposition_msg; 
       
	std_msgs::Float64 lastangle_msg; 
	
        lastposition_msg.data.resize(2);
       
        lastposition_msg.data[0] = lastposition_x;
        lastposition_msg.data[1] = lastposition_y;
	
	lastangle_msg.data = last_rad;
        target_pub.publish(lastposition_msg);
	target_angle.publish(lastangle_msg);
       //fp = fopen("/home/sucro/1.txt", "w");
           //if(fp ==NULL)
        //printf("打开文件aa.txt失败\n");
        //fprintf(fp, nframe,target1x);
        printf(" X1= %4.2f ,Y1= %4.2f \n X2= %4.2f ,Y2= %4.2f \n last_x=%4.2f, last_y =%4.2f\n angle1= %4.2f\n angle2= %4.2f\n last_angle= %4.2f\n" , target1x ,target1y ,target2x ,target2y,lastposition_x,lastposition_y, rad1 , rad2 ,last_rad);
	

        loop_rate.sleep();
}
        return 0;
      }
       
       
       
       
       
       
       
       
       
       
       
       
       
       
