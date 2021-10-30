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
float angle1;
float angle2;
float last_angle;
float kp = 10 / 180 * Pi;

//static ofstream log_ronghe("/home/sucro/log_ronghe.csv");
std::string int2str( int val )  
{  
	std::ostringstream out;  
	out << val;  
}
void pos1Callback(const std_msgs::Float64MultiArray& msg)
{
     target1x = msg.data[0];
     target1y = msg.data[1];
    
}
void angleCallback(const std_msgs::Float64& msg)
{
      angle1 = msg.data;   
}

void pos2Callback(const std_msgs::Float64MultiArray& msg)
{
      target2x = msg.data[0];
      target2y = msg.data[1];
      angle2 = atan2(target2y, target2x) / Pi * 180;   
}


  int main(int argc, char **argv)
{	
    ros::init(argc, argv, "chose_position");
    ros::NodeHandle nh;
    ros::Subscriber target1_sub = nh.subscribe("/shuangmu_position", 1, pos1Callback);
    //ros::Subscriber angle = nh.subscribe("/dynamixel_mx28/command", 1, angleCallback);
    ros::Subscriber angle = nh.subscribe("/dianji", 1, angleCallback);
    ros::Subscriber target2_sub= nh.subscribe("/target_position", 1, pos2Callback);
    ros::Publisher  target_pub = nh.advertise<std_msgs::Float64MultiArray>("last_target", 1);
    ros::Publisher  target_angle = nh.advertise<std_msgs::Float64>("last_angle", 1);
    time_t timer;
    std::time(&timer);
    std::tm* t_tm = std::localtime(&timer);   
    string txt_save_path = "/home/sucro/results/Ctxt/";
    string txt_save_name = "Shuangmu_Data_" + int2str(t_tm->tm_year + 1900) + "_" + int2str(t_tm->tm_mon + 1) 
    + "_" + int2str(t_tm->tm_mday) + "__" + int2str(t_tm->tm_hour) + "_" + int2str(t_tm->tm_min) + ".txt";
    std::ofstream txt_data(txt_save_path + txt_save_name, ios::app); //|ios::noreplace
    
    txt_data << "######### RONGHE data ##########" << endl;
    txt_data << "Experiment date: " ;
    txt_data << int2str(t_tm->tm_year + 1900) + "." + int2str(t_tm->tm_mon + 1) 
			+ "." + int2str(t_tm->tm_mday) + "  " + int2str(t_tm->tm_hour) + ":" + int2str(t_tm->tm_min) << endl << endl<< endl;
    txt_data << "Frame" << " " << "X1" << " " <<  "Y1" << " " << "ANGLE1" << " " << "X2" << " " << "Y2" << " " << "ANGLE2" << " " 
		 << "XLAST" << " " << " YLAST" << " " << "ANGLELAST" << " " 
		 <<endl;
     
     ros::Rate loop_rate(20);
      while(ros::ok()){
       ros::spinOnce();
        ++nframe;
	printf("Frame #%ld: \n", nframe);
       if ( abs(target1y-target2y) < 0.6 && abs(target1x-target2x) < 0.3)
       { 
	 lastposition_x= ( target1x + target2x )/2;
	 lastposition_y = ( target1y + target2y )/2;
       }
       else 
       {
         lastposition_x=  target2x;
	 lastposition_y = target2y;
       
       }
     
        if ( abs(angle1-angle2) < kp )
        last_angle = (angle1 + angle2)/2;
        else
	 last_angle = angle2;

       /*
        log_ronghe<<target1x<<",";
	log_ronghe<<target1y<<",";
	log_ronghe<<angle1<<",";
        log_ronghe<<target2x<<",";
	log_ronghe<<target2y<<",";
	log_ronghe<<angle2<<",";
	log_ronghe<<lastposition_x<<",";
	log_ronghe<<lastposition_y<<",";
	log_ronghe<<last_angle<<",";
        log_ronghe<<endl;
	*/
        std_msgs::Float64MultiArray lastposition_msg; 
       
	std_msgs::Float64 lastangle_msg; 
	
        lastposition_msg.data.resize(2);
       
        lastposition_msg.data[0] = lastposition_x;
        lastposition_msg.data[1] = lastposition_y;
	
	lastangle_msg.data = last_angle;
        target_pub.publish(lastposition_msg);
	target_angle.publish(lastangle_msg);
	
	txt_data << nframe << "     "; 
        txt_data << target1x << "     ";
        txt_data << target1y << "      ";
        txt_data << angle1 << "     ";
        txt_data << target2x<< "    ";
        txt_data << target2y << "     ";
        txt_data << angle2 << "     ";
        txt_data << lastposition_x << "     "; 
	txt_data << lastposition_y << "     ";
	txt_data << last_angle << "     "
	<< endl << endl;
	txt_data.close();
        printf(" X1= %4.2f ,Y1= %4.2f \n, X2= %4.2f ,Y2= %4.2f \n ,last_x=%4.2f, last_y =%4.2f\n, angle1= %4.2f\n , angle2= %4.2f\n,last_angle= %4.2f" , target1x ,target1y ,target2x ,target2y,lastposition_x,lastposition_y, angle1 , angle2 ,last_angle);
	

        loop_rate.sleep();
}
        return 0;
      }
       
       
       
       
       
       
       
       
       
       
       
       
       
       