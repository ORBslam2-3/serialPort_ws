#include <ros/ros.h> 
#include <serial/serial.h>  //ROS已经内置了的串口包 
#include <std_msgs/String.h> 
#include <std_msgs/Empty.h> 
#include <std_msgs/UInt8.h>
#include "serialPort/sbus.h"
#include <geometry_msgs/Twist.h>
#include <string>

serial::Serial ser; //声明串口对象
SbusData data;

// 假设输入数值范围为 300～1800，中间范围为 1000～1050
double min_value = 200.0;
double max_value = 1800.0;
double middle_min = 970.0;
double middle_max = 1050.0;

// 假设线速度范围为 -10.0 到 10.0 m/s
double rear_linear_speed;
double front_linear_speed;

//角速度 -2.0到+2.0 m/s2
double streer_angular_speed;

//端口
std::string port;

int main (int argc, char** argv) 
{ 
    //初始化节点 
    ros::init(argc, argv, "serialPort_node"); 
    //声明节点句柄 
    ros::NodeHandle nh; 
    nh.param<double>("rear_linear_speed", rear_linear_speed, -5);
    nh.param<double>("front_linear_speed", front_linear_speed, 10);
    nh.param<double>("streer_angular_speed", streer_angular_speed, 5);
    nh.param<std::string>("port", port, "/dev/ttyUSB1");
    // 创建一个发布者，将消息发布到名为 "cmd_vel" 的主题上
    ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);

    try 
    { 
        //设置串口属性，并打开串口 
        ser.setPort(port); 
        ser.setBaudrate(115200); 
    
        serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
        ser.setTimeout(to); 
        ser.open(); 
    } 
    catch (serial::IOException& e) 
    { 
        ROS_ERROR_STREAM("Unable to open port "); 
        return -1; 
    } 
    //检测串口是否已经打开，并给出提示信息 
    if(ser.isOpen()) { 
        ROS_INFO_STREAM("Serial Port initialized"); 
    } 
    else { 
        return -1; 
    }
    SbusRx sbus_rx(&ser);
    ros::Rate loop_rate(10); // 10 Hz
    while (ros::ok()) {
    if (sbus_rx.Read()) {
        /* Grab the received data */
        data = sbus_rx.data();
        /* Display the received data */
        // std::cout<<"通道0存储的遥控器左右数据为 ： "<<data.ch[0]<<std::endl;
        // std::cout<<"通道2存储的遥控器前后数据为 ： "<<data.ch[2]<<std::endl;
        // std::cout<<"通道6存储的遥控器使能位数据为 ： "<<data.ch[6]<<std::endl;
        // 计算线速度
        double linear_speed;
        //计算角速度
        double angular_speed;
        //解析使能位数据
        if(data.ch[6]==1800)
        {
            
            if (data.ch[2] < middle_min) {
                // 低于中间范围，映射到后退速度范围
                linear_speed = -rear_linear_speed + (data.ch[2] - min_value) / (middle_min - min_value) * rear_linear_speed;
                
            } else if (data.ch[2] > middle_max) {
                // 高于中间范围，映射到前进速度范围
                linear_speed = 0.0 + (data.ch[2] - middle_max) / (max_value - middle_max) * front_linear_speed;
            } else {
                // 在中间范围，无线速度
                linear_speed = 0.0;
            }
            //根据坐标系 前x左y上z 向左转与x夹角为正 向右转为负
            if (data.ch[0] < middle_min) {
                // 低于中间范围，向左转
                angular_speed = -(-streer_angular_speed + (data.ch[0] - min_value) / (middle_min - min_value) * streer_angular_speed);
                
            } else if (data.ch[0] > middle_max) {
                // 高于中间范围，向右转
                angular_speed = -(-streer_angular_speed + (data.ch[0] - min_value) / (middle_min - min_value) * streer_angular_speed);
            } else {
                // 在中间范围，无转向
                angular_speed = 0.0;
            }
        }else{
            angular_speed = 0.0;
            linear_speed = 0.0;
        }
        // 创建一个 Twist 消息
        geometry_msgs::Twist twist;
        
        // 设置线速度和角速度
        twist.linear.x = linear_speed; // 设置线速度为1.0 m/s
        twist.angular.z = angular_speed; // 设置角速度为0.5 rad/s
        
        // 发布 Twist 消息
        twist_pub.publish(twist);
      
        // 进行循环
        ros::spinOnce();
        loop_rate.sleep();

    }
    }
}
        
     
