
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h> 
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Header.h>
#include <math.h>
#include <unistd.h>
#include <iostream>
#include <cstdlib>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define BUF_LEN 4096
using namespace std;

serial::Serial ser;
std_msgs::String DataTemp;
std_msgs::String result;
int m_iLinearVelocity = 0;
int m_iAngulerVelocity = 0;


int main (int argc, char** argv)
{
    ros::init(argc, argv, "Joy_tetra_node");
    ros::NodeHandle n;
    ros::Publisher joy_tetra_publisher;

    joy_tetra_publisher = n.advertise<sensor_msgs::Joy>("joy",100);
    sensor_msgs::Joy tetra_joy;

    try
    {
        //Bluetooth to USB_serial 
        ser.setPort("/dev/SENA");
        ser.setBaudrate(9600);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialized");
    }
    else
    {
        return -1;
    }

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    ros::Rate loop_rate(10); //HZ
    
    while(ros::ok())
    {
        ros::spinOnce();
        current_time = ros::Time::now();
        double dt = (current_time - last_time).toSec();

        if(ser.available() > 0)
        {
            DataTemp.data = ser.read();
            if(DataTemp.data == "\n")
            {
                ROS_INFO_STREAM("DATA: " << result.data);
                int ilength = result.data.size();
                int iCommaPoint = result.data.find(','); 
                //ROS_INFO_STREAM("iCommaPoint: " << iCommaPoint);
                if(iCommaPoint < 1) //Check Garbege
                {
                    memset(&result.data, 0, sizeof(result.data));
                    ROS_INFO_STREAM("Connect Success!");
                }
                else
                {
                    m_iLinearVelocity = stof(result.data.substr(0, iCommaPoint));
                    m_iAngulerVelocity = stof(result.data.substr(iCommaPoint + 1));
                    //ROS_INFO_STREAM("m_iLinearVelocity: " << m_iLinearVelocity);
                    //ROS_INFO_STREAM("m_iAngulerVelocity: " << m_iAngulerVelocity);

                    //Joy_msg Publish
                    //tetra_joy.axes[1] = (float)m_iLinearVelocity;
                    tetra_joy.buttons.clear();
                    tetra_joy.buttons.push_back(0);
                    tetra_joy.buttons.push_back(0);
                    tetra_joy.buttons.push_back(0);
                    tetra_joy.buttons.push_back(0);
                    tetra_joy.buttons.push_back(0);
                    tetra_joy.buttons.push_back(0);
                    tetra_joy.buttons.push_back(0);
                    tetra_joy.buttons.push_back(0);
                    tetra_joy.buttons.push_back(0);
                    tetra_joy.buttons.push_back(0);
                    tetra_joy.buttons.push_back(0);
                    tetra_joy.buttons.push_back(0);

                    tetra_joy.axes.clear();
                    tetra_joy.axes.push_back(0);
                    tetra_joy.axes.push_back((float)m_iLinearVelocity / 100.0);
                    tetra_joy.axes.push_back((float)m_iAngulerVelocity / 100.0);
                    tetra_joy.axes.push_back(0);
                    tetra_joy.axes.push_back(0);
                    
                    joy_tetra_publisher.publish(tetra_joy);

                    memset(&result.data, 0, sizeof(result.data)); 
                }
                
                //usleep(1000);

            }
            else
            {
                result.data += DataTemp.data;
                //ROS_INFO_STREAM("Read: " << DataTemp.data);
            }
            
        }

        //Todo....
        //tetra_joy.axes[0] = m_iLinearVelocity;
        //joy_tetra_publisher.publish(tetra_joy);

        last_time = current_time;
        //ROS_INFO("Time: %.2f\n",dt);
        loop_rate.sleep();
    }


    return 0;
}

