#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include "dynamixel.h"
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/UInt32.h>

#define total_motor 21
const double degree = M_PI/180;

//geometry_msgs::Quaternion koordinat_kaki_kanan, koordinat_kaki_kiri
unsigned int motion;
/*
void coordinate_callback_kanan(const geometry_msgs::Quaternion &msg)
{
    koordinat_kaki_kanan = msg;
}

void coordinate_callback_kiri(const geometry_msgs::Quaternion &msg)
{
    koordinat_kaki_kiri = msg;
}
*/
void motion_callback(const std_msgs::UInt32 &msg)
{
    motion = msg.data;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "model_publisher");
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(60);

    ros::NodeHandle motion_node[3];
    ros::Subscriber motion_sub[3];

    // robot state
    double id_motor[total_motor];
    double angle = 0;
    // message declarations
    geometry_msgs::TransformStamped odom_trans;
    sensor_msgs::JointState joint_state;

    Dynamixel motors;
/*
    motion_sub[0] = motion_node[0].subscribe("/motion/koordinat/kaki_kanan",1,coordinate_callback_kanan);
    motion_sub[1] = motion_node[1].subscribe("/motion/koordinat/kaki_kiri",1,coordinate_callback_kiri);
    motion_sub[2] = motion_node[2].subscribe("/motion/motion",1,motion_callback);
*/
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    for(int i = 0; i<total_motor; i++)
    {
        id_motor[i] = 0;
    }
    motors.update_data_command();

    while (ros::ok()) {
        //update joint_state

        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(21);
        joint_state.position.resize(21);
        joint_state.name[0] = "tflow_1_kpitch";
        joint_state.name[1] = "tflow_2_kipitch";
        joint_state.name[2] = "tflow_3_kroll";
        joint_state.name[3] = "tflow_4_kiroll";
        joint_state.name[4] = "tflow_5_kroll";
        joint_state.name[5] = "tflow_6_kiroll";
        joint_state.name[6] = "tflow_7_kroll";
        joint_state.name[7] = "tflow_8_kiroll";
        joint_state.name[8] = "tflow_9_kpitch";
        joint_state.name[9] = "tflow_10_kipitch";
        joint_state.name[10] = "tflow_11_kyaw";
        joint_state.name[11] = "tflow_12_kiyaw";
        joint_state.name[12] = "tflow_13_proll";
        joint_state.name[13] = "tflow_14_ppitch";
        joint_state.name[14] = "tflow_15_pyaw";
        joint_state.name[15] = "tflow_16_bkiroll";
        joint_state.name[16] = "tflow_17_bkroll";
        joint_state.name[17] = "tflow_18_bkiyaw";
        joint_state.name[18] = "tflow_19_bkyaw";

        joint_state.name[19] = "tflow_22_kyaw";
	joint_state.name[20] = "tflow_23_kroll";



        for(int a=0; a<21; a++)
        {
            joint_state.position[a] = motors.current_pos[a+1];
        }

  /*      float penjumlah_x = 0.0, penjumlah_y = 0.0, penjumlah_z = 0.0;
        if(motion < 100 && motion > 0)
        {
            if(koordinat_kaki_kanan.z > koordinat_kaki_kiri.z)
            {
                penjumlah_z = koordinat_kaki_kanan.z;
                penjumlah_y = koordinat_kaki_kanan.y;
                penjumlah_x = koordinat_kaki_kanan.x;
            }
            else
            {
                penjumlah_z = koordinat_kaki_kiri.z;
                penjumlah_y = koordinat_kaki_kiri.y;
                penjumlah_x = koordinat_kaki_kiri.x;
            }
            penjumlah_z += 130.0;

            penjumlah_z /= 1000;
            penjumlah_y /= 1000;
            penjumlah_x /= 1000;
        }
*/

        // update transform
        // (moving in a circle with radius)
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.transform.translation.x = 0;
        odom_trans.transform.translation.y = 0;
        odom_trans.transform.translation.z = 0;
        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0);

        //send the joint state and transform
        joint_pub.publish(joint_state);
        broadcaster.sendTransform(odom_trans);

      angle += degree/4;

        // This will adjust as needed per iteration
      ros::spinOnce();
      //loop_rate.sleep();
    }


    return 0;
}

/*#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include "dynamixel.h"
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/UInt32.h>

#define total_motor 20
const double degree = M_PI/180;

geometry_msgs::Quaternion koordinat_kaki_kanan, koordinat_kaki_kiri;
unsigned int motion;

void coordinate_callback_kanan(const geometry_msgs::Quaternion &msg)
{
    koordinat_kaki_kanan = msg;
}

void coordinate_callback_kiri(const geometry_msgs::Quaternion &msg)
{
    koordinat_kaki_kiri = msg;
}

void motion_callback(const std_msgs::UInt32 &msg)
{
    motion = msg.data;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "model_publisher");
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(60);

    ros::NodeHandle motion_node[3];
    ros::Subscriber motion_sub[3];

    // robot state
    double id_motor[total_motor];
    double angle = 0;
    // message declarations
    geometry_msgs::TransformStamped odom_trans;
    sensor_msgs::JointState joint_state;

    Dynamixel motors;

    motion_sub[0] = motion_node[0].subscribe("/motion/koordinat/kaki_kanan",1,coordinate_callback_kanan);
    motion_sub[1] = motion_node[1].subscribe("/motion/koordinat/kaki_kiri",1,coordinate_callback_kiri);
    motion_sub[2] = motion_node[2].subscribe("/motion/motion",1,motion_callback);

    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    for(int i = 0; i<total_motor; i++)
    {
        id_motor[i] = 0;
    }
    motors.update_data_command();

    while (ros::ok()) {
        //update joint_state

        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(20);
        joint_state.position.resize(20);
        joint_state.name[0] = "1.bahu_kanan_roll";
        joint_state.name[1] = "2.bahu_kiri_roll";
        joint_state.name[2] = "3.bahu_kanan_pitch";
        joint_state.name[3] = "4.bahu_kiri_pitch";
        joint_state.name[4] = "5.siku_kanan_yaw";
        joint_state.name[5] = "6.siku_kiri_yaw";
        joint_state.name[6] = "7.hip_kanan_yaw";
        joint_state.name[7] = "8.hip_kiri_yaw";
        joint_state.name[8] = "9.hip_kanan_pitch";
        joint_state.name[9] = "10.hip_kiri_pitch";
        joint_state.name[10] = "11.hip_kanan_roll";
        joint_state.name[11] = "12.hip_kiri_roll";
        joint_state.name[12] = "13.lutut_kanan_roll";
        joint_state.name[13] = "14.lutut_kiri_roll";
        joint_state.name[14] = "15.engkle_kanan_roll";
        joint_state.name[15] = "16.engkle_kiri_roll";
        joint_state.name[16] = "17.telapak_kaki_kanan_pitch";
        joint_state.name[17] = "18.telapak_kaki_kiri_pitch";
        joint_state.name[18] = "19.leher_x";
        joint_state.name[19] = "20.leher_y";

        for(int a=0; a<20; a++)
        {
            joint_state.position[a] = motors.current_pos[a+1];
        }

        float penjumlah_x = 0.0, penjumlah_y = 0.0, penjumlah_z = 0.0;
        if(motion < 100 && motion > 0)
        {
            if(koordinat_kaki_kanan.z > koordinat_kaki_kiri.z)
            {
                penjumlah_z = koordinat_kaki_kanan.z;
                penjumlah_y = koordinat_kaki_kanan.y;
                penjumlah_x = koordinat_kaki_kanan.x;
            }
            else
            {
                penjumlah_z = koordinat_kaki_kiri.z;
                penjumlah_y = koordinat_kaki_kiri.y;
                penjumlah_x = koordinat_kaki_kiri.x;
            }
            penjumlah_z += 130.0;

            penjumlah_z /= 1000;
            penjumlah_y /= 1000;
            penjumlah_x /= 1000;
        }


        // update transform
        // (moving in a circle with radius)
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.transform.translation.x = penjumlah_x;
        odom_trans.transform.translation.y = 0;
        odom_trans.transform.translation.z = penjumlah_z;
        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0);

        //send the joint state and transform
        joint_pub.publish(joint_state);
        broadcaster.sendTransform(odom_trans);

      angle += degree/4;

        // This will adjust as needed per iteration
      ros::spinOnce();
      //loop_rate.sleep();
    }


    return 0;
}
*/
