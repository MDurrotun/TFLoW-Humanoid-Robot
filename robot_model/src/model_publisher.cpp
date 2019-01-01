#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include "dynamixel.h"
#include "imu.h"
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/UInt32.h>

#define total_motor 20
const double degree = M_PI/180;
const float PHI = (float)57.295779513082320876798154814105;
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
    Imu imu;

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
    imu.update_imu();

    while (ros::ok()) {
        //update joint_state

        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(20);
        joint_state.position.resize(20);
        joint_state.position.resize(20);
        joint_state.name[0] = "1.engkle_kanan_pitch"; //done
        joint_state.name[1] = "2.engkle_kiri_pitch"; //done
        joint_state.name[2] = "3.engkle_kanan_roll"; //done
        joint_state.name[3] = "4.engkle_kiri_roll"; //done
        joint_state.name[4] = "5.lutut_kanan_roll"; //done
        joint_state.name[5] = "6.lutut_kiri_roll"; //done
        joint_state.name[6] = "7.hip_kanan_roll"; //done
        joint_state.name[7] = "8.hip_kiri_roll"; //done
        joint_state.name[8] = "9.hip_kanan_pitch"; //done
        joint_state.name[9] = "10.hip_kiri_pitch"; //done
        joint_state.name[10] = "11.hip_kanan_yaw"; //done
        joint_state.name[11] = "12.hip_kiri_yaw"; //done
        joint_state.name[12] = "13.joint_perut_pitch"; //done
        joint_state.name[13] = "14.joint_perut_roll"; //done
        joint_state.name[14] = "15.tangan_kanan_roll"; //done
        joint_state.name[15] = "16.tangan_kiri_roll"; //done
        joint_state.name[16] = "17.tangan_kanan_pitch"; //done
        joint_state.name[17] = "18.tangan_kiri_pitch"; //done
        joint_state.name[18] = "19.kepala_yaw"; //done
        joint_state.name[19] = "20.kepala_roll"; //done

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

        float p = imu.roll / PHI;
        float r = imu.pitch / PHI;
        float y = imu.yaw / PHI;


        // update transform
        // (moving in a circle with radius)
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.transform.translation.x = penjumlah_x;
        odom_trans.transform.translation.y = 0;
        odom_trans.transform.translation.z = penjumlah_z;
        odom_trans.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(r,p,y); //tf::createQuaternionMsgFromYaw(0);

        //send the joint state and transform
        joint_pub.publish(joint_state);
        broadcaster.sendTransform(odom_trans);

      angle += degree/4;

        // This will adjust as needed per iteration
      ros::spinOnce();
      loop_rate.sleep();
    }


    return 0;
}
