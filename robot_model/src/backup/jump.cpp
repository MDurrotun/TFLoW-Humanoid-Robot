#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "jump");
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(2000);//1000

    const double degree = M_PI/180;

    // robot state
    double inc= .0005; 
    double angle= 0;

    //kaki kanan
    double tf_11_kyaw_inc=.0005, tf_9_kpitch_inc=.0005, tf_7_kroll_inc=.0005, tf_5_kroll_inc=.0005, tf_3_kroll_inc=.0005, tf_1_kpitch_inc=.0005;
    double tf_11_kyaw=0, tf_9_kpitch=0, tf_7_kroll=0, tf_5_kroll=0, tf_3_kroll=0, tf_1_kpitch=0;

    //kaki kiri
    double tf_12_kiyaw_inc=.0005, tf_10_kipitch_inc=.0005, tf_8_kiroll_inc=.0005, tf_6_kiroll_inc=.0005, tf_4_kiroll_inc=.0005, tf_2_kipitch_inc=.0005;
    double tf_12_kiyaw=0, tf_10_kipitch=0, tf_8_kiroll=0, tf_6_kiroll=0, tf_4_kiroll=0, tf_2_kipitch=0;

    //perut
    double tf_13_proll_inc=.0005, tf_14_ppitch_inc=.0005, tf_15_pyaw_inc=.0005;
    double tf_13_proll=0, tf_14_ppitch=0, tf_15_pyaw=0;

    //kepala
    double tf_32_kyaw_inc=.0005, tf_33_kpitch_inc=.0005, tf_34_kroll_inc=.0005;
    double tf_32_kyaw=0, tf_33_kpitch=0, tf_34_kroll=0;

    //tangan kanan
    double tf_17_bkroll_inc=.0005, tf_19_bkyaw_inc=.0005, tf_23_skyaw_inc=.0005;
    double tf_17_bkroll=0, tf_19_bkyaw=0, tf_23_skyaw=0;

    //tangan kiri
    double tf_16_bkiroll_inc=.0005, tf_18_bkiyaw_inc=.0005, tf_22_skiyaw_inc=.0005;
    double tf_16_bkiroll=0, tf_18_bkiyaw=0, tf_22_skiyaw =0;

    // message declarations
    geometry_msgs::TransformStamped odom_trans;
    sensor_msgs::JointState joint_state;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    while (ros::ok()) {
        //update joint_state
        joint_state.header.stamp = ros::Time::now();
	joint_state.name.resize(25);
        joint_state.position.resize(25);
        joint_state.name[0] ="tflow_11_kyaw"; joint_state.position[0] = tf_11_kyaw; 
        joint_state.name[1] ="tflow_9_kpitch"; joint_state.position[1] = tf_9_kpitch;
        joint_state.name[2] ="tflow_7_kroll"; joint_state.position[2] = tf_7_kroll;
        joint_state.name[3] ="tflow_5_kroll"; joint_state.position[3] = tf_5_kroll;
        joint_state.name[4] ="tflow_3_kroll"; joint_state.position[4] = tf_3_kroll;
        joint_state.name[5] ="tflow_1_kpitch"; joint_state.position[5] = tf_1_kpitch;

        joint_state.name[6] ="tflow_12_kiyaw"; joint_state.position[6] = tf_12_kiyaw;
        joint_state.name[7] ="tflow_10_kipitch"; joint_state.position[7] = tf_10_kipitch;
        joint_state.name[8] ="tflow_8_kiroll"; joint_state.position[8] = tf_8_kiroll;
        joint_state.name[9] ="tflow_6_kiroll"; joint_state.position[9] = tf_6_kiroll;
        joint_state.name[10] ="tflow_4_kiroll"; joint_state.position[10] = tf_4_kiroll;
        joint_state.name[11] ="tflow_2_kipitch"; joint_state.position[11] = tf_2_kipitch;

	joint_state.name[12] ="tflow_13_proll";joint_state.position[12] = tf_13_proll; 
	joint_state.name[13] ="tflow_14_ppitch";joint_state.position[13] = tf_14_ppitch; 
        joint_state.name[14] ="tflow_15_pyaw"; joint_state.position[14] = tf_15_pyaw; 

	joint_state.name[15] ="tflow_32_kyaw"; joint_state.position[15] = tf_32_kyaw;
	joint_state.name[16] ="tflow_33_kpitch"; joint_state.position[16] = tf_33_kpitch;
	joint_state.name[17] ="tflow_34_kroll"; joint_state.position[17] = tf_34_kroll;

	joint_state.name[18] ="tflow_17_bkroll"; joint_state.position[18] = tf_17_bkroll;
	joint_state.name[19] ="tflow_19_bkyaw"; joint_state.position[19] = tf_19_bkyaw;
	joint_state.name[20] ="tflow_23_skyaw"; joint_state.position[20] = tf_23_skyaw;

	joint_state.name[21] ="tflow_16_bkiroll"; joint_state.position[21] = tf_16_bkiroll;
	joint_state.name[22] ="tflow_18_bkiyaw"; joint_state.position[22] = tf_18_bkiyaw;
	joint_state.name[23] ="tflow_22_skiyaw"; joint_state.position[23] = tf_22_skiyaw;

        // update transform
        // (moving in a circle with radius)
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.transform.translation.x = cos(angle);
        odom_trans.transform.translation.y = sin(angle);
        odom_trans.transform.translation.z = 0;
        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(angle);

        //send the joint state and transform
        joint_pub.publish(joint_state);
        broadcaster.sendTransform(odom_trans);

	// Create new robot state

	//tf_7_kroll=.25;
	//tf_5_kroll=-.25;

        //tf_11_kyaw += tf_11_kyaw_inc;
        //if (tf_11_kyaw > .63 || tf_11_kyaw < -.63) tf_11_kyaw_inc *= -1;

        //tf_9_kpitch += tf_9_kpitch_inc;
        //if (tf_9_kpitch > .53 || tf_9_kpitch < -.53) tf_9_kpitch_inc *= -1;


        tf_7_kroll += tf_7_kroll_inc; int x=tf_7_kroll-.25;
        if (tf_7_kroll > 1.53 || tf_7_kroll < x) tf_7_kroll_inc *= -1;
	
        tf_5_kroll += tf_5_kroll_inc; int kr5 = -.25; 
	if (tf_5_kroll > kr5 || tf_5_kroll < -1.53) tf_5_kroll_inc *= -1;

        tf_3_kroll += tf_3_kroll_inc;
        if (tf_3_kroll > .78 || tf_3_kroll < -.78) tf_3_kroll_inc *= -1;


        //tf_3_kroll += tf_3_kroll_inc;
        //if (tf_3_kroll > .78 || tf_3_kroll < -.78) tf_3_kroll_inc *= -1;

        //tf_1_kpitch += tf_1_kpitch_inc;
        //if (tf_1_kpitch > .35 || tf_1_kpitch < -.35) tf_1_kpitch_inc *= -1;

	//angle += degree/4;

        //tf_12_kiyaw += tf_12_kiyaw_inc;
        //if (tf_12_kiyaw > .63 || tf_12_kiyaw < -.63) tf_12_kiyaw_inc *= -1;

        //tf_10_kipitch += tf_10_kipitch_inc;
        //if (tf_10_kipitch > .53 || tf_10_kipitch < -.53) tf_10_kipitch_inc *= -1;

        tf_8_kiroll += tf_8_kiroll_inc; int i=tf_8_kiroll-.25;
        if (tf_8_kiroll > 1.53 || tf_8_kiroll < i) tf_8_kiroll_inc *= -1; 

        tf_6_kiroll+= tf_6_kiroll_inc; int kr6 = -.25;
        if (tf_6_kiroll > kr6 || tf_6_kiroll < -1.53) tf_6_kiroll_inc *= -1;

        tf_4_kiroll += tf_4_kiroll_inc;
        if (tf_4_kiroll > .78 || tf_4_kiroll < -.78) tf_4_kiroll_inc *= -1;

        //tf_2_kipitch += tf_2_kipitch_inc;
        //if (tf_2_kipitch > .35 || tf_2_kipitch < -.35) tf_2_kipitch_inc *= -1;

//PERUT
/*        tf_13_proll += tf_13_proll_inc;
        if (tf_13_proll > 1. || tf_13_proll < -1.0) tf_13_proll_inc *= -1;

	tf_14_ppitch += tf_14_ppitch_inc;
        if (tf_14_ppitch > 1 || tf_14_ppitch < -1.0) tf_14_ppitch_inc *= -1;

	tf_15_pyaw += tf_15_pyaw_inc;
        if (tf_15_pyaw < 1.5 || tf_15_pyaw > -1.5) tf_15_pyaw_inc *= -1;

//KEPALA
	tf_32_kyaw += tf_32_kyaw_inc;
        if (tf_32_kyaw < -1.5 || tf_32_kyaw > 1.5) tf_32_kyaw_inc *= -1;

	tf_33_kpitch += tf_33_kpitch_inc;
        if (tf_33_kpitch < -1.5 || tf_33_kpitch > 1.5) tf_33_kpitch_inc *= -1;

	tf_34_kroll += tf_34_kroll_inc;
        if (tf_34_kroll < .75 || tf_34_kroll > -.75) tf_34_kroll_inc *= -1;

//TANGAN
	tf_17_bkroll += tf_17_bkroll_inc;
        if (tf_17_bkroll < -1.5 || tf_17_bkroll > 1.5) tf_17_bkroll_inc *= -1;

	tf_19_bkyaw += tf_19_bkyaw_inc;
        if (tf_19_bkyaw < -1.5 || tf_19_bkyaw > 1.5) tf_19_bkyaw_inc *= -1;

	tf_23_skyaw += tf_23_skyaw_inc;
        if (tf_23_skyaw < -1.5 || tf_23_skyaw > 1.5) tf_23_skyaw_inc *= -1;

//TANGAN
	tf_16_bkiroll += tf_16_bkiroll_inc;
        if (tf_16_bkiroll < -1.5 || tf_16_bkiroll > 1.5) tf_16_bkiroll_inc *= -1;

	tf_18_bkiyaw += tf_18_bkiyaw_inc;
        if (tf_18_bkiyaw < -1.5 || tf_18_bkiyaw > 1.5) tf_18_bkiyaw_inc *= -1;

	tf_22_skiyaw += tf_22_skiyaw_inc;
        if (tf_22_skiyaw < -1.5 || tf_22_skiyaw > 1.5) tf_22_skiyaw_inc *= -1;
*/
angle += degree/4;
angle += degree/4;
angle += degree/4;
angle += degree/4;
        // This will adjust as needed per iteration
        loop_rate.sleep();
    }
    return 0;
}

/*
//menendang sederhana
        tf_11_kyaw += tf_11_kyaw_inc;
        if (tf_11_kyaw > .63 || tf_11_kyaw < -.63) {
		tf_11_kyaw_inc *= -1;

        	tf_9_kpitch += tf_9_kpitch_inc;
        	if (tf_9_kpitch > .53 || tf_9_kpitch < -.53) tf_9_kpitch_inc *= -1;

        	tf_7_kroll += tf_7_kroll_inc;
        	if (tf_7_kroll > 1.53 || tf_7_kroll < -1.53) tf_7_kroll_inc *= -1;

        	tf_5_kroll += tf_5_kroll_inc;
        	if (tf_5_kroll > 1.53 || tf_5_kroll < -1.53) tf_5_kroll_inc *= -1;
	}
        tf_3_kroll += tf_3_kroll_inc;
        if (tf_3_kroll > .78 || tf_3_kroll < -.78) tf_3_kroll_inc *= -1;

        tf_1_kpitch += tf_1_kpitch_inc;
        if (tf_1_kpitch > .35 || tf_1_kpitch < -.35) tf_1_kpitch_inc *= -1;
*/
