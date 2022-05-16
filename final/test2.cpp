#include <ros/ros.h>

#include <iostream>
#include <cmath>
#include <stdlib.h>
#include <px4_command/command.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

//topic ͷ�ļ�
#include <darknet_ros_msgs/BoundingBoxes.h> //Ŀ����
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <math_utils.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#define PAI 3.1415926

using namespace std;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>ȫ �� �� ��<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
enum Command {
	Move_ENU,
	Move_Body,
	Hold,
	Takeoff,
	Land,
	Arm,
	Disarm,
	Failsafe_land,
	Idle
};
sensor_msgs::LaserScan Laser;                                   //�����״��������
geometry_msgs::PoseStamped pos_drone;                                  //���˻���ǰλ��
Eigen::Quaterniond q_fcu;
Eigen::Vector3d Euler_fcu;
float target_x;                                                 //����λ��_x
float target_y;                                                 //����λ��_y
float final_x;                                                 //����λ��_x�����ţ� 
float final_y;                                                 //����λ��_y�����ţ� 

float door_x;
float door_y;
int range_min;                                                //�����״�̽�ⷶΧ ��С�Ƕ�
int range_max;                                                //�����״�̽�ⷶΧ ���Ƕ�
float last_time = 0;
float fly_height;
//--------------------------------------------�㷨���--------------------------------------------------
float R_outside, R_inside;                                      //��ȫ�뾶 [�����㷨��ز���]
float p_R;                                                      //��Ȧ��������
float p_r;                                                      //СȦ��������
float distance_c, angle_c;                                      //����ϰ������ �Ƕ�
float distance_cx, distance_cy;                                 //����ϰ������XY
float vel_collision[2];                                         //����ϰ������ٶ�
float vel_collision_max;                                        //����ϰ������ٶ��޷�
float p_xy;                                                     //׷�ٲ���λ�û�P
float vel_track[2];                                             //׷�ٲ����ٶ�
float vel_track_max;                                            //׷�ٲ����ٶ��޷�
int flag_land;                                                  //�����־λ
//--------------------------------------------���--------------------------------------------------
std_msgs::Bool flag_collision_avoidance;                       //�Ƿ�������ģʽ��־λ
float vel_sp_body[2];                                           //���ٶ�
float vel_sp_ENU[2];                                            //ENU�µ����ٶ�
float vel_sp_max;                                               //���ٶ��޷�
px4_command::command Command_now;                               //���͸�position_control.cpp������
//---------------------------------------------���ű�����------------------------------------------- 
float door_center_x[2];                                         //ǰ�����ŵ�x����
float door_center_y[2];                                         //ǰ�����ŵ�y����
bool reach_door_flag[2];                                        //����ǰ�����ŵı�־
//--------------------------------------------Ŀ���⣬ʶ������-------------------------------------------------
int detect_num;                                                  //darknet�����ļ�⵽��������Ŀ
darknet_ros_msgs::BoundingBox darknet_box;                       //����ģʽ4ֻ��ʶ��һ��ͼ�����
darknet_ros_msgs::BoundingBoxes darknet_boxes;                   //����ģʽ5��Ҫʶ������ͼ�����
int flag_hold;                                                   //��ͣ��־
float fx=554.3827;                                               //����ڲ�
float fy=554.3827;
float cx=320;
float cy=240;
float pic_target[2];                                             //ģʽ4��ͼ������ENU����
float abs_distance1=10;                                          //Ϊģʽ4�д�Խ2����ʶ��ͼ��֮��Ĺ��ȶ����õ���С����ֵ
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>�� �� �� ��<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void cal_min_distance();
float satfunc(float data, float Max);
void printf();                                                                       //��ӡ����
void printf_param(); 
void finddoorcentor(int i);                                                                  //��ӡ��������Թ����
void collision_avoidance(float target_x, float target_y);

// ������ϵ��ת������- ����ϵ��enuϵ
// input�ǻ���ϵ,output�ǹ���ϵ��yaw_angle�ǵ�ǰƫ����
void rotation_yaw(float yaw_angle, float input[2], float output[2]) {
	output[0] = input[0] * cos(yaw_angle) - input[1] * sin(yaw_angle);
	output[1] = input[0] * sin(yaw_angle) + input[1] * cos(yaw_angle);
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>�� �� �� ��<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//�����״�����ݣ�������Ӧ����,Ȼ�����ǰ������������С����
void lidar_cb(const sensor_msgs::LaserScan::ConstPtr &scan) {
	sensor_msgs::LaserScan Laser_tmp;
	Laser_tmp = *scan;
	Laser = *scan;
	int count;    //count = 359
	count = Laser.ranges.size();

	//�޳�inf�����
	for (int i = 0; i < count; i++) {
		//�ж��Ƿ�Ϊinf
		int a = isinf(Laser_tmp.ranges[i]);
		//���Ϊinf����ֵ��һ�Ƕȵ�ֵ
		if (a == 1) {
			if (i == 0) {
				Laser_tmp.ranges[i] = Laser_tmp.ranges[count - 1];
			} else {
				Laser_tmp.ranges[i] = Laser_tmp.ranges[i - 1];
			}
		}

	}
	for (int i = 0; i < count; i++) {
		if (i + 180 > 359)
			Laser.ranges[i] = Laser_tmp.ranges[i - 180];
		else
			Laser.ranges[i] = Laser_tmp.ranges[i + 180];
		//cout<<"tmp: "<<i<<" l:"<<Laser_tmp.ranges[i]<<"|| Laser: "<<Laser.ranges[i]<<endl;
	}
	//cout<<"//////////////"<<endl;
	//����ǰ������������С����
	cal_min_distance();
}

void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
	pos_drone = *msg;
	// Read the Quaternion from the Mavros Package [Frame: ENU]
	Eigen::Quaterniond q_fcu_enu(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y,
	                             msg->pose.orientation.z);
	q_fcu = q_fcu_enu;
	//Transform the Quaternion to Euler Angles
	Euler_fcu = quaternion_to_euler(q_fcu);
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>�� �� ��<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//px4_command::command Command_now;
//---------------------------------------�����β���---------------------------------------------
float size_square; //�����α߳�
float height_square;                //���и߶�
float sleep_time;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>�� �� ��<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv) {
	ros::init(argc, argv, "square");//��ʼ��
	ros::NodeHandle nh("~");//���ɽڵ���
	ros::Rate rate(20.0);
	ros::Publisher move_pub = nh.advertise<px4_command::command>("/px4/command", 10);
	//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>������ȡ<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	nh.param<float>("size_square", size_square, 1.0);
	nh.param<float>("height_square", height_square, 0.5);
	nh.param<float>("sleep_time", sleep_time, 10.0);
	
	//����Ϊ�������ݣ����ڴ��ţ� 
    nh.param<float>("R_outside", R_outside, 2);
    nh.param<float>("R_inside", R_inside, 1);

    nh.param<float>("p_xy", p_xy, 0.5);

    nh.param<float>("vel_track_max", vel_track_max, 0.5);

    nh.param<float>("p_R", p_R, 0.0);
    nh.param<float>("p_r", p_r, 0.0);

    nh.param<float>("vel_collision_max", vel_collision_max, 0.0);
    nh.param<float>("vel_sp_max", vel_sp_max, 0.0);

    nh.param<int>("range_min", range_min, 0.0);
    nh.param<int>("range_max", range_max, 0.0);

    nh.param<float>("final_x", final_x, 2.0);
    nh.param<float>("final_y", final_y, 0.0);

    nh.param<float>("door_x", door_x, 1.0);
    nh.param<float>("door_y", door_y, 1.7);

	// ��һ����Ϊ�˳�������ǰ���һ�²����Ƿ���ȷ
	// ����1,�������������˳�����
	int check_flag;
	cout << "size_square: " << size_square << "[m]" << endl;
	cout << "height_square: " << height_square << "[m]" << endl;
	cout << "Please check the parameter and setting��1 for go on�� else for quit: " << endl;
	cin >> check_flag;//��ӡԤ��������۲��ǲ��ǶԵ�
	if (check_flag != 1) {
		return -1;
	}

	//check arm
	int Arm_flag;
	cout << "Whether choose to Arm? 1 for Arm, 0 for quit" << endl;
	cin >> Arm_flag;
	if (Arm_flag == 1) {
		Command_now.command = Arm;//����һ������
		move_pub.publish(Command_now);
	} else
		return -1;

	int takeoff_flag;
	cout << "Whether choose to Takeoff? 1 for Takeoff, 0 for quit " << endl;
	cin >> takeoff_flag;
	if (takeoff_flag != 1) {
		return -1;
	}
	int i = 0;
	int comid = 0;//����ָ����
	
	//	ros::init(argc, argv, "collision_avoidance");
//	ros::NodeHandle nh("~");
//	// Ƶ�� [20Hz]
//	ros::Rate rate(20.0);
	//�����ġ�Lidar����
	ros::Subscriber lidar_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1000, lidar_cb);
	//�����ġ����˻���ǰλ�� ����ϵ NEDϵ
	ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 100, pos_cb);
	// �����������͸�position_control.cpp������

	//��ȡ�������еĲ���
	nh.param<float>("target_x", target_x, 1.0); //dyx
	nh.param<float>("target_y", target_y, 0.0); //dyx

	nh.param<float>("R_outside", R_outside, 2);
	nh.param<float>("R_inside", R_inside, 1);

	nh.param<float>("p_xy", p_xy, 0.5);

	nh.param<float>("vel_track_max", vel_track_max, 0.5);

	nh.param<float>("p_R", p_R, 0.0);
	nh.param<float>("p_r", p_r, 0.0);

	nh.param<float>("vel_collision_max", vel_collision_max, 0.0);
	nh.param<float>("vel_sp_max", vel_sp_max, 0.0);

	nh.param<int>("range_min", range_min, 0.0);
	nh.param<int>("range_max", range_max, 0.0);
	nh.getParam("/px4_pos_controller/Takeoff_height", fly_height);
	//��ӡ��ʵ������
	printf_param();

	//check paramater
//	int check_flag;

	//��ֵ
	vel_track[0] = 0;
	vel_track[1] = 0;

	vel_collision[0] = 0;
	vel_collision[1] = 0;

	vel_sp_body[0] = 0;
	vel_sp_body[1] = 0;

	vel_sp_ENU[0] = 0;
	vel_sp_ENU[1] = 0;
	//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>������<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	//takeoff
	i = 0;

	while (i < sleep_time) { //�ݵ�
		Command_now.command = Move_ENU;//���Ŵ������ϵ�ƶ�������ڵ�ǰ��λ�ö��ԣ�
		Command_now.sub_mode = 0;//����ģʽ
		Command_now.pos_sp[0] = 0;//x=0�ƶ���ĳһ����
		Command_now.pos_sp[1] = 0;//y=0ԭ�����
		Command_now.pos_sp[2] = height_square;//��ɸ߶���1m
		Command_now.yaw_sp = 0;//ƫ������0�ȣ�û��ƫ����
		Command_now.comid = comid;//����һ���������һ����id��ǰ���һ�������id��Ż�ִ�С�
		comid++;//���Զ��ط�����������˵����ط��İ��ڷ������ϻ��ӵ� �����Ҫ�·���һ��������Ҫ++
		//���ѭ�����ܻᷢ���ܶ�����������ͨ�����磬���ܻ��Զ��ط�
		//mid���������ִ�ж��ٴν����һ��
		move_pub.publish(Command_now);
		rate.sleep();
		cout << "Point 0----->takeoff" << endl;
		i++;
	}

	i = 0;
	while (i < sleep_time) {
		Command_now.command = Move_ENU;
		Command_now.sub_mode = 0;
		Command_now.pos_sp[0] = 3.5;//��ǰ��3.5m�ľ���
		Command_now.pos_sp[1] = 0;//��Ϊ��
		Command_now.pos_sp[2] = height_square;//Ѳ���߶���1m ���ֲ���
		Command_now.yaw_sp = 0;
		Command_now.comid = comid;
		comid++;
		move_pub.publish(Command_now);
		rate.sleep();
		cout << "Point 1----->forward" << endl;
		i++;
	}

	i = 0;
	while (i < sleep_time) {
		Command_now.command = Move_ENU;
		Command_now.sub_mode = 0;
		Command_now.pos_sp[0] = 3.5;
		Command_now.pos_sp[1] = 0;//�ߵ���������
		Command_now.pos_sp[2] = height_square;
		Command_now.yaw_sp = 0.5*PAI;
		Command_now.comid = comid;
		comid++;
		move_pub.publish(Command_now);
		rate.sleep();
		cout << "Point 2----->turn left" << endl;
		i++;//ת��
	}
	
	
	
//	i = 0;
//	while (i < sleep_time) {
//		Command_now.command = Move_ENU;
//		Command_now.sub_mode = 0;
//		Command_now.pos_sp[0] = 3.5;
//		Command_now.pos_sp[1] = -3;//�ߵ���������
//		Command_now.pos_sp[2] = height_square;
//		Command_now.yaw_sp = 0;
//		Command_now.comid = comid;
//		comid++;
//		move_pub.publish(Command_now);
//		rate.sleep();
//		cout << "Point 2----->left" << endl;
//		i++;
//	}
////��ͷ�ı䳯��
//	i = 0;
//
//	while (i < sleep_time) {
//		Command_now.command = Move_ENU;
//		Command_now.sub_mode = 0;
//		Command_now.pos_sp[0] = 3.5;
//		Command_now.pos_sp[1] = -3;//�ߵ���������
//		Command_now.pos_sp[2] = height_square;
//		Command_now.yaw_sp = 1;
//		Command_now.comid = comid;
//		comid++;
//		move_pub.publish(Command_now);
//		rate.sleep();
//		cout << "Point 3----->left" << endl;
//		i++;//ת��
//	}
//���뷿�䣡

//--------------------------------------�ָ���-��������----------------------------------------------------------------------- 
    //������С���� ��ֵ
    flag_land = 0;


//    //���ָ���ʼ��
//    comid = 1;
    
while(ros::ok())
    {
        //�ص�һ�� ���´�����״̬
        //1. �����״�������ݣ��洢��Laser��,������������С����
        ros::spinOnce();
        /**************************dyx****************************************/ //change: just cross one door
        //���ԣ�����ԭ�����������һ���Ŷ�ǽ������ͼƬ����ʶ�𲢼���ͼƬ����xy���꣬������ȥ��
        /*
        if(!reach_door_flag[0]) finddoorcentor(0);
        else if(reach_door_flag[0]&&!reach_door_flag[1])		//�����Ǵ������ŵĴ��� 
        {
           if(detect_num&&abs_distance1<0.3)
           {
              cout<<"detectnum "<<detect_num<<endl;
              reach_door_flag[1]=true;
           }
              else finddoorcentor(1);
              abs_distance1 = sqrt((pos_drone.pose.position.x - door_center_x[1]) * (pos_drone.pose.position.x - door_center_x[1]) + (pos_drone.pose.position.y - door_center_y[1]) * (pos_drone.pose.position.y - door_center_y[1]));
        }
        else if(reach_door_flag[0]&&reach_door_flag[1]) detect_nav();
        */
        if(!reach_door_flag[0]) //finddoorcentor(0);		//�����û����
        {
           collision_avoidance(door_x+0.1,door_y);
           float abs_distance;
           abs_distance = sqrt((pos_drone.pose.position.x - door_x - 0.1) * (pos_drone.pose.position.x -door_x - 0.1) + (pos_drone.pose.position.y - door_y) * (pos_drone.pose.position.y - door_y));
           if(abs_distance < 0.3 )
           {
              reach_door_flag[0]=true;
           }
        }
        else if(reach_door_flag[0])			//���������
        {
           collision_avoidance(final_x,final_y);
//           collision_avoidance(target_x,target_y);
//           float abs_distance;
//           abs_distance = sqrt((pos_drone.pose.position.x - final_x) * (pos_drone.pose.position.x -final_x) + (pos_drone.pose.position.y - final_y) * (pos_drone.pose.position.y - final_y));
//           if(abs_distance < 0.3 )
//           {
//              flag_land = 1;	//�����־λ
//           }
        }
//--------------------------------------�ָ��߽���----------------------------------------------------------------------------

////	ros::init(argc, argv, "collision_avoidance");
////	ros::NodeHandle nh("~");
////	// Ƶ�� [20Hz]
////	ros::Rate rate(20.0);
//	//�����ġ�Lidar����
//	ros::Subscriber lidar_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1000, lidar_cb);
//	//�����ġ����˻���ǰλ�� ����ϵ NEDϵ
//	ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 100, pos_cb);
//	// �����������͸�position_control.cpp������
//
//	//��ȡ�������еĲ���
//	nh.param<float>("target_x", target_x, 1.0); //dyx
//	nh.param<float>("target_y", target_y, 0.0); //dyx
//
//	nh.param<float>("R_outside", R_outside, 2);
//	nh.param<float>("R_inside", R_inside, 1);
//
//	nh.param<float>("p_xy", p_xy, 0.5);
//
//	nh.param<float>("vel_track_max", vel_track_max, 0.5);
//
//	nh.param<float>("p_R", p_R, 0.0);
//	nh.param<float>("p_r", p_r, 0.0);
//
//	nh.param<float>("vel_collision_max", vel_collision_max, 0.0);
//	nh.param<float>("vel_sp_max", vel_sp_max, 0.0);
//
//	nh.param<int>("range_min", range_min, 0.0);
//	nh.param<int>("range_max", range_max, 0.0);
//	nh.getParam("/px4_pos_controller/Takeoff_height", fly_height);
//	//��ӡ��ʵ������
//	printf_param();
//
//	//check paramater
////	int check_flag;
//
//	//��ֵ
//	vel_track[0] = 0;
//	vel_track[1] = 0;
//
//	vel_collision[0] = 0;
//	vel_collision[1] = 0;
//
//	vel_sp_body[0] = 0;
//	vel_sp_body[1] = 0;
//
//	vel_sp_ENU[0] = 0;
//	vel_sp_ENU[1] = 0;

	flag_land = 0;

//	comid = 1;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Main Loop<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	while (ros::ok()) {
		//�ص�һ�� ���´�����״̬
		//1. �����״�������ݣ��洢��Laser��,������������С����
		ros::spinOnce();
		collision_avoidance(target_x, target_y);

		Command_now.command = Move_ENU;     //����ϵ���ƶ�
		Command_now.comid = comid;
		comid++;
		Command_now.sub_mode = 2; // xy �ٶȿ���ģʽ z λ�ÿ���ģʽ
		Command_now.vel_sp[0] =  vel_sp_ENU[0];
		Command_now.vel_sp[1] =  vel_sp_ENU[1];  //ENU frame
		Command_now.pos_sp[2] =  fly_height;
		Command_now.yaw_sp = 0 ;

		float abs_distance;
		abs_distance = sqrt((pos_drone.pose.position.x - target_x) * (pos_drone.pose.position.x - target_x) +
		                    (pos_drone.pose.position.y - target_y) * (pos_drone.pose.position.y - target_y));
		if (abs_distance < 0.3 || flag_land == 1) {
			Command_now.command = 3;     //Land
			flag_land = 1;
		}
		if (flag_land == 1)
			Command_now.command = Land;
		move_pub.publish(Command_now);
		//��ӡ
		printf();
		rate.sleep();
	}
	return 0;
	}  //end of ���� 
}//end of main

void printf()
{
	cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>collision_avoidance<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" << endl;
	cout << "Minimun_distance : " << endl;
	cout << "Distance : " << distance_c << " [m] " << endl;
	cout << "Angle :    " << angle_c    << " [du] " << endl;
	cout << "distance_cx :    " << distance_cx    << " [m] " << endl;
	cout << "distance_cy :    " << distance_cy    << " [m] " << endl;
	if (flag_collision_avoidance.data == true) {
		cout << "Collision avoidance Enabled " << endl;
	} else {
		cout << "Collision avoidance Disabled " << endl;
	}
	cout << "vel_track_x : " << vel_track[0] << " [m/s] " << endl;
	cout << "vel_track_y : " << vel_track[1] << " [m/s] " << endl;

	cout << "vel_collision_x : " << vel_collision[0] << " [m/s] " << endl;
	cout << "vel_collision_y : " << vel_collision[1] << " [m/s] " << endl;

	cout << "vel_sp_x : " << vel_sp_ENU[0] << " [m/s] " << endl;
	cout << "vel_sp_y : " << vel_sp_ENU[1] << " [m/s] " << endl;
}

void printf_param()
{
	cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Parameter <<<<<<<<<<<<<<<<<<<<<<<<<<<" << endl;
	cout << "target_x : " << target_x << endl;
	cout << "target_y : " << target_y << endl;

	cout << "R_outside : " << R_outside << endl;
	cout << "R_inside : " << R_inside << endl;

	cout << "p_xy : " << p_xy << endl;
	cout << "vel_track_max : " << vel_track_max << endl;

	cout << "p_R : " << p_R << endl;
	cout << "p_r : " << p_r << endl;

	cout << "vel_collision_max : " << vel_collision_max << endl;

	cout << "vel_sp_max : " << vel_sp_max << endl;
	cout << "range_min : " << range_min << endl;
	cout << "range_max : " << range_max << endl;
	cout << "fly heigh: " << fly_height << endl;
}


//����ǰ������������С����
void cal_min_distance()
{
	distance_c = Laser.ranges[range_min];
	angle_c = 0;
	for (int i = range_min; i <= range_max; i++) {
		if (Laser.ranges[i] < distance_c) {
			distance_c = Laser.ranges[i];
			angle_c = i;
		}
	}
}

void collision_avoidance(float target_x, float target_y)
{
	//2. ������С�����жϣ��Ƿ����ñ��ϲ���
	if (distance_c >= R_outside ) {
		flag_collision_avoidance.data = false;
	} else {
		flag_collision_avoidance.data = true;
	}
	//3. ����׷���ٶ�
	vel_track[0] = p_xy * (target_x - pos_drone.pose.position.x);
	vel_track[1] = p_xy * (target_y - pos_drone.pose.position.y);
	//�ٶ��޷�
	for (int i = 0; i < 2; i++) {
		vel_track[i] = satfunc(vel_track[i], vel_track_max);
	}
	vel_collision[0] = 0;
	vel_collision[1] = 0;
	//4. ���ϲ���
	if (flag_collision_avoidance.data == true) {
		distance_cx = distance_c * cos(angle_c / 180 * 3.1415926);
		distance_cy = distance_c * sin(angle_c / 180 * 3.1415926);

		float F_c;

		F_c = 0;

		if (distance_c > R_outside) {
			//���ٶȲ�������
			vel_collision[0] = vel_collision[0] + 0;
			vel_collision[1] = vel_collision[1] + 0;
			cout << " Forward Outside " << endl;
		}
		//С���������ƶ��ٶ�
		if (distance_c > R_inside && distance_c <= R_outside) {
			F_c = p_R * (R_outside - distance_c);
		}
		//����������ƶ��ٶ�
		if (distance_c <= R_inside ) {
			F_c = p_R * (R_outside - R_inside) + p_r * (R_inside - distance_c);
		}

		if (distance_cx > 0) {
			vel_collision[0] = vel_collision[0] - F_c * distance_cx / distance_c;
		} else {
			vel_collision[0] = vel_collision[0] - F_c * distance_cx / distance_c;
		}

		if (distance_cy > 0) {
			vel_collision[1] = vel_collision[1] - F_c * distance_cy / distance_c;
		} else {
			vel_collision[1] = vel_collision[1] - F_c * distance_cy / distance_c;
		}
		//�����ٶ��޷�
		for (int i = 0; i < 2; i++) {
			vel_collision[i] = satfunc(vel_collision[i], vel_collision_max);
		}
	}

	vel_sp_body[0] = vel_track[0] + vel_collision[0];
	vel_sp_body[1] = vel_track[1] + vel_collision[1]; //dyx

	//�ҵ�ǰλ�õ�Ŀ����xy��ֵ�������������һ����ֵС����һ����ֵ��
	//�ҹ���һ�ỹ�Ǳ��������ֵ�Ϳ�ʼ�Ӳ�ֵ���֡�
	//���磬y����ӽ�0����x����ܶ࣬��x�������ϰ������ʱ��discx cy�Ĵ�С������y�����⡣

	for (int i = 0; i < 2; i++) {
		vel_sp_body[i] = satfunc(vel_sp_body[i], vel_sp_max);
	}
	rotation_yaw(Euler_fcu[2], vel_sp_body, vel_sp_ENU);
}

//���ͺ���
float satfunc(float data, float Max)
{
	if (abs(data) > Max)
		return ( data > 0 ) ? Max : -Max;
	else
		return data;
}

//˼·���������ǽ�ļ������ݻ��кܴ��ͻ�䣬�ڼ����������ҵ����ͻ�䷶Χ����ת����ENU����ϵ�¼�������ŵ�����
void finddoorcentor(int i)
{
    //1.if no centor , findcentor set target
    //2.use collision_avoidance
    //3.judge whether reach target

    //1.
    float a,b,c;
    double l;
    cout<<"********************"<<endl;
    if(!door_center_x[i])				//�����û�е�������
    {
        a=Laser.ranges[0];
        b=Laser.ranges[89];
        c=Laser.ranges[270];			//��ȡ�����״�����
        int theta1=atan(b/a)/3.1415926*180;		//�Ƕ�1		//atan�����������ֵķ�����ֵ
        int theta2=atan(c/a)/3.1415926*180;		//�Ƕ�2
        cout<<"theta1: "<<theta1<<endl;
        cout<<"theta2: "<<theta2<<endl;
        std::vector<int> door_angle;		//��ʼ��vector������һ���ն���
        door_angle.reserve(theta1+theta2);
        for(int k=theta1;k>0;k--){
            float angle=k;
            l=a/cos(angle/180*3.1415926);
            float dl=abs(l-Laser.ranges[k]);
            if(dl>1) door_angle.push_back(k);
            //cout<<"k: "<<k<<" l: "<<l<<" Laser: "<<Laser.ranges[k]<<" dl: "<<dl<<endl;
        }
        for(int k=0;k<=theta2;k++){
            float angle=k;
            l=a/cos(angle/180*3.1415926);
            float dl=abs(l-Laser.ranges[359-k]);
            if(dl>1) door_angle.push_back(359-k);
            //cout<<"k: "<<359-k<<" l: "<<l<<" Laser: "<<Laser.ranges[359-k]<<" dl: "<<dl<<endl;
        }
        cout<<"door angle num: "<<door_angle.size()<<endl;
        cout<<"first :"<<door_angle.front()<<"last one: "<<door_angle.back()<<endl;
        int the1 = door_angle.front();
        int the2 = door_angle.back();
        float angle1,angle2;
        float x1,x2,y1,y2;
        x1=a;
        x2=a;
        if(the1>270)
        {
            angle1=359-the1;
            y1=-a*tan(angle1/180*3.1415926);
        }
        else
        {
            angle1=the1;
            y1=a*tan(angle1/180*3.1415926);
        }
        if(the2>270)
        {
            angle2=359-the2;
            y2=-a*tan(angle2/180*3.1415926);
        }
        else
        {
            angle2=the2;
            y2=a*tan(angle2/180*3.1415926);
        }


        cout<<"x1 y1: "<<x1<<" "<<y1<<endl;
        cout<<"x2 y2: "<<x2<<" "<<y2<<endl;

        door_center_x[i]=(x1+x2)/2+pos_drone.pose.position.x;
        door_center_y[i]=(y1+y2)/2+pos_drone.pose.position.y;
        cout<<"door position: "<<door_center_x[i]<<" "<<door_center_y[i]<<endl;
    }//end of if(!door_center_x[i]) 
    
    //���������ŵ�����λ�����꣨ENU����ϵ��
    collision_avoidance(door_center_x[i]+0.1,door_center_y[i]);
    float abs_distance;
    abs_distance = sqrt((pos_drone.pose.position.x - door_center_x[i]-0.1) * (pos_drone.pose.position.x - door_center_x[i]-0.1) + (pos_drone.pose.position.y - door_center_y[i]) * (pos_drone.pose.position.y - door_center_y[i]));
    //cout<<"abs_distance: "<<abs_distance<<endl;
    cout<<"door position: "<<door_center_x[i]<<" "<<door_center_y[i]<<endl;
    if(abs_distance < 0.3 )
    {
        reach_door_flag[i]=true;

    }
}
