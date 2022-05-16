#include <ros/ros.h>

#include <iostream>
#include <cmath>
#include <stdlib.h>
#include <px4_command/command.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

//topic ͷ�ļ�
#include <iostream>
#include <px4_command/command.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>
#include <stdlib.h>
#include <math_utils.h>

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
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>�� �� �� ��<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void cal_min_distance();
float satfunc(float data, float Max);
void printf();                                                                       //��ӡ����
void printf_param();                                                                 //��ӡ��������Թ����
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
		Command_now.pos_sp[1] = -3;//�ߵ���������
		Command_now.pos_sp[2] = height_square;
		Command_now.yaw_sp = 0;
		Command_now.comid = comid;
		comid++;
		move_pub.publish(Command_now);
		rate.sleep();
		cout << "Point 2----->left" << endl;
		i++;
	}
//��ͷ�ı䳯��
	i = 0;

	while (i < sleep_time) {
		Command_now.command = Move_ENU;
		Command_now.sub_mode = 0;
		Command_now.pos_sp[0] = 3.5;
		Command_now.pos_sp[1] = -3;//�ߵ���������
		Command_now.pos_sp[2] = height_square;
		Command_now.yaw_sp = 1;
		Command_now.comid = comid;
		comid++;
		move_pub.publish(Command_now);
		rate.sleep();
		cout << "Point 3----->left" << endl;
		i++;//ת��
	}
//���뷿�䣡

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

	flag_land = 0;

	comid = 1;
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
}

void printf() {
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

void printf_param() {
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
void cal_min_distance() {
	distance_c = Laser.ranges[range_min];
	angle_c = 0;
	for (int i = range_min; i <= range_max; i++) {
		if (Laser.ranges[i] < distance_c) {
			distance_c = Laser.ranges[i];
			angle_c = i;
		}
	}
}
void collision_avoidance(float target_x, float target_y) {

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
float satfunc(float data, float Max) {
	if (abs(data) > Max)
		return ( data > 0 ) ? Max : -Max;
	else
		return data;
}

