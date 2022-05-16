#include <ros/ros.h>

#include <iostream>
#include <cmath>
#include <stdlib.h>
#include <px4_command/command.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

//topic 头文件
#include <darknet_ros_msgs/BoundingBoxes.h> //目标检测
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
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
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
sensor_msgs::LaserScan Laser;                                   //激光雷达点云数据
geometry_msgs::PoseStamped pos_drone;                                  //无人机当前位置
Eigen::Quaterniond q_fcu;
Eigen::Vector3d Euler_fcu;
float target_x;                                                 //期望位置_x
float target_y;                                                 //期望位置_y
float final_x;                                                 //期望位置_x（穿门） 
float final_y;                                                 //期望位置_y（穿门） 

float door_x;
float door_y;
int range_min;                                                //激光雷达探测范围 最小角度
int range_max;                                                //激光雷达探测范围 最大角度
float last_time = 0;
float fly_height;
//--------------------------------------------算法相关--------------------------------------------------
float R_outside, R_inside;                                      //安全半径 [避障算法相关参数]
float p_R;                                                      //大圈比例参数
float p_r;                                                      //小圈比例参数
float distance_c, angle_c;                                      //最近障碍物距离 角度
float distance_cx, distance_cy;                                 //最近障碍物距离XY
float vel_collision[2];                                         //躲避障碍部分速度
float vel_collision_max;                                        //躲避障碍部分速度限幅
float p_xy;                                                     //追踪部分位置环P
float vel_track[2];                                             //追踪部分速度
float vel_track_max;                                            //追踪部分速度限幅
int flag_land;                                                  //降落标志位
//--------------------------------------------输出--------------------------------------------------
std_msgs::Bool flag_collision_avoidance;                       //是否进入避障模式标志位
float vel_sp_body[2];                                           //总速度
float vel_sp_ENU[2];                                            //ENU下的总速度
float vel_sp_max;                                               //总速度限幅
px4_command::command Command_now;                               //发送给position_control.cpp的命令
//---------------------------------------------穿门避障用------------------------------------------- 
float door_center_x[2];                                         //前两道门的x坐标
float door_center_y[2];                                         //前两道门的y坐标
bool reach_door_flag[2];                                        //到达前两道门的标志
//--------------------------------------------目标检测，识别降落用-------------------------------------------------
int detect_num;                                                  //darknet发布的检测到的物体数目
darknet_ros_msgs::BoundingBox darknet_box;                       //用于模式4只用识别一张图的情况
darknet_ros_msgs::BoundingBoxes darknet_boxes;                   //用于模式5需要识别三张图的情况
int flag_hold;                                                   //悬停标志
float fx=554.3827;                                               //相机内参
float fy=554.3827;
float cx=320;
float cy=240;
float pic_target[2];                                             //模式4的图像中心ENU坐标
float abs_distance1=10;                                          //为模式4中穿越2门与识别图像之间的过度而设置的最小距离值
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>声 明 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void cal_min_distance();
float satfunc(float data, float Max);
void printf();                                                                       //打印函数
void printf_param(); 
void finddoorcentor(int i);                                                                  //打印各项参数以供检查
void collision_avoidance(float target_x, float target_y);

// 【坐标系旋转函数】- 机体系到enu系
// input是机体系,output是惯性系，yaw_angle是当前偏航角
void rotation_yaw(float yaw_angle, float input[2], float output[2]) {
	output[0] = input[0] * cos(yaw_angle) - input[1] * sin(yaw_angle);
	output[1] = input[0] * sin(yaw_angle) + input[1] * cos(yaw_angle);
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回 调 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//接收雷达的数据，并做相应处理,然后计算前后左右四向最小距离
void lidar_cb(const sensor_msgs::LaserScan::ConstPtr &scan) {
	sensor_msgs::LaserScan Laser_tmp;
	Laser_tmp = *scan;
	Laser = *scan;
	int count;    //count = 359
	count = Laser.ranges.size();

	//剔除inf的情况
	for (int i = 0; i < count; i++) {
		//判断是否为inf
		int a = isinf(Laser_tmp.ranges[i]);
		//如果为inf，则赋值上一角度的值
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
	//计算前后左右四向最小距离
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
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//px4_command::command Command_now;
//---------------------------------------正方形参数---------------------------------------------
float size_square; //正方形边长
float height_square;                //飞行高度
float sleep_time;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv) {
	ros::init(argc, argv, "square");//初始化
	ros::NodeHandle nh("~");//生成节点句柄
	ros::Rate rate(20.0);
	ros::Publisher move_pub = nh.advertise<px4_command::command>("/px4/command", 10);
	//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>参数读取<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	nh.param<float>("size_square", size_square, 1.0);
	nh.param<float>("height_square", height_square, 0.5);
	nh.param<float>("sleep_time", sleep_time, 10.0);
	
	//以下为新填内容（用于穿门） 
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

	// 这一步是为了程序运行前检查一下参数是否正确
	// 输入1,继续，其他，退出程序
	int check_flag;
	cout << "size_square: " << size_square << "[m]" << endl;
	cout << "height_square: " << height_square << "[m]" << endl;
	cout << "Please check the parameter and setting，1 for go on， else for quit: " << endl;
	cin >> check_flag;//打印预设参数，观察是不是对的
	if (check_flag != 1) {
		return -1;
	}

	//check arm
	int Arm_flag;
	cout << "Whether choose to Arm? 1 for Arm, 0 for quit" << endl;
	cin >> Arm_flag;
	if (Arm_flag == 1) {
		Command_now.command = Arm;//发送一个命令
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
	int comid = 0;//发布指令编号
	
	//	ros::init(argc, argv, "collision_avoidance");
//	ros::NodeHandle nh("~");
//	// 频率 [20Hz]
//	ros::Rate rate(20.0);
	//【订阅】Lidar数据
	ros::Subscriber lidar_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1000, lidar_cb);
	//【订阅】无人机当前位置 坐标系 NED系
	ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 100, pos_cb);
	// 【发布】发送给position_control.cpp的命令

	//读取参数表中的参数
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
	//打印现实检查参数
	printf_param();

	//check paramater
//	int check_flag;

	//初值
	vel_track[0] = 0;
	vel_track[1] = 0;

	vel_collision[0] = 0;
	vel_collision[1] = 0;

	vel_sp_body[0] = 0;
	vel_sp_body[1] = 0;

	vel_sp_ENU[0] = 0;
	vel_sp_ENU[1] = 0;
	//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主程序<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	//takeoff
	i = 0;

	while (i < sleep_time) { //幂等
		Command_now.command = Move_ENU;//沿着大地坐标系移动（相对于当前的位置而言）
		Command_now.sub_mode = 0;//调整模式
		Command_now.pos_sp[0] = 0;//x=0移动到某一个点
		Command_now.pos_sp[1] = 0;//y=0原地起飞
		Command_now.pos_sp[2] = height_square;//起飞高度是1m
		Command_now.yaw_sp = 0;//偏航角是0度，没有偏航角
		Command_now.comid = comid;//发了一次命令！后面一个的id比前面的一个命令的id大才会执行。
		comid++;//把自动重发的命令给过滤掉，重发的包在服务器上会扔掉 如果想要下发下一个命令则要++
		//这个循环可能会发出很多命令。这个命令通过网络，可能会自动重发
		//mid：无论最后执行多少次结果都一样
		move_pub.publish(Command_now);
		rate.sleep();
		cout << "Point 0----->takeoff" << endl;
		i++;
	}

	i = 0;
	while (i < sleep_time) {
		Command_now.command = Move_ENU;
		Command_now.sub_mode = 0;
		Command_now.pos_sp[0] = 3.5;//向前走3.5m的距离
		Command_now.pos_sp[1] = 0;//右为正
		Command_now.pos_sp[2] = height_square;//巡航高度是1m 保持不变
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
		Command_now.pos_sp[1] = 0;//走到第三个点
		Command_now.pos_sp[2] = height_square;
		Command_now.yaw_sp = 0.5*PAI;
		Command_now.comid = comid;
		comid++;
		move_pub.publish(Command_now);
		rate.sleep();
		cout << "Point 2----->turn left" << endl;
		i++;//转向
	}
	
	
	
//	i = 0;
//	while (i < sleep_time) {
//		Command_now.command = Move_ENU;
//		Command_now.sub_mode = 0;
//		Command_now.pos_sp[0] = 3.5;
//		Command_now.pos_sp[1] = -3;//走到第三个点
//		Command_now.pos_sp[2] = height_square;
//		Command_now.yaw_sp = 0;
//		Command_now.comid = comid;
//		comid++;
//		move_pub.publish(Command_now);
//		rate.sleep();
//		cout << "Point 2----->left" << endl;
//		i++;
//	}
////机头改变朝向
//	i = 0;
//
//	while (i < sleep_time) {
//		Command_now.command = Move_ENU;
//		Command_now.sub_mode = 0;
//		Command_now.pos_sp[0] = 3.5;
//		Command_now.pos_sp[1] = -3;//走到第三个点
//		Command_now.pos_sp[2] = height_square;
//		Command_now.yaw_sp = 1;
//		Command_now.comid = comid;
//		comid++;
//		move_pub.publish(Command_now);
//		rate.sleep();
//		cout << "Point 3----->left" << endl;
//		i++;//转向
//	}
//进入房间！

//--------------------------------------分割线-本次新增----------------------------------------------------------------------- 
    //四向最小距离 初值
    flag_land = 0;


//    //输出指令初始化
//    comid = 1;
    
while(ros::ok())
    {
        //回调一次 更新传感器状态
        //1. 更新雷达点云数据，存储在Laser中,并计算四向最小距离
        ros::spinOnce();
        /**************************dyx****************************************/ //change: just cross one door
        //策略：穿门原理，当穿完最后一道门对墙面人像图片进行识别并计算图片中心xy坐标，导航过去。
        /*
        if(!reach_door_flag[0]) finddoorcentor(0);
        else if(reach_door_flag[0]&&!reach_door_flag[1])		//这里是穿两扇门的代码 
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
        if(!reach_door_flag[0]) //finddoorcentor(0);		//如果还没到门
        {
           collision_avoidance(door_x+0.1,door_y);
           float abs_distance;
           abs_distance = sqrt((pos_drone.pose.position.x - door_x - 0.1) * (pos_drone.pose.position.x -door_x - 0.1) + (pos_drone.pose.position.y - door_y) * (pos_drone.pose.position.y - door_y));
           if(abs_distance < 0.3 )
           {
              reach_door_flag[0]=true;
           }
        }
        else if(reach_door_flag[0])			//如果到门了
        {
           collision_avoidance(final_x,final_y);
//           collision_avoidance(target_x,target_y);
//           float abs_distance;
//           abs_distance = sqrt((pos_drone.pose.position.x - final_x) * (pos_drone.pose.position.x -final_x) + (pos_drone.pose.position.y - final_y) * (pos_drone.pose.position.y - final_y));
//           if(abs_distance < 0.3 )
//           {
//              flag_land = 1;	//降落标志位
//           }
        }
//--------------------------------------分割线结束----------------------------------------------------------------------------

////	ros::init(argc, argv, "collision_avoidance");
////	ros::NodeHandle nh("~");
////	// 频率 [20Hz]
////	ros::Rate rate(20.0);
//	//【订阅】Lidar数据
//	ros::Subscriber lidar_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1000, lidar_cb);
//	//【订阅】无人机当前位置 坐标系 NED系
//	ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 100, pos_cb);
//	// 【发布】发送给position_control.cpp的命令
//
//	//读取参数表中的参数
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
//	//打印现实检查参数
//	printf_param();
//
//	//check paramater
////	int check_flag;
//
//	//初值
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
		//回调一次 更新传感器状态
		//1. 更新雷达点云数据，存储在Laser中,并计算四向最小距离
		ros::spinOnce();
		collision_avoidance(target_x, target_y);

		Command_now.command = Move_ENU;     //机体系下移动
		Command_now.comid = comid;
		comid++;
		Command_now.sub_mode = 2; // xy 速度控制模式 z 位置控制模式
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
		//打印
		printf();
		rate.sleep();
	}
	return 0;
	}  //end of 穿门 
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


//计算前后左右四向最小距离
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
	//2. 根据最小距离判断：是否启用避障策略
	if (distance_c >= R_outside ) {
		flag_collision_avoidance.data = false;
	} else {
		flag_collision_avoidance.data = true;
	}
	//3. 计算追踪速度
	vel_track[0] = p_xy * (target_x - pos_drone.pose.position.x);
	vel_track[1] = p_xy * (target_y - pos_drone.pose.position.y);
	//速度限幅
	for (int i = 0; i < 2; i++) {
		vel_track[i] = satfunc(vel_track[i], vel_track_max);
	}
	vel_collision[0] = 0;
	vel_collision[1] = 0;
	//4. 避障策略
	if (flag_collision_avoidance.data == true) {
		distance_cx = distance_c * cos(angle_c / 180 * 3.1415926);
		distance_cy = distance_c * sin(angle_c / 180 * 3.1415926);

		float F_c;

		F_c = 0;

		if (distance_c > R_outside) {
			//对速度不做限制
			vel_collision[0] = vel_collision[0] + 0;
			vel_collision[1] = vel_collision[1] + 0;
			cout << " Forward Outside " << endl;
		}
		//小幅度抑制移动速度
		if (distance_c > R_inside && distance_c <= R_outside) {
			F_c = p_R * (R_outside - distance_c);
		}
		//大幅度抑制移动速度
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
		//避障速度限幅
		for (int i = 0; i < 2; i++) {
			vel_collision[i] = satfunc(vel_collision[i], vel_collision_max);
		}
	}

	vel_sp_body[0] = vel_track[0] + vel_collision[0];
	vel_sp_body[1] = vel_track[1] + vel_collision[1]; //dyx

	//找当前位置到目标点的xy差值，如果出现其中一个差值小，另一个差值大，
	//且过了一会还是保持这个差值就开始从差值入手。
	//比如，y方向接近0，但x还差很多，但x方向有障碍，这个时候按discx cy的大小，缓解y的难题。

	for (int i = 0; i < 2; i++) {
		vel_sp_body[i] = satfunc(vel_sp_body[i], vel_sp_max);
	}
	rotation_yaw(Euler_fcu[2], vel_sp_body, vel_sp_ENU);
}

//饱和函数
float satfunc(float data, float Max)
{
	if (abs(data) > Max)
		return ( data > 0 ) ? Max : -Max;
	else
		return data;
}

//思路：门相对于墙的激光数据会有很大的突变，在激光数据里找到这个突变范围，再转换到ENU坐标系下即可求出门的中心
void finddoorcentor(int i)
{
    //1.if no centor , findcentor set target
    //2.use collision_avoidance
    //3.judge whether reach target

    //1.
    float a,b,c;
    double l;
    cout<<"********************"<<endl;
    if(!door_center_x[i])				//如果还没有到门中心
    {
        a=Laser.ranges[0];
        b=Laser.ranges[89];
        c=Laser.ranges[270];			//获取激光雷达数据
        int theta1=atan(b/a)/3.1415926*180;		//角度1		//atan函数返回数字的反正切值
        int theta2=atan(c/a)/3.1415926*180;		//角度2
        cout<<"theta1: "<<theta1<<endl;
        cout<<"theta2: "<<theta2<<endl;
        std::vector<int> door_angle;		//初始化vector（创建一个空对象）
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
    
    //以上是求门的中心位置坐标（ENU坐标系）
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
