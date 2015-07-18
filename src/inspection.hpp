// RoboCup@Home Navigation Test 競技用プログラム
// 150608: dfollowを移植

#ifndef DNAVIGATION_HPP
#define DNAVIGATION_HPP

#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include <rospack/rospack.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include <nav_msgs/Odometry.h>
#include <stdio.h>
#include <std_msgs/String.h>
#include <string>
#include <sound_play/sound_play.h>
#include <sstream>
#include <sensor_msgs/LaserScan.h>
#include "/usr/local/include/kobuki-1.0/libkobuki.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


//#define MOMENT_EXEL // 慣性モーメントデータ取得用
//#define DEBUG       // デバッグ用
#define RECORD      // 録画用
#define MOVE       //  動かさないときはコメントアウト
//#define PROPORTIONAL_NAVI /// 比例航法でないときはコメントアウト
///#define HOME         // 自宅用のパラメータ
#define ENGLISH
#define SPEECH     // 音声認識をしないときはコメントアウト


#ifdef ENGLISH
//#define FOLLOW "padsp ~/myprog/src/music/bin/music ~/myprog/src/audio_out/follow_me/follow_me.wav"
#define SPEAK_COMPLETED "padsp ~/myprog/src/music/bin/music ~/myprog/src/voice/mini/completed.wav"
#define SPEAK_COUNTDOWN "padsp ~/myprog/src/music/bin/music ~/myprog/src/voice/mini/countdown.wav"

#define SPEAK_ELEVATOR "padsp ~/myprog/src/music/bin/music ~/myprog/src/voice/english/elevator.wav"
#define SPEAK_FIND "padsp ~/myprog/src/music/bin/music ~/myprog/src/voice/english/find.wav"
#define SPEAK_FIND_OPERATOR "padsp ~/myprog/src/music/bin/music ~/myprog/src/voice/english/findoperator.wav"
#define SPEAK_FOLLOWYOU "padsp ~/myprog/src/music/bin/music ~/myprog/src/voice/english/followyou.wav"
#define SPEAK_KENSEIKO "padsp ~/myprog/src/music/bin/music ~/myprog/src/voice/english/saykenseikochan.wav"

#define SPEAK_LEAVE "padsp ~/myprog/src/music/bin/music ~/myprog/src/voice/english/leaveelevator.wav"
#define SPEAK_LEAVE_ARENA "padsp ~/myprog/src/music/bin/music ~/myprog/src/voice/mini/leaveAreana.wav"

#define SPEAK_LOST "padsp ~/myprog/src/music/bin/music ~/myprog/src/voice/english/lost.wav"
#define SPEAK_MINI "padsp ~/myprog/src/music/bin/music ~/myprog/src/voice/mini/mini.wav"
#define SPEAK_MOVEOUT "padsp ~/myprog/src/music/bin/music ~/myprog/src/voice/mini/moveout.wav"
#define SPEAK_NAVIGATION_TEST "padsp ~/myprog/src/music/bin/music ~/myprog/src/voice/mini/navigationtest.wav"

#define SPEAK_NEXT "padsp ~/myprog/src/music/bin/music ~/myprog/src/voice/mini/next.wav"
#define SPEAK_OK "padsp ~/myprog/src/music/bin/music ~/myprog/src/voice/english/hai.wav"

#define SPEAK_REMEMBER "padsp ~/myprog/src/music/bin/music ~/myprog/src/voice/english/remember.wav"
#define SPEAK_REMEMBERED "padsp ~/myprog/src/music/bin/music ~/myprog/src/voice/english/remembered2.wav"
#define SPEAK_SAY_LEAVE "padsp ~/myprog/src/music/bin/music ~/myprog/src/voice/english/sayleaveelevator.wav"
#define SPEAK_SAY_FOLLOWME "padsp ~/myprog/src/music/bin/music ~/myprog/src/voice/english/sayfollowme.wav"
#define SPEAK_SAY_OPEN "padsp ~/myprog/src/music/bin/music ~/myprog/src/voice/mini/opendoor.wav"


#define SPEAK_START "padsp ~/myprog/src/music/bin/music ~/myprog/src/voice/english/start.wav"
#define SPEAK_START_MINI  "padsp ~/myprog/src/music/bin/music ~/myprog/src/voice/mini/start.wav"
#define SPEAK_STAND "padsp ~/myprog/src/music/bin/music ~/myprog/src/voice/english/stand1.wav"
#define SPEAK_STOP "padsp ~/myprog/src/music/bin/music ~/myprog/src/audio_out/follow_me/stop.wav"
#define SPEAK_WALK "padsp ~/myprog/src/music/bin/music ~/myprog/src/voice/english/walk.wav"
#define SPEAK_WALK_IN_PLACE "padsp ~/myprog/src/music/bin/music ~/myprog/src/voice/english/walkinplace.wav"
#define SPEAK_WAYPOINT  "padsp ~/myprog/src/music/bin/music ~/myprog/src/voice/mini/waypoint.wav"
#define SPEAK_WAYPOINT1 "padsp ~/myprog/src/music/bin/music ~/myprog/src/voice/mini/waypoint1.wav"
#define SPEAK_WAYPOINT2 "padsp ~/myprog/src/music/bin/music ~/myprog/src/voice/mini/waypoint2.wav"
#define SPEAK_WAYPOINT3 "padsp ~/myprog/src/music/bin/music ~/myprog/src/voice/mini/waypoint3.wav"
#define SPEAK_WAYPOINT4 "padsp ~/myprog/src/music/bin/music ~/myprog/src/voice/mini/waypoint4.wav"
#define SPEAK_WAYPOINT5 "padsp ~/myprog/src/music/bin/music ~/myprog/src/voice/mini/waypoint5.wav"



#else
// Japanese
#define SPEAK_HAI "padsp ~/myprog/src/music/bin/music ~/myprog/src/audio_out/follow_me/hai.wav"
//#define FOLLOW "padsp ~/myprog/src/music/bin/music ~/myprog/src/audio_out/follow_me/follow_me.wav"
#define SPEAK_FIND "padsp ~/myprog/src/music/bin/music ~/myprog/src/voice/find.wav"
#define SPEAK_FOLLOWYOU "padsp ~/myprog/src/music/bin/music ~/myprog/src/voice/followyou.wav"
#define SPEAK_LOST "padsp ~/myprog/src/music/bin/music ~/myprog/src/voice/lost.wav"
#define SPEAK_REMEMBER "padsp ~/myprog/src/music/bin/music ~/myprog/src/voice/remember.wav"
#define SPEAK_REMEMBERED "padsp ~/myprog/src/music/bin/music ~/myprog/src/voice/remembered.wav"
#define SPEAK_START "padsp ~/myprog/src/music/bin/music ~/myprog/src/voice/start.wav"
#define SPEAK_STAND "padsp ~/myprog/src/music/bin/music ~/myprog/src/voice/stand.wav"
#define SPEAK_STOP "padsp ~/myprog/src/music/bin/music ~/myprog/src/audio_out/follow_me/stop.wav"
#endif

#define DEG2RAD(DEG) ( (M_PI) * (DEG) / 180.0 )
#define RAD2DEG(RAD) ( 180.0  * (RAD) / (M_PI))

using namespace std;
// アールティ用ロボットネームスペース
using namespace rt_net;

using namespace cv;
const double moving_threshold = 0.01;   // 移動物検出のしきい値[m] 
const double follow_max_distance = 6.0; // follow distance
const double follow_distance = 2.5;     // follow distance
const double follow_min_distance = 0.8; // 0.2follow distance 
const double leg_width_max = 0.3;       // 脚幅の最大値
const double leg_width_min = 0.1;       // 脚幅の最大値
const double follow_angle  = 180;       // 探す範囲は正面のこの角度[deg]  
const double gain_linear   = 2.0;       // P制御比例ゲイン（並進）
const double gain_turn     = 0.5;       // P制御比例ゲイン（回転）
const int    template_number    = 10;   // テンプレート数     

#ifdef HOME // 家庭用　狭い環境
const double kp                 = 10;
const double kd                 =  5;
const double gain_proportion    = 10;    // 比例航法のゲイン FMT用 0.1 家　10
const double linear_max_speed   = 0.3;  // FMT用 0.6  家用 0.3 　最大　
const double turn_max_speed     = 2.4;  // FMT用 0.6　家用 1.2 　最大　3.14
#else  // FMT
const double kp                 = 10; // 4, 10 kp: 30, kd: 0
const double kd                 = 100; // 
const double gain_proportion    = 4;  // 比例航法のゲイン FMT用 0.1 家　10
const double linear_max_speed   = 0.6;  // 0.35 FMT用 0.6  家用 0.3 　最大　 
const double turn_max_speed     = 1.2;  // 0.8 FMT用 0.6　家用 1.2 　最大　3.14 
#endif

//double laser_time, laser_last_time, laser_diff_time; //  [s]
//double target_angle;     // target angle [rad] center is 0 [rad]
//double target_distance;  // minimum distance from a robot

const int IMAGE_WIDTH=500, IMAGE_HEIGHT=500;
const double mToPixel = 50; // mをpixelへ変換 1m == 50 pixel, 1 pixel = 2cm

cv::RNG rng(12345); // 乱数発生器　12345は初期化の種


// 初期化時に塗りつぶす                                                    
cv::Mat lidar_image(cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT), 
   	    CV_8UC3, cv::Scalar::all(255));
cv::Mat lidar_gray_image(cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT),
	    CV_8U, cv::Scalar::all(255));
// 1時刻前の画像_
cv::Mat lidar_gray_old_image(cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT), 
	    CV_8U, cv::Scalar::all(255));
// 脚と推定した輪郭
cv::Mat detect_image;
cv::Mat detect_old_image;

cv::Mat e1_img,e1_old_img,diff_img;
cv::Mat e3_img, e3_old_img, diff3_img;

// テンプレートの輪郭
cv::vector<cv::vector<cv::vector<cv::Point> > >template_contours;
cv::vector<cv::vector<cv::Point> > template_contour;
cv::Scalar red(0,0,255), blue(255,0,0), green(0, 255, 0);

cv::VideoWriter writer1("videofile.avi", CV_FOURCC_MACRO('X', 'V', 'I', 'D'), 30.0,
		       cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT), true);
cv::VideoWriter writer2("videofile2.avi", CV_FOURCC_MACRO('X', 'V', 'I', 'D'), 30.0,
		       cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT), true);

std::vector<std::string> templateFiles;
boost::shared_ptr<sound_play::SoundClient> sc;
std_msgs::String voice_command;



//enum  Robot_State { START_STATE, WP1_STATE, WP2_STATE, WP3_STATE,_BEGIN, STATE1, STATE2, STATE3, STATE_END, STATE_TEST };
enum  Robot_Condition { SAFE = 0, DANGER = 119, IN_ELEVATOR = 999};

struct Pose {
  double x;
  double y;
  double z;
  double theta;
  double velocity; 
};

class Object {
private:
  double x;     // position in the world coordinate
  double y;
  double z;
  double theta;

public:
  Object();
  Pose local,last_local; // local coordinate: ROSに合わせて進行方向がx, 左方向がy
  Pose world,last_world; // world coordinate
  bool leg;   // leg:true  or not:false
  int begin; // right side laser
  int end;   // left side laser
  int diff;
  int intensity_min, intensity_max; // 反射強度の最小、最大値　[0:255]
  double distance,last_distance; // present, last detected distance
  double angle,last_angle; // rad
  double width;
  double radius; // radius of an enclose circle
  cv::Point2f image_pos; // position of an image
  double getX() { return x; }
  double getY() { return y; }
  double getZ() { return z;}
  double getTheta() { return theta;}
  void   setX(double _x) {x = _x;}
  void   setY(double _y) {y = _y;}
  void   setZ(double _z) {z = _z;}
  void   setTheta(double _theta) {theta = _theta;}

};

class Robot {
private:
  bool   human_lost;  // 
  bool   second_section; // true if the robot pass the second section 
  bool   reached_wp4;    // ウェイポイント４に辿り着いたか
  int    state; // state of robots; 
  double time, last_time; // [s]
  double x,y,theta; // position [m], orientation [rad] in the world coordiante
  double vx, vy, vth; // velocity in the world coordinate (odometry)
  double laser_distance[1081]; // hokuyo lidar UTM-30LX                                     
  double laser_last_distance[1081];
  double laser_intensities[1081];
  double laser_last_intensities[1081];

  double last_x, last_y, last_theta; // １時刻前のx, y, theta
  //geometry_msgs::Twist cmd_speed; // tmp_speed;
  double linear_speed, angular_speed; // [m/s]
  int    waypoint_num; // the number of the waypoint


  geometry_msgs::Twist cmd_vel, zero_cmd_vel;
  geometry_msgs::PoseStamped robotPose; //ロボットの自己位置
  move_base_msgs::MoveBaseGoal goal;
  //MoveBaseClient ac;
  ros::NodeHandle nh;
  ros::Publisher  cmdVelPublisher;
  ros::Subscriber amclPoseSubscriber;
  ros::Subscriber speechSubscriber;
  ros::Subscriber laserSubscriber;
  ros::Subscriber odomSubscriber;
  

public:
  Robot();
  // state0: memorizing the operator, state1: 1st section  
  // state2: 2nd seciton,             state3: 3rd section  
  Pose   local; // local coordinate
  Pose   world;
  int    dataCount;
  double laser_angle_min, laser_angle_max; 
  void   actionInElevator();
  void   amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
  void   calcHumanPose(int object_num,  Object *object, Object *human_object);
  void   changeToPicture(int dataCount, double laser_angle_min,
			 double laser_angle_max, double laser_angle_increment);
  int    checkCollision();
  int    checkCondition(double theta);
  bool   checkDoorOpen();
  double checkFrontDistanceMarco(); // by marco
  bool   checkObstacles(double distance);
  bool   choosePathMarco();  // by marco
  bool   findAction(); // State: action
  double findDirection(double distance);
  bool   findHuman(cv::Mat input_image, double *dist, double *angle);
  int    findLegs(cv::Mat input_image, Object *object, cv::Mat result_image,
	   cv::Mat display_image, const string& winname, cv::Scalar color,
	   int contour_min, int contour_max, int width_min, int width_max,
	   double ratio_min, double  ratio_max,double m00_min, double m00_max,
		  double m10_min, double m10_max, double diff_x,  double diff_y);
  bool   followAction();
  void   followPerson(cv::Mat input_image);
  double getAngularSpeed() { return angular_speed; }
  double getLastX(){ return last_x; }
  double getLastY(){ return last_y; }
  double getLastTime() { return last_time; }
  double getLastTheta(){ return last_theta; }
  double getLinearSpeed()  { return linear_speed; }
  bool   getReachedWP4() { return reached_wp4;}
  int    getState() { return state;}
  double getTheta(){ return theta; }
  double getTime() { return time;  }
  int    getWaypointNum() { return waypoint_num;}
  double getX(){ return x;}
  double getY(){ return y;}
  void   goAhead(double distance); // distanceの距離だけ前進
  void   goAhead2Marco(double distance); // by marco
  void   goForwardMarco(double distance); // by marco
  void   goOut(double distance);   // エレベーターから外にでる関数
  void   init(); // initialize
  void   laserCallback(const sensor_msgs::LaserScan laser_scan);
  void   localToWorld(Pose local_pose, Pose *world_pose );
  double measureObstacleDist(double angle);  // angle [rad]
  double median(int no, double *data);
  void   memorizeIntensity();
  void   memorizeOperator();
  void   move();
  void   move(double linear, double angular);
  //bool   navigation(MoveBaseClient *ac);
  int    navigation(MoveBaseClient *ac);
  //int    navigation();
  void   odomCallback(const nav_msgs::OdometryConstPtr& msg);
  void   prepRecord();
  void   prepWindow();
  void   readTemplate();
  bool   recog_followme; // 音声認識用  
  bool   recog_kenseiko;
  bool   recog_leave;    // 音声認識用  
  bool   recog_stop;
  bool   recog_mini;
  //  void   recognizeVoice(const string str);
  void   recognizeSimpleVoice(const string str);
  void   recognizeVoice(const string str);
  void   record(cv::VideoWriter writer, cv::Mat image);
  void   searchOperator();
  void   searchOperatorMarco(); // by marco
  void   setAngularSpeed(double angular);
  void   setLastX(double _last_x){ last_x = _last_x; }
  void   setLastY(double _last_y){ last_y = _last_y; }
  void   setLastTime(double _last_time) { last_time = _last_time; }
  void   setLastTheta(double _last_theta){ last_theta = _last_theta; }
  void   setLinearSpeed(double linear);
  void   setPose(double x, double y, double th);
  void   setReachedWP4();
  void   setState(int _state)   { state = _state;}
  void   setTheta(double _theta){ theta = _theta; }
  void   setTime(double _time) { time = _time;  }
  void   setWaypointNum(int _wp_num) {waypoint_num = _wp_num; }
  void   setX(double _x){ x = _x; }
  void   setY(double _y){ y = _y; }
  void   showWindow();
  double SMAfilter(double value, double *data, const int length);
  void   speak(const string str, int wait_time);  // [ms] 
  void   speechCallback(const std_msgs::String& voice);
  void   stop();
  void   test();
  void   turn(double rad);
  void   welcomeMessage();
  //Kobuki *kobuki;
  };

  Object human;


#endif
