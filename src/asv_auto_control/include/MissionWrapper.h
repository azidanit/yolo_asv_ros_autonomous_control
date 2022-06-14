// //
// // Created by lumpia on 15/07/21.
// //

// #ifndef SRC_MISSIONWRAPPER_H
// #define SRC_MISSIONWRAPPER_H

// #include <mutex>

// #include <ros/ros.h>
// #include <ros/package.h>
// #include <tf/transform_listener.h>

// #include <nav_msgs/Path.h>
// #include <geometry_msgs/Point.h>
// #include <geometry_msgs/Point32.h>
// #include <visualization_msgs/Marker.h>
// #include <visualization_msgs/MarkerArray.h>
// #include <std_msgs/Int32.h>
// #include <std_msgs/UInt16MultiArray.h>
// #include <std_msgs/Float32.h>
// #include <std_msgs/UInt8.h>
// #include <std_msgs/Int32MultiArray.h>
// #include <sensor_msgs/NavSatFix.h>

// #include <rviz_plugin/Selectedwp.h>
// #include <rviz_plugin/ManualCtrl.h>
// #include <rviz_plugin/NextDest.h>
// #include <comm_stm/stm_status.h>

// #include <QThread>
// #include <QPoint>
// #include <thread>
// #include <QFile>
// #include <QTextStream>


// #include "misi/misi.h"
// #include "SensorControl.h"
// #include "utils.hpp"

// #define LAT_TO_METER 111000
// #define LON_TO_METER 113321

// typedef struct {
//     geometry_msgs::Point pos;
//     double roll, pitch, yaw;
// } TF_simplified2;

// typedef struct {
//     double p, i, d;
// } PID_t;

// enum WpList{
//     MAIN_MISSION = 0,
//     GUIDED_RTH = 1,
//     PATROL_BACK = 5,
//     QUICK_RTH = 11,
// };
// class MissionWrapper : public QThread {
//     Q_OBJECT

// public:
//     MissionWrapper(ros::NodeHandle, char*);
//     ~MissionWrapper();

//     ros::NodeHandle nh;

//     void run();

// private:
//     //----CONST DEFINITION----
// //    const double RADIUS_WP_THRESHOLD = .5;
//     const double RADIUS_WP_THRESHOLD = 2;
//     const double DIST_TO_WP_THRESHOLD = 15;
//     const int ANGLE_WP_THRESHOLD = 50;
//     const int MAX_STEERING = 600;
//     const int MAX_THRUST = 500;

//     //----ROS MAIN variable----
//     ros::Subscriber wp_sub, manual_control_sub, state_rviz_sub, gps_sub,
//         marker_srf_sub, local_vel_sub, data_from_stm_sub;
//     ros::Publisher stm_pub, cmpcal_pub, using_compass_pub, orientation_TF_pub;
//     std::mutex param_qt_mtx;

//     //camera param from rviz
//     ros::Subscriber critline_sub, boatside_sub;
//     QPoint boatside[4];
//     void critlineCallback(const std_msgs::Int32MultiArray msg);
//     void boatsideCallback(const geometry_msgs::PoseArray msg);

//     //CLASS PENDUKUNG
//     Misi *ms;
//     SensorControl sensor_control;

//     //----ASV variable----
//     tf::StampedTransform currentBoatTF;
//     TF_simplified ASV_TF;
//     std::string world_frame, robot_frame;
//     std::mutex tf_mutex, sensor_mtx;
//     tf::TransformListener boatTF_sub;

//     int steer_trim, steer_trim_from_remote, thrust_trim, steer_out;
//     int status_auto_from_remote;
//     double thrust_out;
//     double asv_local_speed;


//     //----ASV Function----
//     void loadParams(ros::NodeHandle nh);
//     void listenBoatTF();
//     void publishControlToSTM(int steer, int thrust);

//     //----MISSION variable----
//     bool is_started, is_running, is_rth_after_done, is_quick_rth, rth_called, patroling_back;
//     bool quick_rth_called, quick_rth_running;
//     bool hold_korban_called, is_hold_korban_running;
//     bool lifebuoy_detached;
//     int wp_start, patrol_lap, current_patrol_lap;
//     std::thread misi_thread, quick_rth_thread, hold_korban_thread;
//     std::mutex run_mtx;
//     //----MISSION function----
//     void missionStart();
//     void initQuickRTH();
//     void quickRTHMissionStart();
//     void initHoldKorban();
//     void holdTrackingKorban();

//     void checkIsPatrol(); //if patrol, invert wp

//     //----WAYPOINTS variable----
//     nav_msgs::Path waypoints[20], inverted_waypoints_for_patrol;
//     std::vector<double> waypoints_speed[20];
//     std::vector<double> waypoints_action[20];
//     int start_waypoint;
//     //----WAYPOINTS function----
//     void setInvertedWaypointForPatrol();

//     //----INIT FUNCTION----
//     void initSubscriber();
//     void initPublisher();
//     void initVariable();

//     //----CONTROL param----
//     PID_t pid_wp_angle, pid_wp_dist, pid_speed_control,
//             pid_obstacle_avoidance_srf, pid_camera_tracking,
//             pid_camera_tracking_thrust;
//     bool use_speed_control, use_obstacle_avoidance, use_obstacle_stop, use_find_korban;
//     int target_constant_thrust;
//     std::mutex mission_wp_mtx;
//     double save_front_distance;
//     //----CONTROL param marker----
//     ros::Publisher nextDest_wp_marker_pub, headingMarker_pub,
//             targetMarker_pub, angle_path_pub, compass_helper_pub;
//     std::thread marker_thread;
//     void publishMarkers();


//     //--------------------------------------------------------CONTROL variable----
//     //--------------------------------CONTROL variable----
//     //----------CONTROL variable----
//     int wp_idx_current, selected_mission;
//     double error_angle_wp, error_angle_wp_before, error_dist_wp, error_dist_wp_before;
//     double error_angle_wp_accumulative, error_dist_wp_accumulative;
//     double error_speed, error_speed_before, current_thrust;
//     //camera
//     double error_camera, error_camera_before;
//     int crit_line, horizon;
//     bool is_korban_found;
//     int counter_braking_after_korban_found;

//     //obstacle
//     bool stopped_from_brakking, avoiding_obstacle;
//     //----CONTROL function----
//     double calculateErrorPID(PID_t, double error_now, double error_before, double error_acc);
//     int controlWaypoint();
//     int controlObstacleAvoidanceSRF();
//     int controlFindKorbanCamera();
//     int controlLifebuoyKorbanCamera();
//     double controlSpeed(double target_speed, int sensitivity, PID_t pid_speed);
//     double angleOfTwoWaypoints(int wp1_idx, int wp2_idx);
//     double getDestAngle(const geometry_msgs::Point&);
//     double distanceLineToPoint(const geometry_msgs::Point& initial, const geometry_msgs::Point& destination, const geometry_msgs::Point& current);
//     bool isWaypointArrived();
//     double getAngle(const geometry_msgs::Point&, const geometry_msgs::Point&);
//     void setNextDest();
//     double getAnglePath();

//     //PATH VARIABLE
//     ros::Publisher path_pub, loaded_path_pub, global_path_pub;
//     nav_msgs::Path path_msg, loaded_path_msg, global_path_msg;
//     std::vector<sensor_msgs::NavSatFix> global_path;
//     sensor_msgs::NavSatFix current_gps;
//     bool is_path_recorded;
//     std::mutex path_mtx, gps_mtx;
//     double LatitudeTop, LatitudeBot, LongitudeRight, LongitudeLeft, LongitudeCenter, LatitudeCenter;
//     std::thread path_thread;
//     //PATH FUNCTION
//     void recordPath();


//     void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
//     geometry_msgs::PoseStamped globalToPose(double lat, double lon);
//     void initVenue(const char* venue);

//     //----TOPIC CALLBACK function----
//     void missionStateCallback(std_msgs::UInt16MultiArray msg);
//     void runManual(const rviz_plugin::ManualCtrl& msg);

//     void waypointsCallback(const rviz_plugin::Selectedwp& msg);
//     void markerCallback(const visualization_msgs::MarkerArray& msg);

//     void localVelocityCallback(const geometry_msgs::TwistStamped &msg);

//     void dataFromStmCallback(const comm_stm::stm_status &msg);

// public slots:
//     //DARI UI
//     void startMission();
//     void stopMission();
//     void resumeMission();

//     void changeSteerTrim(int);
//     void changeSpeedTrim(int);
//     void calibrateCompass(double);
//     void changeUseCompass(bool val);

//     void useSpeedControl(bool);
//     void useAccError(bool);
//     void changeUseObstacleAvoidance(bool);
//     void changeUseObstacleStop(bool);
//     void changeUseFindKorban(bool);
//     void changeRTHAfterDone(bool);

//     void changePatrolLap(int val);

//     void changeP(int, double);
//     void changeI(int, double);
//     void changeD(int, double);

//     void changeSpeed(int, int);
//     void changeDistance(int, double);
//     void pitchChanged(int, double val);

//     void changeASVWidth(double);
//     void changeASVOffset(double);

//     void startRecordPath();
//     void stopRecordPath();
//     void clearPath();  // Clear recorded path
//     void savePath(QString filename);
//     void loadPath(QString filename);
//     void loadLocalPath(QString filename);

//     void changeCritLine(int, int);
//     void changeBoatSide(QPoint**);

// signals:
//     void setLogInformationDisplay(QString msg);
//     void steer_trim_changed_from_remote(int val);

//     void setCritLine(int idx, int val);
// };


// #endif //SRC_MISSIONWRAPPER_H
