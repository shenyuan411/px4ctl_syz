#include <ros/ros.h>
#include <Eigen/Dense>
#include <Eigen/Core>

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/RCIn.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/State.h"
#include "ros/subscriber.h"
// #include "sensor_msgs/Image.h"
#include "std_msgs/Float32.h"
#include <dynamic_reconfigure/server.h>

using namespace std;

#define SIM 0

enum MISSION_STATE
{
    INIT,
    OFFBOARD,
    POSITION,  // TAKEOFF
    LAND
};

MISSION_STATE mission_state;

mavros_msgs::State px4_state, px4_state_prev;

#define DEAD_ZONE 0.25
#define MAX_MANUAL_VEL 1.0
#define RC_REVERSE_PITCH 0
#define RC_REVERSE_ROLL 0
#define RC_REVERSE_THROTTLE 0

double last_set_hover_pose_time;

ros::Publisher     target_pose_pub;
ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;

//  W: World; B: Body;  有t是时变
Eigen::Affine3f     T_W_Bt;

geometry_msgs::Pose T_W_B_set,T_W_B_des;

bool                receive_odom = false;

void                odom_callback(const geometry_msgs::PoseStampedConstPtr &odom_msg)
{
    T_W_Bt.matrix().block<3, 3>(0, 0) =
        Eigen::Quaternionf(odom_msg->pose.orientation.w, odom_msg->pose.orientation.x,
                           odom_msg->pose.orientation.y, odom_msg->pose.orientation.z)
            .toRotationMatrix();
    T_W_Bt.matrix().block<3, 1>(0, 3) = Eigen::Vector3f(
        odom_msg->pose.position.x, odom_msg->pose.position.y, odom_msg->pose.position.z);
    receive_odom = true;
}

// bool receive_pose_set = false;
// void pose_set_callback(const geometry_msgs::PoseStampedConstPtr &pose_set_msg)
// {
//     T_W_Bt_des.matrix().block<3, 3>(0, 0) =
//         Eigen::Quaternionf(pose_set_msg->pose.orientation.w, pose_set_msg->pose.orientation.x,
//                            pose_set_msg->pose.orientation.y, pose_set_msg->pose.orientation.z)
//             .toRotationMatrix();
//     T_W_Bt_des.matrix().block<3, 1>(0, 3) =
//         Eigen::Vector3f(pose_set_msg->pose.position.x, pose_set_msg->pose.position.y,
//                         pose_set_msg->pose.position.z);

//     if (mission_state != LAND)
//     {
//         // Eigen::Affine3f T_W_B_set_tmp = T_W_V * T_V_Bt_set;
//         // T_W_B_set.position.x          = T_W_Bt_des.translation().x();
//         // T_W_B_set.position.y          = T_W_Bt_des.translation().y();
//         // T_W_B_set.position.z          = T_W_Bt_des.translation().z();
//         T_W_B_set.position = pose_set_msg->pose.position;
//         // Eigen::Quaternionf q(T_W_B_set_tmp.matrix().block<3, 3>(0, 0));
//         // T_W_B_set.orientation.w = q.w();
//         // T_W_B_set.orientation.x = q.x();
//         // T_W_B_set.orientation.y = q.y();
//         // T_W_B_set.orientation.z = q.z();
//         T_W_B_set.orientation = pose_set_msg->pose.orientation;
//     }
//     receive_pose_set = true;
// }


void rc_callback(const mavros_msgs::RCInConstPtr &rc_msg)
{
    double rc_ch[4];
    for (int i = 0; i < 4; i++)
    {
        // 归一化遥控器输入
        rc_ch[i] = ((double)rc_msg->channels[i] - 1500.0) / 500.0;
        if (rc_ch[i] > DEAD_ZONE)
            rc_ch[i] = (rc_ch[i] - DEAD_ZONE) / (1 - DEAD_ZONE);
        else if (rc_ch[i] < -DEAD_ZONE)
            rc_ch[i] = (rc_ch[i] + DEAD_ZONE) / (1 - DEAD_ZONE);
        else
            rc_ch[i] = 0.0;
    }
    if (rc_msg->channels[4] < 1250){
        // mission_state = INIT;
        if (px4_state.mode != "MANUAL")
        {
            // cout << "debug" << endl;
            mavros_msgs::SetMode offb_set_mode;
            offb_set_mode.request.custom_mode = "MANUAL";
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                ROS_INFO("px4 mode Switch to MANUAL!");
                px4_state_prev      = px4_state;
                px4_state_prev.mode = "MANUAL";
                mission_state = INIT;
                ROS_INFO("From ~ to INIT!");
            }
            else
            {
                ROS_WARN("Failed to enter MANUAL!");
                return;
            }
        }
    }
    if (rc_msg->channels[4] > 1750 && px4_state.armed)
    {
        // if (px4_state.mode != "STABILIZED" && mission_state == OFFBOARD)
        // {
        //     cout << "debug" << endl;
        //     mavros_msgs::SetMode offb_set_mode;
        //     offb_set_mode.request.custom_mode = "STABILIZED";
        //     if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
        //     {
        //         ROS_INFO("px4 mode Switch to STABILIZED!");
        //         px4_state_prev      = px4_state;
        //         px4_state_prev.mode = "STABILIZED";
        //         mission_state = POSITION;
        //         ROS_INFO("From OFFBOARD to STABILIZED!");
        //     }
        //     else
        //     {
        //         ROS_WARN("Failed to enter STABILIZED!");
        //         return;
        //     }
        // }
        if (px4_state.mode == "OFFBOARD" && mission_state == OFFBOARD)
        {
            mission_state = POSITION;
            ROS_INFO("From OFFBOARD to POSITION!");
        }
        // mission_state = INIT;
    }
    else if (rc_msg->channels[4] > 1250 && rc_msg->channels[4] < 1750)  // heading to POSITION
    // else if (rc_msg->channels[4] > 1750)  // heading to POSITION
    {
        if (mission_state == INIT && rc_msg->channels[5] < 1250)
        {
            if (rc_ch[0] == 0.0 && rc_ch[1] == 0.0 && rc_ch[2] == 0.0 && rc_ch[3] == 0.0 
                // && receive_odom && receive_pose_set)
                && receive_odom)
            // )
            {
                // mission_state            = POSITION;
                last_set_hover_pose_time = ros::Time::now().toSec();
                // pose 33
                T_W_B_set.position.x         = 0.5;//????????????
                T_W_B_set.position.y         = 0.0;
                T_W_B_set.position.z         = 0.8;
                T_W_B_set.orientation.w      = 1.0;
                T_W_B_set.orientation.x      = 0.0;
                T_W_B_set.orientation.y      = 0.0;
                T_W_B_set.orientation.z      = 0.0;

                // T_W_Bt_des = T_W_B_set;
                Eigen::Affine3f T_W_B0       = T_W_Bt;
                // Eigen::Affine3f T_W_B0_des   = T_W_B_des;
                // T_W_B0_des.translation().z() = 0;

                // T_W_V                        = T_W_B0 * T_V_B0_set.inverse();
                cout << "T_W_B0" << endl << T_W_B0.matrix() << endl;
                cout << "T_W_B_set.z" << endl << T_W_B_set.position.z << endl;
                // cout << "T_W_V" << endl << T_W_V.matrix() << endl;
                // goal_position = dynamixel_controller.convertRadian2Value(ID, INIT_RAD);
                ROS_INFO("PRETAKEOFF succeed!");
            }
            else
            {
                ROS_WARN("PRETAKEOFF failed! Rockers are not in reset middle! or no odom!");
                std::cout << rc_ch[0] << rc_ch[1] << rc_ch[2] << rc_ch[3] << std::endl;
                return;
            }
        }else if (mission_state == POSITION && px4_state.armed) {
            if (px4_state.mode != "OFFBOARD"){
                mavros_msgs::SetMode offb_set_mode;
                offb_set_mode.request.custom_mode = "OFFBOARD";
                if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                {
                    // ROS_INFO("Offboard enabled");
                    px4_state_prev      = px4_state;
                    px4_state_prev.mode = "OFFBOARD";
                }
                else
                {
                    ROS_WARN("Failed to enter OFFBOARD!");
                    return;
                }
            }
            mission_state = OFFBOARD;
            ROS_INFO("From POSITION to OFFBOARD!");
        }
    }

    if (!SIM)
    {
        if (rc_msg->channels[5] > 1750 && rc_msg->channels[4] > 1250 &&rc_msg->channels[4] < 1750)
        {
            if (mission_state == INIT)
            {
                if (rc_ch[0] == 0.0 && rc_ch[1] == 0.0 && rc_ch[2] == 0.0 && rc_ch[3] == 0.0 &&
                    !px4_state.armed)
                {
                    if (px4_state.mode != "OFFBOARD")
                    {
                        mavros_msgs::SetMode offb_set_mode;
                        offb_set_mode.request.custom_mode = "OFFBOARD";
                        if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                        {
                            ROS_INFO("Offboard enabled");
                            px4_state_prev      = px4_state;
                            px4_state_prev.mode = "OFFBOARD";
                        }
                        else
                        {
                            ROS_WARN("Failed to enter OFFBOARD!");
                            return;
                        }
                    }
                    else if (px4_state.mode == "OFFBOARD")
                    {
                        mavros_msgs::CommandBool arm_cmd;
                        arm_cmd.request.value = true;

                        if (arming_client.call(arm_cmd) && arm_cmd.response.success)
                        {
                            ROS_INFO("Vehicle armed");
                            mission_state = OFFBOARD;
                            ROS_INFO("From INIT to OFFBOARD!");
                        }
                        else
                        {
                            ROS_ERROR("Failed to armed");
                            return;
                        }
                    }
                    // cout << "????"<< endl;
                }
                else if (!px4_state.armed)
                {
                    ROS_WARN("Arm denied! Rockers are not in reset middle!");
                    return;
                }
            }
        }
        else if (rc_msg->channels[5] > 1250 && rc_msg->channels[5] < 1750)
        {
            if (px4_state_prev.mode == "OFFBOARD")
            {
                mission_state = LAND;
                ROS_INFO("From OFFBOARD to LAND!");
            }
        }
        else if (rc_msg->channels[5] < 1250)
        {
            // disarmed的前提是什么？？？？？
            if (px4_state.armed)
            {
                mavros_msgs::CommandBool arm_cmd;
                arm_cmd.request.value = false;

                if (arming_client.call(arm_cmd) && arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle disarmed");
                    mission_state = INIT;
                    ROS_INFO("Swith to INIT state!");
                }
                else
                {
                    ROS_ERROR("Failed to disarmed");
                    return;
                }
            }
        }
    }

    if (mission_state != INIT)
    {
        double now               = ros::Time::now().toSec();
        double delta_t           = now - last_set_hover_pose_time;
        last_set_hover_pose_time = now;

        // body frame: x-forward, y-left, z-up
        if (mission_state == POSITION)
        {
            T_W_B_set.position.x +=
                rc_ch[1] * MAX_MANUAL_VEL * delta_t * (RC_REVERSE_PITCH ? 1 : -1);
            T_W_B_set.position.y +=
                rc_ch[0] * MAX_MANUAL_VEL * delta_t * (RC_REVERSE_ROLL ? 1 : -1);
        }
        if (mission_state == LAND)
        {
            T_W_B_set.position.z -= 0.3 * delta_t;
        }
        if (mission_state == OFFBOARD)
        {
            T_W_B_set = T_W_B_des;
            ROS_WARN("send des point!!!");
        }
        else
        {
            T_W_B_set.position.z +=
                rc_ch[2] * MAX_MANUAL_VEL * delta_t * (RC_REVERSE_THROTTLE ? -1 : 1);//??????????
        }

        if (T_W_B_set.position.z < -0.3)
            T_W_B_set.position.z = -0.3;
        else if (T_W_B_set.position.z > 1.0)
            T_W_B_set.position.z = 1.0;
    }
}

void px4_state_callback(const mavros_msgs::StateConstPtr &state_msg)
{
    px4_state = *state_msg;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "origin_ctl");
    ros::NodeHandle nh("~");
    ros::Rate       rate(5);
    // Parameter_t param;
    // param.config_from_ros_handle(nh_);

    ros::Subscriber odom_sub = nh.subscribe<geometry_msgs::PoseStamped>(
        "/mavros/vision_pose/pose", 100, odom_callback, ros::VoidConstPtr(), ros::TransportHints());
    ros::Subscriber state_sub =
        nh.subscribe<mavros_msgs::State>("/mavros/state", 10, px4_state_callback,
                                         ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());

    ros::Subscriber rc_sub = nh.subscribe<mavros_msgs::RCIn>(
        "/mavros/rc/in", 10, rc_callback, ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());

    // ros::Subscriber pose_set_sub =
    //     nh.subscribe<geometry_msgs::PoseStamped>("/ar/cam_pose_level_set", 100, pose_set_callback);
    // ros::Subscriber pitch_set_sub =
    //     nh.subscribe<std_msgs::Float32>("/ar/cam_pitch_set", 100, pitch_set_callback);

    target_pose_pub =
        nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 100);
    arming_client   = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    ros::Duration(0.5).sleep();

    // dynamic_reconfigure::Server<px4ctrl::fake_rcConfig> server;
    // dynamic_reconfigure::Server<px4ctrl::fake_rcConfig>::CallbackType f;

	//判断从参数服务器读过来的参数设置是否需要遥控：1不需要，0需要
    // if (param.takeoff_land.no_RC)
    // {
    //     f = boost::bind(&Dynamic_Data_t::feed, &fsm.dy_data ,_1); //绑定回调函数
    //     server.setCallback(f); //为服务器设置回调函数， 节点程序运行时会调用一次回调函数来输出当前的参数配置情况
    //     ROS_WARN("PX4CTRL] Remote controller disabled, be careful!");
    // }
    // else
    // {
    //     ROS_INFO("PX4CTRL] Waiting for RC");
    //     while (ros::ok())
    //     {
    //         ros::spinOnce();
    //         if (fsm.rc_is_received(ros::Time::now()))
    //         {
    //             ROS_INFO("[PX4CTRL] RC received.");
    //             break;
    //         }
    //         ros::Duration(0.1).sleep();
    //     }
    // }

    mission_state = INIT;

    T_W_B_des.position.x         = 0.0;
    T_W_B_des.position.y         = 0.0;
    T_W_B_des.position.z         = 0.8;
    T_W_B_des.orientation.w      = 1.0;
    T_W_B_des.orientation.x      = 0.0;
    T_W_B_des.orientation.y      = 0.0;
    T_W_B_des.orientation.z      = 0.0;

    // const char *log;

    // while (!dynamixel_controller.begin("/dev/ttyUSB0", BAUDRATE, &log))  //打开串口
    //     cout << log << endl;
    // uint16_t model_number = 0;
    // bool     result       = dynamixel_controller.ping(ID, &model_number, &log);
    // if (result == false)
    // {
    //     ROS_ERROR("%s", log);
    //     ROS_ERROR("Can't find Dynamixel ID '%d'", ID);
    //     return result;
    // }
    // dynamixel_controller.torqueOff(ID, &log);
    // cout << log << endl;
    // dynamixel_controller.setPositionControlMode(ID, &log);  //设置位置控制模式
    // cout << log << endl;
    // dynamixel_controller.torqueOn(ID, &log);
    // cout << log << endl;
    //上电，只有断电的情况下才能设置控制模式
    // goal_position = dynamixel_controller.convertRadian2Value(ID, INIT_RAD);

    // T_B_C.matrix() << 0.0, 0.0, 1.0, 0.0,
    //                  0.0, 1.0, 0.0, 0.0,
    //                  -1.0, 0.0, 0.0, 0.0,
    //                  0., 0., 0., 1.;



    geometry_msgs::PoseStamped pose;
    // for (int i =0 ; i<20; i++){
    //     pose.header.stamp    = ros::Time::now();
    //     pose.header.frame_id = "map";
    //     pose.pose            = T_W_B_des;
    //     target_pose_pub.publish(pose);
    // }


    while (ros::ok())
    {
        // float radian = 0;
        // dynamixel_controller.getRadian(ID, &radian);

        if(!receive_odom){
            ROS_WARN("no odom!");
        }



        if (mission_state != INIT)
        {
            pose.header.stamp    = ros::Time::now();
            pose.header.frame_id = "map";
            pose.pose            = T_W_B_set;
            target_pose_pub.publish(pose);
        // }
        }else {
            pose.header.stamp    = ros::Time::now();
            pose.header.frame_id = "map";
            pose.pose = T_W_B_des;
            target_pose_pub.publish(pose);
        }
        ROS_INFO_STREAM( "missiont_state:" << mission_state << " mode:" << px4_state.mode );
        if(px4_state.armed)
            cout << "armed:1" << endl;

        // result = dynamixel_controller.itemWrite(ID, "Goal_Position", goal_position,
        //                                         &log);  //写入目标的位置
        // if (!result)
        // {
        //     ROS_ERROR("%s", log);
        //     ROS_ERROR("Failed to write value[%d] on items[Goal_Position] to Dynamixel[ID : %d]",
        //               goal_position, ID);
        // }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}