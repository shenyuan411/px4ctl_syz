#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <thread>

#include "TagDetector.hpp"

// Finite state machine
// VIO-->TAG-->TAG2VIO--
//        ^            |
//        |            |
//        --------------
enum FUSE_STATE
{
    VIO,
    TAG,
    TAG2VIO,
};

FUSE_STATE gFuseState;

ros::Publisher odom_pub;

Eigen::Affine3f T_B_C;

Eigen::Affine3f T_W_V0, T_W_Vt;
double          vio_time;

Eigen::Affine3f T_C0_A, T_Ct_A;
double          tag_time;

Eigen::Affine3f T_W_VA0, T_W_VAt;

void vio_callback(const nav_msgs::OdometryConstPtr &vio_msg)
{
    Eigen::Vector3f    p(vio_msg->pose.pose.position.x, vio_msg->pose.pose.position.y,
                         vio_msg->pose.pose.position.z);
    Eigen::Quaternionf q(vio_msg->pose.pose.orientation.w, vio_msg->pose.pose.orientation.x,
                         vio_msg->pose.pose.orientation.y, vio_msg->pose.pose.orientation.z);
    T_W_Vt.rotate(q);
    T_W_Vt.translation() = p;

    if (gFuseState == VIO)
    {
        vio_time = vio_msg->header.stamp.toSec();
        odom_pub.publish(vio_msg);
    }
    else if (gFuseState == TAG)
    {
        Eigen::Affine3f T_V0_VAt      = T_B_C * T_C0_A * T_Ct_A.inverse() * T_B_C.inverse();
        T_W_VAt                       = T_W_V0 * T_V0_VAt;
        nav_msgs::Odometry odom_msg   = *vio_msg;
        odom_msg.pose.pose.position.x = T_W_VAt.translation().x();
        odom_msg.pose.pose.position.y = T_W_VAt.translation().y();
        odom_msg.pose.pose.position.z = T_W_VAt.translation().z();
        // 不用位姿补偿，可能频率不够
        odom_pub.publish(odom_msg);
    }
    else if (gFuseState == TAG2VIO)
    {
        T_W_Vt                        = T_W_VA0 * T_W_V0.inverse() * T_W_Vt;
        nav_msgs::Odometry odom_msg   = *vio_msg;
        odom_msg.pose.pose.position.x = T_W_Vt.translation().x();
        odom_msg.pose.pose.position.y = T_W_Vt.translation().y();
        odom_msg.pose.pose.position.z = T_W_Vt.translation().z();
        // 不用位姿补偿，可能频率不够
        odom_pub.publish(odom_msg);
    }
}

int                                   tag_len;
std::string                           file     = "../test.jpg";
cv::Mat                               image    = cv::imread(file);
int                                   tag_size = 15;
std::vector<std::vector<cv::Point3f>> tag_coord;
bool                                  ret = LoadBoard("../board.txt", tag_size, tag_coord);
cv::Mat inrinsic = (cv::Mat_<float>(3, 3) << 1200, 0, 1920, 0, 1200, 1440, 0, 0, 1);
cv::Mat distort  = (cv::Mat_<float>(1, 4) << 0, 0, 0, 0);

void tag_process()
{
    tag_len = TagDetector(image, tag_coord, inrinsic, distort, T_Ct_A);

    std::cout << tag_len << std::endl;
    std::cout << T_Ct_A.matrix() << std::endl;
    if (tag_len > 2)  // TODO: 添加距离判断？（ < 2m）
    {
        if (gFuseState == VIO)
        {
            T_W_V0     = T_W_Vt;
            T_C0_A     = T_Ct_A;
            gFuseState = TAG;
        }
        else if (gFuseState == TAG2VIO)
        {
            T_W_V0     = T_W_Vt;
            T_C0_A     = T_Ct_A;
            gFuseState = TAG;
        }
    }
    else if (gFuseState == TAG)
    {
        T_W_V0     = T_W_Vt;
        T_W_VA0    = T_W_VAt;
        gFuseState = TAG2VIO;
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "tag_aided_loc");
    ros::NodeHandle nh("~");

    ros::Subscriber vio_sub =
        nh.subscribe<nav_msgs::Odometry>("/vins_estimator/imu_propagate", 100, vio_callback,
                                         ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());

    odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 100);

    gFuseState = VIO;

    T_B_C = Eigen::Affine3f::Identity();  // TODO: 读取外参

    std::thread tag_thread = std::thread(tag_process);
    ros::spin();
    return 0;
}