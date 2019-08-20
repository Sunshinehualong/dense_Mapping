
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
#include <iostream>

ros::Publisher cam_pose_pub;

void fcu_pose_callback(const geometry_msgs::PoseStampedConstPtr fcu_msg)
{
    geometry_msgs::PoseStamped cam_pose;
    cam_pose.header.frame_id = fcu_msg->header.frame_id;
    cam_pose.header.stamp = fcu_msg->header.stamp;

    Eigen::Isometry3d fcu_eigen_pose;
    tf::poseMsgToEigen(fcu_msg->pose, fcu_eigen_pose);
    
    // std::cout << " fcu_eigen_pose: \r\n "
    //      << fcu_eigen_pose.matrix() << std::endl;


    //Rotation: in Quaternion [-0.500, 0.500, -0.500, 0.500]
    geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromRollPitchYaw(-1.570, -0.000, -1.570);
    // std::cout << "quat: " << quat << std::endl;

    Eigen::Quaterniond eigen_quat;
    tf::quaternionMsgToEigen(quat, eigen_quat);
    // std::cout << "eigen_quat: " << eigen_quat.normalized().coeffs() << std::endl;

    Eigen::Isometry3d   fcu_to_cam = Eigen::Isometry3d::Identity();
    fcu_to_cam.pretranslate(Eigen::Vector3d::Zero());
    fcu_to_cam.rotate(eigen_quat.normalized().matrix());
    // std::cout << " rotation_eigen_pose: \r\n "
    //      << fcu_to_cam.matrix() << std::endl;    

     Eigen::Isometry3d cam_eigen_pose = fcu_eigen_pose*fcu_to_cam;

    cam_pose.pose.position.x = cam_eigen_pose.translation().x();
    cam_pose.pose.position.y = cam_eigen_pose.translation().y();
    cam_pose.pose.position.z = cam_eigen_pose.translation().z();
    Eigen::Quaterniond q(cam_eigen_pose.rotation());
    cam_pose.pose.orientation.w = q.w();
    cam_pose.pose.orientation.x = q.x();
    cam_pose.pose.orientation.y = q.y();
    cam_pose.pose.orientation.z = q.z();

    cam_pose_pub.publish(cam_pose);

}

bool transform(const tf::TransformListener &listener, geometry_msgs::PoseStamped &cam_pose)
{
    geometry_msgs::PoseStamped cam_origin;
    cam_pose.header.frame_id = "camera_link";
    cam_pose.header.stamp = ros::Time();
    cam_origin.pose.position.x = 0;
    cam_origin.pose.position.y = 0;
    cam_origin.pose.position.z = 0;

    cam_origin.pose.orientation.w = 1;
    cam_origin.pose.orientation.x = 0;
    cam_origin.pose.orientation.y = 0;
    cam_origin.pose.orientation.z = 0;

    try
    {
        listener.transformPose("fcu", cam_origin, cam_pose);
    }
    // catch (tf::TransformException &ex)
    // {
    //     ROS_ERROR("can not transform campose");
    // }
    catch (tf::LookupException &ex)
    {
        ROS_ERROR_THROTTLE(1.0, "No Transform available Error : %s\n", ex.what());
        return false;
    }
    catch (tf::ConnectivityException &ex)
    {
        ROS_ERROR_THROTTLE(1.0, "Connectivity Error, : %s\n", ex.what());
        return false;
    }
    catch (tf::ExtrapolationException &ex)
    {
        ROS_ERROR_THROTTLE(1.0, "Extrapolation Error, : %s\n", ex.what());
        return false;
    }
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fcu_to_cam_pose_node");

    ros::NodeHandle node;
    // tf::TransformListener listener(ros::Duration(10.0));
    ROS_INFO("start listening");
    cam_pose_pub = node.advertise<geometry_msgs::PoseStamped>("/camera_pose", 10);
    ros::Subscriber fcu_pose_sub = node.subscribe("/mavros/local_position/pose", 2, fcu_pose_callback);
    ros::spin();
    // ros::Rate rate(50);
    // while (ros::ok())
    // {
    //     geometry_msgs::PoseStamped cam_pose;
    //     transform(listener, cam_pose);
    //     cam_pose_pub.publish(cam_pose);

    //     rate.sleep();
    // }

    return 0;
}