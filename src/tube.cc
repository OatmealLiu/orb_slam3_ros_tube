#include "tube.h"

ros::Publisher pose_pub;
ros::Publisher odom_pub;            // +++ Dev +++
ros::Publisher map_points_pub;
image_transport::Publisher rendered_image_pub;

std::string map_frame_id, pose_frame_id;

// Coordinate transformation matrix from orb coordinate system to ros coordinate systemm
tf::Matrix3x3 tf_orb_to_ros(1, 0, 0,
                            0, 1, 0,
                            0, 0, 1);

// Previous estimated Time stmap, Position and Orientation
double prev_t = 0.0;
double prev_x = 0.0;
double prev_y = 0.0;
double prev_z = 0.0;
Eigen::Vector3d prev_rpy;
bool active_odom = false;



void setup_ros_publishers(ros::NodeHandle &node_handler, image_transport::ImageTransport &image_transport)
{
    pose_pub = node_handler.advertise<geometry_msgs::PoseStamped> ("/orb_slam3_ros_tube/pose", 1);

    odom_pub = node_handler.advertise<nav_msgs::Odometry> ("/orb_slam3_ros_tube/odom", 1);            // +++ Dev

    map_points_pub = node_handler.advertise<sensor_msgs::PointCloud2>("/orb_slam3_ros_tube/map_points", 1);

    rendered_image_pub = image_transport.advertise("/orb_slam3_ros_tube/tracking_image", 1);
}

void publish_ros_pose_tf(cv::Mat Tcw, ros::Time current_frame_time, ORB_SLAM3::System::eSensor sensor_type)
{
    if (!Tcw.empty())
    {
        tf::Transform tf_transform = from_orb_to_ros_tf_transform (Tcw);

        publish_tf_transform(tf_transform, current_frame_time);

        publish_pose_stamped(tf_transform, current_frame_time);
    }
}


void publish_tf_transform(tf::Transform tf_transform, ros::Time current_frame_time)
{
    static tf::TransformBroadcaster tf_broadcaster;

    tf_broadcaster.sendTransform(tf::StampedTransform(tf_transform, current_frame_time, map_frame_id, pose_frame_id));
}

// ++ Dev
Eigen::Vector3d quaternion_to_euler_angle(double w, double x, double y, double z)
{
    Eigen::Quaterniond q(w, x, y, z);
    Eigen::Vector3d eulerAngle = q.matrix().eulerAngles(2,1,0);
    
    return eulerAngle;
}

void publish_odometry(geometry_msgs::PoseStamped pose_msg, ros::Time current_frame_time)
{

    /*  geometry_msgs/PoseStamped Message
    std_msgs/Header header
    geometry_msgs/Pose pose
        geometry_msgs/Point position
        geometry_msgs/Quaternion orientation
    */

    /*  nav_msgs/Odometry Message
    Header header
    string child_frame_id
    geometry_msgs/PoseWithCovariance pose
        geometry_msgs/Pose pose
            geometry_msgs/Point position
            geometry_msgs/Quaternion orientation
        float64[36] covariance
    geometry_msgs/TwistWithCovariance twist
        geometry_msgs/Twist twist
            geometry_msgs/Vector3 linear
            geometry_msgs/Vector3 angular
        float64[36] covariance
    */
    // static bool active_odom = false;

    // static double prev_t = 0.0;
    // static double prev_x = 0.0;
    // static double prev_y = 0.0;
    // static double prev_z = 0.0;
    // static Eigen::Vector3d prev_rpy;

    if (active_odom)
    {   
        // Compute delta_t b.t.w. two poses
        double d_t = pose_msg.header.stamp.toSec() - prev_t;

        // Compute displacement along X-Y-Z b.t.w. two poses
        double d_x =  pose_msg.pose.position.x - prev_x;
        double d_y = pose_msg.pose.position.y - prev_y;
        double d_z = pose_msg.pose.position.z - prev_z;

        // Compute rotation with the sequence of Roll-Pitch-Yaw (X-Y-Z) b.t.w. two poses
        Eigen::Vector3d rpy = quaternion_to_euler_angle(pose_msg.pose.orientation.w, pose_msg.pose.orientation.x,\
                                                        pose_msg.pose.orientation.y, pose_msg.pose.orientation.z);
        double d_roll = rpy[2] - prev_rpy[2];
        double d_pitch = rpy[1] - prev_rpy[1];
        double d_yaw = rpy[0] - prev_rpy[0];

        nav_msgs::Odometry odo_msg;

        // Record header
        odo_msg.header = pose_msg.header;          // set odom.header to current pose
        
        // Record current pose
        odo_msg.pose.pose = pose_msg.pose;         // set odom.pose to current pose
        
        // Linear velocity
        odo_msg.twist.twist.linear.x = d_x / d_t;
        odo_msg.twist.twist.linear.y = d_y / d_t;
        odo_msg.twist.twist.linear.z = d_z / d_t;

        // Angular velocity
        odo_msg.twist.twist.angular.x = d_roll / d_t;
        odo_msg.twist.twist.angular.y = d_pitch / d_t;
        odo_msg.twist.twist.angular.z = d_yaw / d_t;

        // Store the current pose estimation as the previous
        prev_t = pose_msg.header.stamp.toSec();
        prev_x = pose_msg.pose.position.x;
        prev_y = pose_msg.pose.position.y;
        prev_z = pose_msg.pose.position.z;
        prev_rpy = rpy;

        // Publish the VOdometry topic
        odom_pub.publish(odo_msg);  // publish odometry message
    }
    else
    {
        prev_t = pose_msg.header.stamp.toSec();
        prev_x = pose_msg.pose.position.x;
        prev_y = pose_msg.pose.position.y;
        prev_z = pose_msg.pose.position.z;
        prev_rpy = quaternion_to_euler_angle(pose_msg.pose.orientation.w, pose_msg.pose.orientation.x,\
                                             pose_msg.pose.orientation.y, pose_msg.pose.orientation.z);

        active_odom = true;
    }

}

void publish_pose_stamped(tf::Transform tf_transform, ros::Time current_frame_time)
{
    tf::Stamped<tf::Pose> grasp_tf_pose(tf_transform, current_frame_time, map_frame_id);

    geometry_msgs::PoseStamped pose_msg;

    tf::poseStampedTFToMsg(grasp_tf_pose, pose_msg);

    publish_odometry(pose_msg, current_frame_time);

    pose_pub.publish(pose_msg);
}

void publish_ros_tracking_img(cv::Mat image, ros::Time current_frame_time)
{
    std_msgs::Header header;

    header.stamp = current_frame_time;

    header.frame_id = map_frame_id;

    const sensor_msgs::ImagePtr rendered_image_msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();

    rendered_image_pub.publish(rendered_image_msg);
}

void publish_ros_tracking_mappoints(std::vector<ORB_SLAM3::MapPoint*> map_points, ros::Time current_frame_time)
{
    sensor_msgs::PointCloud2 cloud = tracked_mappoints_to_pointcloud(map_points, current_frame_time);
    
    map_points_pub.publish(cloud);
}

void setup_tf_orb_to_ros(ORB_SLAM3::System::eSensor sensor_type)
{
    // The conversion depends on whether IMU is involved:
    //  z is aligned with camera's z axis = without IMU
    //  z is aligned with gravity = with IMU
    if (sensor_type == ORB_SLAM3::System::MONOCULAR || sensor_type == ORB_SLAM3::System::STEREO || sensor_type == ORB_SLAM3::System::RGBD)
    {
        tf_orb_to_ros.setValue(
             0,  0,  1,
            -1,  0,  0,
             0, -1,  0);
    }
    else if (sensor_type == ORB_SLAM3::System::IMU_MONOCULAR || sensor_type == ORB_SLAM3::System::IMU_STEREO)
    {
        tf_orb_to_ros.setValue(
             0,  1,  0,
            -1,  0,  0,
             0,  0,  1);
    }
    else
    {
        tf_orb_to_ros.setIdentity();
    }
    
}

tf::Transform from_orb_to_ros_tf_transform(cv::Mat transformation_mat)
{
    cv::Mat orb_rotation(3, 3, CV_32F);
    cv::Mat orb_translation(3, 1, CV_32F);

    orb_rotation    = transformation_mat.rowRange(0, 3).colRange(0, 3);
    orb_translation = transformation_mat.rowRange(0, 3).col(3);

    tf::Matrix3x3 tf_camera_rotation(
        orb_rotation.at<float> (0, 0), orb_rotation.at<float> (0, 1), orb_rotation.at<float> (0, 2),
        orb_rotation.at<float> (1, 0), orb_rotation.at<float> (1, 1), orb_rotation.at<float> (1, 2),
        orb_rotation.at<float> (2, 0), orb_rotation.at<float> (2, 1), orb_rotation.at<float> (2, 2)
    );

    tf::Vector3 tf_camera_translation(orb_translation.at<float> (0), orb_translation.at<float> (1), orb_translation.at<float> (2));

    // cout << setprecision(9) << "Rotation: " << endl << orb_rotation << endl;
    // cout << setprecision(9) << "Translation xyz: " << orb_translation.at<float> (0) << " " << orb_translation.at<float> (1) << " " << orb_translation.at<float> (2) << endl;

    // Transform from orb coordinate system to ros coordinate system on camera coordinates
    tf_camera_rotation    = tf_orb_to_ros * tf_camera_rotation;
    tf_camera_translation = tf_orb_to_ros * tf_camera_translation;

    // Inverse matrix
    tf_camera_rotation    = tf_camera_rotation.transpose();
    tf_camera_translation = -(tf_camera_rotation * tf_camera_translation);

    // Transform from orb coordinate system to ros coordinate system on map coordinates
    tf_camera_rotation    = tf_orb_to_ros * tf_camera_rotation;
    tf_camera_translation = tf_orb_to_ros * tf_camera_translation;

    return tf::Transform(tf_camera_rotation, tf_camera_translation);
}

sensor_msgs::PointCloud2 tracked_mappoints_to_pointcloud(std::vector<ORB_SLAM3::MapPoint*> map_points, ros::Time current_frame_time)
{
    const int num_channels = 3; // x y z

    if (map_points.size() == 0)
    {
        std::cout << "Map point vector is empty!" << std::endl;
    }

    sensor_msgs::PointCloud2 cloud;

    cloud.header.stamp = current_frame_time;
    cloud.header.frame_id = map_frame_id;
    cloud.height = 1;
    cloud.width = map_points.size();
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.point_step = num_channels * sizeof(float);
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.fields.resize(num_channels);

    std::string channel_id[] = { "x", "y", "z"};

    for (int i = 0; i < num_channels; i++)
    {
        cloud.fields[i].name = channel_id[i];
        cloud.fields[i].offset = i * sizeof(float);
        cloud.fields[i].count = 1;
        cloud.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
    }

    cloud.data.resize(cloud.row_step * cloud.height);

    unsigned char *cloud_data_ptr = &(cloud.data[0]);


    for (unsigned int i = 0; i < cloud.width; i++)
    {
        if (map_points[i])
        {

            tf::Vector3 point_translation(map_points[i]->GetWorldPos().at<float> (0), map_points[i]->GetWorldPos().at<float> (1), map_points[i]->GetWorldPos().at<float> (2));

            point_translation = tf_orb_to_ros * point_translation;

            float data_array[num_channels] = {point_translation.x(), point_translation.y(), point_translation.z()};

            memcpy(cloud_data_ptr+(i*cloud.point_step), data_array, num_channels*sizeof(float));
        }
    }
    return cloud;
}