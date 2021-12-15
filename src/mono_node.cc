/**
* 
* Adapted from ORB-SLAM3: Examples/ROS/src/ros_mono.cc
*
*/

#include "tube.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM) : mpSLAM(pSLAM) {}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM3::System* mpSLAM;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    if (argc > 1)
    {
        ROS_WARN ("Arguments supplied via command line are neglected.");
    }

    ros::NodeHandle node_handler;
    std::string node_name = ros::this_node::getName();
    image_transport::ImageTransport image_transport(node_handler);

    // Params for ORB-SLAM-3 library
    // Will be set in .launch file
    std::string voc_file, settings_file;
    node_handler.param<std::string>(node_name + "/voc_file", voc_file, "file_not_set");
    node_handler.param<std::string>(node_name + "/settings_file", settings_file, "file_not_set");

    if (voc_file == "file_not_set" || settings_file == "file_not_set")
    {
        ROS_ERROR("Please provide voc_file and settings_file in the launch file");       
        ros::shutdown();
        return 1;
    }


    // Params for ROS
    // Will be set in .launch file
    node_handler.param<std::string>(node_name + "/map_frame_id", map_frame_id, "map");
    node_handler.param<std::string>(node_name + "/pose_frame_id", pose_frame_id, "pose");

    // 修改吗？
    bool enable_pangolin = false;
    node_handler.param<bool>(node_name + "/enable_pangolin", enable_pangolin, false);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(voc_file, settings_file, ORB_SLAM3::System::MONOCULAR, enable_pangolin);
    ImageGrabber igb(&SLAM);

    std::string cam_topic_name = "/sam/perception/camera_front_left/image_color";
    node_handler.param<std::string>(node_name + "/cam_topic_name", cam_topic_name, "/sam/perception/camera_front_left/image_color");
    // ros::Subscriber sub_img0 = node_handler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage, &igb);
//    ros::Subscriber sub_img0 = node_handler.subscribe("/sam/perception/camera_front_left/image_color", 1, &ImageGrabber::GrabImage, &igb);  // sam left camera
    ros::Subscriber sub_img0 = node_handler.subscribe(cam_topic_name, 1, &ImageGrabber::GrabImage, &igb);  // sam left camera

    setup_ros_publishers(node_handler, image_transport);

    setup_tf_orb_to_ros(ORB_SLAM3::System::MONOCULAR);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Main algorithm runs here
    // mpSLAM->TrackMonocular return the estimated Camera pose
    // Proccess the given monocular frame and optionally imu data
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails).
    cv::Mat Tcw = mpSLAM->TrackMonocular(cv_ptr->image, cv_ptr->header.stamp.toSec());

    ros::Time current_frame_time = msg->header.stamp;

    publish_ros_pose_tf(Tcw, current_frame_time, ORB_SLAM3::System::MONOCULAR);

    publish_ros_tracking_mappoints(mpSLAM->GetTrackedMapPoints(), current_frame_time);

    // publish_ros_tracking_img(mpSLAM->GetCurrentFrame(), current_frame_time);
}