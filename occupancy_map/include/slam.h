#pragma once

class Slam : public rclcpp::Node {
public:
    Slam(ORB_SLAM3::System* pSLAM, const string &strSettingFile, const string &strRectify, const string &strEqual);
    ~Slam();
private:
    void callback_imu(const sensor_msgs::msg::Imu::SharedPtr msg);
    void callback_RGBD(const sensor_msgs::msg::Image::SharedPtr msgRGB, const sensor_msgs::msg::Image::SharedPtr msgD);

    std::thread *syncThread;

    bool doRectify;
    bool doEqual;
    cv::Mat M1l, M2l, M1r, M2r;

    bool bClahe;
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8,8));

    //IMU
    queue<sensor_msgs::msg::SharedPtr> imuBuf;
    std::mutex bufMutex;
    
    ORB_SLAM3::System* m_SLAM

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subIMU;

    cv_bridge::CvImageConstPtr cv_ptrRGB;
    cv_bridge::CvImageConstPtr cv_ptrD;

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> subRGB;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> subD;
    
    std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> syncApprox;

};