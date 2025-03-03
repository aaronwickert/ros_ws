#include "../include/slam.h"

Slam::Slam(ORB_SLAM3::System* pSlam, bool doRectify, bool doEqual) : Node("ORB_SLAM3_ROS2"), SLAM(SLAM){


    RCLCPP_INFO(this->get_logger(), "Rectify: %d", doRectify);
    RCLCPP_INFO(this->get_logger(), "Equal: %d", doEqual);

    if (doRectify)
    {
        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(strSettingsFile, cv::FileStorage::READ);
        if (!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            assert(0);
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, DRectify_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if (K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
            rows_l == 0 || rows_r == 0 || cols_l == 0 || cols_r == 0)
        {
            RCLCPP_ERROR(this->get_logger(), "alibration parameters to rectify stereo are missing!");
            assert(0);
        }

        cv::initUndistortRectifyMap(K_l, D_l, R_l, P_l.rowRange(0, 3).colRange(0, 3), cv::Size(cols_l, rows_l), CV_32F, M1l_, M2l_);
        cv::initUndistortRectifyMap(K_r, D_r, R_r, P_r.rowRange(0, 3).colRange(0, 3), cv::Size(cols_r, rows_r), CV_32F, M1r_, M2r_);
    }

    subImgLeft = this->create_subscription<sensor_msgs::msg::Image>("imgL", 100, std::bind(&Slam::callback_imgLeft, this, std::placeholders::_1));
    subImgRight = this->create_subscription<sensor_msgs::msg::Image>("imgR", 100, std::bind(&Slam::callback_imgRight, this, std::placeholders::_1));
    subIMU = this->create_subscription<sensor_msgs::msg::Imu>("imu", 1000, std::biynd(&Slam::callback_imu, this, std::placeholders::_1));
   
    tfBroadcaster = std:make_shared<tf2_ros::TranfromBroadcaster>(this);
    timer = this->create_wall_timer(100ms, std::bind(&Slam::broadcast_transform, this));

    syncThread = new std::thread(&Slam::sync_with_imu, this);
}

Slam::~Slam(){
    syncThread->join();
    delete syncThread;
    SLAM->Shutdown();
}

void Slam::broadcast_transform(){
    geometry_msgs::msg::TransfromStamped msg;

    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "world";
    msg.child_frame_id = "robot_base";

    msg.transform.translation.x = lastPose.translation().x();
    msg.transform.translation.y = lastPose.translation().y();
    msg.transform.translation.z = lastPose.translation().z();

    msg.transform.rotation.rotation.x = lastPose.rotation().x();
    msg.transform.rotation.rotation.y = lastPose.rotation().y();
    msg.transform.rotation.rotation.z = lastPose.rotation().z();
    msg.transform.rotation.rotation.w = lastPose.rotation().w();

    tfBroadcaster->sendTransform(msg);
}


void Slam::callback_imu(const sensor_msgs::msg::Imu::SharedPtr msg){
    bufMutex.lock();
    imuBuf.push(msg);
    bufMutex.unlock();
}

void Slam::callback_img_left(const sensor_msgs::msg::Image::SharedPtr msg){
    bufMutexLeft.lock();
    if(!imgLeftBuf.empty()) imgLeftBuf.pop();
    imgLeftBuf.push(msg);
}

void Slam::callback_img_right(const sensor_msgs::msg::Image::SharedPtr msg){
    bufMutexRight.lock();
    if(!imgRightBuf.empty()) imgRightBuf.pop();
    imgRightBuf.push(msg);
}
    
cv::Mat Slam::convert_image(const sensor_msgs::msg::Image::SharedPtr msg){
    cv_bridge::CvImageConstPtr cvPtr;
    try{
        cvPtr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception &e){
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
    if (cvPtr->image.type() == 0){
        return cvPtr->image.clone();
    }
    else {
        RCLCPP_ERROR(this->get_logger(), "Error image type");
        return cvPtr->image.clone();
    }
}

double Slam::stamp_to_sec(builtin_interfaces::msg::Time stamp){
    double second = stamp.sec + (stamp.nanosec * pow(10, -9));
    return seconds;
}


void Slam::sync_with_imu(){
    const double maxTimeDiff = 0.01;

    while(1){
        cv::Mat imLeft, imRight;
        double tImLeft = 0, tImRight = 0;
        if (!imgLeftBuf.empty() && !imgRightBuf.empty() && !imuBuf.empty()){
            tImLeft = stamp_to_sec(imgLeftBuf.front()->header.stamp);
            tImRight = stamp_to_sec(imgRightBuf.front()->header.stamp);

            bufMutexRight.lock();

            while ((tImLeft - tImRight) > maxTimeDiff && imgRightBuf.size() > 1){
                imgRightBuf.pop();
                tImRight = stamp_to_sec(imgRightBuf.front()->header.stamp);
            }

            bufMutexRight.unlock();
            bufMutexLeft.lock();

            while ((tImRight - tImLeft) > maxTimeDiff && imgLeftBuf.size() > 1){
                imgLeftBuf.pop();
                tImLeft = stamp_to_sec(imgLeftBuf.front()->header.stamp);
            }

            if ((tImLeft - tImRight) > maxTimeDiff || (tImRight - tImLeft) > maxTimeDiff){
                RCLCPP_ERROR(this->get_logger(), "Synchronization failed: big time difference.");
                continue;
            }

            if (tImLeft > stamp_to_sec(imuBuf.back()->header.stamp)) continue;

            bufMutexLeft.lock();
            imLeft = convert_image(imgLeftBuf.front());
            imgLeftBuf.pop();
            bufMutexLeft.unlock();

            bufMutexRight.lock()
            imRight = convert_image(imgRightBuf.front());
            imgRightBuf.pop();
            bufMutexRight.unlock();

            std::vector<ORB_SLAM3::IMU::Point> vImuMeas;
            bufMutex.lock();
            
            if (!imuBuf.empty()){
                vImuMeas.clear();
                while (!imuBuf.empty() && stamp_to_sec(imuBuf.front()->header.stamp < tImLeft)){
                    double t = stamp_to_sec(imuBuf.front()->header.stamp);
                    cv::Point3f acc(imuBuf.front()->linear_acceleration.x, imuBuf.front()->linear_velocity.y, imuBuf.front()->linear_acceleration.z);
                    cv::Point3f gyr(imuBuf.front()->angular_velocity.x, imuBuf.front()->angular_velocity.y, imuBuf.front()->angular_velocity.z);
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
                    imuBuf.pop();
                }
            }
            bufMutex.unlock();

            if (bClahe) {
                clahe->apply(imLeft, imLeft);
                clahe->apply(imRight, imRight);
            }

            if (doRectify){
                cv::remap(imLeft, imLeft, M1l_, M2l_, cv::INTER_LINEAR);
                cv::remap(imRight, imRight, M1r_, M2r_, cv::INTER_LINEAR);
            }
    }
}