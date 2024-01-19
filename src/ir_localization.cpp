
#include <memory>
#include <string>
#include <math.h>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "aml_uros_gimbal_interfaces/msg/gimbal_angles.hpp"
#include "aml_uros_gimbal_interfaces/srv/get_gimbal_angles.hpp"
#include "aml_uros_gimbal_interfaces/srv/home_gimbal.hpp"

#include "ortools/graph/assignment.h"
#include "opencv2/opencv.hpp"

#include "areal_ir_localization/beacon.hpp"
#include "areal_ir_localization/rotationMat2Quaternion.hpp"
#include "areal_ir_localization/utility.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class IRLocalization : public rclcpp::Node
{

public:

    // Constructor
    explicit IRLocalization() : Node("ir_localization")
    {

        // ----- Declare all parameters from launch file
        
        // General camera and message parameters
        this->declare_parameter("camera_topic", 
            "/my_camera/pylon_ros2_camera_node/image_raw");
        this->declare_parameter("beacon_baud_rate", 10);
        this->declare_parameter("frame_rate", 50);
        this->declare_parameter("bits_per_message", 12);
        this->declare_parameter("image_max_x", 1440);
        this->declare_parameter("image_max_y", 1080);
        this->declare_parameter("fx", 1700.0);
        this->declare_parameter("fy", 1700.0);
        this->declare_parameter("cx", 720.0);
        this->declare_parameter("cy", 500.0);
        this->declare_parameter("d1", 0.0);
        this->declare_parameter("d2", 0.0);
        this->declare_parameter("d3", 0.0);
        this->declare_parameter("d4", 0.0);

        // Computer vision algorithm parameters
        this->declare_parameter("blur", "Gauss");
        this->declare_parameter("blur_w", 5);
        this->declare_parameter("blur_h", 5);
        this->declare_parameter("blur_sigma_x", 0);
        this->declare_parameter("blur_sigma_y", 0);
        this->declare_parameter("threshold_val", 200);
        this->declare_parameter("pnp_method", "SOLVEPNP_ITERATIVE");
        this->declare_parameter("pnp_ext_guess", false);
        this->declare_parameter("pnp_ransac", false);

        // Gimbal parameters
        this->declare_parameter("gimbal_lockout", 0);
        this->declare_parameter("phi_lockout", 0.0);
        this->declare_parameter("theta_lockout", 0.0);
        this->declare_parameter("publish_gimbal_angles", false);
        this->declare_parameter("exclude_gimbal", false);
        this->declare_parameter("core_id_list", 
            rclcpp::PARAMETER_INTEGER_ARRAY);
        this->declare_parameter("core_edge_buffer", 50);
        this->declare_parameter("phi_change_limit", -1.0);
        this->declare_parameter("theta_change_limit", -1.0);

        // Debug parameters
        this->declare_parameter("debug_topics", false);
        this->declare_parameter("debug_print_all", false);
        this->declare_parameter("debug_print_pnp", false);
        this->declare_parameter("debug_print_id", false);
        this->declare_parameter("debug_print_buffer", false);
        this->declare_parameter("debug_print_process_beacon", false);

        // IR beacon message and id parameters
        this->declare_parameter("messages", rclcpp::PARAMETER_INTEGER_ARRAY);
        this->declare_parameter("msg_ids", rclcpp::PARAMETER_INTEGER_ARRAY);

        // IR beacon real-world dock parameters
        this->declare_parameter("pos_ids", rclcpp::PARAMETER_INTEGER_ARRAY);
        this->declare_parameter("pcb_port", rclcpp::PARAMETER_INTEGER_ARRAY);
        this->declare_parameter("x_pos", rclcpp::PARAMETER_DOUBLE_ARRAY);
        this->declare_parameter("y_pos", rclcpp::PARAMETER_DOUBLE_ARRAY);
        this->declare_parameter("z_pos", rclcpp::PARAMETER_DOUBLE_ARRAY);

        // ----- Setup variables and lookup tables from parameters

        // General camera and message variables (from parameters)
        camera_topic_ = this->get_parameter("camera_topic").as_string();
        beacon_baud_rate_ = this->get_parameter("beacon_baud_rate").as_int();
        frame_rate_ = this->get_parameter("frame_rate").as_int();
        bits_per_message_ =this->get_parameter("bits_per_message").as_int();
        image_max_x_ = this->get_parameter("image_max_x").as_int();
        image_max_y_ = this->get_parameter("image_max_y").as_int();
        fx_ = this->get_parameter("fx").as_double();
        fy_= this->get_parameter("fy").as_double();
        cx_ = this->get_parameter("cx").as_double();
        cy_ = this->get_parameter("cy").as_double();
        d1_ = this->get_parameter("d1").as_double();
        d2_ = this->get_parameter("d2").as_double();
        d3_ = this->get_parameter("d3").as_double();
        d4_ = this->get_parameter("d4").as_double();

        // Computer vision algorithm variables (from parameters)
        blur_ = this->get_parameter("blur").as_string();
        blur_w_ = this->get_parameter("blur_w").as_int();
        blur_h_ = this->get_parameter("blur_h").as_int();
        blur_sigma_x_ = this->get_parameter("blur_sigma_x").as_int();
        blur_sigma_y_ = this->get_parameter("blur_sigma_y").as_int();
        threshold_val_ = this->get_parameter("threshold_val").as_int();
        pnp_method_ = this->get_parameter("pnp_method").as_string();
        pnp_ext_guess_ = this->get_parameter("pnp_ext_guess").as_bool();
        pnp_ransac_ = this->get_parameter("pnp_ransac").as_bool();

        // Create pnp method loopup table
        pnp_lookup_["SOLVEPNP_ITERATIVE"] = cv::SOLVEPNP_ITERATIVE;
        pnp_lookup_["SOLVEPNP_EPNP"] = cv::SOLVEPNP_EPNP;
        pnp_lookup_["SOLVEPNP_IPPE"] = cv::SOLVEPNP_IPPE;

        // Gimbal control variables (from parameters)
        gimbal_lockout_ = this->get_parameter("gimbal_lockout").as_int();
        phi_lockout_ = this->get_parameter("phi_lockout").as_double();
        theta_lockout_ = this->get_parameter("theta_lockout").as_double();
        publish_gimbal_angles_ = \
            this->get_parameter("publish_gimbal_angles").as_bool();
        exclude_gimbal_ = this->get_parameter("exclude_gimbal").as_bool();
        core_id_list_ = this->get_parameter("core_id_list").as_integer_array();
        core_edge_buffer_ = this->get_parameter("core_edge_buffer").as_int();
        phi_change_limit_ = this->get_parameter("phi_change_limit").as_double();
        theta_change_limit_ = this->get_parameter("theta_change_limit").as_double();

        // Debug variables (from parameters)
        debug_topics_ = this->get_parameter("debug_topics").as_bool();
        debug_print_all_ = this->get_parameter("debug_print_all").as_bool();
        debug_print_pnp_ = this->get_parameter("debug_print_pnp").as_bool();
        debug_print_id_ = this->get_parameter("debug_print_id").as_bool();
        debug_print_buffer_ = 
            this->get_parameter("debug_print_buffer").as_bool();
        debug_print_process_beacon_ = 
            this->get_parameter("debug_print_process_beacon").as_bool();

        // Debug print control override
        if (debug_print_all_){
            debug_print_pnp_ = true;
            debug_print_id_ = true;
            debug_print_buffer_ = true;
            debug_print_process_beacon_ = true;
        }

        // Beacon message and ID values (from parameters)
        std::vector<long int> messages = 
            this->get_parameter("messages").as_integer_array();
        std::vector<long int> msg_ids = 
            this->get_parameter("msg_ids").as_integer_array();

        // Setup beacon messages and id values as a map
        for(int i = 0; i < (int)messages.size(); i++){
            msg_lookup_table_[(int)messages[i]] = (int)msg_ids[i];
        }

        // Beacon position and id values as vectors (from parameters)
        std::vector<long int> posID = 
            this->get_parameter("pos_ids").as_integer_array();
        std::vector<long int> pcbPort = 
            this->get_parameter("pcb_port").as_integer_array();
        std::vector<double> xPos = 
            this->get_parameter("x_pos").as_double_array();
        std::vector<double> yPos = 
            this->get_parameter("y_pos").as_double_array();
        std::vector<double> zPos = 
            this->get_parameter("z_pos").as_double_array();

        // Setup beacon position and id info as a map
        for(int i = 0; i < (int)posID.size(); ++i){
        
            // Create struct with needed data
            BeaconPos posStruct;
            posStruct.pcbPort = pcbPort[i];
            posStruct.xPos = xPos[i];
            posStruct.yPos = yPos[i];
            posStruct.zPos = zPos[i];

            // Add struct to map
            pos_lookup_table_[posID[i]] = posStruct;

        }

        // Setup camera matrix 
        camera_matrix_ = cv::Mat::zeros(3, 3, cv::DataType<double>::type);
        camera_matrix_.at<double>(0, 0) = fx_;
        camera_matrix_.at<double>(1, 1) = fy_;
        camera_matrix_.at<double>(0, 2) = cx_;
        camera_matrix_.at<double>(1, 2) = cy_;

        // Setup vector of distortion coefficients from parameters
        distortion_coeffs_ = cv::Mat(4, 1, cv::DataType<double>::type);
        distortion_coeffs_.at<double>(0) = d1_;
        distortion_coeffs_.at<double>(1) = d2_;
        distortion_coeffs_.at<double>(2) = d3_;
        distortion_coeffs_.at<double>(3) = d4_;

        // Init frame buffer.  
        /**
         * This is: frame_rate / baud_rate * bits per message
         * frames/sec * 1/(bits/sec) * bits/message = frames/message
         */
        num_frames_per_message_ = frame_rate_ / beacon_baud_rate_*
            bits_per_message_;
        frame_buffer_ = 
            (Frame *)malloc(sizeof(Frame) * num_frames_per_message_);

        // Init the frame buffer to null pointers
        for (int i = 0; i < num_frames_per_message_; i++)
        {
            frame_buffer_[i].contour_list = nullptr;
            frame_buffer_[i].association_to_prev = nullptr;
        }
        frame_index_ = 0;
        frame_count_ = 0;

        // Setup translation and rotation vectors
        tvec_ = cv::Mat::zeros(3, 1, cv::DataType<double>::type);
        rvec_ = cv::Mat::zeros(3, 1, cv::DataType<double>::type);

        // Initialize the gimbal setpoints to 0
        phi_setpoint_ = 0.0;
        theta_setpoint_ = 0.0;

        sensor_qos_profile_ = rclcpp::SensorDataQoS();

        // Setup the subscriber to camera image topic
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            camera_topic_, sensor_qos_profile_, 
            std::bind(&IRLocalization::camera_callback, this, _1));

        // Setup publisher for camera pose
        ir_pose_publisher_ = 
            this->create_publisher<geometry_msgs::msg::PoseStamped>(
                "ir_cam_pose", 10);

        // Setup publisher for camera position in camera frame
        cam_pos_publisher_ = 
            this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
                "ir_cam_position_C", 10);

        // Setup publisher for gimbal angle setpoints
        gimbal_setpoint_publisher_ = this->create_publisher\
            <aml_uros_gimbal_interfaces::msg::GimbalAngles>\
            ("uROS_gimbal_angle_setpoints", 10);

        // Setup publisher for camera range
        ir_range_publisher_ = 
            this->create_publisher<sensor_msgs::msg::Range>(
                "ir_cam_range", 10);

        // Setup initialization timer
        init_timer_ = this->create_wall_timer(1s, 
            std::bind(&IRLocalization::init_timer_callback, this));

        // ----- Gimbal angle publisher portions
        // Only create the publisher and client if they are needed
        if (publish_gimbal_angles_){

            // Gimbal angle client
            gimbal_angle_client_ = this->create_client\
                <aml_uros_gimbal_interfaces::srv::GetGimbalAngles>\
                ("/getGimbalAngles");

            // Gimbal angle publisher
            gimbal_angle_publisher_ = this->create_publisher\
                <aml_uros_gimbal_interfaces::msg::GimbalAngles>\
                ("uROS_gimbal_angles", sensor_qos_profile_);

        }
        
        // ----- Exclude gimbal portions
        /*
         * Setpoint published in all cases, just no homing service, b/c it
         * stops the program. Gimbal setpoints are also possibly useful for 
         * debugging, and don't stop anything (12/18/2023)
        */
        if (!exclude_gimbal_){

            // Setup the client for the gimbal home service
            home_gimbal_client_ = this->create_client\
                <aml_uros_gimbal_interfaces::srv::HomeGimbal>("/homeGimbal");

        }

        // ----- Debug constructor portions
        if (debug_topics_){

            // Initialize the threshold and contour image variables
            debug_thresh_im_ = cv::Mat( image_max_y_, image_max_x_, CV_8UC3);
            debug_cont_im_ = cv::Mat(image_max_y_, image_max_x_, CV_8UC3);

            // Initialize the threshold and contour message variables
            msg_thresh_im_.header.frame_id = "threshold_image";
            msg_thresh_im_.height = image_max_y_;
            msg_thresh_im_.width = image_max_x_;
            msg_thresh_im_.encoding = "rgb8";
            msg_thresh_im_.is_bigendian = false;
            msg_thresh_im_.step = debug_thresh_im_.cols * 3;

            msg_cont_im_.header.frame_id = "contour_image";
            msg_cont_im_.height = image_max_y_;
            msg_cont_im_.width = image_max_x_;
            msg_cont_im_.encoding = "rgb8";
            msg_cont_im_.is_bigendian = false;
            msg_cont_im_.step = debug_cont_im_.cols * 3;

            // Thresholded image publisher
            thresh_im_publisher_ = 
                this->create_publisher<sensor_msgs::msg::Image>(
                    "DEBUG_thresh_im", 10);

            // Contoured image publisher
            cont_im_publisher_ = 
                this->create_publisher<sensor_msgs::msg::Image>(
                    "DEBUG_cont_im", 10);

        }

    } // IRLocalization()

    // Destructor
    ~IRLocalization()
    {
        for (int i = 0; i < num_frames_per_message_; i++)
        {
        free(frame_buffer_[i].contour_list);
        }
        free(frame_buffer_);
    }

private:

    // Callback method for timer triggered shortly after initialization
    void init_timer_callback(){

        // ----- Wait for and call gimbal home service if desired
        if (!exclude_gimbal_){

            // Setup service request
            auto home_gimbal_request = std::make_shared\
                <aml_uros_gimbal_interfaces::srv::HomeGimbal::Request>();
            rclcpp::Time now = this->get_clock()->now();
            home_gimbal_request->timestamp = now;

            // Wait for service to become avaliable
            while (!home_gimbal_client_->wait_for_service(1s)) {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Home gimbal "
                    "service not available, waiting again...");
            }

            // Send the service request
            auto result = 
                home_gimbal_client_->async_send_request(home_gimbal_request);
                
        }

        // ----- Wait for and call gimbal angle service if desired
        if (publish_gimbal_angles_){

            // Setup service request
            auto gimbal_angle_request = std::make_shared\
                <aml_uros_gimbal_interfaces::srv::GetGimbalAngles::Request>();
            rclcpp::Time now = this->get_clock()->now();
            gimbal_angle_request->timestamp = now;

            // Wait for service to become avaliable
            while (!gimbal_angle_client_->wait_for_service(1s)) {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Gimbal angle "
                    "service not available, waiting again...");
            }

            // Send the gimbal angle service request with callback on future
            auto result = gimbal_angle_client_->async_send_request(
                gimbal_angle_request, 
                std::bind(&IRLocalization::gimbal_service_callback, this, 
                    std::placeholders::_1));

        }

        // ----- SPACE FOR ADDITIONAL 1-TIME BEHAVIOR BELOW

        
        // ----- END OF 1-TIME BEHAVIOR ZONE

        // Deactivate timer
        init_timer_->cancel();

    }

    // Callback method on each new camera frame
    void camera_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {

        frame_count_ = frame_count_ + 1;

        // Get and publish gimbal angles if desired
        if(publish_gimbal_angles_){

            // Setup service request
            auto gimbal_angle_request = std::make_shared\
                <aml_uros_gimbal_interfaces::srv::GetGimbalAngles::Request>();
            rclcpp::Time now = this->get_clock()->now();
            gimbal_angle_request->timestamp = now;

            // Send the gimbal angle service request with callback on future
            auto result = gimbal_angle_client_->async_send_request(
                gimbal_angle_request, 
                std::bind(&IRLocalization::gimbal_service_callback, this, 
                    std::placeholders::_1));

        }

        // Read image data from message
        cv::Mat image = cv::Mat(msg->height, msg->width, CV_8UC1, 
            msg->data.data());

        // Apply image filtering and smoothing
        // Use a gaussian blur
        cv::GaussianBlur(image, image, cv::Size(blur_w_, blur_h_), 
            blur_sigma_x_, blur_sigma_y_);
        
        // Apply a threshold to the image
        cv::threshold(image, image, threshold_val_, 255, cv::THRESH_BINARY);

        if (debug_topics_){
            cv::cvtColor(image, debug_thresh_im_, cv::COLOR_GRAY2RGB);
        }

        // Detect the contours in the image
        std::vector<std::vector<cv::Point>> cv_contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(image, cv_contours, hierarchy, cv::RETR_EXTERNAL, 
            cv::CHAIN_APPROX_SIMPLE);

        // Add contours from this image to the frame buffer
        fill_frame_buffer(cv_contours);

        // Associate contours from frame to frame
        associate_contours();

        // Process the contours as beacons and extract beacon IDs
        identify_beacons();

        // If 4+ beacons compute and publish camera pose
        if (valid_beac_points_.size() > 3)
        {
            solve_pose();

            pose_publisher_call(); // Call publisher for camera pose
        }

        // Compute target gimbal angles from contours and publish if any points
        // Below identification so core 4 beacons can be kept in frame
        compute_gimbal_setpoint();

        // Publish image processing topics if debug set
        if (debug_topics_){
            
            // Convert to color for contour plotting
            cv::cvtColor(image, debug_cont_im_, cv::COLOR_GRAY2RGB);

            // Draw the contours on the debug image
            cv::drawContours(debug_cont_im_, cv_contours, -1, 
                cv::Scalar(255, 50, 50), 3);

            // Set threshold image message data
            msg_thresh_im_.header.stamp = this->now();
            msg_thresh_im_.data = std::vector<uint8_t>(debug_thresh_im_.data, 
                debug_thresh_im_.data + 
                    debug_thresh_im_.total()*debug_thresh_im_.elemSize());

            // Set contour image data
            msg_cont_im_.header.stamp = this->now();
            msg_cont_im_.data = std::vector<uint8_t>(debug_cont_im_.data, 
                debug_cont_im_.data + 
                    debug_cont_im_.total()*debug_cont_im_.elemSize());

            // Publish debug image topics
            thresh_im_publisher_->publish(msg_thresh_im_);
            cont_im_publisher_->publish(msg_cont_im_);

        }

        // Increment the frame index
        frame_index_ = (frame_index_ + 1) % num_frames_per_message_;

    }

    // Method to add contours from current image to the frame buffer
    void fill_frame_buffer(std::vector<std::vector<cv::Point>> cv_contours){
        /*
         * Currently allocates memory for all contours in the image, and then
         * only adds those that are not touching the image edge. The 
         * num_contours variable is set by this count of valid contours. This
         * section should be refactored to use vectors for resizing (12/18/2023)
         */
        
        // Free the old frame
        free(frame_buffer_[frame_index_].contour_list);
        free(frame_buffer_[frame_index_].association_to_prev);

        // Allocate memory for this portion of the frame buffer
        frame_buffer_[frame_index_].contour_list = 
            (Contour *)malloc(sizeof(Contour)*cv_contours.size());
        frame_buffer_[frame_index_].association_to_prev = 
            (int *)malloc(sizeof(int)*cv_contours.size());

        im_points_.clear(); // Reset list of points in this image
        int valid_count = 0; // Start count of contours not touching edge

        centroid_contours_.clear(); // Reset vector of centroid information

        // Initialize debug text for use if needed
        std::string raw_block = "\n\n" + std::to_string(cv_contours.size());
        raw_block += " Raw Contours:\n";
        std::string valid_block = " Valid Contours:\n";
        std::string excluded_block = " Excluded Contours:\n";

        double total_area = 0;
        im_centroid_.x = 0;
        im_centroid_.y = 0;

        // Iterate through each contour found in this frame
        for (int i = 0; i < (int)cv_contours.size(); i++) 
        {
            // Get position and size of contour
            cv::Point2f center;
            float radius;
            double area;
            cv::minEnclosingCircle(cv_contours[i], center, radius);
            area = cv::contourArea(cv_contours[i]);
            // cv::Moments M = cv::moments(cv_contours);
            im_centroid_.x += area*center.x;
            im_centroid_.y += area*center.y;
            total_area += area;

            im_points_.push_back(center); // Add to im points for gimbal

            // Debug print for all contours
            if (debug_print_buffer_){
                std::string IDX = std::to_string(i);
                std::string IMX = std::to_string(center.x);
                std::string IMY = std::to_string(center.y);
                std::string RAD = std::to_string(radius);
                std::string AREA = std::to_string(area);
                std::string raw_line = "IDX: " + IDX + " X: " + IMX + \
                    " Y: " + IMY + " RAD: " + RAD + " AREA: " + AREA + "\n";
                raw_block += raw_line;
            }

            // Check if contour not touching image edge
            if (!((center.x + radius > image_max_x_) || 
                  (center.x - radius < 0) ||
                  (center.y + radius > image_max_y_) || 
                  (center.y - radius < 0)))
            {

                // Add the area and center to the frame buffer
                frame_buffer_[frame_index_].contour_list[valid_count].area = 
                    area; 
                frame_buffer_[frame_index_].contour_list[valid_count].radius = 
                    radius;
                frame_buffer_[frame_index_].contour_list[valid_count].\
                    position.x = center.x;
                frame_buffer_[frame_index_].contour_list[valid_count].\
                    position.y = center.y;

                // Set association to -1
                frame_buffer_[frame_index_].\
                    association_to_prev[valid_count] = -1;

                // Set the bit to -1
                frame_buffer_[frame_index_].contour_list[valid_count].bit = -1;

                valid_count = valid_count + 1; // Increment valid count

                // Add to debug statement if needed
                if (debug_print_buffer_){
                    std::string IDX = std::to_string(i);
                    std::string IMX = std::to_string(center.x);
                    std::string IMY = std::to_string(center.y);
                    std::string RAD = std::to_string(radius);
                    std::string AREA = std::to_string(area);
                    std::string valid_line = "IDX: " + IDX + " X: " + IMX + \
                        " Y: " + IMY + " RAD: " + RAD + " AREA: " + AREA + "\n";
                    valid_block += valid_line;
                }

            } else{

                // Add to centroid contours
                Contour centroid_data;
                centroid_data.area = area;
                centroid_data.position.x = center.x;
                centroid_data.position.y = center.y;
                centroid_contours_.push_back(centroid_data);

                // Add to debug statement if needed
                if (debug_print_buffer_){
                    std::string IDX = std::to_string(i);
                    std::string IMX = std::to_string(center.x);
                    std::string IMY = std::to_string(center.y);
                    std::string RAD = std::to_string(radius);
                    std::string AREA = std::to_string(area);
                    std::string excluded_line = "IDX: " + IDX + " X: " + IMX + \
                        " Y: " + IMY + " RAD: " + RAD + " AREA: " + AREA + "\n";
                    excluded_block += excluded_line;
                }

            }

        } // For all contours in frame

        // Divide by total area
        im_centroid_.x /= total_area;
        im_centroid_.y /= total_area;

        // Set number of valid contours in this frame
        frame_buffer_[frame_index_].num_contours = valid_count;

        // Free any extra entries in frame buffer

        // Debug printing section
        if (debug_print_buffer_){

            // Set starting text
            std::string start_block = "\n";
            start_block += std::string(27, '=');
            start_block += " FRAME BUFFER DEBUG START ";
            start_block += std::string(27, '=');

            // Add counts to text sections
            valid_block = "\n" + std::to_string(valid_count) + valid_block;
            excluded_block = "\n" + std::to_string(cv_contours.size() - 
                valid_count) + excluded_block;

            std::string middle_block = raw_block + valid_block + excluded_block;

            // Set ending text
            std::string end_block = "\n";
            end_block += std::string(28, '=');
            end_block += " FRAME BUFFER DEBUG END ";
            end_block += std::string(28, '=');

            // Join and print all text sections
            std::string print_block = start_block + middle_block + end_block;
            RCLCPP_INFO(this->get_logger(), print_block.c_str());

        } // Debug print

    }

    // Method to associate contours from one frame to the next
    void associate_contours()
    {

        Frame *curr = &frame_buffer_[frame_index_];
        Frame *prev = 
            &frame_buffer_[(frame_index_ - 1 + num_frames_per_message_) % 
            num_frames_per_message_];

        // Check if curr or prev is empty, if so return:
        if (prev->num_contours == 0)
        {
            // RCLCPP_ERROR(this->get_logger(), "Prev frame empty");
            return;
        }
        if (curr->num_contours == 0)
        {
            // RCLCPP_ERROR(this->get_logger(), "Curr frame empty");
            return;
        }

        int difference_of_num_contours = 
            curr->num_contours - prev->num_contours;
        int num_contour_assignments = curr->num_contours > prev->num_contours ? 
            curr->num_contours : prev->num_contours;

        // Create a simple linear sum assignment object and add the distances 
        // between contours to it
        operations_research::SimpleLinearSumAssignment assignment;
        for (int i = 0; i < curr->num_contours; i++)
        {
            for (int ii = 0; ii < prev->num_contours; ii++)
            {
                int dist = sqrt(pow(curr->contour_list[i].position.x - 
                    prev->contour_list[ii].position.x, 2) + 
                    pow(curr->contour_list[i].position.y -
                    prev->contour_list[ii].position.y, 2));
                assignment.AddArcWithCost(i, ii, dist);
            }
        }
            // Pad the distance matrix with 0s to make it square
            if (difference_of_num_contours != 0)
            {

            // If curr has more contours than prev, add more columns
            if (difference_of_num_contours > 0) 
            {

                for (int i = 0; i < difference_of_num_contours; i++)
                {
                for (int ii = 0; ii < curr->num_contours; ii++)
                {
                    assignment.AddArcWithCost(ii, prev->num_contours + i, 0);
                }
                }
            }
            else // if prev has more contours than curr, add more rows
            {
                for (int i = 0; i < -1 * difference_of_num_contours; i++)
                {
                for (int ii = 0; ii < prev->num_contours; ii++)
                {
                    assignment.AddArcWithCost(curr->num_contours + i, ii, 0);
                }
                }
            }
        }

        // Solve the assignment problem
        operations_research::SimpleLinearSumAssignment::Status status = 
            assignment.Solve();

        // If error, print error
        if (status != operations_research::SimpleLinearSumAssignment::OPTIMAL)
        {
            RCLCPP_ERROR(this->get_logger(), "Assignment Error 1: %d", status);
            return;
        }

        int average_distance = 0;
        // Get the average distance by adding up the distances and dividing by 
        // the number of contours
        for (int i = 0; i < num_contour_assignments; i++)
        {
            average_distance += assignment.AssignmentCost(i);
        }
        average_distance /= num_contour_assignments;

        // A TEMPORARY SOLUTION TO PROBLEM OF ASSIGNING CONTOURS TO EACHOTHER 
        // for the one comes in one leaves edge case (11/8/23)
        const int THRESHOLD = 40000;
        int watchdog = 0;
        while (average_distance > THRESHOLD || watchdog > 100)
        {
            watchdog++;

            // Add another row and column of 0s to the assignment object
            for (int i = 0; i < curr->num_contours; i++)
            {
                assignment.AddArcWithCost(i, prev->num_contours + watchdog, 0);
            }
            for (int i = 0; i < prev->num_contours; i++)
            {
                assignment.AddArcWithCost(prev->num_contours + watchdog, i, 0);
            }

            // Solve the assignment problem
            operations_research::SimpleLinearSumAssignment::Status status = 
                assignment.Solve();

            // If error, print error
            if (status != 
                operations_research::SimpleLinearSumAssignment::OPTIMAL)
            {
                RCLCPP_ERROR(this->get_logger(), "Assignment Error: %d", 
                    status);
                return;
            }

            average_distance = 0;
            // Get the average distance by adding up the distances and 
            // dividing by the number of contours
            for (int i = 0; i < num_contour_assignments; i++)
            {
                average_distance += assignment.AssignmentCost(i);
            }
            average_distance /= num_contour_assignments;
        }

        // Iterate through each contour in the current frame and determine its 
        // association to the previous frame
        for (int i = 0; i < curr->num_contours; i++)
        {
            // Check if the contour is associated with a contour in the 
            // previous frame or not
            if (assignment.RightMate(i) >= prev->num_contours)
            {
                // If not, set the association to -1
                curr->association_to_prev[i] = -1;
                continue;
            }
            curr->association_to_prev[i] = assignment.RightMate(i);

        }

    }

    // Method to extract beacon ID and position from associated contours
    void identify_beacons()
    {

        // Determine the number of contours in this frame
        int num_contours = frame_buffer_[frame_index_].num_contours;

        // Make the beacon vector
        std::vector<Beacon> beacon_list(num_contours);
        // beacon_list_.resize(num_contours);

        // Clear image, object point, and id vectors (used by PnP)
        obj_points_.clear();
        valid_beac_points_.clear();
        valid_id_list_.clear();
        valid_rad_list_.clear();

        // Initialze debug text
        std::string beacon_block = "";
        std::string bit_block = "";
        std::string beacon_processing_block = "";

        for (int i = 0; i < num_contours; i++) // Iterate through each contour
        {

            // Initialize the beacon object in the vector
            beacon_list[i] = 
                Beacon(bits_per_message_, frame_rate_, beacon_baud_rate_,
                    &msg_lookup_table_);

            // Add contour to this beacon                       
            beacon_list[i].add(&frame_buffer_[frame_index_].contour_list[i]);
            int association = frame_buffer_[frame_index_].\
                association_to_prev[i];
            int ii = frame_index_ - 1;

            // Create linked list of contour associations
            while (association != -1 && 
                    beacon_list[i].get_size() < num_frames_per_message_){

                beacon_list[i].add(&frame_buffer_\
                    [(ii + num_frames_per_message_) % num_frames_per_message_].\
                    contour_list[association]);
                association = frame_buffer_[(ii + num_frames_per_message_) % 
                    num_frames_per_message_].association_to_prev[association];

                ii--;
            }

            // Process the beacon to assign id if valid
            std::string processing_lines = \
                beacon_list[i].process_beacon(debug_print_process_beacon_);

            // Add beacon data to centroid contours vector
            Contour centroid_data;
            centroid_data.area = beacon_list[i].get_avg_area();
            centroid_data.position.x = beacon_list[i].get_position().x;
            centroid_data.position.y = beacon_list[i].get_position().y;
            centroid_contours_.push_back(centroid_data);

            if (debug_print_process_beacon_){
                std::string IDX = std::to_string(i);
                beacon_processing_block += "Beacon " + IDX + ":\n" + \
                    processing_lines + "\n";
            }

            if(debug_print_id_){
                std::string IDX = std::to_string(i);
                bit_block += "Beacon " + IDX + ": " +
                    beacon_list[i].get_bits_str() + "\n";
            }

            // If the beacon is valid
            if(beacon_list[i].is_valid()){

                int valid_id = beacon_list[i].get_id(); // Get valid beacon id

                // Add entry to debug text if needed
                if (debug_print_id_){
                    std::string IDX = std::to_string(i);
                    std::string MSG = 
                        beacon_list[i].get_message_string().c_str();
                    std::string MSG_NUM = 
                        std::to_string(beacon_list[i].get_message());
                    std::string ID = std::to_string(valid_id);
                    std::string IMX = 
                        std::to_string(beacon_list[i].get_position().x);
                    std::string IMY = 
                        std::to_string(beacon_list[i].get_position().y);
                    std::string beacon_line = "IDX: " + IDX + " MSG: " + MSG +\
                        " MSG #: " + MSG_NUM + " ID: " + ID + " IM: (" + IMX + \
                        ", " + IMY + ")\n";
                    beacon_block += beacon_line;
                }

                // Check for beacon ID = 0 (valid but unidentified)
                if (valid_id != 0){

                    // Add image, object points, and id to vectors
                    valid_beac_points_.push_back(cv::Point2f\
                        (beacon_list[i].get_position().x, 
                        beacon_list[i].get_position().y));
                    valid_rad_list_.push_back(beacon_list[i].get_radius());
                    obj_points_.push_back(cv::Point3f\
                        (pos_lookup_table_[valid_id].xPos, 
                        pos_lookup_table_[valid_id].yPos, 
                        pos_lookup_table_[valid_id].zPos));
                    valid_id_list_.push_back(valid_id);

                } // if beacon is not ID = 0

            } // if beacon is valid

        } // for each beacon

        // Check for and resolve the repeat ID case if at least 2 beacons
        if (valid_id_list_.size() > 1){

            // Generate vector of repeated id numbers
            std::vector<int> repeated_values = 
                findRepeatedValues(valid_id_list_);

            // Loop through all sets of repeated ID numbers
            for (int i : repeated_values) {

                // Get idx's of repeated value in valid_id_list_
                std::vector<int> repeat_idx = 
                    getIndicesOfValue(valid_id_list_, i);

                // If repeated id number is in lookup table
                if (id_im_assignment_table_.count(i)){

                    // Find image coordinates from last frame
                    cv::Point2f prev_coords = id_im_assignment_table_[i];

                    std::vector<float> distances = computeDistances(i, 
                        prev_coords, valid_id_list_, valid_beac_points_);

                    // Assume unique distances b/c floating point
                    int min_dist_idx = findMinIndex(distances);

                    // Remove minimum distance idx from repeat values (to keep)
                    repeat_idx.erase(repeat_idx.begin() + min_dist_idx);

                } 

                // Remove non-mimimum distance elems from id, im, obj vecs
                removeElementsAtIndices(valid_id_list_, repeat_idx);
                removeElementsAtIndices(valid_beac_points_, repeat_idx);
                removeElementsAtIndices(valid_rad_list_, repeat_idx);
                removeElementsAtIndices(obj_points_, repeat_idx);

            } // Loop all repeated values

        } // End of repeat ID resolution

        id_im_assignment_table_.clear(); // Empty pairing table

        // Create lookup table of id-im_point pairings from cleaned vectors
        for(int i = 0; i < (int)valid_id_list_.size(); i++){
            id_im_assignment_table_[(int)valid_id_list_[i]] = 
                valid_beac_points_[i];
        }

        // Debug printing section
        if (debug_print_id_){

            // Set starting text
            std::string start_block = "\n";
            start_block += std::string(29, '=');
            start_block += " BEACON ID DEBUG START ";
            start_block += std::string(28, '=');

            // Number of contours
            std::string intro_block = "\n\n" + std::to_string(num_contours) +
                " Potential Beacons";

            // Detailed becon processing section if desired
            if (debug_print_process_beacon_){
                beacon_processing_block = "\n\nBeacon Processing:\n" + \
                    beacon_processing_block;
            }

            // Full bit data
            bit_block = "Raw Bits:\n" + bit_block;

            // Add header to beacon data
            beacon_block = "\nBeacon Data:\n" + beacon_block;

            // Append content blocks together
            std::string middle_block = intro_block + beacon_processing_block + \
                bit_block + beacon_block;

            // Set ending text
            std::string end_block = "\n";
            end_block += std::string(30, '=');
            end_block += " BEACON ID DEBUG END ";
            end_block += std::string(29, '=');

            // Join and print all text sections
            std::string print_block = start_block + middle_block + end_block;
            RCLCPP_INFO(this->get_logger(), print_block.c_str());

        } // Debug printing

    } // identify_beacons()

    // Method to solve for the camera pose from identified beacons
    void solve_pose()
    {

        // Assign the PnP algorithm
        int pnp_alg_use = pnp_lookup_[pnp_method_];

        // Frame-based external seed - No major effect (12/27/23)
        // if (frame_count_ < 100){
        //     pnp_ext_guess = false;
        // } else{
        //     pnp_ext_guess = true;
        // }

        // Solve PnP
        cv::solvePnP(obj_points_, valid_beac_points_, camera_matrix_, 
            distortion_coeffs_, rvec_, tvec_, pnp_ext_guess_, pnp_alg_use);

        // Pose refinement - No major effect (12/27/23)
        // cv::solvePnPRefineLM(obj_points_, valid_beac_points_, camera_matrix_, 
        //     distortion_coeffs_, rvec_, tvec_);
        // cv::solvePnPRefineVVS(obj_points_, valid_beac_points_, camera_matrix_, 
        //     distortion_coeffs_, rvec_, tvec_);

        // Debug printing section
        if (debug_print_pnp_){

            // Set starting text
            std::string start_block = "\n";
            start_block += std::string(32, '=');
            start_block += " PNP DEBUG START ";
            start_block += std::string(31, '=');

            // Solution points text
            std::string point_block = "\n\n";
            point_block += std::to_string(valid_id_list_.size());
            point_block += " Solution Points: \n";
            for (int i = 0; i < (int)valid_id_list_.size(); i++){
                std::string ID = std::to_string(valid_id_list_[i]);
                std::string IMX = std::to_string(valid_beac_points_[i].x);
                std::string IMY = std::to_string(valid_beac_points_[i].y);
                std::string OBX = std::to_string(obj_points_[i].x);
                std::string OBY = std::to_string(obj_points_[i].y);
                std::string OBZ = std::to_string(obj_points_[i].z);
                std::string point_line = "ID: " + ID + " IM: (" + IMX + ", " + \
                    IMY + ") OBJ: (" + OBX + ", " + OBY + ", " + OBZ + ")\n";            
                point_block += point_line;
            }

            // Solved position text
            std::string x_curr = std::to_string(tvec_.at<double>(0));
            std::string y_curr = std::to_string(tvec_.at<double>(1));
            std::string z_curr = std::to_string(tvec_.at<double>(2));
            std::string pos_block = "\nSolved Position: " + x_curr + ", " + \
                y_curr + ", " + z_curr + "\n";

            // Solved orientation text
            std::string r1_curr = std::to_string(rvec_.at<double>(0));
            std::string r2_curr = std::to_string(rvec_.at<double>(1));
            std::string r3_curr = std::to_string(rvec_.at<double>(2));
            std::string rot_block = "Solved Orientation: " + r1_curr + \
                ", " + r2_curr + ", " + r3_curr + "\n";

            std::string middle_block = point_block + pos_block + rot_block;

            // Set ending text
            std::string end_block = "\n";
            end_block += std::string(33, '=');
            end_block += " PNP DEBUG END ";
            end_block += std::string(32, '=');

            // Join and print all text sections
            std::string print_block = start_block + middle_block + end_block;
            RCLCPP_INFO(this->get_logger(), print_block.c_str());

        } // Debug print

    } // solve_pose()

    // Method to publish camera pose
    void pose_publisher_call()
    {

        // Convert rodrigues vec to rotation matrix
        cv::Mat R_V_C(3, 3, cv::DataType<double>::type);
        cv::Mat R_C_V(3, 3, cv::DataType<double>::type);
        cv::Mat tvec_V(3, 1, cv::DataType<double>::type);
        cv::Rodrigues(rvec_, R_V_C);
        R_C_V = R_V_C.t();

        // Convert to quaternion
        double q[4];
        getQuaternion(R_V_C, q);

        // Rotate tvec into V frame
        tvec_V = -R_C_V*tvec_;

        // Set message header data
        msg_ir_cam_pose_.header.stamp = this->now();
        msg_cam_pos_C_.header.stamp = this->now();
        msg_ir_range_.header.stamp = this->now();

        // Set msg_ir_cam_pose_ pose data
        msg_ir_cam_pose_.pose.position.x = tvec_V.at<double>(0);
        msg_ir_cam_pose_.pose.position.y = tvec_V.at<double>(1);
        msg_ir_cam_pose_.pose.position.z = tvec_V.at<double>(2);
        msg_ir_cam_pose_.pose.orientation.x = q[0];
        msg_ir_cam_pose_.pose.orientation.y = q[1];
        msg_ir_cam_pose_.pose.orientation.z = q[2];
        msg_ir_cam_pose_.pose.orientation.w = q[3];

        // Set C-frame camera position data
        msg_cam_pos_C_.vector.x = tvec_.at<double>(0);
        msg_cam_pos_C_.vector.y = tvec_.at<double>(1);
        msg_cam_pos_C_.vector.z = tvec_.at<double>(2);

        // Set range data
        msg_ir_range_.range = cv::norm(tvec_);

        // Publish topics
        ir_pose_publisher_->publish(msg_ir_cam_pose_);
        cam_pos_publisher_->publish(msg_cam_pos_C_);
        ir_range_publisher_->publish(msg_ir_range_);

    }

    // Method to compute the gimbal angular setpoint from contours
    void compute_gimbal_setpoint(){

        // Compute image centroid for gimbals
        im_centroid_.x = 0.0;
        im_centroid_.y = 0.0;
        double total_area = 0.0;
        for (int i = 0; i < (int)centroid_contours_.size(); i++){

            float cont_area = centroid_contours_[i].area;

            total_area += cont_area;
            im_centroid_.x += centroid_contours_[i].position.x*cont_area;
            im_centroid_.y += centroid_contours_[i].position.y*cont_area;

        }

        im_centroid_.x /= total_area;
        im_centroid_.y /= total_area;

        // Compute the gimbal angle setpoint from beacons if in view
        if (im_points_.size() > 0){

            // Compute the angular errors from the centroid
            theta_setpoint_ = atan((im_centroid_.x - cx_) / fx_) * (180.0/M_PI);
            phi_setpoint_ = -atan((im_centroid_.y - cy_) / fy_) * (180.0/M_PI);

            // Apply gimbal angle change bounds if set
            if (phi_change_limit_ > 0){
                boundByMagnitude(phi_setpoint_, phi_change_limit_);
            }

            if (theta_change_limit_ > 0){
                boundByMagnitude(theta_setpoint_, theta_change_limit_);
            }

        } else{

            // If no points no change in setpoint
            theta_setpoint_ = 0.0;
            phi_setpoint_ = 0.0;

        }

        // Core beacon setpoint modification check 
        core_beacon_setpoint_guard();
        
        /*
         * Determine if absolute or relative gimbal angle command. Very simple
         * right now, just if lockout called or not. Could allow more complex
         * or direct commands of gimbal angle in the future
         */
        bool absolute_cmd = gimbal_lockout_ != 0; 
        uint8_t msg_type = 0;
        double phi_abs, theta_abs;

        if (absolute_cmd){ // If commanding gimbal angles absolutely

            msg_type = 0; // Set absolute angle command type

            // Set the absolute gimbal angle commands
            // Possibly from other sources in the future
            
            if (gimbal_lockout_ == 1){ // If a soft lockout

                phi_abs = phi_lockout_ + phi_setpoint_;
                theta_abs = theta_lockout_ + theta_setpoint_;

            } else if (gimbal_lockout_ == 2){ // If a hard lockout

                phi_abs = phi_lockout_;
                theta_abs = theta_lockout_;

            }
        
            // Form the angular portion of setpoint message    
            gimbal_angles_set_.phi_angle = phi_abs;
            gimbal_angles_set_.theta_angle = theta_abs;
            gimbal_angles_set_.msg_type = msg_type;

        } else{ // If commanding a change in gimbal angle

            if (im_points_.size() > 0){

                msg_type = 1; // Set change in angle command type

                // Form the angular portion of setpoint message    
                gimbal_angles_set_.phi_angle = phi_setpoint_;
                gimbal_angles_set_.theta_angle = theta_setpoint_;

            }else {

                msg_type = 0; // Set absolute angle command type
                phi_abs = 0.0;
                theta_abs = 0.0;
                gimbal_angles_set_.phi_angle = phi_abs;
                gimbal_angles_set_.theta_angle = theta_abs;

            }

            gimbal_angles_set_.msg_type = msg_type;

        }

        // Publish absolute gimbal angle setpoints
        rclcpp::Time now = this->get_clock()->now();
        gimbal_angles_set_.timestamp = now;
        gimbal_setpoint_publisher_->publish(gimbal_angles_set_);

    } // compute_gimbal_setpoint()

    // Method to modify gimbal setpoints to keep core beacons in frame
    void core_beacon_setpoint_guard(){

        // Initialize vectors of guard setpoints
        std::vector<double> guard_phi_set_vec, guard_theta_set_vec;

        // If any beacons in valid id list
        if ((int)valid_id_list_.size() > 0){

            // Loop through all core beacons
            for (int i = 0; i < (int)core_id_list_.size(); i++){

                // Check beacon id is core
                auto core_check = std::find(valid_id_list_.begin(), 
                    valid_id_list_.end(), core_id_list_[i]);

                if (core_check != valid_id_list_.end()){

                    // Get information about valid core beacon
                    int valid_idx = std::distance(valid_id_list_.begin(), 
                        core_check);
                    int core_x = (int)valid_beac_points_[valid_idx].x;
                    int core_y = (int)valid_beac_points_[valid_idx].y;
                    int core_rad = (int)valid_rad_list_[valid_idx];

                    // Check out of frame right (maximum x)
                    if ((core_x + core_rad) > \
                        (image_max_x_ - core_edge_buffer_)){
                        // If core beacon + radius in right edge buffer:

                        // Compute guardrail theta setpoint
                        int x_max_overlap = (image_max_x_ - core_edge_buffer_) - 
                            (core_x + core_rad);
                        double theta_restore = -atan(x_max_overlap / fx_) * \
                            (180.0/M_PI);

                        // Append to setpoint vector
                        guard_theta_set_vec.push_back(theta_restore);

                    }

                    // Check out of frame left (minimum x)
                    if ((core_x - core_rad) < core_edge_buffer_){
                        // If core beacon + radius in left edge buffer:

                        // Compute restoring theta setpoint
                        int x_min_overlap = core_edge_buffer_ - \
                            (core_x - core_rad);
                        double theta_restore = -atan(x_min_overlap / fx_) * \
                            (180.0/M_PI);

                        // Append to setpoint vector
                        guard_theta_set_vec.push_back(theta_restore);

                    }

                    // Check out of frame bottom (maximum y)
                    if ((core_y + core_rad) > \
                        (image_max_y_ - core_edge_buffer_)){
                        // If core beacon + radius in bottom edge buffer:

                        // Compute restoring phi setpoint
                        int y_max_overlap = (image_max_y_ - core_edge_buffer_) - 
                            (core_y + core_rad);
                        double phi_restore = atan(y_max_overlap / fy_) * \
                            (180.0/M_PI);

                        // Append to setpoint vector
                        guard_phi_set_vec.push_back(phi_restore);

                    }

                    // Check out of frame top (minimum y)
                    if ((core_y - core_rad) < core_edge_buffer_){
                        // If core beacon + radius in top edge buffer:

                        // Compute restoring phi correction
                        int y_min_overlap = core_edge_buffer_ - \
                            (core_y - core_rad);
                        double phi_restore = atan(y_min_overlap / fy_) * \
                            (180.0/M_PI);

                        // Append to setpoint vector
                        guard_phi_set_vec.push_back(phi_restore);

                    }

                } // End of check if beacon from core list is in valid list

            } // End of loop through all core beacons

            // Process theta setpoint vector
            if ((int)guard_theta_set_vec.size() > 0){ // If any guard setpoint

                // Check if signs of guard set vector match
                bool vec_sign_match = haveSameSign(guard_theta_set_vec);

                if (vec_sign_match){ // If guard setpoints all same sign

                    // Get maximum magnitude element of vector
                    double max_mag_elem = 
                        getMaxMagnitudeElement(guard_theta_set_vec);

                    // If sign of setpoint and max mag element match
                    bool set_sign_match = signsMatch(theta_setpoint_, 
                        max_mag_elem);
                    
                    if (set_sign_match){

                        // Only change setpoint if guard element has larger mag
                        if (std::abs(max_mag_elem)> std::abs(theta_setpoint_)){
                            theta_setpoint_ = max_mag_elem;
                        }

                    } else{ // If setpoint and max mag element are different

                        // Use maximum magnitude guard setpoint
                        theta_setpoint_ = max_mag_elem;

                    }   

                } else{ // If guard setpoints different signs

                    // Change gimbal setpoint to average of maximum pos and 
                    // minimum negative values
                    std::pair<double, double> min_max_values = \
                        getMinMaxValues(guard_theta_set_vec);

                    theta_setpoint_ = (min_max_values.first + 
                        min_max_values.second)/2;

                }

            } // End of theta guard setpoint processing

            // Process phi setpoint vector
            if ((int)guard_phi_set_vec.size() > 0){ // If any guard setpoint

                // Check if signs of guard set vector match
                bool vec_sign_match = haveSameSign(guard_phi_set_vec);

                if (vec_sign_match){ // If guard setpoints all same sign

                    // Get maximum magnitude element of vector
                    double max_mag_elem = 
                        getMaxMagnitudeElement(guard_phi_set_vec);

                    // If sign of setpoint and max mag element match
                    bool set_sign_match = signsMatch(phi_setpoint_, 
                        max_mag_elem);
                    
                    if (set_sign_match){

                        // Only change setpoint if guard element has larger mag
                        if (std::abs(max_mag_elem) > std::abs(phi_setpoint_)){
                            phi_setpoint_ = max_mag_elem;
                        }

                    } else{ // If setpoint and max mag element are different

                        // Use maximum magnitude guard setpoint
                        phi_setpoint_ = max_mag_elem;

                    }   

                } else{ // If guard setpoints different signs

                    // Change gimbal setpoint to average of maximum pos and 
                    // minimum negative values
                    std::pair<double, double> min_max_values = \
                        getMinMaxValues(guard_phi_set_vec);

                    phi_setpoint_ = (min_max_values.first + 
                        min_max_values.second)/2;

                }

            } // End of phi guard setpoint processing

        } // End of check if and valid beacon points exist

        // Beacon id vector comparison
        // std::vector<int> core_id_sorted(core_id_list_);
        // std::vector<int> valid_id_sorted(valid_id_list_);
        // std::sort(core_id_sorted.begin(), core_id_sorted.end());
        // std::sort(valid_id_sorted.begin(), valid_id_sorted.end());
        // bool beac_id_match = core_id_sorted == valid_id_sorted;

        // Check conditions for core beacon protection
        // If # contours not equal to number of beacons or beacon id list
        // does not match core list exactly, activate
        // if((im_points_.size() == valid_beac_points_.size()) & (beac_id_match)){

        //     // Placeholder: take no action

        // } else{ // Core beacon setpoint modification active

        // }            

    } // End of core beacon protection setpoint control
    
    // Callback method for gimbal angle service
    void gimbal_service_callback(
        rclcpp::Client<aml_uros_gimbal_interfaces::srv::GetGimbalAngles>::\
        SharedFuture angles)
    {

        // Get gimbal angles from result future
        auto gimbal_angle_result = angles.get();

        // Assign to publisher and publish
        rclcpp::Time now = this->get_clock()->now();
        gimbal_angles_.timestamp = now;
        gimbal_angles_.theta_angle = gimbal_angle_result->theta_angle;
        gimbal_angles_.phi_angle = gimbal_angle_result->phi_angle;
        gimbal_angles_.msg_type = 0;
        gimbal_angle_publisher_->publish(gimbal_angles_);

    }

    // ----- IRLocalization class variables

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr 
        ir_pose_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr 
        cam_pos_publisher_;
    rclcpp::Publisher<aml_uros_gimbal_interfaces::msg::GimbalAngles>::SharedPtr 
        gimbal_setpoint_publisher_;
    rclcpp::Publisher<aml_uros_gimbal_interfaces::msg::GimbalAngles>::SharedPtr 
        gimbal_angle_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr thresh_im_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr cont_im_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr ir_range_publisher_;

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;

    // Clients
    rclcpp::Client<aml_uros_gimbal_interfaces::srv::GetGimbalAngles>::SharedPtr
        gimbal_angle_client_;
    rclcpp::Client<aml_uros_gimbal_interfaces::srv::HomeGimbal>::SharedPtr
        home_gimbal_client_;

    // Timers
    rclcpp::TimerBase::SharedPtr init_timer_;

    // Interfaces
    sensor_msgs::msg::Image msg_thresh_im_;
    sensor_msgs::msg::Image msg_cont_im_;
    sensor_msgs::msg::Range msg_ir_range_;
    geometry_msgs::msg::PoseStamped msg_ir_cam_pose_;
    geometry_msgs::msg::Vector3Stamped msg_cam_pos_C_;
    aml_uros_gimbal_interfaces::msg::GimbalAngles gimbal_angles_;
    aml_uros_gimbal_interfaces::msg::GimbalAngles gimbal_angles_set_;

    // Camera and message variables (from parameters)
    std::string camera_topic_;
    int beacon_baud_rate_;
    int frame_rate_;
    int bits_per_message_;
    int image_max_x_;
    int image_max_y_;
    double fx_, fy_, cx_, cy_;
    double d1_, d2_, d3_, d4_;

    // Computer vision algorithm variables (from parameters)
    std::string blur_;
    int blur_w_;
    int blur_h_;
    int blur_sigma_x_;
    int blur_sigma_y_;
    int threshold_val_;
    std::string pnp_method_;
    bool pnp_ext_guess_;
    bool pnp_ransac_;

    // Gimbal control variables (from parameters)
    int gimbal_lockout_;
    double phi_lockout_;
    double theta_lockout_;
    bool publish_gimbal_angles_;
    bool exclude_gimbal_;
    std::vector<long int> core_id_list_;
    int core_edge_buffer_;
    double phi_change_limit_;
    double theta_change_limit_;

    // Gimbal control variables
    double phi_setpoint_;
    double theta_setpoint_;
    double phi_angle_;
    double theta_angle_;
    std::vector<Contour> centroid_contours_;

    // Lookup tables
    std::map<std::string, int> pnp_lookup_;
    std::map<int, int> msg_lookup_table_;
    std::map<int, BeaconPos> pos_lookup_table_;
    std::map<int, cv::Point2f> id_im_assignment_table_;


    
    // Other variables
    cv::Point2f im_centroid_;
    cv::Mat debug_thresh_im_;
    cv::Mat debug_cont_im_;
    Frame *frame_buffer_;
    // int num_beacons_;
    int num_frames_per_message_;
    int frame_index_;
    // std::vector<Beacon> beacon_list_;
    int frame_count_;

    std::vector<cv::Point2f> im_points_; // Vector of centerpoints all contours
    std::vector<cv::Point2f> valid_beac_points_; // Vec all valid ID beacons
    std::vector<int> valid_id_list_;
    std::vector<float> valid_rad_list_;
    std::vector<cv::Point3f> obj_points_;
    cv::Mat camera_matrix_;
    cv::Mat distortion_coeffs_;
    cv::Mat tvec_;
    cv::Mat rvec_;

    

    rclcpp::SensorDataQoS sensor_qos_profile_;

    bool debug_topics_;
    

    bool debug_print_all_;
    bool debug_print_pnp_;
    bool debug_print_id_;
    bool debug_print_buffer_;
    bool debug_print_process_beacon_;

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IRLocalization>());
    rclcpp::shutdown();
    return 0;
}
