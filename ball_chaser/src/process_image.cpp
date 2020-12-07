#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
//
#include <vector>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include <cv_bridge/cv_bridge.h>
#include <math.h>

// Define a global client that can request services
ros::ServiceClient client;

// Target ball color
cv::Vec3b target_color = cv::Vec3b((char)255, (char)255, (char)255);

// speed factors
float angular_z_factor = 0.001;
float linear_x_factor = 0.3;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // ROS_INFO_STREAM("Driveing robot to the target");
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    if (!client.call(srv))
        ROS_ERROR("Failed to call service command_robot");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image &img)
{
    cv::Mat cv_img;
    cv::Mat cv_img_gray;
    
    try
    {
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;

        //convert img to opencv format
        cv_img = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8)->image;

        // grayscale image
        cv::cvtColor(cv_img, cv_img_gray, cv::COLOR_BGR2GRAY);

        // apply Canny filter
        cv::Canny(cv_img_gray, cv_img_gray, 100, 200, 3);

        // find contours, externals only
        findContours(cv_img_gray, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);

        if (contours.size() == 0)
        {
            drive_robot(0, 0);
            return;
        }

        float angular_z = 0.0;
        float linear_x = 0.0;
        for (size_t i = 0; i < contours.size(); i++)
        {
            // calculate contour area
            double c_area = cv::contourArea(contours[i]);

            // calculate hull polygon area
            std::vector<cv::Point> hull(contours[i].size());
            cv::convexHull(cv::Mat(contours[i]), hull, false);
            double c_area_hull = cv::contourArea(hull);

            // calculate circle area by perimeter
            double c_arc_len = cv::arcLength(contours[i], true);

            // calculate minimum enclosing circle
            cv::Point2f mec_center;
            float mec_radius = 0;
            cv::minEnclosingCircle(contours[i], mec_center, mec_radius);

            // calculte the solidity of the target area
            double solidity = c_area_hull / c_area;

            // calculate matching_circle_factor by comparing the contour area and the minimum enclosing circle area
            float min_enclosing_circle_area = 3.141592 * mec_radius * mec_radius;

            float matching_circle_factor = c_area / min_enclosing_circle_area;

            // Using 2 factors to make sure we found a circle
            // matching_circle_factor +-0.2
            // solidity +-0.05
            // and a radius limit to stop when it is too close to the ball or remove noise small circles
            
            // Debug only
            // drawContours(cv_img, contours, (int)i, cv::Scalar(0, 255, 0), 2, cv::LINE_8, hierarchy, 0);
            if (fabs(matching_circle_factor - 1) < 0.2 && fabs(solidity - 1) < 0.05 && mec_radius > 10 && mec_radius < 200)
            {
                // assuming we have only 1 ball with the specified color to follow
                cv::Vec3b circle_color = cv_img.at<cv::Vec3b>(mec_center);

                if (circle_color == target_color)
                {
                    angular_z = (cv_img.size().width / 2 - mec_center.x) * angular_z_factor;
                    linear_x = (1 - fabs((mec_center.x / (cv_img.size().width)) - 0.5)) * linear_x_factor;
                    
                    // Debug only
                    // drawContours(cv_img, contours, (int)i, cv::Scalar(0, 0, 255), 2, cv::LINE_8, hierarchy, 0);
                    // ROS_INFO("linear_x:%1.2f\tangula_z:%1.2f", linear_x, angular_z);

                }
            }
            
        }
        drive_robot(linear_x, angular_z);
        // Debug only:
        // cv::imshow("colored", cv_img);
        // cv::imshow("gray", cv_img_gray);
        // cv::waitKey(3);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR(">> Error: cv_bridge exception: %s", e.what());
        return;
    }
}

int main(int argc, char **argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}