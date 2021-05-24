#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>

/**
 * Stair detection ros ndoe which utilizes OpenCV to classify stairways in a height map 
 * provided in the form of a grid map
 * 
 */

// If true the different filter steps will be visualized and parameters can be adjusted via sliders
bool debug = true;

// Parameters for the Canny filter
int lowThreshold = 10;
int highThreshold = 129;
const int max_lowThreshold = 100;
int canny_kernel_size = 1;

// The different OpenCV images used
cv::Mat src_scaled, dst, zoomedImage, cdst, processed, erosion_dst, dilation_dst, stairs_dst;

// The region of interest in the grid map
cv::Rect roi;

// Parameters for the Probabilistig Hough Line Transform
int lineThreshold = 13;
int minLineLength = 9;
int maxLineGap = 4;
int rho = 1;
int theta_scale = 100;

// Parameters for the Laplacian filter
int scale = 1;
int delta = 0;
int laplacian_kernel_size = 0;

// Radius of the Gaussian blur
int blurRadius = 37;


// Titles of the debug windows
const char *src_window_name = "Original Image";
const char *process_window_name = "Processed Image";
const char *canny_window_name = "Canny";
const char *laplacian_window_name = "Laplacian";
const char *line_window_name = "Hough Line Transform";
const char *extracted_line_window_name = "Extracted Lines";




void updateCanny(int, void *);
void updateLaplacian(int, void *);
void updatePreprocess(int, void *);
static void updateLineDetection(int, void *);
void gridMapCallback(grid_map_msgs::GridMap msg);


/**
 * Applies the Canny filter to create edge image.
 */ 
void updateCanny(int, void *)
{
    Canny(processed, dst, lowThreshold, highThreshold, 3 + canny_kernel_size * 2);
    if (debug)
    {
        imshow(canny_window_name, dst(roi));
        updateLineDetection(0, 0);
    }
}

/**
 * Applies the Laplacien to create alternative edge image. Not used at the moment.
 */
void updateLaplacian(int, void *)
{
    int ddepth = CV_8U;

    Laplacian(processed, dst, ddepth, 3 + laplacian_kernel_size * 2, scale, delta, cv::BORDER_DEFAULT);

    if (debug)
    {
        imshow(laplacian_window_name, dst(roi));
    }
}


/**
 * Applies the Gaussian blur to reduce noise.
 */
void updatePreprocess(int, void *)
{
    blurRadius = blurRadius - (blurRadius % 2) + 1;
    cv::GaussianBlur(src_scaled, processed, cv::Size(blurRadius, blurRadius), 0.0, 0.0);

    if (debug)
    {
        imshow(process_window_name, processed(roi));

        updateLaplacian(0, 0);
        updateCanny(0, 0);
    }
}


/**
 * Detect lines useing the Probabilistic Hough Line Transform
 */ 
static void updateLineDetection(int, void *)
{

    cv::Mat cdstP;
    cvtColor(dst, cdst, cv::COLOR_GRAY2BGR);
    cdstP = cdst.clone();

    // Probabilistic Hough Line Transform
    std::vector<cv::Vec4i> linesP;                                                                            // will hold the results of the detection
    HoughLinesP(dst, linesP, rho, theta_scale / 100 * CV_PI / 180, lineThreshold, minLineLength, maxLineGap); // runs the actual detection

    // Draw the lines
    stairs_dst = cv::Mat(src_scaled.size(), src_scaled.type(), cv::Scalar(255, 255, 255));
    for (size_t i = 0; i < linesP.size(); i++)
    {
        cv::Vec4i l = linesP[i];
        line(stairs_dst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 0, 0), 0.5, cv::LINE_AA);
        line(cdstP, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 0, 255), 0.5, cv::LINE_AA);

    }
    

    if (debug)
    {
        imshow(line_window_name, cdstP(roi));
        imshow(extracted_line_window_name, stairs_dst(roi));
    }
    ROS_INFO_STREAM("Lines extreacted: " << linesP.size());
}


/**
 * Callback method for the grid map. Starts the filtering Process.
 * 
 */
void gridMapCallback(grid_map_msgs::GridMap msg)
{
  
    grid_map::GridMap map;
    grid_map::GridMapRosConverter::fromMessage(msg, map);

    ROS_INFO_STREAM("Time" << map.getTimestamp());

    if (debug)
    {
        grid_map::Matrix &elevation = map.get("elevation");

        float min = 9999.0;
        float max = -9999.0;

        // Calculating the minimum and maximum elevation in the height map
        // to calibrate the grid map to opencv conversion
        for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator)
        {
            const int i = iterator.getLinearIndex();
            if (elevation(i) < min)
                min = elevation(i);
            if (elevation(i) > max)
                max = elevation(i);
        }
        ROS_INFO_STREAM("min: " << min << " max: " << max);
    }
    cv::Mat src;

    // Converting the grid map to an opencv image
    grid_map::GridMapCvConverter::toImage<unsigned char, 1>(map, "elevation", CV_8UC1, -0.075, 1.995, src);

    double resizeFactor = 3.0;

    cv::resize(src, src_scaled, cv::Size(), resizeFactor, resizeFactor);
    if (debug)
    {

        // Creating and resizing windows for each filtering step
        cv::namedWindow(src_window_name, cv::WINDOW_NORMAL);
        cv::namedWindow(process_window_name, cv::WINDOW_NORMAL);
        cv::namedWindow(canny_window_name, cv::WINDOW_NORMAL);
        cv::namedWindow(laplacian_window_name, cv::WINDOW_NORMAL);
        cv::namedWindow(line_window_name, cv::WINDOW_NORMAL);
        cv::namedWindow(extracted_line_window_name, cv::WINDOW_NORMAL);
       
        int width = 920;
        int height = 980;
        cv::resizeWindow(src_window_name, width, height);
        cv::resizeWindow(process_window_name, width, height);
        cv::resizeWindow(canny_window_name, width, height);
        cv::resizeWindow(laplacian_window_name, width, height);
        cv::resizeWindow(line_window_name, width, height);
        cv::resizeWindow(extracted_line_window_name, width, height);
        cv::moveWindow(line_window_name, width + 50, 0);
    }

    // Specifying the region of interist in the image
    roi.width = src_scaled.size().width / 11;
    roi.height = src_scaled.size().height / 11;

   
    roi.x = 500;
    roi.y = 3100;
    ROS_INFO_STREAM("x"<<roi.x);
    ROS_INFO_STREAM("y"<<roi.y);
    

    if (src_scaled.empty())
    {
        ROS_INFO_STREAM("SRC Empty");
    }
    ROS_INFO_STREAM(src_scaled.depth());
    
    
    if (debug)
    {
        cv::imshow(src_window_name, src_scaled(roi));
    }
    updatePreprocess(0, 0);
    
    ROS_INFO_STREAM("Image displayed");

    // Creating Trackbars for the most important parameters
    if (debug)
    {
        cv::createTrackbar("Min Threshold:", canny_window_name, &lowThreshold, max_lowThreshold, updateCanny);
        cv::createTrackbar("Max Threshold:", canny_window_name, &highThreshold, 3000, updateCanny);
        cv::createTrackbar("Kernel:", canny_window_name, &canny_kernel_size, 2, updateCanny);

        cv::createTrackbar("Delta:", laplacian_window_name, &delta, 50, updateLaplacian);
        cv::createTrackbar("Scale:", laplacian_window_name, &scale, 50, updateLaplacian);
        cv::createTrackbar("Kernel Size:", laplacian_window_name, &laplacian_kernel_size, 4, updateLaplacian);

        cv::createTrackbar("Line Threshold:", line_window_name, &lineThreshold, 100, updateLineDetection);
        cv::createTrackbar("Minimum Line Lenth:", line_window_name, &minLineLength, 60, updateLineDetection);
        cv::createTrackbar("Max Line Gap:", line_window_name, &maxLineGap, 10, updateLineDetection);
        cv::createTrackbar("Rho", line_window_name, &rho, 100, updateLineDetection);
        cv::createTrackbar("Theta", line_window_name, &theta_scale, 300, updateLineDetection);

        cv::createTrackbar("Blur Radius", process_window_name, &blurRadius, 500, updatePreprocess);

        
        cv::waitKey(0);
    }
    
}

/**
 * Main method. Gets called when the node is started
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "OpenCVStairClassifierNode");
    ros::NodeHandle ns;
    ros::NodeHandle np;

    ns.param("debug", debug, debug);

    ros::Subscriber zed_pose_subscriber = ns.subscribe("/octomap_to_gridmap_demo/grid_map", 1, gridMapCallback);

    ros::Rate loop_rate(20);
    ros::spinOnce();
    
    // Main loop
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    ROS_INFO("Fin suscriptor");
    return 0;
}
