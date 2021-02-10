#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
//#include <jackal_3d_mapping/DBSCAN.hpp>
bool debug = true;

ros::Publisher pub;
int lowThreshold = 7;
int highThreshold = 349;
const int max_lowThreshold = 100;
int canny_kernel_size = 1;
cv::Mat src_scaled, dst, zoomedImage, cdst, processed, erosion_dst, dilation_dst, stairs_dst;

cv::Rect roi;
int lineThreshold = 8;
int minLineLength = 21;
int maxLineGap = 5;
int rho = 1;
int theta_scale = 100;

int scale = 1;
int delta = 0;
int laplacian_kernel_size = 0;

int blurRadius = 18;

const char *src_window_name = "Original Image";
const char *process_window_name = "Processed Image";
const char *canny_window_name = "Canny";
const char *laplacian_window_name = "Laplacian";
const char *line_window_name = "Hough Line Transform";

int erosion_elem = 0;
int erosion_size = 0;
int dilation_elem = 0;
int dilation_size = 1;
int const max_elem = 2;
int const max_kernel_size = 21;

//DBSCAN dbscan;

void Erosion(int, void *);
void Dilation(int, void *);

void updateCanny(int, void *);
void updateLaplacian(int, void *);
void updatePreprocess(int, void *);
static void updateLineDetection(int, void *);
void gridMapCallback(grid_map_msgs::GridMap msg);

void updateCanny(int, void *)
{
    //cv::morphologyEx(cv::MORPH)
    Canny(processed, dst, lowThreshold, highThreshold, 3 + canny_kernel_size * 2);
    if (debug)
    {
        // imshow(canny_window_name, dst);
        imshow(canny_window_name, dst(roi));
        updateLineDetection(0, 0);
    }
}
void updateLaplacian(int, void *)
{
    int ddepth = CV_8U;

    Laplacian(processed, dst, ddepth, 3 + laplacian_kernel_size * 2, scale, delta, cv::BORDER_DEFAULT);
    //bitwise_not ( dst, dst );

    if (debug)
    {
        // imshow(laplacian_window_name, dst);
        imshow(laplacian_window_name, dst(roi));
        updateLineDetection(0, 0);
    }
}

void updatePreprocess(int, void *)
{
    blurRadius = blurRadius - (blurRadius % 2) + 1;
    cv::GaussianBlur(src_scaled, processed, cv::Size(blurRadius, blurRadius), 0.0, 0.0);

    if (debug)
    {
        imshow(process_window_name, processed(roi));

        updateLaplacian(0, 0);
        updateCanny(0, 0);
        //updateLineDetection(0, 0);
    }
}
void Erosion(int, void *)
{
    int erosion_type = 0;
    if (erosion_elem == 0)
    {
        erosion_type = cv::MORPH_RECT;
    }
    else if (erosion_elem == 1)
    {
        erosion_type = cv::MORPH_CROSS;
    }
    else if (erosion_elem == 2)
    {
        erosion_type = cv::MORPH_ELLIPSE;
    }
    cv::Mat element = cv::getStructuringElement(erosion_type,
                                                cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
                                                cv::Point(erosion_size, erosion_size));
    erode(dilation_dst, erosion_dst, element);
    if (debug)
    {
        imshow("Erosion Demo", erosion_dst);
    }
}
void Dilation(int, void *)
{
    int dilation_type = 0;
    if (dilation_elem == 0)
    {
        dilation_type = cv::MORPH_RECT;
    }
    else if (dilation_elem == 1)
    {
        dilation_type = cv::MORPH_CROSS;
    }
    else if (dilation_elem == 2)
    {
        dilation_type = cv::MORPH_ELLIPSE;
    }
    cv::Mat element = cv::getStructuringElement(dilation_type,
                                                cv::Size(2 * dilation_size + 1, 2 * dilation_size + 1),
                                                cv::Point(dilation_size, dilation_size));
    dilate(dst, dilation_dst, element);
    if (debug)
    {
        imshow("Dilation Demo", dilation_dst);
    }
}

static void updateLineDetection(int, void *)
{
    //cv::GaussianBlur(dst, dst, cv::Size(blurRadius, blurRadius), 0.0, 0.0);

    cv::Mat cdstP;
    cvtColor(dst, cdst, cv::COLOR_GRAY2BGR);
    cdstP = cdst.clone();

    // Probabilistic Line Transform
    std::vector<cv::Vec4i> linesP;                                                                            // will hold the results of the detection
    HoughLinesP(dst, linesP, rho, theta_scale / 100 * CV_PI / 180, lineThreshold, minLineLength, maxLineGap); // runs the actual detection
    // HoughLinesP(dst, linesP, 1, CV_PI / 180, 50, 50, 10); // runs the actual detection
    // Draw the lines
    stairs_dst = cv::Mat(src_scaled.size(), src_scaled.type(), cv::Scalar(255, 255, 255));
    for (size_t i = 0; i < linesP.size(); i++)
    {
        cv::Vec4i l = linesP[i];
        line(stairs_dst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 0, 0), 0.5, cv::LINE_AA);
        line(cdstP, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 0, 255), 0.5, cv::LINE_AA);

        // double angle = atan2(l[0] - l[1], l[2] - l[3]) * 180.0 / CV_PI;
        // int midx=(l[0]+l[2])/2;
        // int midy=(l[1]+l[3])/2;

        // cv::putText(stairs_dst, std::to_string(angle), cv::Point2f(midx, midy), cv::FONT_HERSHEY_PLAIN, 0.8, cv::Scalar(0, 0, 255, 255));
    }
    //dbscan=DBSCAN();
    //dbscan.run(linesP);
    if (debug)
    {
        imshow(line_window_name, cdstP(roi));
        imshow("Extracted Lines", stairs_dst(roi));
    }
    ROS_INFO_STREAM("Lines extreacted: " << linesP.size());
}

void lineClustering(std::vector<cv::Vec4i> lines)
{
    for (size_t i = 0; i < lines.size(); i++)
    {
        cv::Vec4i line = lines[i];
    }
}

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
    //NOCH ANPASSEN!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    grid_map::GridMapCvConverter::toImage<unsigned char, 1>(map, "elevation", CV_8UC1, -0.075, 1.995, src);
    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    double resizeFactor = 3.0;
    cv::resize(src, src_scaled, cv::Size(), resizeFactor, resizeFactor);

    if (debug)
    {
        cv::namedWindow(src_window_name, cv::WINDOW_NORMAL);
        cv::namedWindow(process_window_name, cv::WINDOW_NORMAL);
        cv::namedWindow(canny_window_name, cv::WINDOW_NORMAL);
        cv::namedWindow(laplacian_window_name, cv::WINDOW_NORMAL);
        cv::namedWindow(line_window_name, cv::WINDOW_NORMAL);
        cv::namedWindow("Erosion Demo", cv::WINDOW_NORMAL);
        cv::namedWindow("Dilation Demo", cv::WINDOW_NORMAL);
        int width = 920;
        int height = 980;
        cv::resizeWindow(src_window_name, width, height);
        cv::resizeWindow(process_window_name, width, height);
        cv::resizeWindow(canny_window_name, width, height);
        cv::resizeWindow(laplacian_window_name, width, height);
        cv::resizeWindow(line_window_name, width, height);
        cv::moveWindow(line_window_name, width + 50, 0);
    }
    roi.width = src_scaled.size().width / 3;
    roi.height = src_scaled.size().height / 3;

    roi.x = 0;
    roi.y = src_scaled.size().height - roi.height;

    // zoomedImage = src_scaled(roi);
    // cv::imshow(src_scaled_window_name, zoomedImage);

    if (src_scaled.empty())
    {
        ROS_INFO_STREAM("SRC Empty");
    }
    ROS_INFO_STREAM(src_scaled.depth());
    // Edge detection
    //Canny(src, dst, lowThreshold, highThreshold, 3 + kernel_size * 2);
    // Copy edges to the images that will display the results in BGR
    if (debug)
    {
        cv::imshow(src_window_name, src_scaled(roi));
    }
    updatePreprocess(0, 0);
    //Dilation(0, 0);
    // updateLaplacian(0, 0);
    updateCanny(0, 0);

    updateLineDetection(0, 0);

    Dilation(0, 0);
    Erosion(0, 0);
    ROS_INFO_STREAM("Image displayed");

    // Show results
    // imshow(canny_window_name, dst);
    // imshow("Standard Hough Line Transform", cdst);
    // imshow(line_window_name, cdstP);
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

        cv::createTrackbar("Element:\n 0: Rect \n 1: Cross \n 2: Ellipse", "Erosion Demo",
                           &erosion_elem, max_elem,
                           Erosion);
        cv::createTrackbar("Kernel size:\n 2n +1", "Erosion Demo",
                           &erosion_size, max_kernel_size,
                           Erosion);
        cv::createTrackbar("Element:\n 0: Rect \n 1: Cross \n 2: Ellipse", "Dilation Demo",
                           &dilation_elem, max_elem,
                           Dilation);
        cv::createTrackbar("Kernel size:\n 2n +1", "Dilation Demo",
                           &dilation_size, max_kernel_size,
                           Dilation);

        // imshow("Source", src(roi));
        // imshow("Detected Lines (in red) - Standard Hough Line Transform", cdst(roi));
        // imshow(line_window_name, cdstP(roi));
        cv::waitKey(0);
    }
    cv::Mat stairs_dst_resize;
    cv::resize(dilation_dst, stairs_dst_resize, src.size());

    grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 1>(stairs_dst_resize, "stairs", map, 0.0, 1.0);

    // Publish grid map.
    grid_map_msgs::GridMap message;
    grid_map::GridMapRosConverter::toMessage(map, message);
    pub.publish(message);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "OpenCVStairClassifierNode");
    ros::NodeHandle ns;
    ros::NodeHandle np;

    ns.param("debug", debug, debug);

    ros::Subscriber zed_pose_subscriptor = ns.subscribe("/octomap_to_gridmap_demo/grid_map", 1, gridMapCallback);
    pub = ns.advertise<grid_map_msgs::GridMap>("stairs_grid_map", 1, true);

    ros::Rate loop_rate(20);
    ros::spinOnce();
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    ROS_INFO("Fin suscriptor");
    return 0;
}
