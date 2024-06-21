#include <iostream>
#include <fstream>
#include <istream>
#include <sstream>


#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/videoio.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>

#include <math.h>
#include <stdio.h>
#include <numeric>
#include <chrono>
#include <iomanip>
#include <string>
#include <cmath>
#include <eigen3/Eigen/Dense>

using namespace std;
struct objectinfo {
    float confidence=0.0;
    std::string classnameofdetection;
    cv::Rect boundingbox;
    double depth;
    cv::Mat frame;
    

};

class run_yolo {
private:
     
    cv::String model_path;
    cv::dnn::Net mydnn;
    float total_fps;
    chrono::time_point<chrono::steady_clock> total_start, total_end, dnn_start, dnn_end;
    objectinfo obj;
    cv::Mat depthdata;

    float set_confidence;
    vector<std::string> classnames;

    cv::Mat segment(const cv::Mat& input_image);
    double calculateConfidence(const cv::Mat& segmented_mask, const cv::Rect& bbox);
    //double calculateDepth(const cv::Rect& bbox); // You need to implement this function

public:
    run_yolo(const cv::String model_path);
    ~run_yolo();
    float appro_fps;
    void segment_image(const cv::Mat& input_image);
    void unet(cv::Mat& frame);
    vector<objectinfo> obj_vector;
    void display(const cv::Mat& mask);
    void getdepthdata(cv::Mat depth_data);
    
};


