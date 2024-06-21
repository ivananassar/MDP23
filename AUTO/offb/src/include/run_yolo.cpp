#include <stdio.h>
#include "run_yolo.h"
#include <math.h>
#include <ros/ros.h>

using namespace std;
run_yolo::run_yolo(const cv::String model_path) {
        this->mydnn = cv::dnn::readNetFromTensorflow(model_path);
        this->mydnn.setPreferableBackend(cv::dnn::DNN_BACKEND_DEFAULT);
        this->mydnn.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
        this->model_path=model_path;
    }
run_yolo::~run_yolo() {}

cv::Mat run_yolo::segment(const cv::Mat& input_image){
    // Preprocess input image if needed

    // Set input blob for the network
    cv::Mat blob = cv::dnn::blobFromImage(input_image, 1.0 / 255.0, cv::Size(256, 256), cv::Scalar(), true, false);
    mydnn.setInput(blob);

    // Forward pass through the network
    cv::Mat segmented_mask = mydnn.forward();

    // Postprocess segmented mask if needed

    return segmented_mask;
}
   


void run_yolo::segment_image(const cv::Mat& input_image) {
    //obj_vector.clear();
    //auto total_start = std::chrono::steady_clock::now();

    // Perform segmentation using the loaded UNet model
    cv::Mat segmented_mask = segment(input_image);
    if (!segmented_mask.empty()) {
    // Find contours to get bounding boxes
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(segmented_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // Calculate bounding boxes from contours and store them in obj_vector
    
    
    for (const auto& contour : contours) {
        cv::Rect boundingbox = cv::boundingRect(contour);
        double confidence = calculateConfidence(segmented_mask, boundingbox);
        // Convert bounding box information to YOLO format
        int final_x = boundingbox.x;
        int final_y = boundingbox.y;
        int final_w = boundingbox.width;
        int final_h = boundingbox.height;
        cv::Point center = cv::Point(final_x + final_w / 2, final_y + final_h / 2);
        int depthbox_w = final_w * 0.25;
        int depthbox_h = final_h * 0.25;
        cv::Point depthbox_vertice1 = cv::Point(center.x - depthbox_w / 2, center.y - depthbox_h / 2);
        cv::Point depthbox_vertice2 = cv::Point(center.x + depthbox_w / 2, center.y + depthbox_h / 2);
        cv::Rect letsgetdepth(depthbox_vertice1, depthbox_vertice2);

        cv::Mat ROI(depthdata, letsgetdepth);
        cv::Mat ROIframe;
        ROI.copyTo(ROIframe);
        std::vector<cv::Point> nonzeros;

        cv::findNonZero(ROIframe, nonzeros);
        std::vector<double> nonzerosvalue;
        for (auto temp : nonzeros) {
            double depth = ROIframe.at<ushort>(temp);
            nonzerosvalue.push_back(depth);
        }

        double depth_average=0.0;
        if (nonzerosvalue.size() != 0)
            depth_average = accumulate(nonzerosvalue.begin(), nonzerosvalue.end(), 0.0) / nonzerosvalue.size();

        double depthofboundingbox = 0.001 * depth_average;

        // Add bounding box information to obj_vector in YOLO format
        std::string detectedclass = "solar"; // Assuming the class name is fixed
        float detectedconfidence = confidence * 100; // Assuming confidence is scaled to [0, 100]

        // Store bounding box information in ObjectDetection struct
        objectinfo obj;
        obj.confidence = detectedconfidence;
        obj.classnameofdetection = detectedclass;
        obj.boundingbox = boundingbox;
        obj.depth = depthofboundingbox;
        obj.frame = input_image;
        obj_vector.push_back(obj);
    }

    // Store bounding boxes in the vector passed by reference
    //bounding_boxes.insert(bounding_boxes.end(), obj_vector.begin(), obj_vector.end());
}
    // End timing
    //auto total_end = std::chrono::steady_clock::now();
    //total_fps = 1000.0 / std::chrono::duration_cast<std::chrono::milliseconds>(total_end - total_start).count();
    //appro_fps = total_fps;
}


double run_yolo::calculateConfidence(const cv::Mat& segmented_mask, const cv::Rect& bbox) {
    // Calculate confidence (for UNet, it could be the proportion of non-zero pixels in the contour)
    cv::Mat roi = segmented_mask(bbox);
    double confidence = cv::countNonZero(roi) / static_cast<double>(bbox.area());
    return confidence;
}




 //   void run_unet::unet(cv::Mat& frame) {
 //       std::vector<cv::Rect> bounding_boxes;
//        segment_image(frame, bounding_boxes);

        // Process bounding boxes further if needed
//        for (const auto& bounding_box : bounding_boxes) {
            // Do something with each bounding box
            // For example, draw bounding boxes on the frame
//            cv::rectangle(frame, bounding_box, cv::Scalar(0, 255, 0), 2);
//        }
//    }

    void run_yolo::display(const cv::Mat& mask) {
        cv::imshow("Segmented Mask", mask);
        cv::waitKey(20);
    }

    // Method to set depth data if required (similar to getdepthdata)
    void run_yolo::getdepthdata(cv::Mat depthdata)
{
    this->depthdata = depthdata;
}
void run_yolo::unet(cv::Mat &frame)
{
    obj_vector.clear();
    this->total_start = std::chrono::steady_clock::now();
    std::vector<cv::Rect> bounding_boxes;
    segment_image(frame);
    this->total_end = std::chrono::steady_clock::now();
    total_fps = 1000.0 / std::chrono::duration_cast<std::chrono::milliseconds>(total_end - total_start).count();
    this->appro_fps = total_fps;
    total_end = std::chrono::steady_clock::now();
    double deltatime = std::chrono::duration_cast<std::chrono::milliseconds>(total_end - total_start).count() / 1000.0;
    //double total_fps = 1.0 / deltatime;

    // Print timing information
    cout << "Time: " << deltatime << " s" << endl;
    cout << "FPS: " << total_fps << endl;
}
