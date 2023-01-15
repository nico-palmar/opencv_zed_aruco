 // ZED includes
#include <sl/Camera.hpp>

// OpenCV includes
#include <opencv2/opencv.hpp>
// OpenCV dep
#include <opencv2/cvconfig.h>

#include <opencv2/aruco.hpp>

// convert C4 to C3
#include "opencv2/imgproc.hpp"
// #include "opencv2/imgcodecs.hpp"
// #include "opencv2/highgui.hpp"

using namespace std;
using namespace sl;

// using co = cv::aruco;

// Mapping between MAT_TYPE and CV_TYPE -> used in slMat2cvMat
int getOCVtype(sl::MAT_TYPE type) {
    int cv_type = -1;
    switch (type) {
        case MAT_TYPE::F32_C1: cv_type = CV_32FC1; break;
        case MAT_TYPE::F32_C2: cv_type = CV_32FC2; break;
        case MAT_TYPE::F32_C3: cv_type = CV_32FC3; break;
        case MAT_TYPE::F32_C4: cv_type = CV_32FC4; break;
        case MAT_TYPE::U8_C1: cv_type = CV_8UC1; break;
        case MAT_TYPE::U8_C2: cv_type = CV_8UC2; break;
        case MAT_TYPE::U8_C3: cv_type = CV_8UC3; break;
        case MAT_TYPE::U8_C4: cv_type = CV_8UC4; break;
        default: break;
    }
    return cv_type;
}

// function to check extrinsic calibration parameters for the zed camera
void logCameraParams(sl::CameraParameters cam) {
    std::cout << "Fx: " << cam.fx << " | Fy: " << cam.fy << " | Cx: " << cam.cx << " | Cy: " << cam.cy << std::endl;
    for (const double &dist_param: cam.disto) {
        std::cout << "Param: " << dist_param << "| ";
    }
    std::cout << std::endl;
}

// 

/**
* Conversion function between sl::Mat and cv::Mat
**/
cv::Mat slMat2cvMat(Mat& input) {
    // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
    // cv::Mat and sl::Mat will share a single memory structure
    return cv::Mat(input.getHeight(), input.getWidth(), getOCVtype(input.getDataType()), input.getPtr<sl::uchar1>(MEM::CPU), input.getStepBytes(sl::MEM::CPU));
}


int main(int argc, char **argv) {
    // create zed camera
    Camera zed;      

    // Set configuration parameters
    InitParameters init_params;
    init_params.camera_resolution = RESOLUTION::HD1080;
    // init_params.depth_mode = DEPTH_MODE::ULTRA;
    init_params.camera_fps = 30;
    init_params.coordinate_units = UNIT::METER;

    // open camera
    auto camera_return = zed.open(init_params);
    if (camera_return != ERROR_CODE::SUCCESS) {
        printf("%s\n", toString(camera_return).c_str());
        zed.close();
        return 1;
    }
    // get camera resolution details
    Resolution zed_size = zed.getCameraInformation().camera_resolution;

    CalibrationParameters calibration_params = zed.getCameraInformation().camera_configuration.calibration_parameters;

    // uncomment below to see camera parameters
    logCameraParams(calibration_params.left_cam);
    logCameraParams(calibration_params.right_cam);
    

    // setup sl mat
    // TODO: investiage the different with an sl image being 3 vs 4 channels
    sl::Mat image(zed_size.width, zed_size.height, MAT_TYPE::U8_C4);
    cv::Mat image_open_cv = slMat2cvMat(image);
    cv::Mat image_ocv_c3;
    
    // setup aruco stuff
    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners, rejection_candidates;
    // TODO: play with these params
    cv::Ptr<cv::aruco::DetectorParameters> params = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);


    while (true) {
        // get new img
        camera_return = zed.grab();

        if (camera_return != ERROR_CODE::SUCCESS) {
        printf("%s\n", toString(camera_return).c_str());
        zed.close();
        return 1;
        } else {
            // TODO: change to a better view, just left view for now
            zed.retrieveImage(image, VIEW::LEFT);
            // TODO: move the line below to another spot potentially
            cv::cvtColor(image_open_cv, image_ocv_c3, cv::COLOR_BGRA2BGR);
            // cout<<"Image resolution: "<< image.getWidth()<<"x"<<image.getHeight() <<" || Image timestamp: "<<image.timestamp.data_ns<<endl;

        
            cv::aruco::detectMarkers(image_ocv_c3, dictionary, marker_corners, marker_ids, params, rejection_candidates);

            // identify aruco czed.getCameraInformation().camera_configuration.calibration_parametersodes
            if (marker_corners.size() > 0) {
                // show detected markers
                cv::aruco::drawDetectedMarkers(image_ocv_c3, marker_corners, marker_ids);
            } else {
                // std::cout << "No ARUCO code detected" << std::endl;
            }
        }

        // cv::imshow("Image window", image_ocv_c3);

        // handle key event -> make cv2 show a new image on the window every 5ms
        cv::waitKey(5);
    }
    zed.close();
    return 1;
}