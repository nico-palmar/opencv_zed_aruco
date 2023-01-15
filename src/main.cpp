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

// marker length in meters for ARUCO code. Be sure to change it if the marker length changes
// TODO: change this into a configurable parameter
float ARUCO_MARKER_LEN = 18.4 / 100;


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

    // NOTE: the intrinsic calibration parameters were found to be the same on left and right camera after testing
    // uncomment below to see camera parameters
    // logCameraParams(calibration_params.left_cam);
    // logCameraParams(calibration_params.right_cam);

    // populate the distortion coefficient matrix; should be all 0's but this code is kept in case that changes
    cv::Mat distortionCoefficients = cv::Mat::zeros(1, 12, CV_32FC1);
    int param_num = 0;
    for (const double &disto_param: calibration_params.left_cam.disto) {
        distortionCoefficients.at<double>(0, param_num) = disto_param;
        param_num++; 
    }
    std::cout << distortionCoefficients << std::endl;

    // create the intrinstic camera calibration coefficient matrix to use for aruco pose estimation
    cv::Mat intrinsicCalibMatrix = (cv::Mat_<float>(3,3) << 
        calibration_params.left_cam.fx, 0, calibration_params.left_cam.cx, 
        0, calibration_params.left_cam.fy, calibration_params.left_cam.cy,
        0, 0, 1);

    // set the marker coordinate points (in a 2D plane, mark the distances where the marker points are located)
    cv::Mat objPoints(4, 1, CV_32FC3);
    objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-ARUCO_MARKER_LEN/2.f, ARUCO_MARKER_LEN/2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(ARUCO_MARKER_LEN/2.f, ARUCO_MARKER_LEN/2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(ARUCO_MARKER_LEN/2.f, -ARUCO_MARKER_LEN/2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-ARUCO_MARKER_LEN/2.f, -ARUCO_MARKER_LEN/2.f, 0);

    // vectors for ARUCO code poses. Assume max of 4 ARUCO codes at a time
    std::vector<cv::Vec3d> rvecs(4, 0.0), tvecs(4, 0.0);

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
            int num_markers_detected = marker_corners.size();
            // identify aruco codes
            if (num_markers_detected > 0) {
                // show detected markers
                cv::aruco::drawDetectedMarkers(image_ocv_c3, marker_corners, marker_ids);

                // Calculate pose for each marker
                for (int i = 0; i < num_markers_detected; i++) {
                    cv::solvePnP(objPoints, marker_corners.at(i), intrinsicCalibMatrix, distortionCoefficients, rvecs.at(i), tvecs.at(i));
                }
                // Draw axis for each marker
                for(int i = 0; i < marker_ids.size(); i++) {
                    // draw the detected poses with coordinate axis of 0.2m in length
                    cv::drawFrameAxes(image_ocv_c3, intrinsicCalibMatrix, distortionCoefficients, rvecs[i], tvecs[i], 0.2);
                }

            } else {
                // std::cout << "No ARUCO code detected" << std::endl;
            }
        }

        cv::imshow("Image window", image_ocv_c3);

        // handle key event -> make cv2 show a new image on the window every 5ms
        cv::waitKey(5);
    }
    zed.close();
    return 1;
}