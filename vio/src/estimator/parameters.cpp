/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/
#include <filesystem> 
#include "estimator/parameters.h"

namespace fs = std::filesystem;

double INIT_DEPTH;
double MIN_PARALLAX;
double ACC_N, ACC_W;
double GYR_N, GYR_W;

std::vector<Eigen::Matrix3d> RIC;
std::vector<Eigen::Vector3d> TIC;

Eigen::Vector3d G{0.0, 0.0, 9.8};

double BIAS_ACC_THRESHOLD;
double BIAS_GYR_THRESHOLD;
double SOLVER_TIME;
int NUM_ITERATIONS;
int ESTIMATE_EXTRINSIC;
int ESTIMATE_TD;
int ROLLING_SHUTTER;
std::string EX_CALIB_RESULT_PATH;
std::string OUTPUT_FOLDER;

std::string IMU_TOPIC;
int ROW, COL;
double TD;
int NUM_OF_CAM;
int STEREO;
int USE_IMU;
int MULTIPLE_THREAD;
map<int, Eigen::Vector3d> pts_gt;
std::string IMAGE0_TOPIC, IMAGE1_TOPIC;
std::string FISHEYE_MASK;
std::vector<std::string> CAM_NAMES;
int MAX_CNT;
int MIN_DIST;
double F_THRESHOLD;
int SHOW_TRACK;
int FLOW_BACK;

std::string IMAGE_FOLDER;
std::string VINS_RESULT_PATH;
std::string IMAGE_BIN_FOLDER;
/**
 * ROS参数读取
*/
// template <typename T>
// T readParam(std::string name)
// {
//     T ans;
//     if (n.getParam(name, ans)) {
//         PRINT_DEBUG("Loaded %s\n", name);
//     }
//     else {
//         PRINT_DEBUG("Failed to load %s\n", name);
//         n.shutdown();
//     }
//     return ans;
// }

/**
 * YAML配置读取
*/
void readParameters(std::string config_file)
{
    FILE *fh = fopen(config_file.c_str(),"r");
    if(fh == NULL){          
        PRINT_ERROR("config_file dosen't exist; wrong config_file path\n");
        return;          
    }
    fclose(fh);

    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened()) {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    fsSettings["image0_topic"] >> IMAGE0_TOPIC;
    fsSettings["image1_topic"] >> IMAGE1_TOPIC;
    MAX_CNT = fsSettings["max_cnt"];
    MIN_DIST = fsSettings["min_dist"];
    F_THRESHOLD = fsSettings["F_threshold"];
    SHOW_TRACK = fsSettings["show_track"];
    FLOW_BACK = fsSettings["flow_back"];

    MULTIPLE_THREAD = fsSettings["multiple_thread"];

    USE_IMU = fsSettings["imu"];
    PRINT_INFO("USE_IMU: %d\n", USE_IMU);
    if(USE_IMU) {
        fsSettings["imu_topic"] >> IMU_TOPIC;
        PRINT_INFO("IMU_TOPIC: %s\n", IMU_TOPIC.c_str());
        ACC_N = fsSettings["acc_n"];
        ACC_W = fsSettings["acc_w"];
        GYR_N = fsSettings["gyr_n"];
        GYR_W = fsSettings["gyr_w"];
        G.z() = fsSettings["g_norm"];
    }

    SOLVER_TIME = fsSettings["max_solver_time"];
    NUM_ITERATIONS = fsSettings["max_num_iterations"];
    MIN_PARALLAX = fsSettings["keyframe_parallax"];
    MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;

    fsSettings["output_path"] >> OUTPUT_FOLDER;
    IMAGE_FOLDER = OUTPUT_FOLDER + "images/";
    try {
        if (!fs::exists(IMAGE_FOLDER)) { 
            fs::create_directory(IMAGE_FOLDER); 
            PRINT_DEBUG("create directory %s\n", IMAGE_FOLDER.c_str());
        } else {
            PRINT_DEBUG("%s directory existed\n", IMAGE_FOLDER.c_str());
        }
    } catch (const std::exception& e) {
        PRINT_ERROR("create directory error %s\n", e.what());
    }
    
    IMAGE_BIN_FOLDER = OUTPUT_FOLDER + "images.csv";
    VINS_RESULT_PATH = OUTPUT_FOLDER + "vio.txt";
    std::cout << "images result path " << VINS_RESULT_PATH << std::endl;

    ESTIMATE_EXTRINSIC = fsSettings["estimate_extrinsic"];
    if (ESTIMATE_EXTRINSIC == 2) {
        PRINT_WARNING("have no prior about extrinsic param, calibrate extrinsic param\n");
        RIC.push_back(Eigen::Matrix3d::Identity());
        TIC.push_back(Eigen::Vector3d::Zero());
        EX_CALIB_RESULT_PATH = OUTPUT_FOLDER + "/extrinsic_parameter.csv";
    }
    else 
    {
        if (ESTIMATE_EXTRINSIC == 1) {
            PRINT_WARNING("Optimize extrinsic param around initial guess!\n");
            EX_CALIB_RESULT_PATH = OUTPUT_FOLDER + "/extrinsic_parameter.csv";
        }
        if (ESTIMATE_EXTRINSIC == 0) {
            PRINT_WARNING("fix extrinsic param\n");
        }

        cv::Mat cv_T;
        fsSettings["body_T_cam0"] >> cv_T;
        Eigen::Matrix4d T;
        cv::cv2eigen(cv_T, T);
        RIC.push_back(T.block<3, 3>(0, 0));
        TIC.push_back(T.block<3, 1>(0, 3));
    } 
    
    NUM_OF_CAM = fsSettings["num_of_cam"];
    PRINT_INFO("camera number %d\n", NUM_OF_CAM);

    if(NUM_OF_CAM != 1 && NUM_OF_CAM != 2) {
        PRINT_INFO("num_of_cam should be 1 or 2\n");
        assert(0);
    }
    int pn = config_file.find_last_of('/');
    std::string configPath = config_file.substr(0, pn);
    
    std::string cam0Calib;
    fsSettings["cam0_calib"] >> cam0Calib;
    std::string cam0Path = configPath + "/" + cam0Calib;
    CAM_NAMES.push_back(cam0Path);

    if(NUM_OF_CAM == 2) {
        STEREO = 1;
        std::string cam1Calib;
        fsSettings["cam1_calib"] >> cam1Calib;
        std::string cam1Path = configPath + "/" + cam1Calib; 
        //PRINT_INFO("%s cam1 path\n", cam1Path.c_str() );
        CAM_NAMES.push_back(cam1Path);
        
        cv::Mat cv_T;
        fsSettings["body_T_cam1"] >> cv_T;
        Eigen::Matrix4d T;
        cv::cv2eigen(cv_T, T);
        RIC.push_back(T.block<3, 3>(0, 0));
        TIC.push_back(T.block<3, 1>(0, 3));
    }

    INIT_DEPTH = 5.0;
    BIAS_ACC_THRESHOLD = 0.1;
    BIAS_GYR_THRESHOLD = 0.1;

    TD = fsSettings["td"];
    ESTIMATE_TD = fsSettings["estimate_td"];
    if (ESTIMATE_TD) {
        PRINT_INFO("Unsynchronized sensors, online estimate time offset, initial td: %f\n", TD);
    } else {
        PRINT_INFO("Synchronized sensors, fix time offset: %f\n", TD);
    }
    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    PRINT_INFO("ROW: %d COL: %d\n", ROW, COL);

    if(!USE_IMU) {
        ESTIMATE_EXTRINSIC = 0;
        ESTIMATE_TD = 0;
        PRINT_INFO("no imu, fix extrinsic param; no time offset calibration\n");
    }
    PRINT_DEBUG("[Parameters] Read Params Finished\n");

    fsSettings.release();
}
