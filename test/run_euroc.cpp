#include <unistd.h>
#include <thread>
#include <vector>
#include <string>
#include <csignal>
#include <iostream>

#include <opencv2/opencv.hpp>
#include "utility/Print.h"
#include "utility/EurocDataset.h"
#include "estimator/estimator.h"
#include "estimator/parameters.h"

struct ImageData {
  double timestamp = 0;
  std::string file;
};

std::mutex m_buf;
std::queue<ImageData> img0_buf;
std::queue<ImageData> img1_buf;

Estimator estimator;

void signal_callback_handler(int signum) { std::exit(signum); }

void publish_stereo(const std::string &data_path) {
  CameraCsv cam0_csv, cam1_csv;
  cam0_csv.load(data_path + "/cam0/data.csv");
  cam1_csv.load(data_path + "/cam1/data.csv");

  double cam0_t = 0, cam1_t = 0;
  std::string cam0_file, cam1_file;
  size_t i = 0;
  while (i < cam0_csv.items.size()) {
    i++;
    cam0_t = cam0_csv.items[i].t;
    cam1_t = cam1_csv.items[i].t;
    cam0_file = data_path + "/cam0/data/" + cam0_csv.items[i].filename;
    cam1_file = data_path + "/cam1/data/" + cam1_csv.items[i].filename;
    ImageData img0_msg{cam0_t, cam0_file};
    ImageData img1_msg{cam1_t, cam1_file};
    m_buf.lock();
    img0_buf.push(img0_msg);
    img1_buf.push(img1_msg);
    m_buf.unlock();
    // 图像发布频率 20Hz
    usleep(50000); // 50ms
  }
}


// void publish_stereo(const std::string &data_path) {
//   CameraCsv cam0_csv, cam1_csv;
//   cam0_csv.load(data_path + "/cam0/data.csv");
//   cam1_csv.load(data_path + "/cam1/data.csv");
//   unsigned int i = 0; 
//   unsigned int j = 0;
//   while (i < cam0_csv.items.size()) {
//     double cam0_t = cam0_csv.items[i].t;
//     double cam1_t = cam1_csv.items[j].t;
//     if (STEREO) {
//       cv::Mat image0, image1;
      
//       // 双目相机左右图像时差不得超过 0.003s
//       if(cam0_t < cam1_t - 0.003) {
//         PRINT_DEBUG("throw image 0 \n");
//         i++;
//         continue;
//       } else if(cam0_t > cam1_t + 0.003) {
//         PRINT_DEBUG("throw image 1\n");
//         j++;
//         continue;
//       } else {
//         i++;
//         j++;
//         // 提取缓存队列中最早一帧图像，并从队列中删除
//         std::string cam0_file = data_path + "/cam0/data/" + cam0_csv.items[i].filename;
//         std::string cam1_file = data_path + "/cam1/data/" + cam1_csv.items[j].filename;
//         image0 = cv::imread(cam0_file, cv::IMREAD_GRAYSCALE);
//         image1 = cv::imread(cam1_file, cv::IMREAD_GRAYSCALE);
//         // #ifdef VIO_DEBUG
//         //   cv::imshow("image", image0);
//         //   cv::waitKey(1);
//         // #endif
//       }
//       if (!image0.empty() && !image1.empty()) {
//         estimator.inputImage(cam0_t, image0, image1);
//       }
//     } else {
//       std::string cam0_file = data_path + "/cam0/data/" + cam0_csv.items[i].filename;
//       cv::Mat image = cv::imread(cam0_file, cv::IMREAD_GRAYSCALE);
//       if(!image.empty()) {
//         estimator.inputImage(cam0_t, image);
//       }
//     }
//     // cv::Mat img = cv::imread(cam0_file, cv::IMREAD_GRAYSCALE);
//     // cv::imshow("image", img);
//     // cv::waitKey(10);
//     // 图像发布频率 20Hz
//     usleep(50000); // 50ms
//   }
// }

void publish_imu(const std::string &data_path) {
  ImuCsv imu_csv;
  imu_csv.load(data_path + "/imu0/data.csv");
  for (auto &item : imu_csv.items) {
    double timestamp = item.t;
    Vector3d acc(item.a.x, item.a.y, item.a.z);
    Vector3d gyr(item.w.x, item.w.y, item.w.z);
    estimator.inputIMU(timestamp, acc, gyr);

    // PRINT_DEBUG("imu: %f, %f, %f, %f, %f, %f, %f\n", item.t, item.w.x, item.w.y, item.w.z, item.a.x, item.a.y, item.a.z);
    // IMU 发布频率 200Hz
    usleep(5000); // 5ms 
  }
}


// 从两个图像队列中取出最早的一帧，并从队列删除，双目要求两帧时差不得超过 0.003s
void sync_process() {
  while(1) {
    if (STEREO) {
      cv::Mat image0, image1;
      double time = 0;
      m_buf.lock();
      if (!img0_buf.empty() && !img1_buf.empty()) {
        double time0 = img0_buf.front().timestamp;
        double time1 = img1_buf.front().timestamp;
        // 双目相机左右图像时差不得超过 0.003s
        if(time0 < time1 - 0.003) {
          img0_buf.pop();
          PRINT_DEBUG("throw img0\n");
        } else if(time0 > time1 + 0.003) {
          img1_buf.pop();
          PRINT_DEBUG("throw img1\n");
        } else {
          // 提取缓存队列中最早一帧图像，并从队列中删除
          time = time0;
          image0 = cv::imread(img0_buf.front().file, cv::IMREAD_GRAYSCALE);
          image1 = cv::imread(img1_buf.front().file, cv::IMREAD_GRAYSCALE);
          img0_buf.pop();
          img1_buf.pop();
        }
      }
      m_buf.unlock();
      if (!image0.empty()) {
        estimator.inputImage(time, image0, image1);
      }
    } else {
      cv::Mat image0;
      double time = 0;
      m_buf.lock();
      if (!img0_buf.empty()) {
        time = img0_buf.front().timestamp;
        image0 = cv::imread(img0_buf.front().file, cv::IMREAD_GRAYSCALE);
        img0_buf.pop();
      }
      m_buf.unlock();
      if(!image0.empty()) {
        estimator.inputImage(time, image0);
      }
    }
    std::chrono::milliseconds dura(2);
    std::this_thread::sleep_for(dura);
  }
}

int main(int argc, char **argv) {
  std::string data_path   = "../dataset/EuRoC/MH_02_easy/mav0";
  std::string config_path = "../config/euroc/euroc_stereo_imu_config.yaml";

	if(argc == 3) {
    data_path   = argv[1];
    config_path = argv[2];
	}
  // 读取YAML配置参数
  readParameters(config_path);
  estimator.setParameter();

	std::thread thread_pub_imu(publish_imu, std::ref(data_path));
	std::thread thread_pub_stereo(publish_stereo, std::ref(data_path));
  std::thread thread_sync{sync_process};

  thread_pub_imu.join();
	thread_pub_stereo.join();
  thread_sync.join();

  signal(SIGINT, signal_callback_handler);
  return EXIT_SUCCESS;
}