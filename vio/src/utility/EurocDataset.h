#pragma once

#include <mutex>
#include <deque>
#include <thread>
#include <cstring>
#include <eigen3/Eigen/Core>

struct CameraCsv {    
    struct CamData {
        double t;
        std::string filename;
    };

    std::vector<CamData> items;

    void load(const std::string &filename) {
        items.clear();
        if (FILE *csv = fopen(filename.c_str(), "r")) {
            char header_line[2048];
            int ret = fscanf(csv, "%2047[^\r]\r\n", header_line);
            if (ret <= 0) {
                return;
            }
            char filename_buffer[2048];
            CamData item;
            while (!feof(csv)) {
                memset(filename_buffer, 0, 2048);
                if (fscanf(csv, "%lf,%2047[^\r]\r\n", &item.t,
                           filename_buffer) != 2) {
                    break;
                }
                item.t *= 1e-9; // unit: seconds
                item.filename = std::string(filename_buffer);
                items.emplace_back(std::move(item));
            }
            fclose(csv);
        }
    }
};

struct ImuCsv {
    struct ImuData {
        double t;
        struct {
            double x;
            double y;
            double z;
        } w;
        struct {
            double x;
            double y;
            double z;
        } a;
    };

    std::vector<ImuData> items;

    void load(const std::string &filename) {
        items.clear();
        if (FILE *csv = fopen(filename.c_str(), "r")) {
            char header_line[2048];
            int ret = fscanf(csv, "%2047[^\r]\r\n", header_line);
            if (ret <= 0) {
                return;
            }
            ImuData item;
            while (!feof(csv) &&
                   fscanf(csv, "%lf,%lf,%lf,%lf,%lf,%lf,%lf\r\n", &item.t,
                          &item.w.x, &item.w.y, &item.w.z, &item.a.x, &item.a.y,
                          &item.a.z) == 7) {
                item.t *= 1e-9;
                items.emplace_back(std::move(item));
            }
            fclose(csv);
        }
    }
};