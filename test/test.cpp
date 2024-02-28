#include<iostream>
#include "estimator/parameters.h"

int main(int argc, char **argv) {
    std::string data_path   = "../dataset/EuRoC/MH_02_easy/mav0";
    std::string config_path = "../config/euroc_config.yaml";

    if(argc == 3) {
        data_path   = argv[1];
        config_path = argv[2];
    }

    std::cout << " 111" << std::endl;
    PRINT_DEBUG("dasfasdfasdfasdfasd\n");
    readParameters(config_path);


    return EXIT_SUCCESS;
}