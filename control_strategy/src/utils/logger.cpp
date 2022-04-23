#include "utils/logger.h"

Logger::Logger(/* args */)
{
    packagePath = "/home/robot/explore_pushing_and_grasping/src/Exploring_Grasping/control_strategy";
    now_time = time(0);
    struct tm* stime=localtime(&now_time);
    char tmp[32]{0};
    snprintf(tmp,sizeof(tmp),"%04d-%02d-%02d--%02d:%02d:%02d",1900+stime->tm_year,1+stime->tm_mon,stime->tm_mday, stime->tm_hour,stime->tm_min,stime->tm_sec);
    folderPath = tmp;

    command = "mkdir -p " + packagePath + "/logs/" + folderPath;
    system(command.c_str());

    command = "mkdir -p " + packagePath + "/logs/" + folderPath + "/images";
    system(command.c_str());

    command = "mkdir -p " + packagePath + "/logs/" + folderPath + "/wrench";
    system(command.c_str());

    wrenchFile.open(packagePath + "/logs/" + folderPath + "/wrench/data.csv");
}

Logger::~Logger()
{
    wrenchFile.close();
}

void Logger::save_image(cv::Mat image, int cognition)
{
    std::string filePath;
    switch (cognition)
    {
        case 0:
            filePath = packagePath + "/logs/" + folderPath + "/images/Predict.png";
            cv::imwrite(filePath, image);
            break;

        default:
            break;
    }
}

void Logger::save_wrench(double force_x, double force_y, double force_z, double torque_x, double torque_y, double torque_z)
{
    wrenchFile << force_x <<","<< force_y <<","<< force_z <<","<< torque_x <<","<< torque_y <<","<< torque_z << std::endl;
}