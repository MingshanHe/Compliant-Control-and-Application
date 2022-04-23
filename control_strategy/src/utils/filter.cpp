#include "utils/filter.h"

void Filter::init(const ros::NodeHandle &nh)
{
    std::vector<double> Filter_K_;
    std::vector<double> Filter_K_d_;
    std::vector<double> LocalThres_;
    std::vector<double> GlobalThres_;
    std::vector<double> AddNum_;
    if (!nh.getParam("Filter_K", Filter_K_)) { ROS_ERROR("Couldn't retrieve the Filter K.");}
    if (!nh.getParam("Filter_K_d", Filter_K_d_)) { ROS_ERROR("Couldn't retrieve the Filter K_d.");}
    if (!nh.getParam("LocalThres", LocalThres_)) { ROS_ERROR("Couldn't retrieve the Local Threshold.");}
    if (!nh.getParam("GlobalThres", GlobalThres_)) { ROS_ERROR("Couldn't retrieve the Global Threshold.");}
    if (!nh.getParam("AddNum", AddNum_)) { ROS_ERROR("Couldn't retrieve the Add Number.");}
    Filter_K = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(Filter_K_.data(), Filter_K_.size());
    Filter_K_d = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(Filter_K_d_.data(), Filter_K_d_.size());
    LocalThres = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(LocalThres_.data(), LocalThres_.size());
    GlobalThres = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(GlobalThres_.data(), GlobalThres_.size());
    AddNum = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(AddNum_.data(), AddNum_.size());
    // Filter_K = Filter_K_.data();
    // Filter_K_d = Filter_K_d_.data();
    // LocalThres = LocalThres_.data();
    // GlobalThres = GlobalThres_.data();
    // AddNum = AddNum_.data();

    OldData << 0.0, 0.0, 0.0;
    OldFlag << 0.0, 0.0, 0.0;
    NewFlag << 0.0, 0.0, 0.0;
    AddSum  << 0.0, 0.0, 0.0;
}

Eigen::Vector3d Filter::LowPassFilter(const Eigen::Vector3d &data)
{
    // for (size_t i = 0; i < data.size(); i++)
    // {
    //     NewData(i) = data(i);

    //     if((NewData(i) - OldData(i)) > 0)
    //     {
    //         NewFlag(i) = 1;
    //     }
    //     else
    //     {
    //         NewFlag(i) = 0;
    //     }

    //     if (NewFlag(i) == OldFlag(i))
    //     {
    //         if (fabs(NewData(i) - OldData(i)) > LocalThres(i))
    //         {
    //             AddSum(i) += AddNum(i);
    //         }
    //         if (AddSum(i) > GlobalThres(i))
    //         {
    //             Filter_K(i) += Filter_K_d(i);
    //         }
    //     }
    //     else
    //     {
    //         AddSum(i) = 0;
    //         Filter_K(i) = Filter_K_d(i);
    //         OldFlag(i) = NewFlag(i);
    //     }

    //     if (Filter_K(i) > 0.95)
    //     {
    //         Filter_K(i) = 0.95;
    //     }
    //     NewData(i) = ( 1 - Filter_K(i)) * OldData(i) + Filter_K(i) * NewData(i);
    //     OldData(i) = NewData(i);
    // }
    NewData = ( 1 - 0.2) * OldData + 0.2 * NewData;
    OldData = NewData;
    return NewData;
}