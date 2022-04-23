#include "control_strategy/control_strategy.h"

Control_Strategy::Control_Strategy(
    const ros::NodeHandle &nh_,
    std::vector<double>     workspace_limits_,
    std::vector<double>     home_pose_,
    std::vector<double>     work_start_pose_,
    std::vector<double>     grasp_pose_,
    std::vector<double>     predict_map_size_) :
    nh(nh_), workspace_limits(workspace_limits_.data()),
    home_pose(home_pose_.data()),work_start_pose(work_start_pose_.data()),grasp_pose(grasp_pose_.data()),
    predict_map_size(predict_map_size_.data())
{
    force_x = 0.0;
    force_y = 0.0;
    force_x_pre = 0.0;
    force_y_pre = 0.0;
    // ROS Service
    switch_controller_client = nh.serviceClient<controller_manager_msgs::SwitchController>(SwitchController_Topic);
    // ROS Pub&Sub
    Cartesian_Pose_Pub  = nh.advertise<geometry_msgs::Pose>(CartesianPose_Topic, 1, true);
    Cartesian_Twist_Pub = nh.advertise<geometry_msgs::Twist>(CartesianTwist_Topic, 1, true);
    Predict_IMG_Pub     = nh.advertise<std_msgs::Float64MultiArray>(Predict_IMG_Topic, 1, true);
    Gripper_Pub         = nh.advertise<control_msgs::GripperCommandActionGoal>("/gripper_controller/gripper_cmd/goal", 1, true);
    Cartesian_State_Sub = nh.subscribe(CartesianState_Topic, 5, &Control_Strategy::Cartesian_State_Cb, this, ros::TransportHints().reliable().tcpNoDelay());

    Predict_IMG = cv::Mat(predict_map_size(0),predict_map_size(1),CV_8UC1);
    uchar* data = (uchar*)Predict_IMG.data;
    for ( int i=0; i<predict_map_size(0); i++ )
    {
        for ( int j=0; j<predict_map_size(1); j++ )
        {
            int index = i*predict_map_size(0) + j;
            data[index] = 0;
        }
    }
    data[50] = 0;
    logger.save_image(Predict_IMG, 0);
}

void Control_Strategy::Switch_Controller(const int &cognition)
{
    std::string cartesian_position_controller("cartesian_position_controller");
    std::string cartesian_velocity_controller("cartesian_velocity_controller");
    switch (cognition)
    {
    case 0:
        start_controllers.clear();
        stop_controllers.clear();

        start_controllers.push_back(cartesian_position_controller);
        switch_controller_srv.request.start_controllers = start_controllers;
        switch_controller_srv.request.stop_controllers = stop_controllers;
        switch_controller_srv.request.strictness = 2;
        if(switch_controller_client.call(switch_controller_srv))
        {
            ROS_INFO("Switch 'cartesian_position_controller' Successfully.");
        }
        else
        {
            ROS_ERROR("Switch 'cartesian_position_controller' Failed. Please Check Code");
        }
        break;
    case 1:
        start_controllers.clear();
        stop_controllers.clear();

        start_controllers.push_back(cartesian_velocity_controller);
        switch_controller_srv.request.start_controllers = start_controllers;
        switch_controller_srv.request.stop_controllers = stop_controllers;
        switch_controller_srv.request.strictness = 2;
        if(switch_controller_client.call(switch_controller_srv))
        {
            ROS_INFO("Switch 'cartesian_velocity_controller' Successfully.");
        }
        else
        {
            ROS_ERROR("Switch 'cartesian_velocity_controller' Failed. Please Check Code");
        }
        break;
    case 2:
        start_controllers.clear();
        stop_controllers.clear();

        start_controllers.push_back(cartesian_position_controller);
        stop_controllers.push_back(cartesian_velocity_controller);
        switch_controller_srv.request.start_controllers = start_controllers;
        switch_controller_srv.request.stop_controllers = stop_controllers;
        switch_controller_srv.request.strictness = 2;
        if(switch_controller_client.call(switch_controller_srv))
        {
            ROS_INFO("Switch 'cartesian_position_controller' Successfully.");
        }
        else
        {
            ROS_ERROR("Switch 'cartesian_position_controller' Failed. Please Check Code");
        }
        break;
    case 3:
        start_controllers.clear();
        stop_controllers.clear();

        start_controllers.push_back(cartesian_velocity_controller);
        stop_controllers.push_back(cartesian_position_controller);
        switch_controller_srv.request.start_controllers = start_controllers;
        switch_controller_srv.request.stop_controllers = stop_controllers;
        switch_controller_srv.request.strictness = 2;
        if(switch_controller_client.call(switch_controller_srv))
        {
            ROS_INFO("Switch 'cartesian_velocity_controller' Successfully.");
        }
        else
        {
            ROS_ERROR("Switch 'cartesian_velocity_controller' Failed. Please Check Code");
        }
        break;
    default:
        ROS_ERROR("Switch Controller Cognition Failed. Please Check Code and Choose ( 0 or 1 ).");
        break;
    }
}
void Control_Strategy::Go_Home(void)
{
    ros::Rate loop_rate(10);
    geometry_msgs::Pose msg;
    msg.position.x = home_pose(0);
    msg.position.y = home_pose(1);
    msg.position.z = home_pose(2);
    msg.orientation.x = home_pose(3);
    msg.orientation.y = home_pose(4);
    msg.orientation.z = home_pose(5);
    msg.orientation.w = home_pose(6);
    size_t i = 3;
    while (i>0)
    {
        Cartesian_Pose_Pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        i--;
    }

}
void Control_Strategy::Go_Work(void)
{
    ros::Rate loop_rate(10);
    geometry_msgs::Pose msg;
    msg.position.x = work_start_pose(0);
    msg.position.y = work_start_pose(1);
    msg.position.z = work_start_pose(2);
    msg.orientation.x = work_start_pose(3);
    msg.orientation.y = work_start_pose(4);
    msg.orientation.z = work_start_pose(5);
    msg.orientation.w = work_start_pose(6);
    size_t i = 3;
    while (i>0)
    {
        Cartesian_Pose_Pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        i--;
    }
}
void Control_Strategy::Go(Eigen::Vector3d Position)
{
    ros::Rate loop_rate(10);
    geometry_msgs::Pose msg;

    msg.position.x = Position(0);
    msg.position.y = Position(1);
    msg.position.z = Position(2);
    msg.orientation.x = home_pose(3);
    msg.orientation.y = home_pose(4);
    msg.orientation.z = home_pose(5);
    msg.orientation.w = home_pose(6);
    size_t i = 3;
    while (i>0)
    {
        Cartesian_Pose_Pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        i--;
    }
}
void Control_Strategy::Grasp(void)
{
    ros::Rate loop_rate(10);
    geometry_msgs::Pose msg;
    msg.position.x = grasp_pose(0);
    msg.position.y = grasp_pose(1);
    msg.position.z = grasp_pose(2);
    msg.orientation.x = grasp_pose(3);
    msg.orientation.y = grasp_pose(4);
    msg.orientation.z = grasp_pose(5);
    msg.orientation.w = grasp_pose(6);
    size_t i = 3;
    while (i>0)
    {
        Cartesian_Pose_Pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        i--;
    }
    sleep(2);
    control_msgs::GripperCommandActionGoal gripper_msg;
    gripper_msg.goal.command.position = 0.365;
    gripper_msg.goal.command.max_effort = -5.0;
    i = 3;
    while (i>0)
    {
        Gripper_Pub.publish(gripper_msg);
        ros::spinOnce();
        loop_rate.sleep();
        i--;
    }
    sleep(2);

    msg.position.x = grasp_pose(0);
    msg.position.y = grasp_pose(1);
    msg.position.z = grasp_pose(2)+0.1;
    msg.orientation.x = grasp_pose(3);
    msg.orientation.y = grasp_pose(4);
    msg.orientation.z = grasp_pose(5);
    msg.orientation.w = grasp_pose(6);
    i = 3;
    while (i>0)
    {
        Cartesian_Pose_Pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        i--;
    }

}
void Control_Strategy::Cartesian_State_Cb(const cartesian_state_msgs::PoseTwistConstPtr &msg)
{
    Cartesian_State <<  msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
                        msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w;
}