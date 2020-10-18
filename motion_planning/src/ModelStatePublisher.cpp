#include <ros/ros.h>
#include <gazebo_msgs/ModelState.h>

ros::Subscriber stateSub;
ros::Publisher statePub;


void State_Callback(gazebo_msgs::ModelState msg){
    statePub.publish(msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ModelStatePublisher");
    ros::NodeHandle nh;

    stateSub = nh.subscribe("/Clients/model_state", 100, State_Callback);
    statePub = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 100);
    ros::spin();
    return 0;
}