#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/ModelState.h>

#define USAGE "USAGE\n\t"\
                "rosrun motion_planning ModelStateSeparator -h                          ==> help\n\t" \
                "rosrun motion_planning ModelStateSeparator                             ==> default model name (turtlebot3)\n\t" \
                "rosrun motion_planning ModelStateSeparator -m|--model <model_name>     ==> using specified model name"
                

ros::Subscriber stateSub;
ros::Publisher statePub;
std::string model_name;

int find_index(std::string model, ros::V_string& vect){
    for (int i = 0; i < vect.size(); i++)
    {
        if (vect[i] == model)
        {
            return i;
        }
    }
    return -1;
}

void Separator_Callback(gazebo_msgs::ModelStates states){
    int index = find_index(model_name, states.name);
    if (index == -1)
    {
        ROS_ERROR("Model name (%s) not found.", model_name.c_str());
        exit(-2);
    }
    
    gazebo_msgs::ModelState state;
    state.model_name = states.name[index];
    state.pose = states.pose[index];
    state.twist = states.twist[index];

    statePub.publish(state);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ModelStateSeparator");
    ros::NodeHandle nh;

    if (argc == 1)
    {
        model_name = "turtlebot3";
    }
    else if (argc == 2 && !std::strcmp(argv[1], "-h"))
    {
        puts(USAGE);
        return 0;
    }
    else if (argc == 3)
    {
        if (!std::strcmp(argv[1], "-m") || !std::strcmp(argv[1], "--model"))
        {
            model_name = argv[2];
        }
        else{
            puts(USAGE);
            return -1;
        }
    }
    else{
        puts(USAGE);
        return -1;
    }

    ROS_INFO("Model State Separator initialized successfully.");

    stateSub = nh.subscribe("/gazebo/model_states", 100, Separator_Callback);
    statePub = nh.advertise<gazebo_msgs::ModelState>("/Clients/model_state", 100);

    ROS_INFO("Subscribed to /gazebo/model_states for model name : %s", model_name.c_str());
    ROS_INFO("Advertised Successfully to /ModelState");

    ros::spin();

    return 0;
}