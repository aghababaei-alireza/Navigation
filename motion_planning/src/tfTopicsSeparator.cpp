#include <ros/ros.h>
#include <tf2_msgs/TFMessage.h>
#include <vector>

ros::Subscriber tfSub;
std::vector<ros::Publisher> pubs;
std::string main_topic_name;


std::string find_topic(std::string frame_id){
    if (frame_id.find_first_of("/") == 0)
    {
        frame_id = frame_id.substr(1);
    }
    int index = frame_id.find_first_of("/");
    std::string topic = frame_id.substr(0, index);
    // ROS_INFO("Find Topic: %s", topic.c_str());
    return topic;
}

int find_publisher(std::string topic){
    for (int i = 0; i < pubs.size(); i++)
    {
        // ROS_INFO("pubs[%d].getTopic = %s \t topic = %s", i, pubs[i].getTopic().c_str(), topic.c_str());
        if (pubs[i].getTopic().compare(topic) == 0)
        {
            // ROS_INFO("Find publisher: %s", pubs[i].getTopic().c_str());
            return i;
        }
    }
    // ROS_INFO("~~Find Publisher");
    return -1;
}

void tfSeparator(tf2_msgs::TFMessage tfmsg){
    std::string frame_id = tfmsg.transforms[0].header.frame_id;
    std::string topic = "/" + find_topic(frame_id) + "/" + main_topic_name;
    int i = find_publisher(topic);
    ros::Publisher p;
    if (i == -1)
    {
        ros::NodeHandle nh;
        p = nh.advertise<tf2_msgs::TFMessage>(topic, 100);
        p.publish(tfmsg);
        pubs.push_back(p);
        ROS_INFO("New Topic Created: %s", topic.c_str());
    }
    else
    {
        p = pubs[i];
        p.publish(tfmsg);
    }
    
}

#define USAGE "USAGE:\n\trosrun motion_planning tfTopicsSeparator\n\t" \
    "rosrun motion_planning tfTopicsSeparator -t [topic_name]"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "tfTopicsSeparator", ros::init_options::AnonymousName);
    ros::NodeHandle nh;

    if (argc == 1)
    {
        main_topic_name = "tf";
    }
    else if (argc == 3 && !std::strcmp(argv[1], "-t"))
    {
        main_topic_name = argv[2];
    }
    else
    {
        puts(USAGE);
        return -1;
    }
    
    

    tfSub = nh.subscribe(main_topic_name, 100, tfSeparator);
    ROS_INFO("Subscribing to topic: %s", tfSub.getTopic().c_str());
    ros::spin();
    return 0;
}
