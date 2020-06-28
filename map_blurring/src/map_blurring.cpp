#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

ros::Subscriber map_sub;
ros::Publisher map_pub;

bool check = false;

double robot_dim = 0.3;
std::string FromTopic("/map");
std::string ToTopic("/modified_map");
int kernel_dim = 5;

std::vector<std::vector<int>> data;
std::vector<std::vector<int>> data_blured;
double resolution;
double height, width;

//Calculate 2d gaussian distribution
std::vector< std::vector<double> > CalcKernel(int dim){
    std::vector<std::vector<double>> kernel;
    double A = 1.0, x0 = (dim + 1)/2 - 1, y0 = (dim + 1)/2 - 1, x1 = (dim - 1)/2, y1 = (dim - 1)/2;//TODO: Set the values
    kernel.resize(dim);
    for (int i = 0; i < dim; i++)
    {
        kernel[i].resize(dim);
    }
    for (int i = 0; i < dim; i++)
    {
        for (int j = 0; j < dim; j++)
        {
            kernel[i][j] = A * exp(-((pow(i-x0,2)/(2*pow(x1,2)))+(pow(j-y0,2)/(2*pow(y1,2)))));
        }
    }
    return kernel;
}


void map_Callback(nav_msgs::OccupancyGrid msg){
    check = true;
    resolution = msg.info.resolution;
    height = msg.info.height;
    width = msg.info.width;

    nav_msgs::OccupancyGrid map;

    data.resize(height);
    data_blured.resize(height);
    for (int j = 0; j < height; j++)
    {
        data[j].resize(width);
        data_blured[j].resize(width);
    }

    for (int j = 0; j < height; j++)
    {
        for (int i = 0; i < width; i++)
        {
            data[j][i] = msg.data[i + width*j];
        }
    }
    
    int dim = (int) round(robot_dim / resolution);
    if (dim%2 == 0) dim++;
    std::vector<std::vector<double>> kernel = CalcKernel(dim);
    /*******************************************
    std::string s = "Kernel:\n";
    for (int j = 0; j < kernel.size(); j++)
    {
       for (int i = 0; i < kernel[j].size(); i++)
       {
           s += std::to_string(kernel[j][i]) + "\t";
       }
       s += "\n";
    }
    ROS_INFO("%s", s.c_str());
    /*******************************************/
    //kernel center index
    int ci = (dim + 1)/2 - 1;
    int cj = (dim + 1)/2 - 1;

    for (int j = 0; j < height; j++)
    {
        for (int i = 0; i < width; i++)
        {
            double sum_value = 0.0;
            double sum_coeff = 0.0;

            // ROS_INFO("sum_value = %f, sum_coeff = %f", sum_value, sum_coeff);

            for (int m = -(int)((dim-1)/2); m <= (int)((dim-1)/2); m++)
            {
                for (int n = -(int)((dim-1)/2); n <= (int)((dim-1)/2); n++)
                {
                    // ROS_INFO("j = %d, i = %d, m = %d, n = %d", j, i, m, n);
                    // std::string ss = "m = " + std::to_string(m) + ", n = " + std::to_string(n) + ", ";

                    if (j+m >= 0 && j+m < height && i+n >= 0 && i+n < width)
                    {
                        if (data[j+m][i+n] == -1) continue;
                        sum_value += (data[j+m][i+n] * kernel[cj+m][ci+n]);
                        sum_coeff += kernel[cj+m][ci+n];
                        // ss += "sum_val += " + std::to_string(data[j+m][i+n]) + " * " + std::to_string(kernel[cj+m][ci+n]) + " = " + std::to_string(sum_value) + ", sum_coeff = " + std::to_string(sum_coeff);
                        // ROS_INFO("%s", ss.c_str());
                    }
                    
                    
                }
            }

            data_blured[j][i] = sum_value / sum_coeff;

        }
    }
    
    map.header.frame_id = msg.header.frame_id;
    map.header.stamp = ros::Time::now();

    map.info.height = height;
    map.info.width = width;
    map.info.origin = msg.info.origin;
    map.info.map_load_time = ros::Time::now();
    map.info.resolution = resolution;

    map.data.resize(height * width);
    for (int j = 0; j < height; j++)
    {
        for (int i = 0; i < width; i++)
        {
            map.data[i + width*j] = data_blured[j][i];
        }
    }
    
    map_pub.publish(map);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "Map_Server");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    
    ROS_INFO("%s node started.", ros::this_node::getName().c_str());

    private_nh.param("Robot_Width", robot_dim, 0.3);
    private_nh.param("FromTopic", FromTopic, std::string("/map"));
    private_nh.param("ToTopic", ToTopic, std::string("/modified_map"));

    map_sub = nh.subscribe(FromTopic, 1, map_Callback);
    map_pub = nh.advertise<nav_msgs::OccupancyGrid>(ToTopic, 1);

    

    while (!check)
    {
        ROS_INFO("Waiting for map ...");
        ros::spinOnce();
        ros::Duration(1).sleep();
    }
    ROS_INFO("Map Recieved.");

    ros::spin();
}