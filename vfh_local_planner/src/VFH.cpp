#include <vfh_local_planner/VFH.h>

namespace local_planner
{
    VFH::VFH(std::string _namespace){
        private_nh = ros::NodeHandle("~");
        robot_namespace = _namespace;
        odomSub = nh.subscribe(robot_namespace + odom_topic_name, 1, &VFH::OdometryCallback, this);
        laserSub = nh.subscribe(robot_namespace + laser_topic_name, 1, &VFH::LaserCallback, this);
        velPub = nh.advertise<geometry_msgs::Twist>(robot_namespace + velocity_topic_name, 1);
        localSrv = nh.advertiseService(robot_namespace + local_service_name, &VFH::LocalPlanCallback, this);
        ROS_INFO("private_nh namespace: %s", private_nh.getNamespace().c_str());
        checkPose = false;
        checkLaser = false;
        isShutDown = false;

        ROS_INFO("VFH initialized successfully.");
    }

    void VFH::OdometryCallback(nav_msgs::Odometry msg){
        currentPos.position = msg.pose.pose.position;
        currentPos.orientation = msg.pose.pose.orientation;
        currentVel = msg.twist.twist;
        checkPose = true;
    }

    void VFH::LaserCallback(sensor_msgs::LaserScan msg){
        laser = msg;
        for (int i = 0; i < laser.ranges.size(); i++)
        {
            if (laser.ranges[i] >= laser.range_max)
            {
                laser.ranges[i] = laser.range_max;
            }
        }
        certainty_grid.UpdateSensorValue(laser, VFH::getYawFromQuaternion(currentPos.orientation));
        checkLaser = true;
    }

    bool VFH::LocalPlanCallback(navigation_msgs::LocalPosePlan::Request& req, navigation_msgs::LocalPosePlan::Response& resp){
        ROS_INFO("Waiting for odometry and laser scanner topics.");
        while (!checkPose || !checkLaser)
        {
            ros::spinOnce();
            ros::Duration(1).sleep();
        }
        ROS_INFO("Subscribed to odometry and laser scanner topics.");
        for (int i = 0; i < req.posePlan.size(); i++)
        {
            if (isShutDown)
            {
                VFH::SetSpeed(0.0, 0.0);
                velPub.publish(speed);
                break;
            }
            
            goalPos.position.x = req.posePlan[i].x; goalPos.position.y = req.posePlan[i].y; goalPos.position.z = req.posePlan[i].z;
            if(!VFH::ComputeVelocityCommand(speed)){
                resp.success = false;
                return false;
            }
            checkPose = false;
            checkLaser = false;
            ROS_INFO("Reached target i = %d", i);
        }
        if (isShutDown)
        {
            isShutDown = false;
            resp.success = false;
            return false;
        }
        resp.success = true;
        return true;
    }

    void VFH::SetSpeed(double Lx, double Az){
        speed.linear.x = Lx;
        speed.linear.y = 0.0;
        speed.linear.z = 0.0;
        speed.angular.x = 0.0;
        speed.angular.y = 0.0;
        speed.angular.z = Az;
    }

    double VFH::getYawFromQuaternion(geometry_msgs::Quaternion q){
        tf::Quaternion Q(q.x, q.y, q.z, q.w);
        tf::Matrix3x3 m(Q);
        double r, p, y;
        m.getRPY(r, p, y);
        return y;
    }

    double VFH::GetDistance(geometry_msgs::Pose poseA, geometry_msgs::Pose poseB){
        return sqrt(pow(poseA.position.x - poseB.position.x, 2) + pow(poseA.position.y - poseB.position.y, 2) + pow(poseA.position.z - poseB.position.z, 2));
    }

    bool VFH::ComputeVelocityCommand(geometry_msgs::Twist& cmd_vel){
        while (true)
        {
            if (isShutDown)
            {
                VFH::SetSpeed(0.0, 0.0);
                velPub.publish(speed);
                return false;
            }
            
            ros::spinOnce();

            VFH::Polar_Histogram();
            VFH::Smoothed_Polar_Histogram();
            int k_c = VFH::Data_Reduction();
            VFH::Speed_Control(k_c);

            double delta_theta = DeltaRad(steer_direction, VFH::getYawFromQuaternion(currentPos.orientation));
            double Az = delta_theta * 2;
            Az = (std::abs(Az) > 1.82)? 1.82 * std::abs(Az)/Az: Az;
            VFH::SetSpeed(steer_magnitude, Az);
            velPub.publish(speed);

            ros::spinOnce();

            if (isGoalReached())
            {
                VFH::SetSpeed(0.0, 0.0);
                velPub.publish(speed);
                ROS_INFO("Reached target.");
                break;
            }
        }
        return true;
    }

    bool VFH::isGoalReached(){
        if (GetDistance(currentPos, goalPos) < 1.0e-1)
        {
            return true;
        }
        return false;
    }

    void VFH::Polar_Histogram(){
        double L_max = certainty_grid.cellSize * (certainty_grid.windowSize - 1) / 2;

        memset(POD, 0, 360 * sizeof(double));

        for (int yi = 0; yi < certainty_grid.windowSize; yi++)
        {
            for (int xi = 0; xi < certainty_grid.windowSize; xi++)
            {
                double c_ij = certainty_grid.cells[yi][xi];
                if (c_ij > 0)
                {
                    double dy = certainty_grid.CU2M(yi);
                    double dx = certainty_grid.CU2M(xi);

                    double d_ij = sqrt(dx*dx + dy*dy);
                    double beta_ij = atan2(dy, dx);

                    int index = H_ID(beta_ij * _RAD2DEG);

                    if (d_ij < L_max)
                    {
                        double a = 4.0;
                        double b = a / L_max;
                        double m_ij = c_ij*c_ij*(a - b * d_ij);
                        POD[index] = std::max(POD[index], m_ij);
                    }
                }
            }
        }
    }

    void VFH::Smoothed_Polar_Histogram(){
        int l = 25;
        for (int i = 0; i < 360; i++)
        {
            smoothed_POD[i] = (l+1) * POD[i];
            int n = l + 1;

            for (int j = 1; j <= l; j++)
            {
                int i_p = H_ID(i - j);
                int i_m = H_ID(i + j);

                smoothed_POD[i] += (l - j + 1) * POD[i_p];
                smoothed_POD[i] += (l - j + 1) * POD[i_m];

                n += 2 * (l - j + 1);
            }
            smoothed_POD[i] /= n;
        }
        
    }

    int VFH::Data_Reduction(){
        int S_MAX;
        private_nh.param("VFH_Local_Planner/S_max", S_MAX, 54);
        double THRESHOLD;
        private_nh.param("VFH_Local_Planner/Threshold", THRESHOLD, 0.1);

        target_direction = atan2(goalPos.position.y - currentPos.position.y, goalPos.position.x - currentPos.position.x);

        int min_i = -1;
        double min_value = 1000;

        for (int i = 0; i < 360; i++)
        {
            if (smoothed_POD[i] < THRESHOLD)
            {
                double dth = std::abs(DeltaRad(target_direction, i*_DEG2RAD));
                if (dth < min_value)
                {
                    min_value = dth;
                    min_i = i;
                }
            }
        }
        
        if (min_i < 0)
        {
            steer_magnitude = 0.0;
        }
        
        int k_n = min_i - 1;
        int k_f = min_i + 1;

        for (int count = 0; count < S_MAX; )
        {
            bool out = false;
            if (smoothed_POD[H_ID(k_n)] < THRESHOLD)
            {
                --k_n;
                ++count;
            }
            else
            {
                out = true;
            }
            
            if (smoothed_POD[H_ID(k_f)] < THRESHOLD)
            {
                ++k_f;
                ++count;
            }
            else
            {
                if (out)    break;
            }
        }
        int k_c = (k_n + k_f) / 2;
        return H_ID(k_c);
    }

    void VFH::Speed_Control(int k_c){
        double V_MAX, V_MIN, OMEGA_MAX;
        private_nh.param("Robot/V_max", V_MAX, 0.26);
        private_nh.param("Robot/V_min", V_MIN, 0.01);
        private_nh.param("Robot/Omega_max", OMEGA_MAX, 1.82);

        steer_direction = k_c * _DEG2RAD;
        steer_magnitude = V_MAX * (1 - smoothed_POD[(int)(k_c)]/1.0);

        if (steer_magnitude < V_MIN) steer_magnitude = V_MIN;

        double Omega = DeltaRad(steer_direction, VFH::getYawFromQuaternion(currentPos.orientation));
        if (std::abs(Omega) > OMEGA_MAX)
        {
            Omega = OMEGA_MAX * std::abs(Omega) / Omega;
        }
        

        steer_magnitude = steer_magnitude * (1.0 - std::abs(Omega)/OMEGA_MAX);

        if(steer_magnitude < V_MIN) steer_magnitude = V_MIN;
    }
} // namespace local_planner
