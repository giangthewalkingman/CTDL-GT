#include "offboard/offboard.h"
#include "offboard/queue.h"
#include <stack>

//constructor of Offboard class
OffboardControl::OffboardControl(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private, bool input_setpoint) : nh_(nh),
                                                                                                                      nh_private_(nh_private)                                        
                                                                                                                      {
    state_sub_ = nh_.subscribe("/mavros/state", 10, &OffboardControl::stateCallback, this);
    odom_sub_ = nh_.subscribe("/mavros/local_position/odom", 10, &OffboardControl::odomCallback, this);
    setpoint_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    odom_error_pub_ = nh_.advertise<nav_msgs::Odometry>("odom_error", 1, true);
    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    nh_private_.param<bool>("/offboard_node/simulation_mode_enable", simulation_mode_enable_, simulation_mode_enable_);
    nh_private_.param<bool>("/offboard_node/delivery_mode_enable", delivery_mode_enable_, delivery_mode_enable_);
    nh_private_.param<bool>("/offboard_node/return_home_mode_enable", return_home_mode_enable_, return_home_mode_enable_);
    nh_private_.getParam("/offboard_node/z_takeoff", z_takeoff_);
    nh_private_.getParam("/offboard_node/z_delivery", z_delivery_);
    nh_private_.getParam("/offboard_node/land_error", land_error_);
    nh_private_.getParam("/offboard_node/hover_time", hover_time_);
    nh_private_.getParam("/offboard_node/unpack_time", unpack_time_);
    nh_private_.getParam("/offboard_node/desired_velocity", vel_desired_);
    nh_private_.getParam("/offboard_node/odom_error", odom_error_);

    waitForPredicate(10.0);
    dequeueFlight();
}

//destructor
OffboardControl::~OffboardControl() {

}

/* wait for connect
   input: ros rate in hertz, at least 2Hz */
void OffboardControl::waitForPredicate(double hz) {
    ros::Rate rate(hz);

    std::printf("\n[ INFO] Waiting for FCU connection \n");
    while (ros::ok() && !current_state_.connected) {
        ros::spinOnce();
        rate.sleep();
    }
    std::printf("[ INFO] FCU connected \n");
    if (simulation_mode_enable_) {
        std::printf("\n[ NOTICE] Parameter 'simulation_mode_enable' is set true\n");
        std::printf("          OFFBOARD node will automatic ARM and set OFFBOARD mode\n");
        std::printf("          Continue if run a simulation OR SHUTDOWN node if run in drone\n");
        std::printf("          Set parameter 'simulation_mode_enable' to false or not set (default = false)\n");
        std::printf("          and relaunch node for running in drone\n");
        std::printf("          > roslaunch offboard offboard.launch simulation_mode_enable:=false\n");
    }
    else {
        std::printf("\n[ NOTICE] Prameter 'simulation_mode_enable' is set false or not set (default = false)\n");
        std::printf("          OFFBOARD node will wait for ARM and set OFFBOARD mode from RC controller\n");
        std::printf("          Continue if run in drone OR shutdown node if run a simulation\n");
        std::printf("          Set parameter 'simulation_mode_enable' to true and relaunch node for simulation\n");
        std::printf("          > roslaunch offboard offboard.launch simulation_mode_enable:=true\n");
    }
    operation_time_1_ = ros::Time::now();
}

/* send a few setpoints before publish
   input: ros rate in hertz (at least 2Hz) and first setpoint */
void OffboardControl::setOffboardStream(double hz, geometry_msgs::PoseStamped first_target) {
    home_enu_pose_ = targetTransfer(current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, current_odom_.pose.pose.position.z);
    ros::Rate rate(hz);
    std::printf("[ INFO] Setting OFFBOARD stream \n");
    for (int i = 50; ros::ok() && i > 0; --i) {
        target_enu_pose_ = first_target;
        target_enu_pose_.header.stamp = ros::Time::now();
        setpoint_pose_pub_.publish(target_enu_pose_);
        ros::spinOnce();
        rate.sleep();
    }
    std::printf("\n[ INFO] OFFBOARD stream is set\n");
}

/* wait for ARM and OFFBOARD mode switch (in SITL case or HITL/Practical case)
   input: ros rate in hertz, at least 2Hz */
void OffboardControl::waitForArmAndOffboard(double hz) {
    ros::Rate rate(hz);
    if (simulation_mode_enable_) {
        std::printf("\n[ INFO] Ready to takeoff\n");
        while (ros::ok() && !current_state_.armed && (current_state_.mode != "OFFBOARD")) {
            mavros_msgs::CommandBool arm_amd;
            arm_amd.request.value = true;
            if (arming_client_.call(arm_amd) && arm_amd.response.success) {
                ROS_INFO_ONCE("Vehicle armed");
            }
            else {
                ROS_INFO_ONCE("Arming failed");
            }

            // mavros_msgs::SetMode offboard_setmode_;
            offboard_setmode_.request.base_mode = 0;
            offboard_setmode_.request.custom_mode = "OFFBOARD";
            if (set_mode_client_.call(offboard_setmode_) && offboard_setmode_.response.mode_sent) {
                ROS_INFO_ONCE("OFFBOARD enabled");
            }
            else {
                ROS_INFO_ONCE("Failed to set OFFBOARD");
            }
            ros::spinOnce();
            rate.sleep();
        }
        //DuyNguyen
        if (odom_error_) {
            odom_error_pub_.publish(current_odom_);
        }
    }
    else {
        std::printf("\n[ INFO] Waiting switching (ARM and OFFBOARD mode) from RC\n");
        while (ros::ok() && !current_state_.armed && (current_state_.mode != "OFFBOARD")) {
            ros::spinOnce();
            rate.sleep();
        }
        //DuyNguyen
        if (odom_error_) {
            odom_error_pub_.publish(current_odom_);
        }
    }
}

void OffboardControl::stateCallback(const mavros_msgs::State::ConstPtr &msg) {
    current_state_ = *msg;
}

void OffboardControl::odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    current_odom_ = *msg;
}

void OffboardControl::dequeueFlight() {
    ros::Rate rate(10.0);
    bool target_reached = true;
    int i = 0;
    geometry_msgs::PoseStamped setpoint;
    double x, y, z, yaw;
    std::printf("[ INFO] Manual enter ENU target position(s) to drop packages\n");
    std::printf(" Number of target(s): ");
    std::cin >> num_of_enu_target_;
    ArrayQueue q1(num_of_enu_target_);
    std::cout << "Start to enqueue each setpoint to the queue..." << std::endl;
    for (int i = 0; i < num_of_enu_target_; i++) {
        std::printf(" Target (%d) postion x, y, z (in meter): ", i + 1);
        std::cin >> x >> y >> z;
        geometry_msgs::PoseStamped point_to_push;
        point_to_push.pose.position.x = x;
        point_to_push.pose.position.y = y;
        point_to_push.pose.position.z = z;
        q1.enQueue(point_to_push);
        ros::spinOnce();
        rate.sleep();
    }
    std::cout << "Enqueue completed!" << std::endl;
    q1.printQueue();
    std::printf(" Error to check target reached (in meter): ");
    std::cin >> target_error_;
    pushIdxToStack(myStack);
    setOffboardStream(10.0, targetTransfer(current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, z_takeoff_));
    waitForArmAndOffboard(10.0);
    takeOff(targetTransfer(current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, z_takeoff_), hover_time_);
    std::printf("\n[ INFO] Flight with ENU setpoint and Yaw angle\n");
    while (ros::ok() && target_reached) {
        std::cout << "Dequeueing point " << i+1 << std::endl;
        target_reached = false;
        if (i < (num_of_enu_target_ - 1)) {
            final_position_reached_ = false;
            // setpoint = targetTransfer(x_target_[i], y_target_[i], z_target_[i]);
            setpoint = q1.deQueue();
        }
        else {
            final_position_reached_ = true;
            // setpoint = targetTransfer(x_target_[num_of_enu_target_ - 1], y_target_[num_of_enu_target_ - 1], z_target_[num_of_enu_target_ - 1]);
            setpoint = q1.deQueue(); //try it
        }
        std::cout << "Final position reached check: " << final_position_reached_ << std::endl;
        while(ros::ok()){
            components_vel_ = velComponentsCalc(vel_desired_, targetTransfer(current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, current_odom_.pose.pose.position.z), setpoint);

            target_enu_pose_ = targetTransfer(current_odom_.pose.pose.position.x + components_vel_.x, current_odom_.pose.pose.position.y + components_vel_.y, current_odom_.pose.pose.position.z + components_vel_.z);
            target_enu_pose_.header.stamp = ros::Time::now();
            setpoint_pose_pub_.publish(target_enu_pose_);

            distance_ = distanceBetween(targetTransfer(current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, current_odom_.pose.pose.position.z), setpoint);
            std::printf("Distance to target: %.1f (m) \n", distance_);

            target_reached = checkPositionError(target_error_, setpoint);

            if (target_reached && !final_position_reached_) {
                std::printf("\n[ INFO] Reached position: [%.1f, %.1f, %.1f]\n", current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, current_odom_.pose.pose.position.z);

                hovering(setpoint, hover_time_);
                if (delivery_mode_enable_)
                {
                    delivery(setpoint, unpack_time_);
                }
                i += 1;
                break;
            }
            if (target_reached && final_position_reached_) {
                std::printf("\n[ INFO] Reached Final position: [%.1f, %.1f, %.1f]\n", current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, current_odom_.pose.pose.position.z);
                hovering(setpoint, hover_time_);
                if (!return_home_mode_enable_) {
                    landing(targetTransfer(setpoint.pose.position.x, setpoint.pose.position.y, 0.0));
                }
                else {
                    if (delivery_mode_enable_) {
                        delivery(setpoint, unpack_time_);
                    }
                    std::printf("\n[ INFO] Returning home [%.1f, %.1f, %.1f]\n", home_enu_pose_.pose.position.x, home_enu_pose_.pose.position.y, home_enu_pose_.pose.position.z);
                    returnHome(targetTransfer(home_enu_pose_.pose.position.x, home_enu_pose_.pose.position.y, setpoint.pose.position.z));
                    landing(home_enu_pose_);
                }
            }
            ros::spinOnce();
            rate.sleep();
        }
    }
    
}


/* calculate distance between current position and setpoint position
   input: current and target poses (ENU) to calculate distance */
double OffboardControl::distanceBetween(geometry_msgs::PoseStamped current, geometry_msgs::PoseStamped target) {
    Eigen::Vector3d distance;
    distance << target.pose.position.x - current.pose.position.x,
        target.pose.position.y - current.pose.position.y,
        target.pose.position.z - current.pose.position.z;

    return distance.norm();
}

/* calculate components of velocity about x, y, z axis
   input: desired velocity, current and target poses (ENU)
   key: vx/v = dx/d
    */
geometry_msgs::Vector3 OffboardControl::velComponentsCalc(double v_desired, geometry_msgs::PoseStamped current, geometry_msgs::PoseStamped target) {
    double xc = current.pose.position.x;
    double yc = current.pose.position.y;
    double zc = current.pose.position.z;

    double xt = target.pose.position.x;
    double yt = target.pose.position.y;
    double zt = target.pose.position.z;

    double dx = xt - xc;
    double dy = yt - yc;
    double dz = zt - zc;

    double d = sqrt(sqr(dx) + sqr(dy) + sqr(dz));

    geometry_msgs::Vector3 vel;

    vel.x = ((dx / d) * v_desired);
    vel.y = ((dy / d) * v_desired);
    vel.z = ((dz / d) * v_desired);

    return vel;
}



/* perform takeoff task
   input: setpoint to takeoff and hover time */
void OffboardControl::takeOff(geometry_msgs::PoseStamped setpoint, double hover_time) {
    ros::Rate rate(10.0);
    std::printf("\n[ INFO] Takeoff to [%.1f, %.1f, %.1f]\n", setpoint.pose.position.x, setpoint.pose.position.y, setpoint.pose.position.z);
    bool takeoff_reached = false;
    while (ros::ok() && !takeoff_reached) {
        components_vel_ = velComponentsCalc(vel_desired_, targetTransfer(current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, current_odom_.pose.pose.position.z), setpoint);

        target_enu_pose_ = targetTransfer(current_odom_.pose.pose.position.x + components_vel_.x, current_odom_.pose.pose.position.y + components_vel_.y, current_odom_.pose.pose.position.z + components_vel_.z);
        target_enu_pose_.header.stamp = ros::Time::now();
        setpoint_pose_pub_.publish(target_enu_pose_);

        takeoff_reached = checkPositionError(target_error_, setpoint);
        if (takeoff_reached) {
            hovering(setpoint, hover_time);
        }
        else {
            ros::spinOnce();
            rate.sleep();
        }
    }
}

geometry_msgs::PoseStamped OffboardControl::targetTransfer(double x, double y, double z) {
    geometry_msgs::PoseStamped target;
    target.pose.position.x = x;
    target.pose.position.y = y;
    target.pose.position.z = z;
    return target;
}

/* perform hover task
   input: setpoint to hover and hover time */
void OffboardControl::hovering(geometry_msgs::PoseStamped setpoint, double hover_time) {
    ros::Rate rate(10.0);
    ros::Time t_check;

    std::printf("\n[ INFO] Hovering at [%.1f, %.1f, %.1f] in %.1f (s)\n", setpoint.pose.position.x, setpoint.pose.position.y, setpoint.pose.position.z, hover_time);
    t_check = ros::Time::now();
    while ((ros::Time::now() - t_check) < ros::Duration(hover_time)) {
        // ROS_INFO_STREAM(setpoint);
        setpoint.header.stamp = ros::Time::now();
        setpoint_pose_pub_.publish(setpoint);
        // std::cout << "Hello World" <<std::endl;
        ros::spinOnce();
        rate.sleep();
    }
}

/* perform land task
   input: set point to land (e.g., [x, y, 0.0]) */
void OffboardControl::landing(geometry_msgs::PoseStamped setpoint) {
    ros::Rate rate(10.0);
    bool land_reached = false;
    std::printf("[ INFO] Landing\n");
    while (ros::ok() && !land_reached) {
        components_vel_ = velComponentsCalc(vel_desired_, targetTransfer(current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, current_odom_.pose.pose.position.z), setpoint);

        target_enu_pose_ = targetTransfer(current_odom_.pose.pose.position.x + components_vel_.x, current_odom_.pose.pose.position.y + components_vel_.y, current_odom_.pose.pose.position.z + components_vel_.z);
        target_enu_pose_.header.stamp = ros::Time::now();
        // target_enu_pose_.pose.orientation = setpoint.pose.orientation;
        setpoint_pose_pub_.publish(target_enu_pose_);

        land_reached = checkPositionError(land_error_, setpoint);

        if (current_state_.system_status == 3) {
            std::printf("\n[ INFO] Land detected\n");
            flight_mode_.request.custom_mode = "AUTO.LAND";
            if (set_mode_client_.call(flight_mode_) && flight_mode_.response.mode_sent) {
                break;
            }
        }
        else if (land_reached) {
            flight_mode_.request.custom_mode = "AUTO.LAND";
            if (set_mode_client_.call(flight_mode_) && flight_mode_.response.mode_sent) {
                std::printf("\n[ INFO] LANDED\n");
            }
        }
        else {
            ros::spinOnce();
            rate.sleep();
        }
    }

    operation_time_2_ = ros::Time::now();
    std::printf("\n[ INFO] Operation time %.1f (s)\n\n", (operation_time_2_ - operation_time_1_).toSec());
    ros::shutdown();
}

/* perform return home task
   input: home pose in ENU (e.g., [home x, home y, 10.0])*/
void OffboardControl::returnHome(geometry_msgs::PoseStamped home_pose) {
    ros::Rate rate(10.0);
    bool home_reached = false;
    while (ros::ok() && !home_reached) {
        components_vel_ = velComponentsCalc(vel_desired_, targetTransfer(current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, current_odom_.pose.pose.position.z), home_pose);

        target_enu_pose_ = targetTransfer(current_odom_.pose.pose.position.x + components_vel_.x, current_odom_.pose.pose.position.y + components_vel_.y, current_odom_.pose.pose.position.z + components_vel_.z);
        target_enu_pose_.header.stamp = ros::Time::now();
        setpoint_pose_pub_.publish(target_enu_pose_);

        home_reached = checkPositionError(target_error_, home_pose);
        if (home_reached) {
            hovering(home_pose, hover_time_);
        }
        else {
            ros::spinOnce();
            rate.sleep();
        }
    }
}

/* perform delivery task
   input: current setpoint in trajectory and time to unpack */
void OffboardControl::delivery(geometry_msgs::PoseStamped setpoint, double unpack_time) {
    ros::Rate rate(10.0);
    bool land_reached = false;
    std::printf("[ INFO] Land for unpacking\n");
    while (ros::ok() && !land_reached) {
        components_vel_ = velComponentsCalc(vel_desired_, targetTransfer(current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, current_odom_.pose.pose.position.z), targetTransfer(setpoint.pose.position.x, setpoint.pose.position.y, z_delivery_));

        target_enu_pose_ = targetTransfer(current_odom_.pose.pose.position.x + components_vel_.x, current_odom_.pose.pose.position.y + components_vel_.y, current_odom_.pose.pose.position.z + components_vel_.z);
        target_enu_pose_.header.stamp = ros::Time::now();
        setpoint_pose_pub_.publish(target_enu_pose_);

        if (current_state_.system_status == 3) {
            land_reached = true;
        }
        else {
            land_reached = checkPositionError(land_error_, targetTransfer(setpoint.pose.position.x, setpoint.pose.position.y, z_delivery_));
        }

        if (land_reached) {
            if (current_state_.system_status == 3) {
                hovering(targetTransfer(current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, current_odom_.pose.pose.position.z), unpack_time);
                // TODO: unpack service
            }
            else {
                hovering(targetTransfer(setpoint.pose.position.x, setpoint.pose.position.y, z_delivery_), unpack_time);
                // TODO: unpack service
            }
            std::cout << "Pop out the top element containing value " << myStack.top() << std::endl;
            myStack.pop();
            std::printf("\n[ INFO] Done! Return setpoint [%.1f, %.1f, %.1f]\n", setpoint.pose.position.x, setpoint.pose.position.y, setpoint.pose.position.z);
            returnHome(setpoint);
        }
        else {
            ros::spinOnce();
            rate.sleep();
        }
    }
}

bool OffboardControl::checkPositionError(double error, geometry_msgs::PoseStamped target) {
    Eigen::Vector3d geo_error;
    geo_error << target.pose.position.x - current_odom_.pose.pose.position.x, target.pose.position.y - current_odom_.pose.pose.position.y, target.pose.position.z - current_odom_.pose.pose.position.z;

    return (geo_error.norm() < error) ? true : false;
}

void OffboardControl::pushIdxToStack(std::stack<int> &stack) {
    int *A = new int[num_of_enu_target_];
    std::cout << "Input the index: " << std::endl;
    for(int i = 0; i < num_of_enu_target_; i++) {
        std::cin >> A[i];
        stack.push(A[i]);
    }
}

/////////// QUEUE


ArrayQueue::~ArrayQueue() {
    delete [] qArr;
}

void ArrayQueue::enQueue(geometry_msgs::PoseStamped x) {
    if((rear+1)%cap == front) {
        qArr[rear] = x; //must add this line, unless the qArr[rear] value will not be added
    }
    else {
        qArr[rear] = x;
        rear = (rear+1)%cap;
    }
    return;
}

geometry_msgs::PoseStamped ArrayQueue::deQueue() {
    geometry_msgs::PoseStamped retObj;
    geometry_msgs::PoseStamped emptyObj;
    if(rear==front) {
        retObj = qArr[front];
        qArr[front] = geometry_msgs::PoseStamped();  
    }
    else {
        retObj = qArr[front];
        qArr[front] = geometry_msgs::PoseStamped();
        std::cout << qArr[front].pose.position.x << " " << qArr[front].pose.position.y << " " << qArr[front].pose.position.z << std::endl;
        front = (front+1)%cap;
    }
    return retObj;
}

void ArrayQueue::printQueue() {
    std::cout << "Printing the queue..." << std::endl;
    if(front==rear) {
        std::cout << "The front element = the rear element = ";
        ROS_INFO_STREAM(qArr[front]);
    }
    else{
        for(int i = front; i <= rear; i++) {
        ROS_INFO_STREAM(qArr[i]);
    }
    }
    std::cout << "The queue is printed!" << std::endl;
}

//two functions to check whether the queue is empty or not
bool ArrayQueue::isFull() {
    if((rear - front + 1) == cap) {
        std::cout << "The queue is full" << std::endl;
        return 1;
    }
    else {
        std::cout << "The queue is not full" << std::endl;
        return 0;
    }
    //return 1;
}

bool ArrayQueue::isEmpty() {
    geometry_msgs::PoseStamped null;
    null = geometry_msgs::PoseStamped();
    if(front==rear && poseCompare(qArr[front], null)) {
        std::cout << "The queue is empty" << std::endl;
        return 1;
    }
    else {
        std::cout << "The queue is not empty" << std::endl;
        return 0;
    }
}

bool ArrayQueue::poseCompare(geometry_msgs::PoseStamped x, geometry_msgs::PoseStamped y) {
    if(x.pose.position.x == y.pose.position.x && x.pose.position.y == y.pose.position.y && x.pose.position.z == y.pose.position.z) {
        return true;
    }
    else {
        return false;
    }
}

