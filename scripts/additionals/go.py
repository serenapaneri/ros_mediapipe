#include <spot_msgs/TrajectoryActionGoal.h>

actionlib::SimpleActionClient<spot_msgs::TrajectoryAction> ac_ ("spot/trajectory", true);

    ac_.cancelGoal();
    geometry_msgs::PoseStamped target;
    geometry_msgs::PoseStamped curr_pose;

    spot_msgs::TrajectoryGoal goal;
    //goal.header.frame_id="body";
    //goal.goal_id.id=1;
    goal.target_pose.header.frame_id="body";
    goal.duration.data.sec=10;
    // invoking robot pose and orientaiton
    geometry_msgs::PoseStamped global_pose;
    costmap_ros_->getRobotPose(global_pose);
    robot_curr_pose[0] = global_pose.pose.position.x;
    robot_curr_pose[1] = global_pose.pose.position.y;
    robot_curr_orien=getEulerAngles(global_pose)[2];


    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY( 0, 0, angularvel);
    myQuaternion.normalize();
   goal.target_pose.pose.orientation.x=myQuaternion.getX(); 
    goal.target_pose.pose.orientation.y=myQuaternion.getY(); 
    goal.target_pose.pose.orientation.z=myQuaternion.getZ(); 
    goal.target_pose.pose.orientation.w=myQuaternion.getW(); 
    ROS_ERROR("linear vel: %f    angular vel: %f %f %f %f ",goal.target_pose.pose.position.x, goal.target_pose.pose.orientation.x,goal.target_pose.pose.orientation.y,goal.target_pose.pose.orientation.z,goal.target_pose.pose.orientation.w );
    //pub_.publish(goal);
    ac_.sendGoal(goal); 


        // invoking robot pose and orientaiton
        geometry_msgs::PoseStamped global_pose;
        costmap_ros_->getRobotPose(global_pose);
                    robot_curr_pose[0] = global_pose.pose.position.x;
            robot_curr_pose[1] = global_pose.pose.position.y;

        float linear_distance= hypot((global_plan_[global_plan_.size() - 1].pose.position.x-robot_curr_pose[0]),(global_plan_[global_plan_.size() - 1].pose.position.y-robot_curr_pose[1]));
        // last pose
        //cout << "Orinet: "<< fabs(final_orientation[2] - robot_curr_orien) << " - " << o_precision_ << "  Pose: " << getGoalPositionDistance(global_plan_.back(), robot_curr_pose[0],robot_curr_pose[1]) << " - " << p_precision_ <<  endl;
        if ( linear_distance <= 0.3) 
        {
            goal_reached_ = true;
            ROS_ERROR("Goal has been reached!");
        }
