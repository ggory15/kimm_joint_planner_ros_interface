#include "ros/ros.h"

#include "kimm_joint_planner_ros_interface/plan_joint_path.h"
#include "kimm_joint_planner_ros_interface/action_joint_path.h"

#include "std_msgs/Bool.h"
#include "sensor_msgs/JointState.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <kimm_trajectory_smoother/Trajectory.h>
#include <kimm_trajectory_smoother/Path.h>
#include <kimm_hqp_controller/trajectory/trajectory_euclidian.hpp>

using namespace std;
using namespace Eigen;
using namespace kimmhqp::trajectory;

int playTime_;
bool is_calc_, is_run_;

ros::ServiceServer planning_server_;
ros::ServiceServer action_server_;

std::shared_ptr<TrajectoryEuclidianCubic> traj_cubic_;
std::shared_ptr<TrajectoryEuclidianTimeopt> traj_timeopt_;
std::shared_ptr<TrajectoryEuclidianConstant> traj_const_;
int type_ = 0;
Eigen::VectorXd c_joint_, t_joint_, kp_, kv_, vel_limit_, acc_limit_;
Eigen::VectorXi mask_;
std::vector<Eigen::VectorXd> res_traj_;
std::vector<sensor_msgs::JointState> res_j_traj_;
double duration_ = 0;

bool calculation(kimm_joint_planner_ros_interface::plan_joint_path::Request &req, kimm_joint_planner_ros_interface::plan_joint_path::Response &res)
{
    type_ = req.traj_type;
    int nq = req.current_joint.position.size();
    c_joint_.setZero(nq);
    t_joint_.setZero(nq);
    mask_.setZero(nq);
    kp_.setZero(nq);
    kv_.setZero(nq);

    vel_limit_.setOnes(nq);
    acc_limit_.setOnes(nq);
    
    for (int i=0; i<nq; i++){
        c_joint_(i) = req.current_joint.position[i];
        mask_(i) = req.mask[i].data;
        kp_(i) = req.kp[i];
        kv_(i) = req.kv[i];
    }

    if (type_ == 0){
        for (int i=0; i<nq; i++){
            t_joint_(i) = req.target_joint[0].position[i];
        }
        traj_const_->setReference(t_joint_);
        res_traj_= traj_const_->getWholeTrajectory();

    }
    else if (type_ == 1){
        for (int i=0; i<nq; i++){
            t_joint_(i) = req.target_joint[0].position[i];
        }
        duration_ = req.duration;
        traj_cubic_->setInitSample(c_joint_);
        traj_cubic_->setStartTime(0.);
        traj_cubic_->setDuration(duration_);
        traj_cubic_->setGoalSample(t_joint_);

        res_traj_= traj_cubic_->getWholeTrajectory();
    }
    else{
        vel_limit_ *= req.vel_limit;
        acc_limit_ *= req.acc_limit;

        traj_timeopt_->setMaxVelocity(vel_limit_);
        traj_timeopt_->setMaxAcceleration(acc_limit_);
        traj_timeopt_->clearWaypoints();
        traj_timeopt_->setStartTime(0.0);
        traj_timeopt_->addWaypoint(c_joint_);

        for (int j=0; j<req.target_joint.size(); j++){
            for (int i=0; i<nq; i++)
                t_joint_(i) = req.target_joint[j].position[i];
            traj_timeopt_->addWaypoint(t_joint_);
        }
    
        res_traj_= traj_timeopt_->getWholeTrajectory();
    }
    sensor_msgs::JointState step;
    step.position.resize(nq);
    for (int i=0; i<res_traj_.size(); i++){
        for (int j=0; j<nq; j++)
            step.position[j] = res_traj_[i](j);
        res.res_traj.push_back(step);
    }
    res_j_traj_ = res.res_traj;
    is_calc_ = true;
    is_run_ = false;
    return true;
}
bool updateTrajectory(kimm_joint_planner_ros_interface::action_joint_path::Request &req, kimm_joint_planner_ros_interface::action_joint_path::Response &res)
{
    if (is_calc_ && !is_run_){
    	is_run_ = true;
 	    ROS_INFO("is_run: [%d]", is_run_ ); // res.q_trajectory.joint_names[0].c_str()
        res.kp.clear();
        res.kv.clear();

        res.res_traj = res_j_traj_;
        for (int i=0; i<7; i++){
            res.kp.push_back(kp_(i));
            res.kv.push_back(kv_(i));
        }

        return true;
    }
    else{
    	res_j_traj_.clear();
    	res.res_traj.clear();
        return true;
    }
    return true;
}

int main(int argc, char **argv)
{

    struct timespec spec;
    clock_gettime(CLOCK_REALTIME, &spec);
    srand(spec.tv_nsec);

    ros::init(argc, argv, "JointTrajPlannerServer");
    ros::NodeHandle nh("~");  
    
    planning_server_ = nh.advertiseService("plan_joint_path", calculation);
    action_server_ = nh.advertiseService("action_joint_path", updateTrajectory);
    
    traj_cubic_ = std::make_shared<TrajectoryEuclidianCubic>("traj_cubic");
    traj_timeopt_ = std::make_shared<TrajectoryEuclidianTimeopt>("traj_timeopt");
    traj_const_ = std::make_shared<TrajectoryEuclidianConstant>("traj_constant");

    ROS_INFO("ready srv server!");
    
    while (ros::ok())
    { 


        ros::spinOnce();       
    } 

    return 0;
}
