#include <slam/action_model.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <iostream>


ActionModel::ActionModel(void)
: k1_(0.1f)
, k2_(0.1f)
, initialized_(false)
{
    //////////////// TODO: Handle any initialization for your ActionModel 
    std::random_device rd;
    numberGenerator_ = std::mt19937(rd());

}


bool ActionModel::updateAction(const pose_xyt_t& odometry)
{
    ////////////// TODO: Implement code here to compute a new distribution of the motion of the robot 
    if(!initialized_){
        previousOdometry_ = odometry;
        initialized_ = true;
    }

    float delta_x = odometry.x - previousOdometry_.x;
    float delta_y = odometry.y - previousOdometry_.y;
    float delta_theta = odometry.theta - previousOdometry_.theta;
    
    trans_ = std::sqrt(delta_x * delta_x + delta_y * delta_y);
    rot1_ = angle_diff(std::atan2(delta_y, delta_x), previousOdometry_.theta);

    float direction = 1.0;
    if(std::abs(rot1_) > M_PI / 2.0){
        rot1_ = angle_diff(M_PI, rot1_);
        direction = -1.0;
    }
    rot2_ = angle_diff(delta_theta, rot1_);

    moved_ = (delta_x != 0.0) || (delta_y != 0.0) || (delta_theta != 0.0);
    
    
    if(moved_){
        rot1Std_ = k1_ * std::sqrt(std::abs(rot1_));
        transStd_ = k2_ * std::sqrt(std::abs(trans_));
        rot2Std_ = k1_ * std::sqrt(std::abs(rot2_));
    }

    trans_ = trans_ * direction;
    previousOdometry_ = odometry;

    utime_ = odometry.utime;

    return moved_;
    
}


particle_t ActionModel::applyAction(const particle_t& sample)
{
    ////////////// TODO: Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////
    // Make sure you create a new valid particle_t. 
    //Don't forget to set the new time and new parent_pose.
    particle_t newSample = sample;
    float sampleRot1 = std::normal_distribution<>(rot1_, rot1Std_)(numberGenerator_);
    float sampleTrans = std::normal_distribution<>(trans_, transStd_)(numberGenerator_);
    float sampleRot2 = std::normal_distribution<>(rot2_, rot2Std_)(numberGenerator_);
    newSample.pose.x += sampleTrans * cos(sample.pose.theta + sampleRot1);
    newSample.pose.y += sampleTrans * sin(sample.pose.theta + sampleRot1);
    newSample.pose.theta = wrap_to_pi(sample.pose.theta + sampleRot1 + sampleRot2);
    
    newSample.pose.utime = utime_;
    newSample.parent_pose = sample.pose;

    return newSample;
}
