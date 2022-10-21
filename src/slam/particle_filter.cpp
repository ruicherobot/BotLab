#include <slam/particle_filter.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <cassert>
#include <common/angle_functions.hpp>
#include <algorithm>

bool ParticleFilter::compareByWeight(const particle_t &a, const particle_t &b)
{
    return a.weight > b.weight;
}

ParticleFilter::ParticleFilter(int numParticles)
: kNumParticles_ (numParticles)
{
    assert(kNumParticles_ > 1);
    posterior_.resize(kNumParticles_);
}


void ParticleFilter::initializeFilterAtPose(const pose_xyt_t& pose)
{
    ///////////// TODO: Implement your method for initializing the particles in the particle filter 
    // it's just one method which is not good, we can try sample in whole area.
    posteriorPose_ = pose;

    for(auto& p : posterior_){
        p.pose.x = posteriorPose_.x;
        p.pose.y = posteriorPose_.y;
        p.pose.theta = wrap_to_pi(posteriorPose_.theta);
        p.pose.utime = posteriorPose_.utime;
        p.parent_pose = p.pose;
        p.weight = 1.0 / kNumParticles_;
    }
}


pose_xyt_t ParticleFilter::updateFilter(const pose_xyt_t&      odometry,
                                        const lidar_t& laser,
                                        const OccupancyGrid&   map)
{
    // Only update the particles if motion was detected. If the robot didn't move, then
    // obviously don't do anything.
    bool hasRobotMoved = actionModel_.updateAction(odometry);
    
    if(hasRobotMoved)
    {
        auto prior = resamplePosteriorDistribution();
        auto proposal = computeProposalDistribution(prior);
        posterior_ = computeNormalizedPosterior(proposal, laser, map);
        posteriorPose_ = estimatePosteriorPose(posterior_);
    }
    
    posteriorPose_.utime = odometry.utime;
    return posteriorPose_;
}

pose_xyt_t ParticleFilter::updateFilterActionOnly(const pose_xyt_t&      odometry)
{
    // Only update the particles if motion was detected. If the robot didn't move, then
    // obviously don't do anything.
    bool hasRobotMoved = actionModel_.updateAction(odometry);
    
    if(hasRobotMoved)
    {
        //auto prior = resamplePosteriorDistribution();
        auto proposal = computeProposalDistribution(posterior_);
        posterior_ = proposal;
    }
    
    posteriorPose_ = odometry;
    
    return posteriorPose_;
}


pose_xyt_t ParticleFilter::poseEstimate(void) const
{
    return posteriorPose_;
}


particles_t ParticleFilter::particles(void) const
{
    particles_t particles;
    particles.num_particles = posterior_.size();
    particles.particles = posterior_;
    return particles;
}


std::vector<particle_t> ParticleFilter::resamplePosteriorDistribution(void)
{
    //////////// TODO: Implement your algorithm for resampling from the posterior distribution

    // algorithm 1

    /*
    std::random_device rd;
    std::mt19937 generator(rd());
    std::uniform_real_distribution<> dist_uniform(0.0, 1.0 / kNumParticles_);
    std::normal_distribution<> dist_normal(0.0, 0.01);
    std::vector<particle_t> prior = posterior_;
    
    for(auto& p : prior){
        p.pose.x = posteriorPose_.x + dist_normal(generator);
        p.pose.y = posteriorPose_.y + dist_normal(generator);
        p.pose.theta = posteriorPose_.theta + dist_normal(generator);
        p.parent_pose = posteriorPose_;
        p.weight = 1.0 / kNumParticles_;
    }
    */
    
    
    // algorithm 2
    std::random_device rd;
    std::mt19937 generator(rd());
    std::uniform_real_distribution<> dist_uniform(0.0, 1.0 / kNumParticles_);
    std::normal_distribution<> dist_normal(0.0, 0.01);
    std::vector<particle_t> prior;
    std::sort(posterior_.begin(), posterior_.end(), compareByWeight);
    double r = dist_uniform(generator);
    double c = posterior_[0].weight;
    int i = 0;
    double U = r - 1.0 / kNumParticles_;
    float ratio = 0.5;
    for(int j = 0; j < kNumParticles_ * ratio; j++){
        U = U + 1.0 / kNumParticles_;
        while(U > c){
            i += 1;
            c += posterior_[i].weight;
        }
        prior.push_back(posterior_[i]);
    }
    for(int j = kNumParticles_ * ratio; j < kNumParticles_; j++){
        particle_t newParticle;
        newParticle.pose.x = posteriorPose_.x + 0.8 * dist_normal(generator);
        newParticle.pose.y = posteriorPose_.y + 0.8 * dist_normal(generator);
        newParticle.pose.theta = posteriorPose_.theta + 0.2 * dist_normal(generator);
        newParticle.weight = 1.0 / kNumParticles_;
        prior.push_back(newParticle);
    }

    /**
    std::random_device rd;
    std::mt19937 generator(rd());

    std::vector<particle_t> prior;
    
    std::uniform_int_distribution<int> particle_index(0, kNumParticles_ - 1);

    int current_index = particle_index(generator);

    float beta = 0.0;

    float max_weight = 0;
    for(int j = 0; j < kNumParticles_; j++){
        if(max_weight < posterior_[j].weight){
            max_weight = posterior_[j].weight;
        }
    }
    
    float max_weight_2 = 2 * max_weight;

    for (int i = 0; i < kNumParticles_; i++) {
        
        std::uniform_real_distribution<float> random_weight(0.0, max_weight_2);
        beta += random_weight(generator);

        while (beta > posterior_[current_index].weight) {
            beta -= posterior_[current_index].weight;
            current_index = (current_index + 1) % kNumParticles_;
        }..
        prior.push_back(posterior_[current_index]);
    }    
    */
    return prior;
}


std::vector<particle_t> ParticleFilter::computeProposalDistribution(const std::vector<particle_t>& prior)
{
    //////////// TODO: Implement your algorithm for creating the proposal distribution by sampling from the ActionModel
    std::vector<particle_t> proposal;
    for(auto& p : prior){
        proposal.push_back(actionModel_.applyAction(p));
    }
    return proposal;
}


std::vector<particle_t> ParticleFilter::computeNormalizedPosterior(const std::vector<particle_t>& proposal,
                                                                   const lidar_t& laser,
                                                                   const OccupancyGrid&   map)
{
    /////////// TODO: Implement your algorithm for computing the normalized posterior distribution using the 
    ///////////       particles in the proposal distribution
    std::vector<particle_t> posterior; //not the private member
    double sumWeights = 0.0;
    for(auto& p : proposal){
        particle_t weighted = p;
        weighted.weight = sensorModel_.likelihood(weighted, laser, map);
        sumWeights += weighted.weight;
        posterior.push_back(weighted);
    }

    for(auto& p : posterior){
        p.weight /= sumWeights;
    }

    return posterior;
}


pose_xyt_t ParticleFilter::estimatePosteriorPose(const std::vector<particle_t>& posterior)
{
    //////// TODO: Implement your method for computing the final pose estimate based on the posterior distribution
    //average of good poses set thereshold
    //K-mean-clustering
    //depends on the variance
    pose_xyt_t pose;
    // or mabe we can set a thereshold and dispose those below the value
    double x_mean = 0.0;
    double y_mean = 0.0;
    double cos_mean = 0.0;
    double sin_mean = 0.0;
    double weightSum = 0.0;
    
    std::vector<particle_t> particlesVector;
    for(auto p: posterior){
        particlesVector.push_back(p);
    }
    float ratio = 0.9;
    std::sort (particlesVector.begin(), particlesVector.end(), compareByWeight);
    for(int idx = 0; idx < ratio * kNumParticles_; idx++){
        
        x_mean += particlesVector[idx].weight * particlesVector[idx].pose.x;
        y_mean += particlesVector[idx].weight * particlesVector[idx].pose.y;
        cos_mean += particlesVector[idx].weight * cos(particlesVector[idx].pose.theta);
        sin_mean += particlesVector[idx].weight * sin(particlesVector[idx].pose.theta);
        weightSum += particlesVector[idx].weight;
    }
    if(weightSum == 0){
        std::cout<<"weightsum == 0 \n";
    }
    else{
        pose.x = x_mean / weightSum;
        pose.y = y_mean / weightSum;
        pose.theta = std::atan2(sin_mean, cos_mean);
    }

    return pose;
}
