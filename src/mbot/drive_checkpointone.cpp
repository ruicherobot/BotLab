#include <common/lcm_config.h>
#include <mbot/mbot_channels.h>
#include <lcmtypes/robot_path_t.hpp>
#include <lcm/lcm-cpp.hpp>
#include <iostream>
#include <unistd.h>
#include <cmath>

int main(int argc, char** argv)
{
    
    robot_path_t path;
        
    pose_xyt_t p;
    for (int i =0; i < 4; i++) {
        p.x = 0;
        p.y = 0;
        path.path.push_back(p);
        for (int j = 0; j < 3; j++) {
            p.x = .5*j;
            p.y = 0;
            path.path.push_back(p);
        }
        for (int j = 0; j < 3; j++) {
            p.x = 1;
            p.y = .5*j;
            path.path.push_back(p);
        }
        for (int j = 0; j < 3; j++) {
            p.x = 1-.5*j;
            p.y = 1;
            path.path.push_back(p);
        }
        for (int j = 0; j < 3; j++) {
            p.x = 0;
            p.y = 1-.5*j;
            path.path.push_back(p);
        }
    }
    p.x = 0.05;
    p.y = 0.0;
    for (int i = 0; i < 4; i++){
        path.path.push_back(p);
    }
    
    path.path_length = path.path.size();
    
    lcm::LCM lcmInstance(MULTICAST_URL);
	std::cout << "publish to: " << CONTROLLER_PATH_CHANNEL << std::endl;
    lcmInstance.publish(CONTROLLER_PATH_CHANNEL, &path);
    sleep(1);

    return 0;
}
