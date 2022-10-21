#include <slam/sensor_model.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/grid_utils.hpp>


SensorModel::SensorModel(void)
{
    ///////// TODO: Handle any initialization needed for your sensor model
    portion = 1.0 / 2.0;
}


double SensorModel::likelihood(const particle_t& sample, const lidar_t& scan, const OccupancyGrid& map)
{
    ///////////// TODO: Implement your sensor model for calculating the likelihood of a particle given a laser scan //////////
    //very simple one we have to update!!!
    // laser range larger than maxinmun or small than minimun should be ignored
    const int xDeltas[8] = {1, 1, 0, -1, -1, -1, 0, 1};
    const int yDeltas[8] = {0, 1, 1, 1, 0, -1, -1, -1};
    MovingLaserScan movingScan(scan, sample.parent_pose, sample.pose);
    double scanScore = 0.0;
    for(auto& ray : movingScan){
        Point<double> endpoint(ray.origin.x + ray.range * std::cos(ray.theta), 
                                ray.origin.y + ray.range * std::sin(ray.theta));
        auto rayBegin = global_position_to_grid_position(ray.origin, map);
        auto rayEnd = global_position_to_grid_position(endpoint, map);
        if(map.logOdds(rayEnd.x, rayEnd.y) > 0.0){
            //+= map.logOdds(rayEnd.x, rayEnd.y);
            scanScore += map.logOdds(rayEnd.x, rayEnd.y);
        }
        
        else {
            
            for (int i = 0; i < 8; i++){
                if(map.logOdds(rayEnd.x + xDeltas[i], rayEnd.y + yDeltas[i]) > 0.0){
                    scanScore += 0.1 * map.logOdds(rayEnd.x + xDeltas[i], rayEnd.y + yDeltas[i]);
                }
            }
            
            /*
            int dx = std::abs(rayEnd.x - rayBegin.x);
            int dy = std::abs(rayEnd.y - rayBegin.y);
            int sx = rayEnd.x < rayBegin.x ? 1 : -1;
            int sy = rayEnd.y < rayBegin.y ? 1 : -1;
            int err = dx - dy;
            int x_after = rayEnd.x;
            int x_before = rayEnd.x;
            int y_after = rayEnd.y;
            int y_before = rayEnd.y;
            int e2 = 2*err;
            if(e2 >= -dy){
                err -= dy;
                x_before = rayEnd.x + sx;
                x_after = rayEnd.x - sx;
            }
            if(e2 <= dx){
                err += dx;
                y_before = rayEnd.y + sy;
                y_after = rayEnd.y - sy;
            }
            if(map.logOdds(x_before,y_before) > 0.0){
                scanScore += 0.5 * map.logOdds(x_before,y_before);
            } 
            // odd_after
            if(map.isCellInGrid(x_after, y_after)){
                if(map.logOdds(x_after, y_after) > 0.0){
                    scanScore += 0.5 * map.logOdds(x_after, y_after);
                }
            }
            */         
        }
    }

    return scanScore;
}

