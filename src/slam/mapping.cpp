#include <slam/mapping.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <common/grid_utils.hpp>
#include <numeric>


Mapping::Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds)
: kMaxLaserDistance_(maxLaserDistance)
, kHitOdds_(hitOdds)
, kMissOdds_(missOdds)
{
}


void Mapping::updateMap(const lidar_t& scan, const pose_xyt_t& pose, OccupancyGrid& map)
{
    if(!initialized_){
        previousPose_ = pose;
    }
    
    MovingLaserScan movingScan(scan, previousPose_, pose);
    for(auto& ray : movingScan){
        scoreEndpoint(ray, map);
        scoreRay(ray, map);
    }

    initialized_ = true;
    previousPose_ = pose;

}

void Mapping::scoreEndpoint(const adjusted_ray_t& ray, OccupancyGrid& map){
    if(ray.range < kMaxLaserDistance_){
        Point<float> rayStart = global_position_to_grid_cell(ray.origin, map);
        Point<int> rayCell;

        rayCell.x = static_cast<int>(ray.range * std::cos(ray.theta) * map.cellsPerMeter() + rayStart.x);
        rayCell.y = static_cast<int>(ray.range * std::sin(ray.theta) * map.cellsPerMeter() + rayStart.y);

        if(map.isCellInGrid(rayCell.x, rayCell.y)){
            increaseCellOdds(rayCell.x, rayCell.y, map);
        }
    }
}

void Mapping::scoreRay(const adjusted_ray_t& ray, OccupancyGrid& map){
    if(ray.range < kMaxLaserDistance_){
        Point<float> rayStart = global_position_to_grid_cell(ray.origin, map);
        Point<int> rayEnd;
        Point<int> rayBegin;
        rayEnd.x = static_cast<int>(ray.range * std::cos(ray.theta) * map.cellsPerMeter() + rayStart.x);
        rayEnd.y = static_cast<int>(ray.range * std::sin(ray.theta) * map.cellsPerMeter() + rayStart.y);
        rayBegin.x = static_cast<int>(rayStart.x);
        rayBegin.y = static_cast<int>(rayStart.y);
        Point<int> rayCell;
        int dx = std::abs(rayEnd.x - rayBegin.x);
        int dy = std::abs(rayEnd.y - rayBegin.y);
        int sx = rayEnd.x > rayBegin.x ? 1 : -1;
        int sy = rayEnd.y > rayBegin.y ? 1 : -1;
        int err = (dx > dy ? dx : -dy) / 2;
        int e2 = 0;
        rayCell.x = rayBegin.x;
        rayCell.y = rayBegin.y;
        //std::cout<<"++++++++rayBegin.x:"<<rayBegin.x<<"    rayBegin.y:"<<rayBegin.y<<"++++++++\n";
        while(rayCell.x != rayEnd.x || rayCell.y != rayEnd.y){
            //std::cout<<"--------rayCell.x:"<<rayCell.x<<"    rayCell.y:"<<rayCell.y<<"--------\n";
            decreaseCellOdds(rayCell.x, rayCell.y, map);
            e2 = err;
            if(e2 >= -dx){
                err -= dy;
                rayCell.x += sx;
            }
            if(e2 < dy){
                err += dx;
                rayCell.y += sy;
            }
        }
        //std::cout<<"********rayEnd.x:"<<rayEnd.x<<"    rayEnd.y:"<<rayEnd.y<<"********\n\n";
    }
}

void Mapping::decreaseCellOdds(int x, int y, OccupancyGrid& map){
    map(x, y) = (map(x, y) - kMissOdds_ < -127 ? -127 : map(x, y) - kMissOdds_);
}

void Mapping::increaseCellOdds(int x, int y, OccupancyGrid& map){
    map(x, y) = (map(x, y) + kHitOdds_ > 127 ? 127 : map(x, y) + kHitOdds_);
}
