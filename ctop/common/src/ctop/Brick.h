//
// Created by Michal Němec on 11/03/2020.
//

#ifndef CTOP_PLANNER_BRICK_H
#define CTOP_PLANNER_BRICK_H


#include <ctop/log.h>

#include <array>
#include <ctop/util/has_unique_id.h>

#include "pt2eigen.h"

#include "Plane.h"
#include "WallInfo.h"

namespace ctop {

struct BrickWalls {
    enum WallPos : int {
        LEFT = 0, RIGHT, FRONT, BACK, BOTTOM, TOP,  MAX
    };

    static std::string wall_pos2string_set(const std::set<int>& poss) {
        int i = 0;
        std::ostringstream oss;
        for(auto s : poss) {
            auto str = wall_pos2string(s);
            oss << str;
            ++i;
            if(i != poss.size()) {
                oss << " | ";
            }
        }
        return oss.str();
    }

    static std::string wall_pos2string(int pos) {
        if(pos < 0 || pos >= static_cast<int>(WallPos::MAX)) {
            return "UNKNOWN";
        }
        switch(pos) {
            case LEFT: return "LEFT";
            case TOP: return "TOP";
            case RIGHT: return "RIGHT";
            case FRONT: return "FRONT";
            case BACK: return "BACK";
            case BOTTOM: return "BOTTOM";
            default:
                return "UNKNOWN";
        }
    }
    std::array<Plane, 6> walls;
};



struct BrickPlacement {
    unsigned int type = 0;
    int duration = 0;
    int reservoir_time = 0;
};

struct BrickPlacementData {
    std::map<int, BrickPlacement> brick_2_data;
};

struct Brick : has_unique_id<Brick> {

    enum CornerPos : int {
        LEFT_BOTTOM = 0, LEFT_TOP, RIGHT_TOP, RIGHT_BOTTOM
    };

    int layer = 0;

    double x = std::numeric_limits<double>::quiet_NaN();
    double y = std::numeric_limits<double>::quiet_NaN();
    double z = std::numeric_limits<double>::quiet_NaN();

    bool yaw_rotated = false;
    int id = 0;
    std::string name;

    int reward = 0;
    int req_robots = 0;

    unsigned int type = 0;

    BrickWalls walls{};
    std::array<ctop::Point3D, 4> corners;

    template<typename T>
    static T string_to(const std::string &val) {
        T v;
        std::istringstream iss{val};
        auto valid = static_cast<bool>(iss >> v);
        if(!valid) {
            CTOP_LOG_D("failed on {}", val);
        }
        CTOP_ASSERT(valid);
        return v;
    }

    static Brick from_string_vector(ctop::WallInfo &info, const std::vector<std::string> &vals) {

        /*
         * #brick_id brick_type brick_x_pos brick_y_pos brick_z_pos brick_yaw_rotated
         * 1 3 0.72 0.10 0.10 0
         */

        auto brick_id = string_to<int>(vals[0]);
        auto brick_type = string_to<int>(vals[1]);
        auto brick_x_pos = string_to<double>(vals[2]);
        auto brick_y_pos = string_to<double>(vals[3]);
        auto brick_z_pos = string_to<double>(vals[4]);
        auto brick_yaw_rotated = string_to<int>(vals[5]) == 1;

        Brick b;
        b.x = brick_x_pos;
        b.y = brick_y_pos;
        b.z = brick_z_pos;
        b.yaw_rotated = brick_yaw_rotated;
        b.id = brick_id;
        b.type = brick_type;

        b.name = info.brick_ids_map_reversed[brick_type];
        b.reward = info.brick_rewards_map[b.name];
        b.req_robots = info.brick_num_robot_requirements_map[b.name];
        b.corners = b.get_corners(info);
        b.walls = b.get_walls(info);

        return b;
    }

    std::array<ctop::Point3D, 4> get_corners(ctop::WallInfo &info) {
        std::array<ctop::Point3D, 4> corners;
        CTOP_LOG_D("brick_node.type={}", type);
        const auto &brick_type = info.brick_ids_map_reversed[type];

        CTOP_LOG_D("brick_type={}", brick_type);
        const auto &brick_l_h_d_cm = info.brick_size_length_depth_height_cm_map[brick_type];

        CTOP_LOG_D("dimensions readed {}", brick_l_h_d_cm.size());
        int x_index = 0;
        int y_index = 1;
        int z_index = 2;

        if (yaw_rotated) {
            x_index = 1;
            y_index = 0;
        }

        //PRINT_DEF("center of "<<brick_node.id<<" is "<<brick_node.x<<" "<<brick_node.y);
        //PRINT_DEF("yaw_rotated "<<brick_node.yaw_rotated<<" x_index "<<x_index<<" y_index "<<y_index);
        //PRINT_DEF("brick_l_h_d_cm[x_index] "<<brick_l_h_d_cm[x_index]<< " brick_l_h_d_cm[y_index] "<<brick_l_h_d_cm[y_index])

        const auto &width = brick_l_h_d_cm[x_index];
        const auto &height = brick_l_h_d_cm[y_index];

        auto width_half_m = width / 200.0; // is /2 /100
        auto height_half_m = height / 200.0;

        // corner map
        // x-y plane
        //    --x-->
        //   |  2 --------- 1  ---
        //   |  |           |   |
        //   y  |   (mid)   | height
        //   |  |           |   |
        //      3 --------- 4  ---
        //      |---width---|

        ctop::Point3D corner1{};
        corner1.x = x + width_half_m; // is /2 /100
        corner1.y = y + height_half_m;
        corner1.z = z;
        corners[CornerPos::RIGHT_TOP] = corner1;
        CTOP_LOG_D("corner1 [{}, {}]", corner1.x, corner1.y);

        ctop::Point3D corner2{};
        corner2.x = x - width_half_m;
        corner2.y = y + height_half_m;
        corner2.z = z;
        corners[CornerPos::LEFT_TOP] = corner2;
        CTOP_LOG_D("corner2 [{}, {}]", corner2.x, corner2.y);

        ctop::Point3D corner3{};
        corner3.x = x - width_half_m;
        corner3.y = y - height_half_m;
        corner3.z = z;
        corners[CornerPos::LEFT_BOTTOM] = corner3;
        CTOP_LOG_D("corner3 [{}, {}]", corner3.x, corner3.y);

        ctop::Point3D corner4{};
        corner4.x = x + width_half_m;
        corner4.y = y - height_half_m;
        corner4.z = z;
        corners[CornerPos::RIGHT_BOTTOM] = corner4;
        CTOP_LOG_D("corner4 [{}, {}]", corner4.x, corner4.y);
        return corners;
    }

    BrickWalls get_walls(ctop::WallInfo &info) {
        BrickWalls w;

        const auto &brick_type = info.brick_ids_map_reversed[type];
        const auto &brick_l_h_d_cm = info.brick_size_length_depth_height_cm_map[brick_type];
        auto depth_2_m = brick_l_h_d_cm[2] / 200.0;

        auto corners_top = get_corners(info);
        auto corners_bottom = corners_top;

        for (int i = 0; i < corners_top.size(); i++) {
            auto &pt_top = corners_top[i];
            auto &pt_bottom = corners_bottom[i];

            pt_top.z += depth_2_m;
            pt_bottom.z -= depth_2_m;
        }

        {
            auto &plane = w.walls[BrickWalls::FRONT];
            plane.p0 = pt2eigen(corners_bottom[LEFT_BOTTOM]);
            plane.w1 = pt2eigen(corners_bottom[RIGHT_BOTTOM]) - plane.p0;
            plane.w2 = pt2eigen(corners_top[LEFT_BOTTOM]) - plane.p0;
        }

        {
            auto &plane = w.walls[BrickWalls::BACK];
            plane.p0 = pt2eigen(corners_bottom[LEFT_TOP]);
            plane.w1 = pt2eigen(corners_bottom[RIGHT_TOP]) - plane.p0;
            plane.w2 = pt2eigen(corners_top[LEFT_TOP]) - plane.p0;
        }

        {
            auto &plane = w.walls[BrickWalls::LEFT];
            plane.p0 = pt2eigen(corners_bottom[LEFT_BOTTOM]);
            plane.w1 = pt2eigen(corners_bottom[LEFT_TOP]) - plane.p0;
            plane.w2 = pt2eigen(corners_top[LEFT_BOTTOM]) - plane.p0;
        }

        {
            auto &plane = w.walls[BrickWalls::RIGHT];
            plane.p0 = pt2eigen(corners_bottom[RIGHT_BOTTOM]);
            plane.w1 = pt2eigen(corners_bottom[RIGHT_TOP]) - plane.p0;
            plane.w2 = pt2eigen(corners_top[RIGHT_BOTTOM]) - plane.p0;
        }

        {
            auto &plane = w.walls[BrickWalls::TOP];
            plane.p0 = pt2eigen(corners_top[LEFT_BOTTOM]);
            plane.w1 = pt2eigen(corners_top[RIGHT_BOTTOM]) - plane.p0;
            plane.w2 = pt2eigen(corners_top[LEFT_TOP]) - plane.p0;
        }

        {
            auto &plane = w.walls[BrickWalls::BOTTOM];
            plane.p0 = pt2eigen(corners_bottom[LEFT_BOTTOM]);
            plane.w1 = pt2eigen(corners_bottom[RIGHT_BOTTOM]) - plane.p0;
            plane.w2 = pt2eigen(corners_bottom[LEFT_TOP]) - plane.p0;
        }

        return w;
    }

    void print() {
        CTOP_LOG_D("id={} type={} pos=[{:2}, {:2}, {:2}] yaw={}", id, type, x, y, z, yaw_rotated);
    }

    static bool
    isPointInside(double px, double py, const std::array<ctop::Point3D, 4>& corners) {
        int count = 0;
        // Cast a horizontal ray and see how many times it intersects all the edges
        auto sz = corners.size();
        for (int i = 0; i < sz; ++i) {

            const auto& corner1 = corners[i];
            const auto& corner2 = corners[(i + 1) % sz];

            double v1x = corner1.x;
            double v1y = corner1.y;
            double v2x = corner2.x;
            double v2y = corner2.y;

            if (v1y > py && v2y > py) continue;
            if (v1y <= py && v2y <= py) continue;

            double intersect_x = (v1y == v2y) ? v1x : (py - v1y) * (v2x - v1x) / (v2y - v1y) + v1x;
            bool does_intersect = intersect_x > px;
            if (does_intersect) {
                ++count;
            }
        }

        return count % 2;
    }

    static bool is_on_top_of_each_other(Brick* b1, Brick* b2) {
        if(b1 == nullptr || b2 == nullptr) return false;
        for(auto&& c_top : b2->corners) {
            if(isPointInside(c_top.x, c_top.y, b1->corners)) {
                return true;
            }
        }
        return false;
    }

    static double brick_distance(Brick* b1, Brick* b2) {
        if(b1 == nullptr || b2 == nullptr) return std::numeric_limits<double>::max();

        auto dx = b1->x - b2->x;
        auto dy = b1->y - b2->y;
        auto dz = b1->z - b2->z;

        return std::sqrt(dx*dx+dy*dy+dz*dz);
    }
};



}


#endif //CTOP_PLANNER_BRICK_H