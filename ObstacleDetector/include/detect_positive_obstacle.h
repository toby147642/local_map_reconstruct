/* Created  :   Ye Yuwen
 * Date     :   2018-09-21
 * Usage    :   define detect positive obstacle class
*/
#ifndef DETECT_POSITIVE_OBSTACLE_H
#define DETECT_POSITIVE_OBSTACLE_H

#include "alv_data.h"
#include <mutex>

static const int WATER_GRID_THRESHOLD = 60;
static const float POS_PRECENT = 0.3;
static const int grd_height_ths = 25;


class POSITIVE_DETECTOR
{
public:
    static POSITIVE_DETECTOR *p_instance;
    static POSITIVE_DETECTOR *get_instance();   // singleton mode
    void detect(ALV_DATA *alv_data);
    void mark_car_area(ALV_DATA *alv_data);
    void update_grids(ALV_DATA *alv_data, const alv_Point3f **pointcloud, const int BEAM_NUM, const int BEAM_POINTSIZE);
    void process_lidar4_data(ALV_DATA *alv_data);
    void process_lidar1_data(ALV_DATA *alv_data);
    void detect_obstacle_grid(ALV_DATA *alv_data);
    void detect_water_surface(ALV_DATA *alv_data);
    void remove_suspended_obs(ALV_DATA *alv_data); // 滤除悬空障碍物，height_ths是悬浮障碍物离地高度阈值

    void classify_dangerous_grid(ALV_DATA *alv_data);
    void filt_grid(ALV_DATA *alv_data, int win_size = 2, int ths = 1); // 在以障碍栅格为中心，(2*win_size+1)*(2*win_size+1)范围内正障碍栅格个数小于ths时，该障碍栅格会被滤除
    void filt_dangerous(ALV_DATA *alv_data);
    void retrieve_pos(ALV_DATA *alv_data);
    void classify_shadow_grid(ALV_DATA *alv_data);
    void ground_estimate(ALV_DATA *alv_data);

private:
    POSITIVE_DETECTOR();
    ~POSITIVE_DETECTOR();

    class GarbageRobot
    {
    public:
        ~GarbageRobot()
        {
            delete POSITIVE_DETECTOR::p_instance;
        }
    };
    static GarbageRobot grobot;     // used to delete p_instance automatically when whole program ends
};

#endif // DETECT_POSITIVE_OBSTACLE_H
