#ifndef NEGATIVEOBSTACLE_H
#define NEGATIVEOBSTACLE_H

#include "alv_data.h"
#include <mutex>

static const int NEG_RISE_FALL_THS_CM = 60;
static const int NEG_RISE_FALL_THS_MM = 300;
static const int NEG_DENSE_THS = 1;
static const int NEG_Z_CM = 10;
static const int BEGIN_BEAM = 4;
static const int END_BEAM = 12;


class negativeObstacle
{
public:



    static negativeObstacle *p_instance;
    static negativeObstacle *get_instance();   // singleton mode
    void detect(ALV_DATA *alv_data);
    void mark_car_area(ALV_DATA *alv_data);
    void update_grids(ALV_DATA *alv_data, const alv_Point3f **pointcloud, const int BEAM_NUM, const int BEAM_POINTSIZE);
    void multiBeamdetect(ALV_DATA *alv_data);
    void oneBeamdetect(ALV_DATA *alv_data, const alv_Point3f **pointcloud, const int BEAM_NUM, const int BEAM_POINTSIZE);
    void oneBeamdetect_z(ALV_DATA *alv_data, const alv_Point3f **pointcloud, const int BEAM_NUM, const int BEAM_POINTSIZE);
    void oneBeamdetect_delta(ALV_DATA *alv_data, const alv_Point3f **pointcloud, const int BEAM_NUM, const int BEAM_POINTSIZE);
    void multiBeamdetect_h(ALV_DATA *alv_data);
    void detect_obstacle_grid(ALV_DATA *alv_data);
    void reflect(ALV_DATA *alv_data);
    void maxWindowFilter(const alv_Point3f **pointcloud, alv_Point3f **max_pointcloud, const int BEAM_NUM, const int BEAM_POINTSIZE);
    void diff_beam_dis(ALV_DATA *alv_data);
    //    void morphCluster(alv_data);

private:
    negativeObstacle();
    ~negativeObstacle();

    class GarbageRobot
    {
    public:
        ~GarbageRobot()
        {
            delete negativeObstacle::p_instance;
        }
    };
    static GarbageRobot grobot;     // used to delete p_instance automatically when whole program ends
};

#endif // NEGATIVEOBSTACLE_H
