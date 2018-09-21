/*  Created :   YeYuwen
 *  Date    :   2018-09-21
 *  Usage   :   definition of negative obstacle detector class
*/
#include "negativeobstacle.h"
#include <iostream>
#include <strstream>
#include <vector>
#include <queue>
#include <opencv/cv.h>
#include <exception>
#include <algorithm>
#include <opencv2/highgui/highgui.hpp>
using namespace std;
using namespace cv;
using std::vector;

negativeObstacle *negativeObstacle::p_instance = nullptr;
static std::mutex n_lock;   // 线程锁，防止多个线程同时访问 negativeObstacle::p_instance

negativeObstacle *negativeObstacle::get_instance()
{
    if(p_instance == nullptr)
    {
        n_lock.lock();  // prevent multi-thread ambiguous
        if(p_instance == nullptr)
            p_instance = new negativeObstacle;
        n_lock.unlock();
    }
    return p_instance;
}

negativeObstacle::negativeObstacle(){}
negativeObstacle::~negativeObstacle(){}

void negativeObstacle::mark_car_area(ALV_DATA *alv_data)
{
    const PARA_TABLE &para = alv_data->para_table;// alv_data->para_table包含了指针申请的动态内存，所以要么用引用的形式，要么自定义=重载，否则形参析构时会将实参的动态内存释放掉！！！
    GRID **grids = alv_data->grid_map.grids;
    // mark car area
    for(int row = para.grid_center_row - (para.car_length/2)/para.grid_size;
            row <= para.grid_center_row + (para.car_length/2)/para.grid_size;
            row++)
    {
        for(int col = para.grid_center_col - (para.car_width/2)/para.grid_size;
                col <= para.grid_center_col + (para.car_width/2)/para.grid_size;
                col++)
        {
            grids[row][col].attribute = GRID_CAR_AREA;
            grids[row][col].known = true;
        }
    }
}

void negativeObstacle::detect_obstacle_grid(ALV_DATA *alv_data)
{
    mark_car_area(alv_data);
    update_grids(alv_data, (const alv_Point3f**)alv_data->lidar32_pointcloud, HDL32_BEAM_NUM, HDL32_BEAM_POINTSIZE);

    GRID **grids = alv_data->grid_map.grids;
    int Rows = alv_data->para_table.grid_rows;
    int Cols = alv_data->para_table.grid_cols;
    for(int row = 0; row < Rows; row++)
    {
        for(int col = 0; col < Cols; col++)
        {
            if( !grids[row][col].known || grids[row][col].attribute == GRID_CAR_AREA)
                continue;
            grids[row][col].attribute = GRID_TRAVESABLE;
        }
    }
}

void negativeObstacle::multiBeamdetect(ALV_DATA *alv_data){
    const int *angle_grid_num = alv_data->para_table.polar_table.angle_grid_num;
    PIXEL **polar_table = alv_data->para_table.polar_table.table;
    GRID **grids = alv_data->grid_map.grids;
    int Rows = alv_data->para_table.grid_rows;
    int Cols = alv_data->para_table.grid_cols;

    // occulusion
    for(int angle_cnt = 0; angle_cnt < POLAR_ANGLE_NUM; angle_cnt++){
        for (int i = 5; i < angle_grid_num[angle_cnt]-1; i++){
            int row = polar_table[angle_cnt][i].row;        //find the polar to grid
            int col = polar_table[angle_cnt][i].col;
            int row_next = polar_table[angle_cnt][i+1].row;        //find the polar to grid
            int col_next = polar_table[angle_cnt][i+1].col;

            if(grids[row_next][col_next].far_dis > grids[row][col].far_dis + NEG_RISE_FALL_THS_CM
                    && (grids[row_next][col_next].attribute == GRID_TRAVESABLE
                    || grids[row][col].attribute == GRID_TRAVESABLE)){
                grids[row][col].attribute = GRID_RISING_NEG_EDGE;

            }
            else if (grids[row_next][col_next].far_dis < grids[row][col].far_dis - NEG_RISE_FALL_THS_CM
                     && (grids[row_next][col_next].attribute == GRID_TRAVESABLE
                     || grids[row][col].attribute == GRID_TRAVESABLE)){
                grids[row_next][col_next].attribute = GRID_FALLING_NEG_EDGE;
            }
        }
    }

    //filter to expand the neg area
    int win_size = 2;
    int ths = 1;
    for(int row = win_size; row < Rows-win_size; row++)
    {
        for(int col = win_size; col < Cols-win_size; col++)
        {
            if(grids[row][col].attribute == GRID_TRAVESABLE)
            {
                int neg_cnt = 0;
                for(int i=row-win_size; i<=row+win_size; i++)
                {
                    for(int j = col-win_size; j <= col+win_size; j++)
                    {
                        if(grids[row][col].attribute == GRID_RISING_NEG_EDGE
                                || grids[row][col].attribute == GRID_FALLING_NEG_EDGE
                                || grids[row][col].attribute == GRID_NEG_OBS)
                            neg_cnt++;
                    }
                }
                if(neg_cnt >= ths)
                    grids[row][col].attribute = GRID_NEG_OBS;
            }
        }
    }
}

void negativeObstacle::multiBeamdetect_h(ALV_DATA *alv_data){
    const int *angle_grid_num = alv_data->para_table.polar_table.angle_grid_num;
    PIXEL **polar_table = alv_data->para_table.polar_table.table;
    GRID **grids = alv_data->grid_map.grids;
    int Rows = alv_data->para_table.grid_rows;
    int Cols = alv_data->para_table.grid_cols;

    // occulusion
    for(int angle_cnt = 0; angle_cnt < POLAR_ANGLE_NUM; angle_cnt++){
        for (int i = 5; i < angle_grid_num[angle_cnt]-1; i++){
            int row = polar_table[angle_cnt][i].row;        //find the polar to grid
            int col = polar_table[angle_cnt][i].col;
            int row_next = polar_table[angle_cnt][i+1].row;        //find the polar to grid
            int col_next = polar_table[angle_cnt][i+1].col;

            if(grids[row_next][col_next].far_dis > grids[row][col].far_dis + NEG_RISE_FALL_THS_CM
                    && (grids[row_next][col_next].attribute == GRID_TRAVESABLE
                    || grids[row][col].attribute == GRID_TRAVESABLE)){
                grids[row][col].attribute = GRID_RISING_NEG_EDGE;

            }
            else if (grids[row_next][col_next].far_dis < grids[row][col].far_dis - NEG_RISE_FALL_THS_CM
                     && (grids[row_next][col_next].attribute == GRID_TRAVESABLE
                     || grids[row][col].attribute == GRID_TRAVESABLE)){
                grids[row_next][col_next].attribute = GRID_FALLING_NEG_EDGE;
            }
        }
    }

    //filter to expand the neg area
    int win_size = 2;
    int ths = 1;
    for(int row = win_size; row < Rows-win_size; row++)
    {
        for(int col = win_size; col < Cols-win_size; col++)
        {
            if(grids[row][col].attribute == GRID_TRAVESABLE)
            {
                int neg_cnt = 0;
                for(int i=row-win_size; i<=row+win_size; i++)
                {
                    for(int j = col-win_size; j <= col+win_size; j++)
                    {
                        if(grids[row][col].attribute == GRID_RISING_NEG_EDGE
                                || grids[row][col].attribute == GRID_FALLING_NEG_EDGE
                                || grids[row][col].attribute == GRID_NEG_OBS)
                            neg_cnt++;
                    }
                }
                if(neg_cnt >= ths)
                    grids[row][col].attribute = GRID_NEG_OBS;
            }
        }
    }
}

void negativeObstacle::oneBeamdetect(ALV_DATA *alv_data, const alv_Point3f **pointcloud, const int BEAM_NUM, const int BEAM_POINTSIZE){
    const PARA_TABLE &para = alv_data->para_table;
    GRID **grids = alv_data->grid_map.grids;

    // detect each beam
    for(int beam = 0; beam < BEAM_NUM; beam++){
        bool flag = false;
//        bool flag_pt = false;
        for(int cnt = 0; cnt < BEAM_POINTSIZE-1; cnt++){
            const alv_Point3f *pt = (alv_Point3f *)pointcloud + beam*BEAM_POINTSIZE+cnt;
            if(pt->distance <1500 || !pt->valid)
                continue;
            const alv_Point3f *pt_next = nullptr;
            for(int lap = 15; lap < BEAM_POINTSIZE-cnt; lap++){
                pt_next = (alv_Point3f *)pointcloud + beam*BEAM_POINTSIZE + (cnt+lap);
                if(pt_next->distance <1500|| !pt_next->valid)
                    continue;
                break;
            }
            if(pt_next->distance <1500 || !pt_next->valid )
                continue;
            int row = (pt->y + para.map_range_rear)/para.grid_size;
            int col = (pt->x + para.map_range_left)/para.grid_size;
            if(!(row >= 0 && row < para.grid_rows && col >= 0 && col < para.grid_cols))
                continue;
            if(grids[row][col].attribute == GRID_CAR_AREA)
                continue;

            //there to be continue........
            if(pt->distance > pt_next->distance + NEG_RISE_FALL_THS_MM){
                /*if(grids[row][col].attribute == GRID_TRAVESABLE)*/
                if(grids[row][col].attribute != GRID_POS_OBS){
                    grids[row][col].attribute = GRID_FALLING_NEG_EDGE;
                    flag = !flag;
                }
            }
            else if(pt->distance < pt_next->distance - NEG_RISE_FALL_THS_MM){
                if(/*grids[row][col].attribute == GRID_TRAVESABLE
                        ||*/grids[row][col].attribute != GRID_POS_OBS || grids[row][col].attribute == GRID_FALLING_NEG_EDGE){
                    int row_next = (pt_next->y + para.map_range_rear)/para.grid_size;
                    int col_next = (pt_next->x + para.map_range_left)/para.grid_size;
                    if(!(row_next >= 0 && row_next < para.grid_rows && col_next >= 0 && col_next < para.grid_cols))
                        continue;
//                    cout<<row_next<<"   "<<col_next<<endl;
                    grids[row_next][col_next].attribute = GRID_RISING_NEG_EDGE;
                    flag = !flag;
                }
            }
        }
    }
}

void negativeObstacle::oneBeamdetect_z(ALV_DATA *alv_data, const alv_Point3f **pointcloud, const int BEAM_NUM, const int BEAM_POINTSIZE){
    const PARA_TABLE &para = alv_data->para_table;
    GRID **grids = alv_data->grid_map.grids;

    // detect each beam
    for(int beam = 0; beam < BEAM_NUM; beam++){
        bool flag = false;
//        bool flag_pt = false;
        for(int cnt = 0; cnt < BEAM_POINTSIZE-1; cnt++){
            const alv_Point3f *pt = (alv_Point3f *)pointcloud + beam*BEAM_POINTSIZE+cnt;
            if(pt->distance <1500 || !pt->valid)
                continue;
            const alv_Point3f *pt_next = nullptr;
            for(int lap = 5; lap < BEAM_POINTSIZE-cnt; lap++){
                pt_next = (alv_Point3f *)pointcloud + beam*BEAM_POINTSIZE + (cnt+lap);
                if(pt_next->distance <1500|| !pt_next->valid)
                    continue;
                break;
            }
            if(pt_next->distance <1500 || !pt_next->valid )
                continue;
            int row = (pt->y + para.map_range_rear)/para.grid_size;
            int col = (pt->x + para.map_range_left)/para.grid_size;
            if(!(row >= 0 && row < para.grid_rows && col >= 0 && col < para.grid_cols))
                continue;
            if(grids[row][col].attribute == GRID_CAR_AREA)
                continue;

            //there to be continue........
            if(pt->z > pt_next->z + NEG_Z_CM){
                /*if(grids[row][col].attribute == GRID_TRAVESABLE)*/
                if(grids[row][col].attribute != GRID_POS_OBS){
                    grids[row][col].attribute = GRID_FALLING_NEG_EDGE;
                    flag = !flag;
                }
            }
            else if(pt->z < pt_next->z - NEG_Z_CM){
                if(/*grids[row][col].attribute == GRID_TRAVESABLE
                        ||*/grids[row][col].attribute != GRID_POS_OBS || grids[row][col].attribute == GRID_FALLING_NEG_EDGE){
                    int row_next = (pt_next->y + para.map_range_rear)/para.grid_size;
                    int col_next = (pt_next->x + para.map_range_left)/para.grid_size;
                    if(!(row_next >= 0 && row_next < para.grid_rows && col_next >= 0 && col_next < para.grid_cols))
                        continue;
//                    cout<<row_next<<"   "<<col_next<<endl;
                    grids[row_next][col_next].attribute = GRID_RISING_NEG_EDGE;
                    flag = !flag;
                }
            }
        }
    }

    // expand the negative obstacle
    int Rows = alv_data->para_table.grid_rows;
    int Cols = alv_data->para_table.grid_cols;
    int win_size = 1;
    int ths = 2;
    for(int row = win_size; row < Rows-win_size; row++)
    {
        for(int col = win_size; col < Cols-win_size; col++)
        {
            if(grids[row][col].attribute == GRID_TRAVESABLE)
            {
                int neg_cnt = 0;
                for(int i=row-win_size; i<=row+win_size; i++)
                {
                    for(int j = col-win_size; j <= col+win_size; j++)
                    {
                        if(grids[i][j].attribute == GRID_RISING_NEG_EDGE
                                || grids[i][j].attribute == GRID_FALLING_NEG_EDGE)
                            neg_cnt+=2;
                        if(grids[i][j].attribute == GRID_NEG_OBS)
                            neg_cnt++;
                    }
                }
                if(neg_cnt >= ths)
                    grids[row][col].attribute = GRID_NEG_OBS;
            }
        }
    }
}

void negativeObstacle::oneBeamdetect_delta(ALV_DATA *alv_data, const alv_Point3f **pointcloud, const int BEAM_NUM, const int BEAM_POINTSIZE){
    const PARA_TABLE &para = alv_data->para_table;
    GRID **grids = alv_data->grid_map.grids;

    // detect each beam
    for(int beam = 0; beam < BEAM_NUM; beam++){
        int lap = 1;
        for(int cnt = 0; cnt < BEAM_POINTSIZE-1; cnt+=lap){
            lap = 1;
            const alv_Point3f *pt = (alv_Point3f *)pointcloud + beam*BEAM_POINTSIZE+cnt;
            if(pt->distance <1500 || !pt->valid)
                continue;
            const alv_Point3f *pt_next = nullptr;
            for(lap = 1; lap < BEAM_POINTSIZE-cnt; lap++){
                pt_next = (alv_Point3f *)pointcloud + beam*BEAM_POINTSIZE + (cnt+lap);
                if(pt_next->distance < 1500 || !pt_next->valid)
                    continue;
                break;
            }
            if(!pt_next->valid)
                continue;
            int row = (pt->y + para.map_range_rear)/para.grid_size;
            int col = (pt->x + para.map_range_left)/para.grid_size;
            if(!(row >= 0 && row < para.grid_rows && col >= 0 && col < para.grid_cols))
                continue;
            if(grids[row][col].attribute == GRID_CAR_AREA)
                continue;
            //there to be continue........
            float delta_angle = 0.0;
            if(delta_angle = pt_next->angleH - pt->angleH == 0.0 )
                continue;
            float delta_xy_dis = pt_next->xy_dis - pt->xy_dis;
            float grad = delta_xy_dis/delta_angle;

            if(grad < -10 && pt_next->z > pt->z+NEG_Z_CM){
                if(grids[row][col].attribute != GRID_POS_OBS){
//                    grids[row][col].attribute = GRID_FALLING_NEG_EDGE;
                    grids[row][col].attribute = GRID_CANDIDAT_NEG;
                }
            }
            else if(grad >10 && pt_next->z < pt->z-NEG_Z_CM){
                if(/*grids[row][col].attribute == GRID_TRAVESABLE
                        ||*/grids[row][col].attribute != GRID_POS_OBS
                        || grids[row][col].attribute == GRID_FALLING_NEG_EDGE
                        || grids[row][col].attribute == GRID_CANDIDAT_NEG){
                    int row_next = (pt_next->y + para.map_range_rear)/para.grid_size;
                    int col_next = (pt_next->x + para.map_range_left)/para.grid_size;
                    if(!(row_next >= 0 && row_next < para.grid_rows && col_next >= 0 && col_next < para.grid_cols))
                        continue;
                    grids[row_next][col_next].attribute = GRID_CANDIDAT_NEG;
                }
            }
        }
    }

    // expand the negative obstacle
    int Rows = alv_data->para_table.grid_rows;
    int Cols = alv_data->para_table.grid_cols;
    int win_size = 3;
//    int ths = 2;
    int ths = 1;
    for(int row = win_size; row < Rows-win_size; row++)
    {
        for(int col = win_size; col < Cols-win_size; col++)
        {
            if(grids[row][col].attribute == GRID_TRAVESABLE)
            {
                int neg_cnt = 0;
                for(int i=row-win_size; i<=row+win_size; i++)
                {
                    for(int j = col-win_size; j <= col+win_size; j++)
                    {
                        if(grids[i][j].attribute == GRID_RISING_NEG_EDGE
                                || grids[i][j].attribute == GRID_FALLING_NEG_EDGE)
                            neg_cnt+=2;
                        if(grids[i][j].attribute == GRID_NEG_OBS ||
                                grids[i][j].attribute == GRID_CANDIDAT_NEG)
                            neg_cnt++;
                    }
                }
                if(neg_cnt >= ths)
                    grids[row][col].attribute = GRID_CANDIDAT_NEG;
            }
        }
    }
}


void negativeObstacle::update_grids(ALV_DATA *alv_data, const alv_Point3f **pointcloud, const int BEAM_NUM, const int BEAM_POINTSIZE)
{
    const PARA_TABLE &para = alv_data->para_table;
    GRID **grids = alv_data->grid_map.grids;

    // update grids
    for(int beam = 0; beam < BEAM_NUM; beam++)
    {
        for(int cnt = 0; cnt < BEAM_POINTSIZE; cnt++)
        {
            const alv_Point3f *pt = (alv_Point3f *)pointcloud + beam*BEAM_POINTSIZE+cnt;

            if(!pt->valid)
                continue;
            int row = (pt->y + para.map_range_rear)/para.grid_size;
            int col = (pt->x + para.map_range_left)/para.grid_size;

            if(!(row >= 0 && row < para.grid_rows && col >= 0 && col < para.grid_cols))
                continue;
            if(grids[row][col].attribute == GRID_CAR_AREA)
                continue;

            if(grids[row][col].known == false)
            {
                grids[row][col].known = true;
                grids[row][col].max_height = grids[row][col].min_height = pt->z;
                grids[row][col].ground_height = grids[row][col].min_height;
                grids[row][col].far_dis = sqrt(pow(pt->x,2)+pow(pt->y,2)+pow(pt->z,2));
                grids[row][col].total_dis = grids[row][col].far_dis;
            }
            else
            {
                grids[row][col].max_height = pt->z > grids[row][col].max_height? pt->z:grids[row][col].max_height;
                grids[row][col].min_height = pt->z < grids[row][col].min_height? pt->z:grids[row][col].min_height;
                grids[row][col].ground_height = grids[row][col].min_height;
                float tmp = sqrt(pow(pt->x,2)+pow(pt->y,2)+pow(pt->z,2));
                grids[row][col].total_dis += tmp;
                grids[row][col].far_dis = tmp > grids[row][col].far_dis? tmp:grids[row][col].far_dis;
            }
            grids[row][col].ptNum++;
            grids[row][col].points.push_back(*pt);
            if(pt->intensity > 0 && pt->intensity < para.weak_flatness_threshold)
                grids[row][col].weak_intensity_cnt++;
        }
    }

    for(int row=0; row<para.grid_rows; row++)
    {
        for(int col=0; col<para.grid_cols; col++)
        {
            grids[row][col].dis_height = grids[row][col].max_height - grids[row][col].min_height;
            if(grids[row][col].ptNum != 0)
                grids[row][col].ave_dis = grids[row][col].total_dis / grids[row][col].ptNum;
        }
    }

}

void negativeObstacle::reflect(ALV_DATA *alv_data){

}

void negativeObstacle::maxWindowFilter(const alv_Point3f **pointcloud, alv_Point3f **max_pointcloud, const int BEAM_NUM, const int BEAM_POINTSIZE){
    for(int beam = 0; beam < BEAM_NUM; beam++){
        for(int cnt = 1; cnt < BEAM_POINTSIZE-1; cnt++){
            const alv_Point3f *pt1 = (alv_Point3f *)pointcloud + beam*BEAM_POINTSIZE+cnt-1;
            const alv_Point3f *pt2 = (alv_Point3f *)pointcloud + beam*BEAM_POINTSIZE+cnt;
            const alv_Point3f *pt3 = (alv_Point3f *)pointcloud + beam*BEAM_POINTSIZE+cnt+1;
            alv_Point3f *pt_tmp = (alv_Point3f *)max_pointcloud + beam*BEAM_POINTSIZE+cnt;

            if(!pt1->valid || !pt2->valid || !pt3->valid)
                continue;

            if(pt1->distance > pt2->distance && pt1->distance > pt3->distance){
                memcpy(pt_tmp, pt1, sizeof(alv_Point3f));
            }

            else if(pt2->distance > pt1->distance && pt2->distance > pt3->distance){
                memcpy(pt_tmp, pt2, sizeof(alv_Point3f));
            }
            else{
                memcpy(pt_tmp, pt3, sizeof(alv_Point3f));
            }
//            cout << "pt_tmp: "<< pt_tmp->distance<<endl;
        }
        // 0 && BEAM_POINTSIZE-1
        alv_Point3f *pt_tmp1 = (alv_Point3f *)max_pointcloud + beam*BEAM_POINTSIZE;
        alv_Point3f *pt_tmp2 = (alv_Point3f *)pointcloud + beam*BEAM_POINTSIZE;
        if(!pt_tmp1->valid || !pt_tmp2->valid)
            continue;
        memcpy(pt_tmp1, pt_tmp2, sizeof(alv_Point3f));
        alv_Point3f *pt_tmp3 = (alv_Point3f *)max_pointcloud + beam*BEAM_POINTSIZE + BEAM_POINTSIZE-1;
        alv_Point3f *pt_tmp4 = (alv_Point3f *)pointcloud + beam*BEAM_POINTSIZE + BEAM_POINTSIZE-1;
        if(!pt_tmp3->valid || !pt_tmp4->valid)
            continue;
        memcpy(pt_tmp3, pt_tmp4, sizeof(alv_Point3f));
    }
}

void negativeObstacle::diff_beam_dis(ALV_DATA *alv_data){
    const PARA_TABLE &para = alv_data->para_table;
    GRID **grids = alv_data->grid_map.grids;

    for(int i=0; i < ANGLE_NUM; i++)
        for(int j=BEGIN_BEAM; j<END_BEAM; j++){
            if(abs(alv_data->beam_dis_max_mm[i][j]<250))
                continue;
            // part 1:
            int x0 = (int)alv_data->angle_beam_dis_max_mm[i][j]/10 * para.lidar32_inpara.cos_angleV[para.lidar32_inpara.beam_order[j]]*para.lidar32_inpara.sin_angleH[i*100];
            int y0 = (int)alv_data->angle_beam_dis_max_mm[i][j]/10 * para.lidar32_inpara.cos_angleV[para.lidar32_inpara.beam_order[j]]*para.lidar32_inpara.cos_angleH[i*100];

            int row = (x0 + para.map_range_rear)/para.grid_size;
            int col = (y0 + para.map_range_left)/para.grid_size;

            if(!(row >= 0 && row < para.grid_rows && col >= 0 && col < para.grid_cols))
                continue;
//            if(grids[row][col].attribute == GRID_CAR_AREA || grids[row][col].attribute == GRID_POS_OBS)
            if(grids[row][col].attribute != GRID_CANDIDAT_NEG)
                continue;
            grids[row][col].attribute = GRID_NEG_OBS;
            // part 2:
            int x1 = (int)alv_data->angle_beam_dis_max_mm[i][j-1]/10 * para.lidar32_inpara.cos_angleV[para.lidar32_inpara.beam_order[j-1]]*para.lidar32_inpara.sin_angleH[i*100];
            int y1 = (int)alv_data->angle_beam_dis_max_mm[i][j-1]/10 * para.lidar32_inpara.cos_angleV[para.lidar32_inpara.beam_order[j-1]]*para.lidar32_inpara.cos_angleH[i*100];

            int row_next = (x1 + para.map_range_rear)/para.grid_size;
            int col_next = (y1 + para.map_range_left)/para.grid_size;

            if(!(row_next >= 0 && row_next < para.grid_rows && col_next >= 0 && col_next < para.grid_cols))
                continue;
//            if(grids[row_next][col_next].attribute == GRID_CAR_AREA || grids[row_next][col_next].attribute == GRID_POS_OBS)
            if(grids[row_next][col_next].attribute != GRID_CANDIDAT_NEG)
                continue;
            grids[row_next][col_next].attribute = GRID_NEG_OBS;
        }
}
void negativeObstacle::retrieve_neg(ALV_DATA *alv_data){
    GRID **grids = alv_data->grid_map.grids;
    int Rows = alv_data->para_table.grid_rows;
    int Cols = alv_data->para_table.grid_cols;
    const PARA_TABLE &para = alv_data->para_table;// alv_data->para_table包含了指针申请的动态内存，所以要么用引用的形式，要么自定义=重载，否则形参析构时会将实参的动态内存释放掉！！！

    int win = 2;
    int ths = 2;

    // retrieve neg from pos+neg
    for(int row=win; row<Rows-win; row++){
        for(int col=win; col<Cols-win; col++){
            if(grids[row][col].attribute != GRID_POS_OBS)
                continue;

            int cnt;
            // too high filter
            cnt= 0;
            float ave_arround_max_height = 0.0;
            for(int i=row-win; i<=row+win; i++){
                for(int j=col-win; j<=col+win; j++){
                    if(!grids[i][j].known ||
                            (i==row && j==col)||
                            grids[i][j].attribute == GRID_RETRIEVED_POS ||
                            grids[i][j].attribute == GRID_SUSPEND_OBS ||
                            grids[i][j].attribute == GRID_SHADOW ||
                            grids[i][j].attribute == GRID_POS_OBS)
                        continue;
                    cnt++;
                    ave_arround_max_height +=grids[i][j].max_height;
                }
            }
            if(cnt!=0){
                if(grids[row][col].dis_height > alv_data->para_table.pos_obs_threshold)
                    continue;
                if(grids[row][col].max_height-ave_arround_max_height/cnt>alv_data->para_table.pos_obs_threshold)
                    continue;
            }


            // neg retrieve
            cnt= 0;
            float ave_arround_min_height = 0.0;
            for(int i=row-win; i<=row+win; i++){
                for(int j=col-win; j<=col+win; j++){
                    if(!grids[i][j].known ||
                            (i==row && j==col)||
                            grids[i][j].attribute == GRID_RETRIEVED_POS ||
                            grids[i][j].attribute == GRID_SHADOW ||
                            grids[i][j].attribute == GRID_POS_OBS)
                        continue;
                    cnt++;
                    ave_arround_min_height +=grids[i][j].min_height;
                }
            }
            if(cnt!=0){
                ave_arround_min_height/=cnt;
                int dis2car = sqrt(pow((double)row-para.grid_center_row,2)+pow((double)col-para.grid_center_col,2))*para.grid_size;
                if(grids[row][col].min_height-ave_arround_min_height<NEG_THS && dis2car < 1000)
                    grids[row][col].attribute = GRID_NEG_OBS;
            }
        }
    }

    // retrieve pos from neg
    for(int row=win; row<=Rows-win; row++){
        for(int col=win; col<=Cols-win; col++){
            int neig = 0;
            // neg->pos because of wrong
            if(grids[row][col].attribute != GRID_NEG_OBS)
                continue;

            for(int i=row-win; i<=row+win; i++){
                for(int j=col-win; j<=col+win; j++){
                    if(!grids[i][j].known ||(i==row && j==col))
                        continue;
                    if(grids[i][j].attribute == GRID_POS_OBS ||
                            grids[i][j].attribute == GRID_RETRIEVED_POS ||
                            grids[i][j].attribute == GRID_SUSPEND_OBS ||
                            grids[i][j].attribute == GRID_SHADOW)
                        neig++;
                }
            }

            if(neig > ths || grids[row][col].far_dis >1200)
                grids[row][col].attribute = GRID_POS_OBS;

        }
    }
}


void negativeObstacle::neg_complete(ALV_DATA *alv_data){
    GRID **grids = alv_data->grid_map.grids;
    int Rows = alv_data->para_table.grid_rows;
    int Cols = alv_data->para_table.grid_cols;

    int win = 3;

    for(int row = win; row < Rows-win;  row++){
        for(int col = win; col < Cols-win; col++){

            if(grids[row][col].attribute != GRID_NEG_OBS)
                continue;

            if(row == 59){
                for(int j = col-1; j>=col-win; j--){
                    if(
                            grids[row][j].attribute == GRID_UNKNOWN ||
                            !grids[row][j].known ||
                            grids[row][j].attribute == GRID_TRAVESABLE ||
                            grids[row][j].attribute == GRID_SUSPEND_OBS ||
                            grids[row][j].attribute == GRID_CANDIDAT_NEG ||
                            grids[row][j].attribute == GRID_ROAD_EDGE  ||
                            grids[row][j].attribute == GRID_SHADOW
                            )
                        grids[row][j].attribute = GRID_FALLING_NEG_EDGE;
                }
            }
            else if(col == 79 && row <59){
                for(int j=row; j<=row+win;j++){
                    if(grids[j][col].attribute == GRID_UNKNOWN ||
                            !grids[j][col].known ||
                            grids[j][col].attribute == GRID_TRAVESABLE ||
                            grids[j][col].attribute == GRID_SUSPEND_OBS ||
                            grids[j][col].attribute == GRID_CANDIDAT_NEG ||
                            grids[j][col].attribute == GRID_ROAD_EDGE  ||
                            grids[j][col].attribute == GRID_SHADOW
                            )
                        grids[j][col].attribute = GRID_FALLING_NEG_EDGE;
                }
            }
            else if(col == 79 && row >59){
                for(int j=row; j>=row-win;j--){
                    if(grids[j][col].attribute == GRID_UNKNOWN ||
                            !grids[j][col].known ||
                            grids[j][col].attribute == GRID_TRAVESABLE ||
                            grids[j][col].attribute == GRID_SUSPEND_OBS ||
                            grids[j][col].attribute == GRID_CANDIDAT_NEG ||
                            grids[j][col].attribute == GRID_ROAD_EDGE  ||
                            grids[j][col].attribute == GRID_SHADOW
                            )
                        grids[j][col].attribute = GRID_FALLING_NEG_EDGE;
                }
            }
            else{
                // begining
                for(int i=row-win; i<=row+win; i++){
                    for(int j=col-win; j<=col+win ;j++){
                        if(i==row && j==col)
                            continue;
                        if(abs(i-59)+abs(j-79) > abs(row-59)+abs(col-79))
                            continue;

                        if(i == row){
                            if(  (1.0*row-i)*(1.0*col-79)/((1.0*col-j)*(1.0*row-50))<=1.25 &&
                                 (1.0*row-i)*(1.0*col-79)/((1.0*col-j)*(1.0*row-50))>=0.8 &&
                                   ( grids[i][j].attribute == GRID_UNKNOWN ||
                                    !grids[i][j].known ||
                                    grids[i][j].attribute == GRID_TRAVESABLE ||
                                    grids[i][j].attribute == GRID_SUSPEND_OBS ||
                                     grids[i][j].attribute == GRID_CANDIDAT_NEG ||
                                     grids[i][j].attribute == GRID_ROAD_EDGE  ||
                                     grids[i][j].attribute == GRID_SHADOW
                                     )
                                    )
                                grids[i][j].attribute = GRID_FALLING_NEG_EDGE;
                        }else{
                            float flop = (1.0*col-j)*(1.0*row-50)/((1.0*row-i)*(1.0*col-79));
                            if( flop <=1.25 && flop >= 0.8 &&
                                ( grids[i][j].attribute == GRID_UNKNOWN ||
                                 !grids[i][j].known ||
                                 grids[i][j].attribute == GRID_TRAVESABLE ||
                                 grids[i][j].attribute == GRID_SUSPEND_OBS ||
                                  grids[i][j].attribute == GRID_CANDIDAT_NEG ||
                                  grids[i][j].attribute == GRID_ROAD_EDGE  ||
                                  grids[i][j].attribute == GRID_SHADOW
                                  )
                                    )
                                grids[i][j].attribute = GRID_FALLING_NEG_EDGE;
                        }
    //ending

                    }
                }
            }
        }
    }

}

void negativeObstacle::classify_occlusion_grid(ALV_DATA *alv_data)
{
    const int *angle_grid_num = alv_data->para_table.polar_table.angle_grid_num;
    PIXEL **polar_table = alv_data->para_table.polar_table.table;
    GRID **grids = alv_data->grid_map.grids;
    const float lidar_height = alv_data->para_table.lidar32_expara.T[2];

    // occulusion
    for(int angle_cnt = 0; angle_cnt < POLAR_ANGLE_NUM; angle_cnt++){
        int grid_shadowtop = 0;
        //statistic from the fifth polar_grid
        for (int i = 5; i < angle_grid_num[angle_cnt]-1; i++){
            int row = polar_table[angle_cnt][i].row;
            int col = polar_table[angle_cnt][i].col;

            if (grids[row][col].attribute == GRID_POS_OBS || grids[row][col].attribute == GRID_RETRIEVED_POS){
                //calculate shadow top and update
                int grid_height = grids[row][col].max_height;
                int top;
                if(lidar_height > grid_height)
                    top = (int)((lidar_height * i) / (lidar_height - grid_height)); //calculate the farest position, cm
                else
                    top = angle_grid_num[angle_cnt];    //shadow is the farest point
                if (top > grid_shadowtop)
                    grid_shadowtop = top;
            }
            else if(i <= grid_shadowtop && (grids[row][col].attribute== GRID_TRAVESABLE
                                            ||grids[row][col].attribute ==GRID_UNKNOWN
                                            ||grids[row][col].attribute ==GRID_SHADOW))
                grids[row][col].attribute = GRID_OCCULUSION;
        }
    }
}


void negativeObstacle::detect(ALV_DATA *alv_data){
    retrieve_neg(alv_data);
    neg_complete(alv_data);
    classify_occlusion_grid(alv_data);// 标记阴影区、危险区域
}
