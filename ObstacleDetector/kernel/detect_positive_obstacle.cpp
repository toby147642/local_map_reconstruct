/* Created  :   Linhui
 * Date     :   2016-08-05
 * Usage    :
*/
#include "detect_positive_obstacle.h"
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <algorithm>
#include <iostream>
using std::cout;
using std::endl;
using std::vector;

using namespace cv;

POSITIVE_DETECTOR * POSITIVE_DETECTOR::p_instance = nullptr;
static std::mutex n_lock;   // 线程锁，防止多个线程同时访问 POSITIVE_DETECTOR::p_instance

POSITIVE_DETECTOR * POSITIVE_DETECTOR::get_instance()
{
    if(p_instance == nullptr)
    {
        n_lock.lock();  // prevent multi-thread ambiguous
        if(p_instance == nullptr)
            p_instance = new POSITIVE_DETECTOR;
        n_lock.unlock();
    }
    return p_instance;
}

POSITIVE_DETECTOR::POSITIVE_DETECTOR(){}
POSITIVE_DETECTOR::~POSITIVE_DETECTOR(){}


void POSITIVE_DETECTOR::mark_car_area(ALV_DATA *alv_data)
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

void POSITIVE_DETECTOR::retrieve_pos(ALV_DATA *alv_data){
    int rows = alv_data->para_table.grid_rows;
    int cols = alv_data->para_table.grid_cols;
    int grid_size = alv_data->para_table.grid_size;
    GRID ** lidar_grids = alv_data->grid_map.grids;

    // binary the obstacle image
    Mat img = Mat::zeros(rows, cols, CV_8U);
    for (int row = 0; row < rows; row++) {
        uint8_t* di = img.ptr<uint8_t>(row);
        for (int col = 0; col < cols; col++){
            if (lidar_grids[row][col].attribute == GRID_POS_OBS){
                di[col] = 1;
            }
        }
    }

    // find the contours
    vector< vector<Point> > contours;
    findContours(img, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    int n = contours.size();



    cv::Size dsize = cv::Size(2*cols, 2*rows);
    Mat dresult = Mat::zeros(dsize, CV_8UC1);
    drawContours(img, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    for(int i=0; i<n; i++){
        vector<Point> &contour = contours[i];
        int r_first = contour[0].y;
        int c_first = contour[0].x;
        for(int j=0; j<(int)contour.size(); j++){
            int r = contour[j].y;
            int c = contour[j].x;
            cv::line(img, cv::Point(c_first, r_first), cv::Point(c, r), (i+1)*10, 1);
            r_first = r;
            c_first = c;
            img.at<uchar>(r,c) = 255;//(i+1)*10;
        }
    }
    for(int i=0; i<rows; i++)
        for(int j=0; j<cols; j++)
            if(lidar_grids[i][j].attribute == GRID_CAR_AREA)
                img.at<uchar>(i,j) = 255;


    resize(img, dresult, dsize);
    //imshow("allala", dresult);

    // contour analysis
    vector<Point> center(n);
    vector<float> min_height(n, FLT_MAX);
//    vector<float> real_min_height(n, FLT_MAX);
    vector<float> max_height(n, FLT_MIN);
    vector<float> max_height_dis(n, FLT_MIN);
    vector<int> left(n, cols);
    vector<int> right(n, 0);
    vector<int> top(n, rows);
    vector<int> bottom(n, 0);

    for (int i=0; i<n; ++i) {
        vector<Point>& contour = contours[i];
        for (int j=0; j<(int)contour.size(); ++j) {
            int row = contour[j].y;
            int col = contour[j].x;

            // find the bounding rectangle for each contour
            if (row < top[i])
                top[i] = row;
            else if (row > bottom[i])
                bottom[i] = row;
            if (col < left[i])
                left[i] = col;
            else if (col > right[i])
                right[i] = col;

            // calculate the center for each contour
            center[i].x += col;
            center[i].y += row;
            if (lidar_grids[row][col].known) {
                // find the min_height for each contour (thought to be ground)
                if (lidar_grids[row][col].min_height < min_height[i])
                    min_height[i] = lidar_grids[row][col].min_height;
//                if (obs_ft.real_min_height < real_min_height[i])
//                    real_min_height[i] = obs_ft.real_min_height;
                if (lidar_grids[row][col].max_height > max_height[i])
                    max_height[i] = lidar_grids[row][col].max_height;
                if (lidar_grids[row][col].dis_height > max_height_dis[i])
                    max_height_dis[i] = lidar_grids[row][col].dis_height;
            }
        }
        if (n != 0) {
            center[i].x /= contour.size();
            center[i].y /= contour.size();
        }
    }

    // retrieve the flat obstacle near true obstacle from detect_obstacle_grid
    for (int i=0; i<n; ++i) {
        if (fabs(max_height[i]-min_height[i]) > 300.f || fabs(max_height[i]-min_height[i]) < 50.f ||
            max(right[i]-left[i], bottom[i]-top[i])*grid_size > 1000.f)
            continue;
        float search_range = 25.f;
        int s = cvRound(search_range / grid_size);
        int r_min = max(0, top[i]-s);
        int r_max = min(rows-1, bottom[i]+s);
        int c_min = max(0, left[i]-s);
        int c_max = min(cols-1, right[i]+s);
        for (int r=r_min; r<=r_max; ++r) {
            for (int c=c_min; c<=c_max; ++c) {
                if (/*lidar_grids[r][c].known && */lidar_grids[r][c].attribute != GRID_POS_OBS) {
                    if (lidar_grids[r][c].min_height >= min_height[i]+5 &&
                            (lidar_grids[r][c].attribute == GRID_SHADOW/*||lidar_grids[r][c].attribute == GRID_NEG_OBS*/)){
//                        int aa = 1;
                        lidar_grids[r][c].attribute = GRID_RETRIEVED_POS;
                    }

                }
            }
        }
    }
}


void POSITIVE_DETECTOR::update_grids(ALV_DATA *alv_data, const alv_Point3f **pointcloud, const int BEAM_NUM, const int BEAM_POINTSIZE)
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
                grids[row][col].beam_min = para.lidar32_inpara.order_beam[beam];
                grids[row][col].beam_max = para.lidar32_inpara.order_beam[beam];

            }
            else
            {
                grids[row][col].max_height = pt->z > grids[row][col].max_height? pt->z:grids[row][col].max_height;
                grids[row][col].min_height = pt->z < grids[row][col].min_height? pt->z:grids[row][col].min_height;
                grids[row][col].ground_height = grids[row][col].min_height;
                float tmp = sqrt(pow(pt->x,2)+pow(pt->y,2)+pow(pt->z,2));
                grids[row][col].total_dis += tmp;
                grids[row][col].far_dis = tmp > grids[row][col].far_dis? tmp:grids[row][col].far_dis;
                grids[row][col].beam_min = grids[row][col].beam_min<para.lidar32_inpara.order_beam[beam]?grids[row][col].beam_min:para.lidar32_inpara.order_beam[beam];
                grids[row][col].beam_max = grids[row][col].beam_max>para.lidar32_inpara.order_beam[beam]?grids[row][col].beam_max:para.lidar32_inpara.order_beam[beam];
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
            if(grids[row][col].beam_min ==-1)
                grids[row][col].beam_dis = 0;
            else grids[row][col].beam_dis = grids[row][col].beam_max - grids[row][col].beam_min + 1;

            if(row==0 || row==para.grid_rows-1 || col==0 || col==para.grid_cols-1){
                alv_data->neig_grid_map_beam_max[row][col] = grids[row][col].beam_max;
                alv_data->neig_grid_map_beam_min[row][col] = grids[row][col].beam_min;
            } else {
                int max_beam = -2;
                int min_beam = 100;
              for(int i=row-1 ; i<=row+1; i++)
                  for(int j=col-1;j<=col+1;j++){
                      max_beam = max_beam>grids[i][j].beam_max?max_beam:grids[i][j].beam_max;
                      if(grids[i][j].beam_min >=0)
                          min_beam = min_beam<grids[i][j].beam_min?min_beam:grids[i][j].beam_min;
                  }
              alv_data->neig_grid_map_beam_max[row][col] = max_beam;
              if(min_beam==100)
                  alv_data->neig_grid_map_beam_min[row][col] = -1;
              else alv_data->neig_grid_map_beam_min[row][col] = min_beam;
            }
            if(grids[row][col].ptNum != 0)
                grids[row][col].ave_dis = grids[row][col].total_dis / grids[row][col].ptNum;
        }
    }

/*
    // z axis is not available? if it is a positive obstacle?
    for(int row=0; row<para.grid_rows; row++)
    {
        for(int col=0; col<para.grid_cols; col++)
        {
            vector<float> grid_x;
            vector<float> grid_y;
            vector<float> grid_z;
            for(int i=0; i<grids[row][col].points.size();i++){
                if(!grids[row][col].points[i].valid)
                    continue;
                grid_x.push_back(grids[row][col].points[i].x);
                grid_y.push_back(grids[row][col].points[i].y);
                grid_z.push_back(grids[row][col].points[i].z);
            }

            if(int ptsnum = grid_x.size()>=2){
                float sum_x = std::accumulate(grid_x.begin(), grid_x.end(), 0.0);
                float mean_x =  sum_x / ptsnum; //均值
                float sum_y = std::accumulate(grid_y.begin(), grid_y.end(), 0.0);
                float mean_y =  sum_y / ptsnum; //均值
                float sum_z = std::accumulate(grid_z.begin(), grid_z.end(), 0.0);
                float mean_z =  sum_z / ptsnum; //均值

                float accum_x  = 0.0;
                std::for_each (std::begin(grid_x), std::end(grid_x), [&](const float d) {
                    accum_x  += (d-mean_x)*(d-mean_x);
                });
                grids[row][col].stdev_x = sqrt(accum_x/(ptsnum-1)); //方差

                float accum_y  = 0.0;
                std::for_each (std::begin(grid_y), std::end(grid_y), [&](const float d) {
                    accum_y  += (d-mean_y)*(d-mean_y);
                });
                grids[row][col].stdev_y = sqrt(accum_y/(ptsnum-1)); //方差

                float accum_z  = 0.0;
                std::for_each (std::begin(grid_z), std::end(grid_z), [&](const float d) {
                    accum_z  += (d-mean_z)*(d-mean_z);
                });
                grids[row][col].stdev_z = sqrt(accum_z/(ptsnum-1)); //方差

            }else{
                grids[row][col].stdev_x = 0;
                grids[row][col].stdev_y = 0;
                grids[row][col].stdev_z = 0;
            }
        }
    }
    */

}


void POSITIVE_DETECTOR::process_lidar1_data(ALV_DATA *alv_data)
{
    PARA_TABLE &para = alv_data->para_table;
    GRID **grids = alv_data->grid_map.grids;

    // process LIDAR 1 data
    for(int row=0;row<para.grid_rows;row++){
        for(int col=0;col<para.grid_cols;col++){
            if (grids[row][col].attribute == GRID_CAR_AREA)
                continue;
            if(alv_data->lidar1_grids[row][col] == 255){
                grids[row][col].known = true;
                grids[row][col].attribute = GRID_POS_OBS;
                double old = (double)grids[row][col].dis_height;
                grids[row][col].dis_height =std::max(50.0,old);// lidar1 default height 50cm
            }
        }
    }
}


void POSITIVE_DETECTOR::remove_suspended_obs(ALV_DATA *alv_data) // 滤除悬空障碍物
{
    const PARA_TABLE &para = alv_data->para_table;
    GRID **grids = alv_data->grid_map.grids;
    int Rows = alv_data->para_table.grid_rows;
    int Cols = alv_data->para_table.grid_cols;

    // 估计地面高度
    const int estimate_grid_front = 1500/para.grid_size;
    const int estimate_grid_rear = 1000/para.grid_size;
    const int estimate_grid_left = 500/para.grid_size;
    const int estimate_grid_right = 500/para.grid_size;
    int win_size = 2;
    for(int row = para.grid_center_row-estimate_grid_rear; row<=para.grid_center_row+estimate_grid_front; row++)
    {
        for(int col = para.grid_center_col-estimate_grid_left; col<=para.grid_center_col+estimate_grid_right; col++)
        {
            if(grids[row][col].attribute == GRID_CAR_AREA || grids[row][col].attribute == GRID_TRAVESABLE)
                continue;

            int validNum = 0;
            float neighbor_height = 0.0;
            for(int i = row-win_size; i <= row+win_size; i++)
            {
                for(int j = col-win_size; j <= col+win_size; j++)
                {
                    if(grids[i][j].attribute == GRID_CAR_AREA)
                        continue;
                    if(grids[i][j].attribute == GRID_TRAVESABLE ||
                            (grids[i][j].attribute != GRID_TRAVESABLE &&
                             grids[i][j].update_ground))
                    {
                        neighbor_height += grids[i][j].ground_height;
                        validNum++;
                    }
                }
            }
            if(validNum)
            {
                grids[row][col].ground_height = neighbor_height/(float)validNum;
                grids[row][col].update_ground = true;
            }
        }
    }

    // 标记悬空障碍物栅格
    for(int row = 0; row < Rows; row++)
    {
        for(int col = 0; col < Cols; col++)
        {
            if(grids[row][col].attribute == GRID_POS_OBS/* && grids[row][col].dis_height > (float)para.suspend_obs_ths*/)
            {
                // 障碍栅格内点云根据z轴按递增排序
                sort(grids[row][col].points.begin(), grids[row][col].points.end(), [](const alv_Point3f &p1, const alv_Point3f &p2)->bool{return p1.z < p2.z;});
				// 初始化prev_height为ground_height，然后依次检查后一个点是否比前一个点高度差超过阈值，若是，则标记为悬空障碍物
                float prev_height = grids[row][col].ground_height;
                for(int i=0; i<grids[row][col].points.size(); i++)
                {
                    if(grids[row][col].points[i].z - prev_height > (float)para.suspend_obs_ths)
                    {
                        grids[row][col].attribute = GRID_SUSPEND_OBS;
                        break;
                    }
                    prev_height = grids[row][col].points[i].z;
                }
            }
        }
    }

	// 再次过滤一部分障碍物栅格
    win_size = 2;
    int ths = 1;
    for(int row = win_size; row < Rows-win_size; row++)
    {
        for(int col = win_size; col < Cols-win_size; col++)
        {
            if(grids[row][col].attribute == GRID_POS_OBS)
            {
                int susp_cnt = 0;
                for(int i=row-win_size; i<=row+win_size; i++)
                {
                    for(int j = col-win_size; j <= col+win_size; j++)
                    {
                        if(grids[row][col].attribute == GRID_SUSPEND_OBS)
                            susp_cnt++;
                    }
                }
                if(susp_cnt >= ths)
                    grids[row][col].attribute = GRID_SUSPEND_OBS;
            }
        }
    }
}


void POSITIVE_DETECTOR::classify_occlusion_grid(ALV_DATA *alv_data)
{
    const int *angle_grid_num = alv_data->para_table.polar_table.angle_grid_num;
    PIXEL **polar_table = alv_data->para_table.polar_table.table;
    GRID **grids = alv_data->grid_map.grids;
    int Rows = alv_data->para_table.grid_rows;
    int Cols = alv_data->para_table.grid_cols;
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


void POSITIVE_DETECTOR::classify_neg_occlusion_grid(ALV_DATA *alv_data)
{
    const int *angle_grid_num = alv_data->para_table.polar_table.angle_grid_num;
    PIXEL **polar_table = alv_data->para_table.polar_table.table;
    GRID **grids = alv_data->grid_map.grids;
    int Rows = alv_data->para_table.grid_rows;
    int Cols = alv_data->para_table.grid_cols;
    const float lidar_height = alv_data->para_table.lidar32_expara.T[2];

    // occulusion
    for(int angle_cnt = 0; angle_cnt < POLAR_ANGLE_NUM; angle_cnt++){
        //statistic from the fifth polar_grid
        for (int i = 5; i < angle_grid_num[angle_cnt]-1; i++){
            int row = polar_table[angle_cnt][i].row;
            int col = polar_table[angle_cnt][i].col;

            if (grids[row][col].attribute == GRID_NEG_OBS){
                //calculate shadow top and update
                int grid_height = abs(grids[row][col].min_height);
                int top;
                if(lidar_height > grid_height)
                    top = (int)((grid_height * i) / (lidar_height + grid_height)); //calculate the farest position, cm
                if(top>=i)
                    continue;
                for(int j=i; j>=top; j--){
                    grids[row][col].attribute = GRID_NEG_OBS;
                }

            }

        }
    }
}

void POSITIVE_DETECTOR::classify_shadow_grid(ALV_DATA *alv_data)
{
    const int *angle_grid_num = alv_data->para_table.polar_table.angle_grid_num;
    PIXEL **polar_table = alv_data->para_table.polar_table.table;
    GRID **grids = alv_data->grid_map.grids;
    int Rows = alv_data->para_table.grid_rows;
    int Cols = alv_data->para_table.grid_cols;
    const float lidar_height = alv_data->para_table.lidar32_expara.T[2];

    // occulusion
    for(int angle_cnt = 0; angle_cnt < POLAR_ANGLE_NUM; angle_cnt++){
        int grid_shadowtop = 0;
        //statistic from the fifth polar_grid
        for (int i = 5; i < angle_grid_num[angle_cnt]-1; i++){
            int row = polar_table[angle_cnt][i].row;
            int col = polar_table[angle_cnt][i].col;

            if (grids[row][col].attribute == GRID_POS_OBS){
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
            else if(i <= grid_shadowtop && (grids[row][col].attribute== GRID_TRAVESABLE ||grids[row][col].attribute ==GRID_UNKNOWN))
                grids[row][col].attribute = GRID_SHADOW;
        }
    }
}

void POSITIVE_DETECTOR::classify_dangerous_grid(ALV_DATA *alv_data)
{
    GRID **grids = alv_data->grid_map.grids;
    int Rows = alv_data->para_table.grid_rows;
    int Cols = alv_data->para_table.grid_cols;

    // dangerous
    for(int row = 0; row < Rows; row++)
    {
        for(int col = 0; col < Cols; col++)
        {
            if(grids[row][col].attribute != GRID_UNKNOWN)
            {
                int beam = alv_data->para_table.grid_2_polar_beam[row][col];
                int angle = alv_data->para_table.grid_2_polar_angle[row][col];
                alv_data->polar_grids[beam][angle].valid++;
            }
        }
    }

    int win_size = 1;
    for(int beam = win_size; beam < POLAR_BEAM_NUM_2-win_size; beam++)
    {
        for(int angle = win_size; angle < POLAR_ANGLE_NUM_2-win_size; angle++)
        {
            int validCnt = 0;
            for(int i = beam-win_size; i <= beam + win_size; i++)
            {
                for(int j = angle-win_size; j <= angle + win_size; j++)
                {
                    if(alv_data->polar_grids[i][j].valid != 0)
                        validCnt++;
                }
            }
            if(validCnt > 5)
                alv_data->polar_grids[beam][angle].valid = 1;
        }
    }
    filt_dangerous(alv_data);
}

void POSITIVE_DETECTOR::filt_dangerous(ALV_DATA *alv_data)
{
    int Rows = alv_data->para_table.grid_rows;
    int Cols = alv_data->para_table.grid_cols;
    int area_ths = alv_data->para_table.dangerous_area_threshold;
    Mat dangerous = Mat::zeros(Rows, Cols, CV_8U);
    for(int row = 0; row < Rows; row++)
    {
        uint8_t* di = dangerous.ptr<uint8_t>(row);
        for(int col = 0; col < Cols; col++)
        {
            if(row >= alv_data->para_table.grid_center_row - 600/ alv_data->para_table.grid_size &&
                    row <= alv_data->para_table.grid_center_row + 600/ alv_data->para_table.grid_size &&
                    col >= alv_data->para_table.grid_center_col - 600/ alv_data->para_table.grid_size &&
                    col <= alv_data->para_table.grid_center_col + 600/ alv_data->para_table.grid_size)
                continue;
            int beam = alv_data->para_table.grid_2_polar_beam[row][col];
            int angle = alv_data->para_table.grid_2_polar_angle[row][col];
            if(alv_data->polar_grids[beam][angle].valid == 0)
                di[col] = 255;
        }
    }

//    flip(dangerous, dangerous, 0);
//    imshow("dangerous", dangerous);
//    waitKey(1);

    // find contours
    vector<vector<Point> > contours;
    findContours(dangerous, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    int n = contours.size();

    //find big area
    Mat img_tmp = Mat::zeros(Rows, Cols, CV_8U);
    Scalar color( 255, 255, 255);
    for (int i=0; i<n; ++i) {
        vector<Point>& contour = contours[i];
        int m = contour.size();
        double area = contourArea(contour, false);
        if(area > area_ths){
            int mark = 0;
            for(int j = 0; j < m; j++){
                if((contour[j].x-75)*(contour[j].x-75) + (contour[j].y-75-12)*(contour[j].y-75-12) < 150*150)
                    mark = 1;
            }
            if(mark == 1)
                drawContours( img_tmp, contours, i, color, CV_FILLED);
        }

    }

    GRID_ATTRIBUTE **known = new GRID_ATTRIBUTE*[Rows];
    for(int i=0; i<Rows; i++)
        known[i] = new GRID_ATTRIBUTE[Cols];
    for(int row = 0; row < Rows; row++){
        for(int col = 0; col < Cols; col++){
            known[row][col] = alv_data->grid_map.grids[row][col].attribute;
        }
    }

    for(int row = 0; row < Rows; row ++){
        uint8_t* di = img_tmp.ptr<uint8_t>(row);
        for(int col = 0; col < Cols; col ++){
            if(di[col] == 255)
                alv_data->grid_map.grids[row][col].attribute = GRID_DANGEROUS;
                //int a =1;
        }
    }

//    // 将被抹掉的非危险区域重新填入
//    for(row = 0; row < row_max; row ++){
//        for(col = 0; col < col_max; col ++){
//            if(known[row][col] != 4)
//                lidar_grids[row][col].grid_prop_feature.known = known[row][col];
//        }
//    }

    // free memory
    for(int i=0; i<Rows; i++)
        delete[] known[i];
    delete[] known;
}

void POSITIVE_DETECTOR::filt_grid(ALV_DATA *alv_data, int win_size, int ths)
{
    GRID **grids = alv_data->grid_map.grids;
    int Rows = alv_data->para_table.grid_rows;
    int Cols = alv_data->para_table.grid_cols;
    for(int row = win_size; row<Rows-win_size; row++)
    {
        for(int col = win_size; col<Cols-win_size; col++)
        {
            if(grids[row][col].attribute != GRID_POS_OBS)
                continue;

            int obs_cnt = 0;
            for(int i=row-win_size; i<=row+win_size; i++)
            {
                for(int j=col-win_size; j<=col+win_size; j++)
                {
                    if(grids[i][j].attribute == GRID_POS_OBS)
                        obs_cnt++;
                }
            }
            if(obs_cnt <= ths)
                grids[row][col].attribute = GRID_TRAVESABLE;
        }
    }
}


void POSITIVE_DETECTOR::detect_obstacle_grid(ALV_DATA *alv_data)
{
    mark_car_area(alv_data);
    update_grids(alv_data, (const alv_Point3f**)alv_data->lidar32_pointcloud, HDL32_BEAM_NUM, HDL32_BEAM_POINTSIZE);
//    update_grids(alv_data, (const alv_Point3f**)alv_data->lidar16_pointcloud_L, VLP16_BEAM_NUM, VLP16_BEAM_POINTSIZE);
//    update_grids(alv_data, (const alv_Point3f**)alv_data->lidar16_pointcloud_R, VLP16_BEAM_NUM, VLP16_BEAM_POINTSIZE);
//    update_grids(alv_data, (const alv_Point3f**)alv_data->lidar4_pointcloud, LIDAR4_BEAM_NUM, LIDAR4_BEAM_POINTSIZE);

    GRID **grids = alv_data->grid_map.grids;
    int Rows = alv_data->para_table.grid_rows;
    int Cols = alv_data->para_table.grid_cols;
    for(int row = 0; row < Rows; row++)
    {
        for(int col = 0; col < Cols; col++)
        {
            if( !grids[row][col].known || grids[row][col].attribute == GRID_CAR_AREA)
                continue;
            if(grids[row][col].dis_height >= alv_data->para_table.neg_obs_threshold)        //dis_height higher than threshold
            {
                grids[row][col].attribute = GRID_POS_OBS;
                if(row>=alv_data->para_table.grid_center_row
                   && row <= alv_data->para_table.grid_center_row+7
                   && col >= alv_data->para_table.grid_center_col-7
                   && col <= alv_data->para_table.grid_center_col+7
                   && grids[row][col].dis_height < alv_data->para_table.near_obstacle_threshold)
                    grids[row][col].attribute = GRID_TRAVESABLE;
            }/*else if(grids[row][col].dis_height >= alv_data->para_table.neg_obs_threshold){
                grids[row][col].attribute = GRID_NEG_OBS;
                if(row>=alv_data->para_table.grid_center_row
                   && row <= alv_data->para_table.grid_center_row+7
                   && col >= alv_data->para_table.grid_center_col-7
                   && col <= alv_data->para_table.grid_center_col+7
                   && grids[row][col].dis_height < alv_data->para_table.near_obstacle_threshold)
                    grids[row][col].attribute = GRID_TRAVESABLE;
            }*/
            else
                grids[row][col].attribute = GRID_TRAVESABLE;
        }
    }

//#ifndef OFF_LINE
//    process_lidar1_data(alv_data);
//#endif

    // single point filter
    // can't detect the neg is largely because of this
    filt_grid(alv_data, 1, 1);
}


void POSITIVE_DETECTOR::detect_water_surface(ALV_DATA *alv_data)
{
    GRID **grids = alv_data->grid_map.grids;
    int Rows = alv_data->para_table.grid_rows;
    int Cols = alv_data->para_table.grid_cols;
    for(int row = 0; row<Rows; row++)
    {
        for(int col = 0; col<Cols; col++)
        {
            if(row<=alv_data->para_table.grid_center_row + 40
                && row >= alv_data->para_table.grid_center_row
                && col >= alv_data->para_table.grid_center_col-20
                && col <= alv_data->para_table.grid_center_col+20
                && grids[row][col].attribute == GRID_TRAVESABLE)
            {
                if(grids[row][col].weak_intensity_cnt >= WATER_GRID_THRESHOLD)
                    grids[row][col].attribute = GRID_WATER;
            }
        }
    }
    int win_size = 2;
    for(int row = alv_data->para_table.grid_center_row + 10; row <= alv_data->para_table.grid_center_row+35; row++)
    {
        for(int col = alv_data->para_table.grid_center_col-7; col <= alv_data->para_table.grid_center_col+7; col++)
        {
            int neighbor = 0;
            if(grids[row][col].attribute != GRID_WATER)
                continue;
            for(int i=row-win_size; i<=row+win_size; i++)
            {
                for(int j = col-win_size; j <= col+win_size; j++)
                {
                    if(grids[i][j].attribute == GRID_WATER)
                        neighbor++;
                }
            }
            if(neighbor < 3)
                grids[row][col].attribute = GRID_TRAVESABLE;
        }
    }
}

void POSITIVE_DETECTOR::retrieve_neg(ALV_DATA *alv_data){
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
                    int aaa = 1;
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


//  couldn't work
void POSITIVE_DETECTOR::expand_neg(ALV_DATA *alv_data, const alv_Point3f **pointcloud, const int BEAM_POINTSIZE){
    const int *angle_grid_num = alv_data->para_table.polar_table.angle_grid_num;
    PIXEL **polar_table = alv_data->para_table.polar_table.table;
    GRID **grids = alv_data->grid_map.grids;
    int Rows = alv_data->para_table.grid_rows;
    const PARA_TABLE &para = alv_data->para_table;
    int Cols = alv_data->para_table.grid_cols;


    bool flag1 = false;
    bool flag2 = false;
    for(int row=0; row<Rows; row++){
        for(int col=0; col<Cols; col++){
            if(grids[row][col].attribute != GRID_NEG_OBS)
                continue;
//            bool flag_neg = false;

            int row_begin1 = -1;
            int col_begin1 = -1;
            int row_begin2 = -1;
            int col_begin2 = -1;
            int row_end1 = -1;
            int col_end1 = -1;
            int row_end2 = -1;
            int col_end2 = -1;
            if(grids[row][col].beam_min ==-1 && grids[row][col].beam_max ==-1)
                continue;
            int beam_now = grids[row][col].beam_min ==-1 ? grids[row][col].beam_max:grids[row][col].beam_min;
//            cout <<endl;
//            cout << beam_now <<endl;

            for(int cnt = 0; cnt < BEAM_POINTSIZE-2; cnt++){
                const alv_Point3f *pt = (alv_Point3f *)pointcloud + alv_data->para_table.lidar32_inpara.beam_order[beam_now]*BEAM_POINTSIZE+cnt;
//                cout <<endl;
//                cout<<grids[row][col].beam_min<<"   "<<alv_data->para_table.lidar32_inpara.beam_order[grids[row][col].beam_min]<<endl;
                if(!pt->valid)
                    continue;
                int _row = (pt->y + para.map_range_rear)/para.grid_size;
                int _col = (pt->x + para.map_range_left)/para.grid_size;
                if(_row<0 || _row>=Rows || _col<=0||_col>=Cols)
                    continue;
                int row_next = -1;
                int col_next = -1;

                const alv_Point3f *pt_next = nullptr;
                for(int cnt_next = cnt+1; cnt_next<BEAM_POINTSIZE;cnt_next++){
                    pt_next = (alv_Point3f *)pointcloud + alv_data->para_table.lidar32_inpara.beam_order[beam_now]*BEAM_POINTSIZE+cnt_next;
                    if(!pt_next->valid)
                        continue;
                    row_next = (pt_next->y + para.map_range_rear)/para.grid_size;
                    col_next = (pt_next->x + para.map_range_left)/para.grid_size;
                    if(row_next<0 || row_next>=Rows || col_next<0||col_next>=Cols)
                        continue;
                    break;
                }

//                cout << _row<< "  "<< _col << " "<<row_next<<" "<< col_next<<endl;
                if(row_next<0 || row_next>=Rows || col_next<0||col_next>=Cols)
                    continue;
                if(grids[_row][_col].attribute != GRID_NEG_OBS && grids[row_next][col_next].attribute == GRID_NEG_OBS ){
                    row_begin1 = _row;
                    col_begin1 = _col;
                    row_begin2 = row_next;
                    col_begin2 = col_next;
                    flag1 = true;

                }else if(grids[_row][_col].attribute == GRID_NEG_OBS && grids[row_next][col_next].attribute != GRID_NEG_OBS ){
                    row_end1 = row_next;
                    col_end1 = col_next;
                    row_end2 = _row;
                    col_end2 = _col;
                    flag2 = true;
                }
            }

            if(flag1&&flag2){
                flag1 = false;
                flag2 = false;
                int start_row = min(row_begin1, min(row_begin2, min(row_end1,row_end2)));
                int start_col = min(col_begin1, min(col_begin2, min(col_end1,col_end2)));
                int end_row = max(row_begin1, max(row_begin2, max(row_end1,row_end2)));
                int end_col = max(col_begin1, max(col_begin2, max(col_end1,col_end2)));
                cout << start_row <<"   "<<start_col<<"   "<<end_row<<"   "<<end_col<<endl;

                if(start_row<0 || start_col<0 ||end_row<0||end_col<0)
                    continue;
                for(int i=start_row;i<end_row;i++){
                    for(int j=start_col;j<end_col;j++){
                        grids[i][j].attribute = GRID_NEG_OBS;
                    }
                }
            }
        }
    }
}

void POSITIVE_DETECTOR::filterWeeds(ALV_DATA *alv_data){
    GRID **grids = alv_data->grid_map.grids;
    int Rows = alv_data->para_table.grid_rows;
    int Cols = alv_data->para_table.grid_cols;
    for(int row = 0; row < Rows; row++)
    {
        for(int col = 0; col < Cols; col++)
        {
            if( !grids[row][col].known ||
                    grids[row][col].attribute != GRID_POS_OBS||
                    grids[row][col].attribute != GRID_RETRIEVED_POS||
                    //grids[row][col].attribute != GRID_NEG_OBS||
                    grids[row][col].attribute != GRID_NEG_OBS )
                continue;
            if(grids[row][col].dis_height <= alv_data->para_table.pos_obs_threshold)        //dis_height higher than threshold
            {
                grids[row][col].attribute = GRID_TRAVESABLE;
//                if(row>=alv_data->para_table.grid_center_row
//                   && row <= alv_data->para_table.grid_center_row+7
//                   && col >= alv_data->para_table.grid_center_col-7
//                   && col <= alv_data->para_table.grid_center_col+7
//                   && grids[row][col].dis_height < alv_data->para_table.near_obstacle_threshold)
//                    grids[row][col].attribute = GRID_TRAVESABLE;
            }
            /*else
                grids[row][col].attribute = GRID_TRAVESABLE;*/
        }
    }
    filt_grid(alv_data, 1, 1);
}

void POSITIVE_DETECTOR::stdevWeeds(ALV_DATA *alv_data){
    int rows = alv_data->para_table.grid_rows;
    int cols = alv_data->para_table.grid_cols;
    GRID ** grids = alv_data->grid_map.grids;
    for(int row=0; row<rows; row++){
        for(int col=0; col<cols; col++){
            if( !grids[row][col].known ||
                    grids[row][col].attribute != GRID_POS_OBS ||
                    grids[row][col].attribute != GRID_RETRIEVED_POS ||
                    //grids[row][col].attribute != GRID_NEG_OBS ||
                    grids[row][col].attribute != GRID_NEG_OBS )
                continue;
            if(grids[row][col].stdev_x > x_stdev_ths &&
                    grids[row][col].stdev_y > y_stdev_ths &&
                    grids[row][col].stdev_z > z_stdev_ths){
                // it is weed grid, to distinguish
                // threshold is waiting for confine
                grids[row][col].attribute = GRID_FALLING_NEG_EDGE;
            }
        }
    }
    // if weed grid neighbor has enough pos+neg+retri+fall_neg_edge, it's pos; else it is traversable
    int win = 1;
    int neig = 3;
    for(int i=win; i < rows - win; i++){
        for(int j=win; j < cols - win; j++){
            if( !grids[i][j].known ||
                    grids[i][j].attribute != GRID_FALLING_NEG_EDGE)
                continue;
            int cnt_neig = 0;
            for(int row = i - win; row <= i + win; row++){
                for(int col = j - win; col <= j + win; col++){
                    if((i==row && j==col) ||
                            grids[row][col].attribute != GRID_POS_OBS &&
                            grids[row][col].attribute != GRID_NEG_OBS &&
                            grids[row][col].attribute != GRID_RETRIEVED_POS &&
                            grids[row][col].attribute != GRID_FALLING_NEG_EDGE)
                        continue;
                    cnt_neig ++;
                }
            }
            if(cnt_neig >= neig)
                grids[i][j].attribute = GRID_POS_OBS;
            else grids[i][j].attribute = GRID_TRAVESABLE;

        }
    }
}

void POSITIVE_DETECTOR::expand_pos(ALV_DATA *alv_data){
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

/* 正障碍检测 */
void POSITIVE_DETECTOR::detect(ALV_DATA *alv_data)
{
    detect_obstacle_grid(alv_data);	// 标记出障碍物栅格、车体范围
    detect_water_surface(alv_data);
    remove_suspended_obs(alv_data);	// 标记悬空障碍物


    //continue!!!
    // because of occlusion is traversable
    classify_shadow_grid(alv_data);// 标记阴影区、危险区域
    retrieve_pos(alv_data);
    classify_dangerous_grid(alv_data);// 标记阴影区、危险区域
    retrieve_neg(alv_data);
    expand_pos(alv_data);
//    classify_neg_occlusion_grid(alv_data);// 标记阴影区、危险区域
//    expand_neg(alv_data, (const alv_Point3f**)alv_data->lidar32_pointcloud, HDL32_BEAM_POINTSIZE);
//    contour_filter(alv_data);
    // filterWeeds(alv_data);
//    stdevWeeds(alv_data);
    // classify_occlusion_grid(alv_data);// 标记阴影区、危险区域
//    retrieve_pos(alv_data);
}
