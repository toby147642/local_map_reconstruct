/* Created  :   Ye Yuwen
 * Date     :   2018-09-21
 * Usage    :   definition of positive obstacle detector class
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
            }
            else
                grids[row][col].attribute = GRID_TRAVESABLE;
        }
    }

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

/* 正障碍检测 */
void POSITIVE_DETECTOR::detect(ALV_DATA *alv_data)
{
    detect_obstacle_grid(alv_data);	// 标记出障碍物栅格、车体范围
    detect_water_surface(alv_data);
    remove_suspended_obs(alv_data);	// 标记悬空障碍物
    classify_shadow_grid(alv_data);// 标记阴影区、危险区域
    retrieve_pos(alv_data);
    classify_dangerous_grid(alv_data);// 标记阴影区、危险区域

}
