/* Created  :   Ye Yuwen
 * Date     :   2018-09-21
 * Usage    :   Declaration of ALV_DATA class, which deals with data parsing
 *              and data cleanup operation.
*/
#include "alv_data.h"
#include "program.h"
#include <iostream>
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <string>
#include <fstream>
#include <sstream>
#include <cassert>
#include <cmath>
#include <cctype>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace std;
using namespace cv;
/* alv_Point3f
*/
#define OUTPUT
cv::Point3f alv_Point3f::toCvPoint3f()
{
    return Point3f(x,y,z);
}

/* HDL32_INTRINSIC_PARA part:
 *  constructor
*/
HDL32_INTRINSIC_PARA::HDL32_INTRINSIC_PARA()
{
    memset(angleV, 0, sizeof(float)*HDL32_BEAM_NUM);
    memset(beam_order, 0, sizeof(int)*HDL32_BEAM_NUM);
    memset(order_beam, 0, sizeof(int)*HDL32_BEAM_NUM);
    memset(sin_angleH, 0, sizeof(float)*36000);
    memset(cos_angleH, 0, sizeof(float)*36000);
}


ALV_DATA::ALV_DATA()
{

    lidar1_grids = new unsigned char*[GRIDMAP_HEIGHT];
    for(int i=0; i<GRIDMAP_HEIGHT; i++)
    {
        lidar1_grids[i] = new unsigned char[GRIDMAP_WIDTH];
        memset(lidar1_grids[i], 0, sizeof(unsigned char)*GRIDMAP_WIDTH);
    }

    neig_grid_map_beam_max = new int*[GRIDMAP_HEIGHT];
    for(int i=0; i<GRIDMAP_HEIGHT;i++){
        neig_grid_map_beam_max[i] = new int[GRIDMAP_WIDTH];
        memset(neig_grid_map_beam_max[i], -1, sizeof(int)*GRIDMAP_WIDTH);
    }

    neig_grid_map_beam_min = new int * [GRIDMAP_HEIGHT];
    for(int i=0; i<GRIDMAP_HEIGHT;i++){
        neig_grid_map_beam_min[i] = new int[GRIDMAP_WIDTH];
        memset(neig_grid_map_beam_min[i], -1, sizeof(int)*GRIDMAP_WIDTH);
    }
}

ALV_DATA::~ALV_DATA()
{
    for(int i=0; i<GRIDMAP_HEIGHT; i++){
        delete[] lidar1_grids[i];
        delete[] neig_grid_map_beam_max[i];
        delete[] neig_grid_map_beam_min[i];
    }
    delete[] lidar1_grids;
    delete[] neig_grid_map_beam_max;
    delete[] neig_grid_map_beam_min;

}

/* LIDAR_EXTRINSIC_PARA part:
 *  constructor
*/
LIDAR_EXTRINSIC_PARA::LIDAR_EXTRINSIC_PARA()
{
    memset(R, 0, sizeof(float)*3*3);
    memset(T, 0, sizeof(float)*3);
}

PARA_TABLE::PARA_TABLE()
{

//    para_base_dir = "/home/user/workspace/kyxz2018_G1_zju/src/local_map/parameters";
    // Local version
    para_base_dir = "/home/yyw/ObstacleDetector_yyw_withoutRCS/parameters";

    grid_size = 0;
    grid_rows = 0;
    grid_cols = 0;
    map_range_front = 0;
    map_range_rear = 0;
    map_range_left = 0;
    map_range_right = 0;
    blind_area_front = 0;
    blind_area_rear = 0;
    blind_area_left = 0;
    blind_area_right = 0;
    pos_obs_threshold = 0;
    neg_obs_threshold = 0;
    near_obstacle_threshold = 0;
    weak_flatness_threshold = 0;
    suspend_obs_ths = 0;
    grid_center_row = 0;
    grid_center_col = 0;
    car_length = 0;
    car_width = 0;

    for(int i=0; i<POLAR_BEAM_NUM_2; i++)
        polar_beam_grids[i] = 0;
    for(int i = 0; i<HDL32_BEAM_NUM; i++){
        abs_dis[i] = 0;
        diff_abs_dis[i] = 0;
    }


    environment = ENV_UNKNOWN;
}


PARA_TABLE::~PARA_TABLE()
{
    for(int row=0; row<grid_rows; row++)
    {
        delete[] grid_2_polar_angle[row];
        delete[] grid_2_polar_beam[row];
    }
    delete[] grid_2_polar_angle;
    delete[] grid_2_polar_beam;
}


/* GRID part:
 *  constructor, each grid is init empty with unknown attribute
*/
GRID::GRID()
{
    known = false;
    points.clear();
    attribute = GRID_UNKNOWN;
    max_height = 0.0;
    min_height = 0.0;
    dis_height = 0.0;
    beam_dis = 0;
    beam_max = -1;
    beam_min = -1;
    ground_height = 99999.0;    // initially great enough
    far_dis = 0.0;
    total_dis = 0.0;
    ave_dis = 0.0;
    weak_intensity_cnt = 0;
    stdev_x = 0.0;
    stdev_y = 0.0;
    stdev_z = 0.0;
    update_ground = false;
    ptNum = 0;
}

void GRID::cleanup()
{
    known = false;
    points.clear();
    attribute = GRID_UNKNOWN;
    max_height = 0.0;
    min_height = 0.0;
    dis_height = 0.0;
    beam_max = -1;
    beam_min = -1;
    beam_dis = 0;
    ground_height = 99999.0;    // initially great enough
    far_dis = 0.0;
    total_dis = 0.0;
    ave_dis = 0.0;
    weak_intensity_cnt = 0;
    stdev_x = 0.0;
    stdev_y = 0.0;
    stdev_z = 0.0;
    update_ground = false;
    ptNum = 0;
}

/* GRID_MAP part:
 *  constructor & destructor
*/
GRID_MAP::GRID_MAP()
{
    grids = NULL;
    rows = 0;
    cols = 0;
}

// overload constructor
void GRID_MAP::setup(int row_num, int col_num)
{
    rows = row_num;
    cols = col_num;
    grids = new GRID *[rows];
    for(int i=0; i<rows; i++)
        grids[i] = new GRID[cols];
}

GRID_MAP::~GRID_MAP()
{
    for(int i=0; i<rows; i++)
        delete[] grids[i];
    delete[] grids;
}

void GRID_MAP::cleanup()
{
    for(int i=0; i<rows; i++)
    {
        for(int j=0; j<cols; j++)
        {
            grids[i][j].cleanup();
        }
    }
}


/* ALV_DATA part:
 *  constructor & destructor
 *  initiation of parameters
 *  setup
 *  cleanup
*/


bool ALV_DATA::init_para()
{
    if(/*init_lidar4_para() && init_lidar16_para() &&*/ init_lidar32_para() && read_alv_config())
        return true;
    else
        return false;
}

bool ALV_DATA::init_lookup_table()
{
    for(int angle_cnt = 0; angle_cnt < POLAR_ANGLE_NUM; angle_cnt++)
    {
        for (int grid_cnt = 0; grid_cnt < ANGLE_GRID_NUM; grid_cnt++)
        {
            float angle = (float)angle_cnt/(float)POLAR_ANGLE_NUM*360.0;
            int row = grid_cnt*sin(angle*M_PI/180.0) + para_table.grid_center_row;
            int col = grid_cnt*cos(angle*M_PI/180.0) + para_table.grid_center_col;
            if(row>=0 && row<para_table.grid_rows && col>=0 && col<para_table.grid_cols)
            {
                para_table.polar_table.table[angle_cnt][grid_cnt] = PIXEL(row, col);
                para_table.polar_table.angle_grid_num[angle_cnt]++;
            }
            else
                break;
        }
    }

    int prev = 0;
    int tmp_table[POLAR_BEAM_NUM_2+1];
    tmp_table[0] = 0;
    for(int i=0; i<POLAR_BEAM_NUM_2; i++)
    {
        float height = para_table.lidar32_expara.T[2];
        float angleV = 90.0 + para_table.lidar32_inpara.angleV[para_table.lidar32_inpara.beam_order[i]];
        if(angleV >= 90.0)
            tmp_table[i+1] = prev;
        float dis = tan(angleV/180.0*M_PI)*height;
        tmp_table[i+1] = dis/para_table.grid_size;
        prev = tmp_table[i+1];
    }
    for(int i=0; i<POLAR_BEAM_NUM_2; i++)
    {
        para_table.polar_beam_grids[i] = (tmp_table[i] + tmp_table[i+1])/2;
    }


    para_table.grid_2_polar_beam = new int*[para_table.grid_rows];
    para_table.grid_2_polar_angle = new int*[para_table.grid_rows];
    for(int row = 0; row < para_table.grid_rows; row++)
    {
        para_table.grid_2_polar_beam[row] = new int[para_table.grid_cols];
        para_table.grid_2_polar_angle[row] = new int[para_table.grid_cols];
    }
    for(int row = 0; row < para_table.grid_rows; row++)
    {
        for(int col = 0; col < para_table.grid_cols; col++)
        {
            double act_row = row - para_table.grid_center_row;
            double act_col = col - para_table.grid_center_col;
            if(act_row == 0 && act_col == 0)
            {
                para_table.grid_2_polar_angle[row][col] = 0;
                para_table.grid_2_polar_beam[row][col] = 0;
                continue;
            }
            double  dis = sqrt(act_row*act_row + act_col*act_col);
            int beam = 0;
            for(beam = 0; beam < POLAR_BEAM_NUM_2; beam++)
            {
                if(dis <= para_table.polar_beam_grids[beam])
                    break;
            }
            para_table.grid_2_polar_beam[row][col] = beam;


            int angle = acos(act_col / dis)/M_PI*180.0;
            if(act_row < 0)
                angle = 359-angle;
            para_table.grid_2_polar_angle[row][col] = angle;
        }
    }

    return true;
}

bool CMP(pair<float, int> a, pair<float, int> b)
{
    return a.first < b.first;
}

bool ALV_DATA::arrange_beam_order()
{
    vector<pair<float, int> > beams;
    for(int i=0; i<HDL32_BEAM_NUM; i++)
    {
        beams.push_back(pair<float, int>(para_table.lidar32_inpara.angleV[i], i));
    }
    sort(beams.begin(), beams.end(), CMP);
    for(int i=0; i<beams.size(); i++){
        para_table.lidar32_inpara.beam_order[i] = beams[i].second;
        para_table.lidar32_inpara.order_beam[beams[i].second] = i;
    }
}

bool ALV_DATA::init_lidar32_para()
{
    string lidar32para_filename = para_table.para_base_dir + "/lidar32para_offline.ini";
    ifstream infile;
    stringstream sline;
    string line;

    for(int k = 0; k<36000; k++)
    {
        para_table.lidar32_inpara.sin_angleH[k] = (float)sin((double)k/18000.0*M_PI);
        para_table.lidar32_inpara.cos_angleH[k] = (float)cos((double)k/18000.0*M_PI);
    }

    infile.open(lidar32para_filename, ios::in);
    if(!infile)
    {
        cerr<<"***Error: can't open hdl32 para file \""<<lidar32para_filename<<"\""<<endl;
        return false;
    }

    sline.str("");
    sline.clear();
    line.clear();
    while(getline(infile, line))
    {
        if(line.empty())
            continue;
        if(line[0] == '#')      // '#' means comment
            continue;
        if(line[0] == '[')      // '[' means this is a flag line
        {
            string flag;
            for(auto c : line)
            {
                if(c != '[' && c != ']' && c != '\\')
                    flag = flag + c;
                if(c == '\\')
                    break;
            }
            if(flag == "Angle_V")
            {
                for(int i=0; i<32; i++)
                {
                    line.clear();
                    getline(infile, line);
                    sline.str("");
                    sline.clear();
                    sline<<line;
                    sline>>para_table.lidar32_inpara.angleV[i];

                    para_table.lidar32_inpara.sin_angleV[i] = (float)sin(para_table.lidar32_inpara.angleV[i]/180.0*M_PI);
                    para_table.lidar32_inpara.cos_angleV[i] = (float)cos(para_table.lidar32_inpara.angleV[i]/180.0*M_PI);
                }
                arrange_beam_order();
            }
            else if(flag == "Rotation")
            {
                for(int i=0; i<3; i++)
                {
                    line.clear();
                    getline(infile, line);
                    sline.str("");
                    sline.clear();
                    sline<<line;
                    sline>>para_table.lidar32_expara.R[i][0]>>para_table.lidar32_expara.R[i][1]>>para_table.lidar32_expara.R[i][2];
                }
            }
            else if(flag == "Translation")
            {
                line.clear();
                getline(infile, line);
                sline.str("");
                sline.clear();
                sline<<line;
                sline>>para_table.lidar32_expara.T[0]>>para_table.lidar32_expara.T[1]>>para_table.lidar32_expara.T[2];
            }
        }
        line.clear();
    }
    infile.close();
    // adding:
    para_table.abs_dis[0] = para_table.diff_abs_dis[0] = abs(para_table.lidar32_expara.T[2]/para_table.lidar32_inpara.sin_angleV[0]);
    for(int i=1; i<HDL32_BEAM_NUM; i++)
        para_table.abs_dis[para_table.lidar32_inpara.order_beam[i]] = abs(para_table.lidar32_expara.T[2]/para_table.lidar32_inpara.sin_angleV[i]);

    for(int i=1; i<HDL32_BEAM_NUM; i++)
        para_table.diff_abs_dis[i] = para_table.abs_dis[i] - para_table.abs_dis[i-1];
    return true;
}

void ALV_DATA::update_angle_beam_points(){
    for(int beam = 0; beam < HDL32_BEAM_NUM; beam++){
        int cnt_point[360] = {0};
        for(int cnt = 0; cnt < HDL32_BEAM_POINTSIZE; cnt++){
            const alv_Point3f *pt = &lidar32_pointcloud[beam][cnt];
            if(!pt->valid)
                continue;
            int angle = pt->angleH /100;
            if(angle==360)
                angle = 359;
            alv_Point3f *tmp = &angle_beam_points[angle][para_table.lidar32_inpara.order_beam[beam]][cnt_point[angle]];
            memcpy(tmp, pt, sizeof(alv_Point3f));
            cnt_point[angle]++;
        }
    }
}

void ALV_DATA::arrange_lidar32(){
    for(int beam = 0; beam <HDL32_BEAM_NUM; beam++)
        for(int cnt = 0; cnt<HDL32_BEAM_POINTSIZE; cnt++){
            const alv_Point3f *pt = &lidar32_pointcloud[beam][cnt];
            if(!pt->valid)
                continue;
            alv_Point3f *tmp = &lidar32_pointcloud_beam[para_table.lidar32_inpara.order_beam[beam]][cnt];
            memcpy(tmp, pt, sizeof(alv_Point3f));
        }
}


void ALV_DATA::arrange_lidar32_max(){
    for(int beam = 0; beam <HDL32_BEAM_NUM; beam++)
        for(int cnt = 0; cnt<HDL32_BEAM_POINTSIZE; cnt++){
            const alv_Point3f *pt = &max_lidar32_pointcloud[beam][cnt];
            if(!pt->valid)
                continue;
            alv_Point3f *tmp = &max_lidar32_pointcloud_beam[para_table.lidar32_inpara.order_beam[beam]][cnt];
            memcpy(tmp, pt, sizeof(alv_Point3f));
        }
}

void ALV_DATA::maxWindowFilter(){
    for(int beam = 0; beam < HDL32_BEAM_NUM; beam++){
        for(int cnt = 1; cnt < HDL32_BEAM_POINTSIZE-1; cnt++){
            const alv_Point3f *pt1 = (alv_Point3f *)lidar32_pointcloud + beam*HDL32_BEAM_POINTSIZE+cnt-1;
            const alv_Point3f *pt2 = (alv_Point3f *)lidar32_pointcloud + beam*HDL32_BEAM_POINTSIZE+cnt;
            const alv_Point3f *pt3 = (alv_Point3f *)lidar32_pointcloud + beam*HDL32_BEAM_POINTSIZE+cnt+1;
            alv_Point3f *pt_tmp = (alv_Point3f *)max_lidar32_pointcloud + beam*HDL32_BEAM_POINTSIZE+cnt;

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
        alv_Point3f *pt_tmp1 = (alv_Point3f *)max_lidar32_pointcloud + beam*HDL32_BEAM_POINTSIZE;
        alv_Point3f *pt_tmp2 = (alv_Point3f *)lidar32_pointcloud + beam*HDL32_BEAM_POINTSIZE;
        if(!pt_tmp1->valid || !pt_tmp2->valid)
            continue;
        memcpy(pt_tmp1, pt_tmp2, sizeof(alv_Point3f));
        alv_Point3f *pt_tmp3 = (alv_Point3f *)max_lidar32_pointcloud + beam*HDL32_BEAM_POINTSIZE + HDL32_BEAM_POINTSIZE-1;
        alv_Point3f *pt_tmp4 = (alv_Point3f *)lidar32_pointcloud + beam*HDL32_BEAM_POINTSIZE + HDL32_BEAM_POINTSIZE-1;
        if(!pt_tmp3->valid || !pt_tmp4->valid)
            continue;
        memcpy(pt_tmp3, pt_tmp4, sizeof(alv_Point3f));
    }
}

void ALV_DATA::find_angle_ave_max_dis(){
    for(int i=0; i<ANGLE_NUM;i++)
        for(int j=0; j<HDL32_BEAM_NUM;j++){
            int cnt = 0;
            float sum= 0.0;
            float max_dis = -10000;
            for(int k=0; k<ANGLE_BEAM_POINTS_NUM;k++){
                alv_Point3f *tmp = &angle_beam_points[i][j][k];
                if(!tmp->valid)
                    continue;
                max_dis = tmp->distance > max_dis ? tmp->distance : max_dis;
                cnt++;
                sum += tmp->distance;
            }
            if(cnt!=0)
                angle_beam_dis_ave_mm[i][j] = sum /cnt;
            angle_beam_dis_max_mm[i][j] = max_dis;
        }
}

void ALV_DATA::diff_ave_max_dis(){
    for(int j=1; j<HDL32_BEAM_NUM;j++){
        for(int i=0; i<ANGLE_NUM;i++){
            diff_dis_ave_mm[i][j] = angle_beam_dis_ave_mm[i][j]-angle_beam_dis_ave_mm[i][j-1];
            diff_dis_max_mm[i][j] = angle_beam_dis_max_mm[i][j]-angle_beam_dis_max_mm[i][j-1];
            beam_dis_ave_mm[i][j] = diff_dis_ave_mm[i][j] - para_table.diff_abs_dis[j] * 10;
            beam_dis_max_mm[i][j] = diff_dis_max_mm[i][j] - para_table.diff_abs_dis[j] * 10;
        }
    }
}

bool ALV_DATA::parse_32data_offline(const std::string filename)
{
    char file[256];
    strcpy(file, filename.c_str());
    FILE *fp = fopen(file, "rb");
    if(fp==NULL)
        return false;

    u_int8_t *cache = new u_int8_t[HDL32_BUFFER_SIZE];
    if(!fread(cache, sizeof(u_int8_t), HDL32_BUFFER_SIZE, fp))
    {
        cerr<<"***Error: fread file "<<filename<<" failed!"<<endl;
        return false;
    }
    fclose(fp);

    parse_32_process(cache);    //cache point to the first address
    calib_32data();
    // adding: update angle_beam_points
    update_angle_beam_points();
    arrange_lidar32();
    maxWindowFilter();
    arrange_lidar32_max();
    find_angle_ave_max_dis();
    diff_ave_max_dis();
    delete[] cache;
    return true;
}


// read from a single configure file line, determine which type the parameter is,
// and get the value of the parameter
void ReadConfigLine(const string line, string &flag, int &value)
{
    flag.clear();
    value = 0;
    for(auto c : line)
    {
        if(isalpha(c) || c == '_')
            flag = flag + c;
        if(c == '=' || c == ' ')
            continue;
        if(isdigit(c))
            value = value*10+(c-'0');
    }
}
/* init extrinsic para table
*/
bool ALV_DATA::read_alv_config()
{
    ifstream infile;
    stringstream sline;
    string line;

    string alv_config_filename = para_table.para_base_dir + "/alv_config.conf";
    infile.open(alv_config_filename, ios::in);
    if(!infile)
    {
        cerr<<"***Error: can't open config file \""<<alv_config_filename<<"\""<<endl;
        return false;
    }

    line.clear();
    while(getline(infile, line))
    {
        if(line[0] == '#' || line[0] == '[' || line.empty())
            continue;
        else
        {
            string flag;
            int value;
            ReadConfigLine(line, flag, value);
            if(flag == "map_front_range")
                para_table.map_range_front = value;
            else if (flag == "map_rear_range")
                para_table.map_range_rear = value;
            else if (flag == "map_left_range")
                para_table.map_range_left = value;
            else if (flag == "map_right_range")
                para_table.map_range_right = value;
            else if (flag == "blind_area_front")
                para_table.blind_area_front = value;
            else if (flag == "blind_area_rear")
                para_table.blind_area_rear = value;
            else if (flag == "blind_area_left")
                para_table.blind_area_left = value;
            else if (flag == "blind_area_right")
                para_table.blind_area_right = value;
            else if (flag == "pos_obstacle_threshold")
                para_table.pos_obs_threshold = value;
            else if (flag == "neg_obstacle_threshold")
                para_table.neg_obs_threshold = value;
            else if(flag == "near_obstacle_threshold")
                para_table.near_obstacle_threshold = value;
            else if(flag == "weak_flatness_threshold")
                para_table.weak_flatness_threshold = value;
            else if(flag == "suspend_obs_threshold")
                para_table.suspend_obs_ths = value;
            else if (flag == "grid_size")
                para_table.grid_size = value;
            else if (flag == "car_length")
                para_table.car_length = value;
            else if (flag == "car_width")
                para_table.car_width = value;
            else if (flag == "dangerous_area_threshold")
                para_table.dangerous_area_threshold = value;
        }
        line.clear();
    }
    infile.close();

    para_table.grid_center_row = para_table.map_range_rear / para_table.grid_size;
    para_table.grid_center_col = para_table.map_range_left / para_table.grid_size;

    return true;
}

bool ALV_DATA::setup()
{
    if(!init_para())
        return false;

    assert(para_table.map_range_front > 0);
    assert(para_table.map_range_rear > 0);
    assert(para_table.map_range_left > 0);
    assert(para_table.map_range_right > 0);
    assert(para_table.grid_size > 0);
    para_table.grid_rows = (para_table.map_range_front + para_table.map_range_rear) / para_table.grid_size;
    para_table.grid_cols = (para_table.map_range_left + para_table.map_range_right) / para_table.grid_size;
    grid_map.setup(para_table.grid_rows, para_table.grid_cols);

    //adding

    //end

    cv::Mat color_map = cv::Mat::zeros(para_table.grid_rows, para_table.grid_cols, CV_8UC3);
    cv::Mat out_put = cv::Mat::zeros(80, 80, CV_8UC1);
    result = color_map;
    local_map = out_put;

    init_lookup_table();
    return true;
}

void ALV_DATA::cleanup()
{

    memset(lidar32_pointcloud, 0, sizeof(alv_Point3f)*HDL32_BEAM_NUM*HDL32_BEAM_POINTSIZE);
    memset(lidar32_pointcloud_beam, 0, sizeof(alv_Point3f)*HDL32_BEAM_NUM*HDL32_BEAM_POINTSIZE);
    memset(max_lidar32_pointcloud, 0, sizeof(alv_Point3f)*HDL32_BEAM_NUM*HDL32_BEAM_POINTSIZE);
    memset(max_lidar32_pointcloud_beam, 0, sizeof(alv_Point3f)*HDL32_BEAM_NUM*HDL32_BEAM_POINTSIZE);
    memset(angle_beam_points, 0, sizeof(alv_Point3f)*ANGLE_NUM*HDL32_BEAM_NUM*ANGLE_BEAM_POINTS_NUM);
    memset(angle_beam_dis_ave_mm, 0, sizeof(float)*ANGLE_NUM*HDL32_BEAM_NUM);
    memset(angle_beam_dis_max_mm, 0, sizeof(float)*ANGLE_NUM*HDL32_BEAM_NUM);
    memset(diff_dis_ave_mm, 0, sizeof(float)*ANGLE_NUM*HDL32_BEAM_NUM);
    memset(diff_dis_max_mm, 0, sizeof(float)*ANGLE_NUM*HDL32_BEAM_NUM);
    memset(beam_dis_ave_mm, 0, sizeof(float)*ANGLE_NUM*HDL32_BEAM_NUM);
    memset(beam_dis_max_mm, 0, sizeof(float)*ANGLE_NUM*HDL32_BEAM_NUM);

    grid_map.cleanup();
    for(int i=0; i<para_table.grid_rows; i++)
    {
        for(int j=0; j<para_table.grid_cols; j++)
        {
//            cout << para_table.grid_rows << " " << para_table.grid_size <<endl;
            neig_grid_map_beam_max[i][j] = -1;
            neig_grid_map_beam_min[i][j] = -1;
        }
    }

    for(int beam=0; beam<POLAR_BEAM_NUM_2; beam++)
    {
        for(int angle = 0; angle < POLAR_ANGLE_NUM_2; angle++)
            polar_grids[beam][angle].cleanup();
    }

    // clean up picture
    int Rows = para_table.grid_rows;
    int Cols = para_table.grid_cols;
    for(int row = 0; row < Rows; row++)
        for(int col = 0; col < Cols; col++)
          result.at<Vec3b>(row, col) = Vec3b(0,0,0);      	// BGR, 黑
}

void ALV_DATA::show_result(){
    GRID **grids = grid_map.grids;
    int Rows = para_table.grid_rows;
    int Cols = para_table.grid_cols;
    cv::Size re2s = cv::Size(Cols, Rows);
    Mat re2 = Mat::zeros(re2s, CV_8UC1);
    Mat beam_diff = Mat::zeros(re2s, CV_8UC1);
    Mat neig_beam_diff = Mat::zeros(re2s,CV_8UC1);

    cv::Size out = cv::Size(160, 160);
    Mat output_img = Mat::zeros(out, CV_8UC3);

    for(int row = 0; row < Rows; row++)
    {
        for(int col = 0; col < Cols; col++)
        {
            beam_diff.at<uchar>(row, col) = grids[row][col].beam_dis*25;
            neig_beam_diff.at<uchar>(row, col) = (neig_grid_map_beam_max[row][col] - neig_grid_map_beam_min[row][col]+1) * 20;
            re2.at<uchar>(row,col) = 255;
            if(grids[row][col].attribute == GRID_UNKNOWN){
                result.at<Vec3b>(row, col) = Vec3b(0,0,0);      	// BGR, 黑
                re2.at<uchar>(row,col) = 0;
            }
            else if(grids[row][col].attribute == GRID_CAR_AREA)
                result.at<Vec3b>(row, col) = Vec3b(255,255,255);   	// 白
            else if(grids[row][col].attribute == GRID_TRAVESABLE ||
                    grids[row][col].attribute == GRID_CANDIDAT_NEG ||
                    grids[row][col].attribute == GRID_SHADOW/* || grids[row][col].attribute == GRID_ESTIMATED_GROUND*/)
                result.at<Vec3b>(row, col) = Vec3b(100,50,50);   	// 灰
            else if(grids[row][col].attribute == GRID_POS_OBS || grids[row][col].attribute == GRID_RETRIEVED_POS)
                result.at<Vec3b>(row, col) = Vec3b(0,0,255);     	// 红
            else if(grids[row][col].attribute == GRID_NEG_OBS)
                result.at<Vec3b>(row, col) = Vec3b(255,255,0);  	// 浅蓝
            else if(grids[row][col].attribute == GRID_RISING_NEG_EDGE)
                result.at<Vec3b>(row, col) = Vec3b(255,255,0);
            else if(grids[row][col].attribute == GRID_FALLING_NEG_EDGE)
                result.at<Vec3b>(row, col) = Vec3b(255,255,0);
            else if(grids[row][col].attribute == GRID_DANGEROUS)
//                result.at<Vec3b>(row, col) = Vec3b(0,255,255);  	// 黄
                result.at<Vec3b>(row, col) = Vec3b(50,50,50); 	// 灰
            else if(grids[row][col].attribute == GRID_ROAD_EDGE)
                result.at<Vec3b>(row, col) = Vec3b(255,0,255);  	// 紫
            else if(grids[row][col].attribute == GRID_OCCULUSION)
                result.at<Vec3b>(row, col) = Vec3b(255,0,0);  		// 蓝
            else if(grids[row][col].attribute == GRID_WATER)
                result.at<Vec3b>(row, col) = Vec3b(50,50,50); 	// 灰
            else if(grids[row][col].attribute == GRID_SUSPEND_OBS)
                result.at<Vec3b>(row, col) = Vec3b(0,0,255);    	// 绿 0 255 0
        }
    }
    for(int i=0;i<160;i++){
        for(int j=0;j<160;j++){
            output_img.at<Vec3b>(159-i, j) = result.at<Vec3b>(i+60, j);
        }
    }

    // output local map
    //int cnt[4] = {0};
    int cnt[80][80][7] = {};
    for(int i=0; i<80; i++){
        for(int j=0; j<80; j++){
            int tmp_r, tmp_c;
            tmp_r = (i * 2 +60);
            tmp_c = j * 2;

            // cnt  for  attribute
            for(int row=tmp_r; row<tmp_r+2; row++){
                for(int col=tmp_c; col<tmp_c+2; col++){
                    // UNkown
                    if(grids[row][col].attribute == GRID_OCCULUSION)
                        cnt[i][j][3] ++;
                    // Pos
                    else if(grids[row][col].attribute == GRID_POS_OBS ||
                            grids[row][col].attribute == GRID_RETRIEVED_POS)
                        cnt[i][j][1] ++;
                    // Neg
                    else if(grids[row][col].attribute == GRID_NEG_OBS ||
                            grids[row][col].attribute == GRID_RISING_NEG_EDGE ||
                            grids[row][col].attribute == GRID_FALLING_NEG_EDGE)
                        cnt[i][j][2] ++;
                    // Traver
                    else if(grids[row][col].attribute == GRID_CAR_AREA ||
                            grids[row][col].attribute == GRID_SUSPEND_OBS || //
                            grids[row][col].attribute == GRID_UNKNOWN ||
                            grids[row][col].attribute == GRID_TRAVESABLE ||
                            grids[row][col].attribute == GRID_CANDIDAT_NEG ||
                            grids[row][col].attribute == GRID_ROAD_EDGE  ||
                            grids[row][col].attribute == GRID_SHADOW )
                        cnt[i][j][3] ++;
                    // Water
                    else if(grids[row][col].attribute == GRID_WATER)
                        cnt[i][j][4] ++;

                    // Danger
                    else if(grids[row][col].attribute == GRID_DANGEROUS)
                        cnt[i][j][5] ++;
                    else //if(grids[row][col].attribute == GRID_SUSPEND_OBS)
                        cnt[i][j][6] ++;
                }
            }
        }
    }

#ifdef OUTPUT
    for(int i=0; i<80; i++){
        for(int j=0; j<80; j++){
            if(cnt[i][j][0]>=1)
                // unk
                local_map.at<uchar>(i, j) = 3;

            if(cnt[i][j][3]>=1)
                // trav
                local_map.at<uchar>(i, j) = 3;

            if(cnt[i][j][5]>=1)
                // dangerrous
                local_map.at<uchar>(i, j) = 4;

            if(cnt[i][j][1]>=1)
                // pos
                local_map.at<uchar>(i, j) = 1;

            if(cnt[i][j][2]>=1)
                // neg
                local_map.at<uchar>(i, j) = 2;

            if(cnt[i][j][4]>=1)
                // water
                local_map.at<uchar>(i, j) = 4;
        }
    }
#else
    for(int i=0; i<80; i++){
        for(int j=0; j<80; j++){
            if(cnt[i][j][0]>=1)
                // unk
                local_map.at<uchar>(i, j) = 3 * 60;

            if(cnt[i][j][3]>=1)
                // trav
                local_map.at<uchar>(i, j) = 3 * 60;

            if(cnt[i][j][5]>=1)
                // dangerrous
                local_map.at<uchar>(i, j) = 4 * 60;

            if(cnt[i][j][1]>=1)
                // pos
                local_map.at<uchar>(i, j) = 1 * 60;

            if(cnt[i][j][2]>=1)
                // neg
                local_map.at<uchar>(i, j) = 2 * 60;

            if(cnt[i][j][4]>=1)
                // water
                local_map.at<uchar>(i, j) = 4 * 60;

            if(cnt[i][j][6]>=1)  local_map.at<uchar>(i, j) = 3 * 60;
        }
    }
#endif



    int offset = para_table.car_length/2;
    // 5 m
//    line(result, Point(0,para_table.grid_center_row+(offset+500)/para_table.grid_size), Point(Cols-1,para_table.grid_center_row+(offset+500)/para_table.grid_size), Scalar(150,150,150), 1, 4);
//    // 10 m
//    line(result, Point(0,para_table.grid_center_row+(offset+1000)/para_table.grid_size), Point(Cols-1,para_table.grid_center_row+(offset+1000)/para_table.grid_size), Scalar(150,150,150), 1, 4);
//    // 15 m
//    line(result, Point(0,para_table.grid_center_row+(offset+1500)/para_table.grid_size), Point(Cols-1,para_table.grid_center_row+(offset+1500)/para_table.grid_size), Scalar(150,150,150), 1, 4);
//    // 20 m
//    line(result, Point(0,para_table.grid_center_row+(offset+2000)/para_table.grid_size), Point(Cols-1,para_table.grid_center_row+(offset+2000)/para_table.grid_size), Scalar(150,150,150), 1, 4);

//    line(result, Point(0, 128), Point(Cols-1, 128), Scalar(100,100,100), 1);
//    line(result, Point(100,0), Point(100,Rows-1), Scalar(100,100,100), 1);

    int MatHight = para_table.grid_rows - 1;
    flip(result, result, 0);
//    putText(result, "5m", Point(0, MatHight-(para_table.grid_center_row+(offset+500)/para_table.grid_size)),
//            cv::FONT_HERSHEY_COMPLEX, 0.5, Scalar(250,250,250));
//    putText(result, "10m", Point(0, MatHight-(para_table.grid_center_row+(offset+1000)/para_table.grid_size)),
//            cv::FONT_HERSHEY_COMPLEX, 0.5, Scalar(250,250,250));
//    putText(result, "15m", Point(0, MatHight-(para_table.grid_center_row+(offset+1500)/para_table.grid_size)),
//            cv::FONT_HERSHEY_COMPLEX, 0.5, Scalar(250,250,250));
//    putText(result, "20m", Point(0, MatHight-(para_table.grid_center_row+(offset+2000)/para_table.grid_size)),
//            cv::FONT_HERSHEY_COMPLEX, 0.5, Scalar(250,250,250));

    cv::Size dsize = cv::Size(2*Cols, 2*Rows);
    Mat dresult = Mat::zeros(dsize, CV_8UC3);
    resize(result, dresult, dsize, INTER_NEAREST);
    //imshow("result", dresult);

    cv::Size dsize2 = cv::Size(2*160, 2*160);
    Mat dresult2 = Mat::zeros(dsize2, CV_8UC3);
    resize(output_img, dresult2, dsize2, INTER_NEAREST);
    imshow("output_img", dresult2);

    flip(local_map, local_map, 0);
    Mat dresult3 = Mat::zeros(dsize2, CV_8UC1);
    resize(local_map, dresult3, dsize2, INTER_NEAREST);
    imshow("local_map", dresult3);

    cv::waitKey(1);

    // cv::imwrite(string("/home/user/img/")+std::to_string(cnt_frame)+".jpg", output_img);
    // cnt_frame++;

    
//    Mat dresult2 = Mat::zeros(dsize, CV_8UC1);
//    resize(re2, dresult2, dsize);
//    imshow("result2", dresult2);

//    Mat dbeam_diff = Mat::zeros(dsize,CV_8UC1);
//    resize(beam_diff,dbeam_diff,dsize);
//    imshow("beam_dense",dbeam_diff);

//    Mat dneig_beam_diff = Mat::zeros(dsize,CV_8UC1);
//    resize(neig_beam_diff,dneig_beam_diff,dsize);
//    imshow("neig_beam_dense",dneig_beam_diff);
//    waitKey(100);

}

/*
 * Lidar32 part
 * what is the mean to every packet?
 * read a fream point clouds
*/
bool ALV_DATA::parse_32_process(u_int8_t *cache)
{
    int packet_cnt = 0;
    u_int8_t *fp = cache + HDL32_PACKET_SIZE*packet_cnt;
    u_int8_t *fq = fp + 1;
    u_int16_t val = (*fp)*256 + *fq;

    float *cos_angH = para_table.lidar32_inpara.cos_angleH;
    float *sin_angH = para_table.lidar32_inpara.sin_angleH;
    float *cos_angV = para_table.lidar32_inpara.cos_angleV;
    float *sin_angV = para_table.lidar32_inpara.sin_angleV;
    float *angleV = para_table.lidar32_inpara.angleV;


    u_int16_t start_angleH = 1;	// 记录这一帧初始的水平向激光角度
    bool start_angleH_generated = false;
    while(val == 0xffee)
    {
        // parse packet
        for(int i=0; i<HDL32_NUM_SHOTS; i++)
        {
            u_int8_t *pAngL = fp + 100*i + 2;
            u_int8_t *pAngH = pAngL + 1;
            u_int16_t angleH = ((*pAngH)*256 + (*pAngL) + 9000)%36000;
            u_int8_t *pL = fp + 100*i + 4;
            u_int8_t *pH = pL + 1;
            u_int8_t *pVal = pH + 1;
            int idx = packet_cnt*HDL32_NUM_SHOTS + i;

            for(int j=0; j<HDL32_BEAM_NUM; j++)
            {
                int distance = ((*pH)*256 + (*pL))*2;       // in mm
                if(distance >= HDL32_VALID_RADIUS)
                {
                    lidar32_pointcloud[j][idx].valid = true;
                    lidar32_pointcloud[j][idx].distance = distance;
                    lidar32_pointcloud[j][idx].intensity = *pVal;
                    lidar32_pointcloud[j][idx].angleH = angleH;
                    lidar32_pointcloud[j][idx].angleV = int16_t(angleV[j]*100);
                    lidar32_pointcloud[j][idx].x = (float)distance/10.0 * cos_angV[j] * sin_angH[angleH];
                    lidar32_pointcloud[j][idx].y = (float)distance/10.0 * cos_angV[j] * cos_angH[angleH];
                    lidar32_pointcloud[j][idx].xy_dis = (float)distance/10.0 * cos_angV[j];
                    lidar32_pointcloud[j][idx].z = (float)distance/10.0 * sin_angV[j];
                }
                pL += 3;
                pH += 3;
                pVal += 3;
            }
        }

        // next packet
        packet_cnt++;
        if(packet_cnt > HDL32_PACKET_NUM - 1)       // in case of exceeding memory
            break;
        fp = cache + HDL32_PACKET_SIZE*packet_cnt;
        fq = fp + 1;
        val = (*fp)*256 + *fq;
    }
    return true;
}

void ALV_DATA::calib_32data()
{
    for(int beam=0; beam<HDL32_BEAM_NUM; beam++)
    {
        for(int cnt=0; cnt<HDL32_BEAM_POINTSIZE; cnt++)
        {
            alv_Point3f *p = &lidar32_pointcloud[beam][cnt];
            const alv_Point3f tmp = lidar32_pointcloud[beam][cnt];
            if(p->valid)
            {
                p->x = tmp.x*para_table.lidar32_expara.R[0][0]+tmp.y*para_table.lidar32_expara.R[0][1]+tmp.z*para_table.lidar32_expara.R[0][2] + para_table.lidar32_expara.T[0];
                p->y = tmp.x*para_table.lidar32_expara.R[1][0]+tmp.y*para_table.lidar32_expara.R[1][1]+tmp.z*para_table.lidar32_expara.R[1][2] + para_table.lidar32_expara.T[1];
                p->z = tmp.x*para_table.lidar32_expara.R[2][0]+tmp.y*para_table.lidar32_expara.R[2][1]+tmp.z*para_table.lidar32_expara.R[2][2] + para_table.lidar32_expara.T[2];
            }
        }
    }
}

void ALV_DATA::save_32pt_txt(const std::string &filename)
{
    fstream fileout;
    fileout.open(filename, ios::out);
    if(!fileout)
    {
        cerr<<"***Error: Can't open file \""<<filename<<"\""<<endl;
        return;
    }
    for(int beam = 0; beam < HDL32_BEAM_NUM; beam++)
    {
        for(int angle = 0; angle < HDL32_BEAM_POINTSIZE; angle++)
        {
            if(lidar32_pointcloud[beam][angle].valid)
            {
                fileout << lidar32_pointcloud[beam][angle].x << "\t\t"<<
                           lidar32_pointcloud[beam][angle].y << "\t\t"<<
                           lidar32_pointcloud[beam][angle].z << endl;
            }
        }
    }
    fileout.close();
}

void ALV_DATA::save_32pt_txt_dis_cnt_beam(const std::string &filename)
{
    fstream fileout;
    fileout.open(filename, ios::out);
    if(!fileout)
    {
        cerr<<"***Error: Can't open file \""<<filename<<"\""<<endl;
        return;
    }
    for(int beam = 0; beam < HDL32_BEAM_NUM; beam++)
    {
        for(int angle = 0; angle < HDL32_BEAM_POINTSIZE; angle++)
        {
            if(lidar32_pointcloud[beam][angle].valid)
            {
                fileout << beam << " "<<
                           angle << " "<<
                           para_table.lidar32_inpara.order_beam[beam] << " " <<
                           max_lidar32_pointcloud[beam][angle].angleV << " " <<
                           max_lidar32_pointcloud[beam][angle].angleH << " " <<
                           max_lidar32_pointcloud[beam][angle].distance << endl;
            }
        }
    }
    fileout.close();
}

void ALV_DATA::save_32pt_txt_angle_diff(const std::string &filename)
{
    fstream fileout;
    fileout.open(filename, ios::out);
    if(!fileout)
    {
        cerr<<"***Error: Can't open file \""<<filename<<"\""<<endl;
        return;
    }
    for(int beam = 0; beam < HDL32_BEAM_NUM; beam++)
    {
        for(int angle = 0; angle < ANGLE_NUM; angle++)
        {
                fileout << beam << " "<<
                           angle << " "<<
                           diff_dis_ave_mm[angle][beam]<< " "<<
                           diff_dis_max_mm[angle][beam]<< endl;
        }
    }
    fileout.close();
}


float point_distance(const alv_Point3f &p1, const alv_Point3f &p2)
{
    return sqrt(pow(double(p1.x-p2.x), 2) + pow(double(p1.y-p2.y),2) + pow(double(p1.z-p2.z),2));
}


void ALV_DATA::save_grid_map(const std::string &filename)
{
    fstream fileout;
    fileout.open(filename, ios::out);
    if(!fileout)
    {
        cerr<<"***Error: Can't open file \""<<filename<<"\""<<endl;
        return;
    }
    for(int row = 0; row<para_table.grid_rows; row++)
        for(int col=0; col<para_table.grid_cols; col++)
            fileout<<col*100<<"\t"<<row*100<<"\t"
                  <<grid_map.grids[row][col].dis_height<<"\t"
                  <<grid_map.grids[row][col].dis_height<<endl;
    fileout.close();
}
