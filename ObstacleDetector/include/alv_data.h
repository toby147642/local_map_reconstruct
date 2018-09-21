/* Created  :   Linhui
 * Date     :   2016-05-17
 * Usage    :   Declaration of ALV_DATA class, which deals with data parsing
 *              and data cleanup operation.
*/
#ifndef ALV_DATA_H
#define ALV_DATA_H

#include "Lidar32DataStruct.h"
//#include "Lidar16DataStruct.h"
//#include "lidar4Datastruct.h"
//#include "Lidar1_Datan.h"
#include "program.h"
#include <vector>
#include <string>
#include <cmath>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <algorithm>

/* alv data structure, which contians:
 *  1.  point cloud: one frame consists of 16 beams, in Strongest or Last mode
 *      of VLP16 lidar, each beam is made up of around 75*12*2 points, but in
 *      Dual mode, each beam consists of about 150*12*2 points.
 *  2.  grid map
 *  3.  parameter table
*/

static const int INVALID_VAL = 9999999;   // indicating invalid value
static const int POLAR_ANGLE_NUM = 1800;
static const int ANGLE_GRID_NUM = 270;
static const int POLAR_BEAM_NUM_2 = 32;
static const int POLAR_ANGLE_NUM_2 = 360;

enum POINT_ATTRIBUTE
{
    PT_UNDEFINE,     // default value
    PT_FLAT,         // little flatness value, indicating flat surface
    PT_LESS_FLAT,    // not as flat as PT_FLAT, but flat enough
    PT_SHARP,        // large flatness value, indicating sharp corner
    PT_LESS_SHARP    // not as sharp as PT_SHARP, but sharp enough
};

struct alv_Point3f
{
    alv_Point3f(float xval, float yval, float zval):x(xval),y(yval),z(zval){}
    alv_Point3f(){}
    cv::Point3f toCvPoint3f();    // convert to opencv point style
    float x;            // in cm
    float y;            // in cm
    float z;            // in cm
    bool valid;         // flag indicating whether this point is valid
    u_int8_t intensity; // 0-255
    u_int16_t angleH;   // 0-35999, in 0.01 degree
    int16_t angleV;     // -18000~18000, in 0.01 degree
    int distance;       // real distance, 0-65536*2, in mm
    float xy_dis;
    int flatness;       // local flatness, computed with WIN_SIZE neighbor points
    int sub_flatness;   // local flatness, computed with MIN_WIN_SIZE neighbor points
    int neighbor_dis;   // neighbor distance, in mm
    POINT_ATTRIBUTE attribute;      // local point attribute
    POINT_ATTRIBUTE sub_attribute;  // sub local point attribute

    alv_Point3f *prev;  // pointer to first previous valid point
};


/* parameter table structure:
 *  1.  intrinsic parameters, such as sin & cos values of angleV & angleH
 *  2.  extrinsic parameters, such as rotation & translation of lidar
*/
struct HDL32_INTRINSIC_PARA
{
    HDL32_INTRINSIC_PARA();
    HDL32_INTRINSIC_PARA & operator = (const HDL32_INTRINSIC_PARA & other)
    {
        memcpy(angleV, other.angleV, sizeof(float)*HDL32_BEAM_NUM);
        memcpy(sin_angleV, other.sin_angleV, sizeof(float)*HDL32_BEAM_NUM);
        memcpy(cos_angleV, other.cos_angleV, sizeof(float)*HDL32_BEAM_NUM);
        memcpy(sin_angleH, other.sin_angleH, sizeof(float)*36000);
        memcpy(cos_angleH, other.cos_angleH, sizeof(float)*36000);
        memcpy(beam_order, other.beam_order, sizeof(int)*HDL32_BEAM_NUM);
        memcpy(order_beam, other.beam_order, sizeof(int)*HDL32_BEAM_NUM);
        return *this;
    }

    float angleV[HDL32_BEAM_NUM];
    float sin_angleV[HDL32_BEAM_NUM];
    float cos_angleV[HDL32_BEAM_NUM];
    float sin_angleH[36000];
    float cos_angleH[36000];
    int beam_order[HDL32_BEAM_NUM];
    int order_beam[HDL32_BEAM_NUM];
};

struct LIDAR_EXTRINSIC_PARA
{
    LIDAR_EXTRINSIC_PARA();
    LIDAR_EXTRINSIC_PARA & operator = (const LIDAR_EXTRINSIC_PARA & other)
    {
        memcpy(R, other.R, sizeof(float)*3*3);
        memcpy(T, other.T, sizeof(float)*3);
        return *this;
    }

    float R[3][3];
    float T[3];
};


enum ENVIRONMENT
{
    ENV_RURAL,
    ENV_URBAN,
    ENV_UNKNOWN
};

struct PIXEL
{
    PIXEL():row(0),col(0){}
    PIXEL(int row_val, int col_val):row(row_val), col(col_val){}
    int row;
    int col;
};

struct POLAR_TABLE
{
    POLAR_TABLE()
    {
        table = new PIXEL*[POLAR_ANGLE_NUM];
        for(int i=0; i<POLAR_ANGLE_NUM; i++)
            table[i] = new PIXEL[ANGLE_GRID_NUM];
        angle_grid_num = new int[POLAR_ANGLE_NUM];
        memset(angle_grid_num, 0, sizeof(int)*POLAR_ANGLE_NUM);
    }
    ~POLAR_TABLE()
    {
        for(int i=0; i<POLAR_ANGLE_NUM; i++)
            delete[] table[i];
        delete[] table;
        delete[] angle_grid_num;
    }


//    // Important!!!包含有指针成员的类，一定要定义其拷贝构造函数，否则形参析构会把实参的内存也释放掉！！！
//    POLAR_TABLE(const POLAR_TABLE& other)
//    {
//        std::cout<<"polar table = overide"<<std::endl;
//        for(int i=0; i<POLAR_ANGLE_NUM; i++)
//        {
//            for(int j=0; j<ANGLE_GRID_NUM; j++)
//                table[i][j] = other.table[i][j];
//            angle_grid_num[i] = other.angle_grid_num[i];
//        }
//    }

    PIXEL **table;
    int *angle_grid_num;
};

struct PARA_TABLE
{
    PARA_TABLE();
    ~PARA_TABLE();

    HDL32_INTRINSIC_PARA lidar32_inpara;
    LIDAR_EXTRINSIC_PARA lidar32_expara;
    LIDAR_EXTRINSIC_PARA lidar16_expara_L;
    LIDAR_EXTRINSIC_PARA lidar16_expara_R;
    LIDAR_EXTRINSIC_PARA ibeo4_expara;
    int polar_beam_grids[POLAR_BEAM_NUM_2];
    int **grid_2_polar_beam;
    int **grid_2_polar_angle;

    std::string para_base_dir;
    int grid_size;          // grid resolution, usually 20 cm
    int grid_rows;
    int grid_cols;
    int map_range_front;    // map range of front, rear, left & right, in cm
    int map_range_rear;     // to grid map,
    int map_range_left;     // rows = (map_range_front + map_range_rear)/grid_size
    int map_range_right;    // cols = (map_range_left + map_range_right)/grid_size
    int blind_area_front;
    int blind_area_rear;
    int blind_area_left;
    int blind_area_right;
    int pos_obs_threshold;      // obstacle threshold in cm
    int neg_obs_threshold;      // obstacle threshold in cm
    int near_obstacle_threshold;    // near front area obstacle threshold
    int weak_flatness_threshold;    // used to detect water surface
    int suspend_obs_ths;    // suspended obstacle height threshold in cm
    int grid_center_row;
    int grid_center_col;
    int car_length;         // car length in cm
    int car_width;          // car width in cm
    int dangerous_area_threshold;   // dangerous area size threshold
    POLAR_TABLE polar_table;

    ENVIRONMENT environment;

    // adding:
    float abs_dis[HDL32_BEAM_NUM];
    float diff_abs_dis[HDL32_BEAM_NUM];

};


/* Grid map structure:
 *  grid map is at size rows*cols
 *  each grid stores point cloud
 *  each grid is classified one of the following attributes
*/
enum GRID_ATTRIBUTE
{
    GRID_UNKNOWN,           // unknown attribute
    GRID_TRAVESABLE,        // travesable
    GRID_NEG_OBS,           // negative obstacle
    GRID_POS_OBS,           // positive obstacle
    GRID_ROAD_EDGE,         // road edge
    GRID_OCCULUSION,        // shadow area
    GRID_SHADOW,
    GRID_WATER,             // water
    GRID_SUSPEND_OBS,       // suspended obstacle
    GRID_DANGEROUS,         // dangerous area
    GRID_CAR_AREA,          // car area
    GRID_ESTIMATED_GROUND,  // estimated ground grid
    GRID_FALLING_NEG_EDGE,
    GRID_RISING_NEG_EDGE,

    GRID_RETRIEVED_POS,
    // adding
    GRID_CANDIDAT_NEG,

    GRID_FRONT_EDGE,
    GRID_BACK_EDGE,
    GRID_FLAT,
    GRID_CORNER
};

struct GRID
{
    GRID();
    void cleanup();

    bool known;
    float max_height;
    float min_height;		// 实际扫描得到点云最大高度
    float dis_height;		// 实际扫描得到点云最小高度
    float ground_height;    // 对于非障碍栅格，此值为min_height，对于未知栅格 or 障碍栅格，此值应该为根据周围地面栅格做的估计值
    bool update_ground;     // 是否根据周围地面高度进行了地面估计
    int weak_intensity_cnt;  // 反射强度很弱的点数，用于判断水面
    float far_dis;
    float total_dis;
    float ave_dis;
    int ptNum;
    int beam_max;
    int beam_min;
    int beam_dis;
    std::vector<alv_Point3f> points; // use a vector to store point cloud
    float stdev_x;
    float stdev_y;
    float stdev_z;
    GRID_ATTRIBUTE attribute;
};

struct GRID_MAP
{
    GRID_MAP();
    ~GRID_MAP();

    void setup(int row_num, int col_num);
    void cleanup(); // clean up all data after one loop circle

    GRID **grids;   // GRID[rows][cols]
private:
    int rows;
    int cols;
};

/* Polar grid
*/
struct POLAR_GRID{
    POLAR_GRID():valid(0){}
    void cleanup()
    {
        valid = 0;
    }
    int valid;
};

/* alv data structure:
 *  1.  point cloud data of left & right lidar
 *  2.  parameter table
 *  3.  grid map
*/
class ALV_DATA
{
public:
    ALV_DATA();
    ~ALV_DATA();

    bool init_para();
    bool arrange_beam_order();
    bool init_lidar32_para();
    bool init_lidar16_para();
    bool init_lidar4_para();
    bool read_alv_config();
    bool init_lookup_table();
    bool setup();
    void cleanup();

    bool parse_16_process(u_int8_t *cache_L, u_int8_t *cache_R);
    bool parse_16data_offline(const std::string filename_L, const std::string filename_R);
    void calib_16data();                                   // calib point cloud with extrinsic calibration parameters
    void save_16pt_txt(const std::string &filename_L, const std::string &filename_R);
                                                            // save point cloud into files
    bool parse_32_process(u_int8_t *cache);
    bool parse_32data_offline(const std::string filename);
    void calib_32data();
    void save_32pt_txt(const std::string &filename);
    void save_32pt_txt_dis_cnt_beam(const std::string &filename);

    bool parse_4data_offline(const std::string &filename);
    void calib_4data();
    void save_4pt_txt(const std::string filename);

    bool parse_1data_offline(const std::string &filename);

    void show_result();     // show detection result
    void show_cloud();      // show point cloud
    void save_grid_map(const std::string &filename);   // save grid_map
    void update_angle_beam_points();
    void arrange_lidar32();
    void arrange_lidar32_max();
    void maxWindowFilter();
    void find_angle_ave_max_dis();
    void diff_ave_max_dis();
    void save_32pt_txt_angle_diff(const std::string &filename);

    alv_Point3f lidar32_pointcloud[HDL32_BEAM_NUM][HDL32_BEAM_POINTSIZE];   // hdl32 lidar point cloud
    // ADDING
    alv_Point3f max_lidar32_pointcloud[HDL32_BEAM_NUM][HDL32_BEAM_POINTSIZE];   // hdl32 lidar point cloud
    alv_Point3f angle_beam_points[ANGLE_NUM][HDL32_BEAM_NUM][ANGLE_BEAM_POINTS_NUM];
    float angle_beam_dis_ave_mm[ANGLE_NUM][HDL32_BEAM_NUM];
    float angle_beam_dis_max_mm[ANGLE_NUM][HDL32_BEAM_NUM];
    float diff_dis_ave_mm[ANGLE_NUM][HDL32_BEAM_NUM];
    float diff_dis_max_mm[ANGLE_NUM][HDL32_BEAM_NUM];

    float beam_dis_ave_mm[ANGLE_NUM][HDL32_BEAM_NUM];
    float beam_dis_max_mm[ANGLE_NUM][HDL32_BEAM_NUM];

    alv_Point3f lidar32_pointcloud_beam[HDL32_BEAM_NUM][HDL32_BEAM_POINTSIZE];
    alv_Point3f max_lidar32_pointcloud_beam[HDL32_BEAM_NUM][HDL32_BEAM_POINTSIZE];
    unsigned char **lidar1_grids;  // lidar1 data

    PARA_TABLE para_table;
    GRID_MAP grid_map;
    int **neig_grid_map_beam_max;
    int **neig_grid_map_beam_min;

    POLAR_GRID polar_grids[POLAR_BEAM_NUM_2][POLAR_ANGLE_NUM_2];
    cv::Mat local_map;
    int cnt_frame = 0;

protected:
    cv::Mat result;

};

// calculate point distance
float point_distance(const alv_Point3f &p1, const alv_Point3f &p2);

#endif // ALV_DATA_H
