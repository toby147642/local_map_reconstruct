//*********************************************************************************************/
//* 基础数据格式定义头文件
//*
//*********************************************************************************************/

#ifndef DEFINITIONBASIC_H
#define DEFINITIONBASIC_H

//#include "nml.hh"
//#include "nmlmsg.hh"

///////////////////////////////////////////////////////////////////////////////////////
//
//	Basic Definition
//
///////////////////////////////////////////////////////////////////////////////////////
#ifndef __INT64__
#define __INT64__
typedef  signed     long        INT64;
#endif

#ifndef __UINT64__
#define __UINT64__
typedef  unsigned   long        UINT64;
#endif

#ifndef __INT32__
#define __INT32__
typedef  signed     int         INT32;
#endif

#ifndef __UINT32__
#define __UINT32__
typedef  unsigned   int         UINT32;
#endif

#ifndef __INT16__
#define __INT16__
typedef  signed     short       INT16;
#endif

#ifndef __UINT16__
#define __UINT16__
typedef  unsigned   short       UINT16;
#endif

#ifndef __INT8__
#define __INT8__
typedef  signed     char        INT8;
#endif

#ifndef __UINT8__
#define __UINT8__
typedef  unsigned   char        UINT8;
#endif

#ifndef PI
#define PI 3.14159265358979
#endif

#ifndef Pi
#define Pi 3.14159265358979
#endif

#ifndef GPS_TIME_INTERVAL
#define GPS_TIME_INTERVAL 0.1
#endif
///////////////////////////////////////////////////////////////////////////////////////
//
//	Data Source Definition
//
///////////////////////////////////////////////////////////////////////////////////////

//	IMG Define
  #define IMGCOLOR_ROWS 612   // rows
#define IMGCOLOR_COLS 812   // cols
#define IMGCOLOR_DIMS 3     // dimentions
#define IMGCOLOR_SIZE IMGCOLOR_ROWS*IMGCOLOR_COLS*IMGCOLOR_DIMS

#define IMGCOLOR_ROWS_RESIZED 306   // rows
#define IMGCOLOR_COLS_RESIZED 406   // cols
#define IMGCOLOR_DIMS_RESIZED 3     // dimentions
#define IMGCOLOR_SIZE_RESIZED (IMGCOLOR_ROWS_RESIZED*IMGCOLOR_COLS_RESIZED*IMGCOLOR_DIMS_RESIZED)


//	3D	Define
//	各个接口模块填写下
#define PACKETNUM 200
#define PACKETNUM_LIDAR32 200
#define PACKETNUM_LIDAR16 200
#define PACKETNUM_LIDARM 200

#define GLOBALPOINTNUM  2000

#define LIDAR1_BUFFER_SIZE 2000    /* 1-laser lidar, unsettaled, developer should replace it with a proper value */

#define LIDAR4_BUFFER_SIZE 20000    /* 4-laser lidar, unsettaled, developer should replace it with a proper value */

#define VLP16_PACKET_SIZE 1206      // 16-laser lidar
#define VLP16_PACKET_NUM  80
#define VLP16_BUFFER_SIZE (VLP16_PACKET_NUM*VLP16_PACKET_SIZE)

#define HDL32_PACKET_SIZE 1206      // 32-laser lidar
#define HDL32_PACKET_NUM  200
#define HDL32_BUFFER_SIZE (HDL32_PACKET_NUM*HDL32_PACKET_SIZE)

///////////////////////////////////////////////////////////////////////////////////////
//
//	Function Definition
//
///////////////////////////////////////////////////////////////////////////////////////

//	Tracking
#define MAX_VEHICLE_OBJ 30
#define MAX_PEDESTRIAN_OBJ 30

//	正负障碍检测输出栅格图: 局部坐标系，前方80米，后方20米，左右25米
#define GRIDMAP_WIDTH   200
#define GRIDMAP_HEIGHT  325
#define GRIDMAP_VEHICLEX   100
#define GRIDMAP_VEHICLEY   75
#define GRIDMAP_SIZE (GRIDMAP_WIDTH*GRIDMAP_HEIGHT)
#define GRID_SIZE 20

//	PathPlan_Global
#define IMG_GLOBALMAP_SIZE 100*10000


//	Road Segmentation
#define IMGMAP_WIDTH 406
#define IMGMAP_HEIGHT 306
#define IMGMAP_SIZE (IMGMAP_WIDTH*IMGMAP_HEIGHT)

///////////////////////////////////////////////////////////////////////////////////////
//
//	Manager Definition
//
///////////////////////////////////////////////////////////////////////////////////////


// define MSG_TYPE

#define GPS_MSG_TYPE 10001
#define INS_MSG_TYPE 10002
#define IMGCOLOR_MSG_TYPE 10003
#define LIDAR32_MSG_TYPE 10004
#define LIDAR16_MSG_TYPE 10005
#define LIDAR4_MSG_TYPE 10006
#define LIDARM_MSG_TYPE 10007
#define LIDAR1_MSG_TYPE 10008
#define GPSINS_MSG_TYPE 10009
#define IMGCOLOR_RIGHT_MSG_TYPE 10010

#define POSITIVEINFO_MSG_TYPE 12001
#define NEGATIVEINFO_MSG_TYPE 12002
#define VEHICLEINFO_MSG_TYPE 12003
#define ROADSEGINFO_MSG_TYPE 12004
#define PATHPLANLINFO_MSG_TYPE 12005
#define PATNPLANGINFO_TYPE 12006
#define CONVERGENCEINFO_MSG_TYPE 12007
#define LOCALDEM_MSG_TYPE 12008

#define POSITIVE_STATUS_TYPE 13001
#define NEGATIVE_STATUS_TYPE 13002
#define VEHICLE_STATUS_TYPE 13003
#define ROADSEG_STATUS_TYPE 13004
#define PATHPLANL_STATUS_TYPE 13005
#define PATNPLANG_STATUS_TYPE 13006
#define CONVERGENCE_STATUS_TYPE 13007

#define PATHPLANG_CMD_TYPE 19001
#define PATHPLANL_CMD_TYPE 19002
#define SENSORCONTROL_CMD_TYPE 19003
#define SENSORCONTROL_STATUS_TYPE 19004
#define MONITORCONTROL_CMD_MSG_TYPE 19005

#define OBJECTDETECTIONINFO_MSG_TYPE	20001
#define ROADCROSSINFO_MSG_TYPE			20002
#define LADYBUGIMAGEINFO_MSG_TYPE		20003
#define TRACEINFO_MSG_TYPE				20004

#define ESTOPINFO_MSG_TYPE                      30001
#define RADIO_MSG_TYPE                          30002
// define map property
#define GRIDMAP_UNKNOWN     0
#define GRIDMAP_GROUND      1
#define GRIDMAP_OBS_STAND   2
#define GRIDMAP_SHADOW      3
#define GRIDMAP_DANGEROUS   4
#define GRIDMAP_WATER       5

#define GRIDMAP_DYN_CAR     10
#define GRIDMAP_DYN_CARPRE  11
#define GRIDMAP_DYN_PED     12
#define GRIDMAP_DYN_PEDPRE  13

#define GRIDMAP_NEGA        20

#define GRIDMAP_ROAD        30

#define GRIDMAP_PATH        40

#ifndef LAT0
#define LAT0 34.267163
#endif

#ifndef LNG0
#define LNG0 108.093283
#endif



#endif
