//// Created  :   Linhui
//// Date     :   2016-05-16
//// Usage    :   Definition of negtive object detection class module
//#include <iostream>
//#include <opencv2/opencv.hpp>
//#include <stdio.h>
//#include "ObstacleDetector.h"
//using namespace cv;
//using namespace std;

///* Constructor
// * something important: before initialize nml, one should creat an LIDAR16_MSG first.
//*/
//OBSTACLEDETECTOR_MODULE::OBSTACLEDETECTOR_MODULE()
//{
//    /*
//    pLidar32Data = new LIDAR32_MSG;
//    pLidar16Data = new LIDAR16_MSG;
//    pLidar4Data = new LIDAR4_MSG;
//    pLidar1Data = new LIDAR1_MSG;
//    monitor_cmd = new MONITORCONTROL_CMD_MSG;
//    Obstacle_data = new POSITIVEINFO_MSG;
//    LocalDEM_data = new LOCALDEM_MSG;

//    bRecvData_32 = false;
//    bRecvData_16 = false;
//    bRecvData_4 = false;
//    bRecvData_1 = false;
//    bObsGenerate = false;
//    bLocalDEMGenerate = false;
//    bRecvMonitor = false;

//#ifndef OFF_LINE
//    INTIALIZE_NML();
//#endif
//*/
//}
///* Destructor
//*/
////OBSTACLEDETECTOR_MODULE::~OBSTACLEDETECTOR_MODULE()
////{
////    delete HDL32DATA_CHANNEL;
////    delete VLP16DATA_CHANNEL;
////    delete LIDAR4DATA_CHANNEL;
////    delete LIDAR1DATA_CHANNEL;
//////    delete MONITORCMD_CHANNEL;
////    delete OBSTACLE_CHANNEL;
////    delete LOCALDEM_CHANNEL;
//////    delete CMD_CHANNEL;
//////    delete STAT_CHANNEL;

////    delete pLidar32Data;
////    delete pLidar16Data;
////    delete pLidar4Data;
////    delete pLidar1Data;
////    delete monitor_cmd;
////    delete Obstacle_data;
////    delete LocalDEM_data;
////}

///* INTIALIZE_NML
// */
//void OBSTACLEDETECTOR_MODULE::INTIALIZE_NML()
//{
////    STAT_CHANNEL = new NML(PositiveDetectorFormat, "Positive_sts", "PositiveDetector", "ZJUALV.nml");

//    /*
//    HDL32DATA_CHANNEL = new NML(Lidar32_DataFormat, "Lidar32Data", "PositiveDetector", "ZJUALV.nml");
//    pLidar32Data = (LIDAR32_MSG *)HDL32DATA_CHANNEL->get_address();

//    VLP16DATA_CHANNEL = new NML(Lidar16_DataFormat, "Lidar16Data", "PositiveDetector", "ZJUALV.nml");
//    pLidar16Data = (LIDAR16_MSG *)VLP16DATA_CHANNEL->get_address();

//    LIDAR4DATA_CHANNEL = new NML(Lidar4_DataFormat, "Lidar4Data", "PositiveDetector", "ZJUALV.nml");
//    pLidar4Data = (LIDAR4_MSG *)LIDAR4DATA_CHANNEL->get_address();

//    LIDAR1DATA_CHANNEL = new NML(Lidar1_DataFormat, "Lidar1Data", "PositiveDetector", "ZJUALV.nml");
//    pLidar1Data = (LIDAR1_MSG *)LIDAR1DATA_CHANNEL->get_address();

////    MONITORCMD_CHANNEL = new NML(MONITORCONTROL_CMDFormat, "MonitorControl_cmd", "PositiveDetector", "ZJUALV.nml");
////    monitor_cmd = (MONITORCONTROL_CMD_MSG *)MONITORCMD_CHANNEL->get_address();

//    //PositiveObstacle
//    OBSTACLE_CHANNEL = new NML(PositiveInfoFormat, "PositiveInfo", "PositiveDetector", "ZJUALV.nml");

//    //TerrianMap
//    LOCALDEM_CHANNEL = new NML(LocalDEMFormat, "LocalDEM", "PositiveDetector", "ZJUALV.nml");
//    */
//}

///* PRE_PROCESS
// *
// * The PRE_PROCESS function is called every cycle after the command and
// * subordinates status have been read but before DECISION_PROCESS is called.
// * It is intended to be used for tasks such as sensory processing that should
// * be performed every cycle regardless of the current command or state.
//*/
//void OBSTACLEDETECTOR_MODULE::PRE_PROCESS()
//{
//    bRecvData_32 = false;
//    bRecvData_16 = false;
//    bRecvData_4 = false;
//    bRecvData_1 = false;
//    bRecvMonitor = false;
//    bObsGenerate = false;
//    bLocalDEMGenerate = false;

//    switch(HDL32DATA_CHANNEL->read())
//    {
//    case 0:
//        cout<<"No new  lidar32 Data!!"<<endl;
//        break;
//    case -1:
//        cerr<<"lidar32 communication Error!!"<<endl;
//        break;
//    case LIDAR32_MSG_TYPE:
//        bRecvData_32 = true;
//        break;
//    default:
//        break;
//    }

//    switch(VLP16DATA_CHANNEL->read())
//    {
//    case 0:
//        cout<<"No new  lidar16 Data!!"<<endl;
//        break;
//    case -1:
//        cerr<<"lidar16 communication Error!!"<<endl;
//        break;
//    case LIDAR16_MSG_TYPE:
//        bRecvData_16 = true;
//        break;
//    default:
//        break;
//    }

//    switch(LIDAR4DATA_CHANNEL->read())
//    {
//    case 0:
//        cout<<"No new  lidar4 Data!!"<<endl;
//        break;
//    case -1:
//        cerr<<"lidar4 communication Error!!"<<endl;
//        break;
//    case LIDAR4_MSG_TYPE:
//        bRecvData_4 = true;
//        break;
//    default:
//        break;
//    }

//    switch(LIDAR1DATA_CHANNEL->read())
//    {
//    case 0:
//        cout<<"No new  lidar1 Data!!"<<endl;
//        break;
//    case -1:
//        cerr<<"lidar1 communication Error!!"<<endl;
//        break;
//    case LIDAR1_MSG_TYPE:
//        bRecvData_1 = true;
//        break;
//    default:
//        break;
//    }
//}

///* DECISION_PROCESS
// *
// * The DECISION_PROCESS function is called every cycle as long as there is a non-zero
// * command. It is expected to call a command function based on commandInData->type.
//*/
//void OBSTACLEDETECTOR_MODULE::DECISION_PROCESS()
//{
//    ;
//}


//void OBSTACLEDETECTOR_MODULE::generate_output(ALV_DATA *alv_data)
//{
//    bObsGenerate = true;
//    GRID **grids = alv_data->grid_map.grids;
//    Obstacle_data->VehicleGridX = alv_data->para_table.grid_center_col;
//    Obstacle_data->VehicleGridY = alv_data->para_table.grid_center_row;
//    const int MAP_HEIGHT = alv_data->para_table.grid_rows;
//    const int MAP_WIDTH = alv_data->para_table.grid_cols;

//    for(int row=0; row<MAP_HEIGHT; row++)
//    {
//        for(int col=0; col<MAP_WIDTH; col++)
//        {
//            if(grids[row][col].attribute == GRID_POS_OBS && grids[row][col].dis_height >= 60.0)
//                grids[row][col].attribute = GRID_DANGEROUS;
//        }
//    }

//    for(int row=0; row<MAP_HEIGHT; row++)
//    {
//        for(int col=0; col<MAP_WIDTH; col++)
//        {
//            if(grids[MAP_HEIGHT-1-row][col].attribute == GRID_TRAVESABLE)       // traversable
//                Obstacle_data->PositiveObstacle[row*MAP_WIDTH+col] = GRIDMAP_GROUND;
//            else if(grids[MAP_HEIGHT-1-row][col].attribute == GRID_POS_OBS)     // positive ob
//                Obstacle_data->PositiveObstacle[row*MAP_WIDTH+col] = GRIDMAP_OBS_STAND;
//            else if(grids[MAP_HEIGHT-1-row][col].attribute == GRID_NEG_OBS)     // negative ob
//                Obstacle_data->PositiveObstacle[row*MAP_WIDTH+col] = GRIDMAP_NEGA;
//            else if(grids[MAP_HEIGHT-1-row][col].attribute == GRID_UNKNOWN)     // unknown
//                Obstacle_data->PositiveObstacle[row*MAP_WIDTH+col] = GRIDMAP_UNKNOWN;
//            else if(grids[MAP_HEIGHT-1-row][col].attribute == GRID_DANGEROUS)   // cliff
//                Obstacle_data->PositiveObstacle[row*MAP_WIDTH+col] = GRIDMAP_DANGEROUS;
//            else if(grids[MAP_HEIGHT-1-row][col].attribute == GRID_OCCULUSION)  // occlusion
//                Obstacle_data->PositiveObstacle[row*MAP_WIDTH+col] = GRIDMAP_SHADOW;
//            else if(grids[MAP_HEIGHT-1-row][col].attribute == GRID_WATER)       // water
//                Obstacle_data->PositiveObstacle[row*MAP_WIDTH+col] = GRIDMAP_WATER;
//            else if(grids[MAP_HEIGHT-1-row][col].attribute == GRID_SUSPEND_OBS) // suspended ob
//                Obstacle_data->PositiveObstacle[row*MAP_WIDTH+col] = GRIDMAP_GROUND;
////            else if(grids[MAP_HEIGHT-1-row][col].attribute == GRID_ROAD_EDGE)   // road edge
////                Obstacle_data->PositiveObstacle[row*MAP_WIDTH+col] = 8;
//        }
//    }

//    // copy lidar32 data
//    memcpy(Obstacle_data->frame_data, pLidar32Data->frame_data, sizeof(unsigned char)*HDL32_BUFFER_SIZE);
//    // copy lidar16 data
//    memcpy(Obstacle_data->vlp_data_L, pLidar16Data->frame_dataL, sizeof(unsigned char)*VLP16_BUFFER_SIZE);
//    memcpy(Obstacle_data->vlp_data_R, pLidar16Data->frame_dataR, sizeof(unsigned char)*VLP16_BUFFER_SIZE);
//    Obstacle_data->WorkingFlag_Lidar16 = true;

//    // copy lidar4 data
//    Obstacle_data->WorkingFlag_Lidar4 = true;
//    for(int i=0; i<LIDAR4_MAX_POINT_SIZE; i++)
//    {
//        Obstacle_data->layer[i] = alv_data->lidar4_Data->origine_point[i].layer;
//        Obstacle_data->angle[i] = alv_data->lidar4_Data->origine_point[i].angle;
//        Obstacle_data->distance[i] = alv_data->lidar4_Data->origine_point[i].distance;
//    }
//    // copy lidar1 data
//    Obstacle_data->WorkingFlag_Lidar1 = true;
//    memcpy(Obstacle_data->Lidar1_dataL, pLidar1Data->frame_dataL, sizeof(unsigned char)*LIDAR1_BUFFER_SIZE*2);
//    memcpy(Obstacle_data->Lidar1_dataR, pLidar1Data->frame_dataR, sizeof(unsigned char)*LIDAR1_BUFFER_SIZE*2);
//    Obstacle_data->cntL = pLidar1Data->cntL;
//    Obstacle_data->cntR = pLidar1Data->cntR;
//}


///* POST_PROCESS
// *
// * The POST_PROCESS function is called every cycle after DECISION_PROCESS is
// * called but before the status and the subordinates commands  have been written.
// * It is intended to be used for tasks such as output filters that should be
// * performed every cycle regardless of the current command or state.
//*/
//void OBSTACLEDETECTOR_MODULE::POST_PROCESS()
//{
//    if(bObsGenerate)
//    {
//        OBSTACLE_CHANNEL->write(Obstacle_data);
//    }
//}
