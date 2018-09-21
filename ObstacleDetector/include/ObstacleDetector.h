/* Created  :   Linhui
 * Date     :   2016-05-16
 * Usage    :   Declaration of object detection class module, which concerns about
 *              data receiving and sending, as well as loop control
*/

#ifndef NEGATIVEDETECTOR_HH
#define NEGATIVEDETECTOR_HH

//#include "rcs.hh"               // Common RCS definitions
//#include "nml_mod.hh"           // NML_MODULE definitions
//#include "Lidar32_Datan.h"
//#include "Lidar16_Datan.h"
//#include "Lidar4_Datan.h"
//#include "Lidar1_Datan.h"
//#include "GPSINS_Datan.h"
//#include "LocalDEMn.h"          // NML Messages for LocalDEMMap
//#include "Monitor_CMDn.h"
//#include "PositiveInfon.h"      // NML Messages for TerrianMap
#include "alv_data.h"


class OBSTACLEDETECTOR_MODULE
{
public:
    OBSTACLEDETECTOR_MODULE();   // Constructor
    ~OBSTACLEDETECTOR_MODULE();  // Destructor

    // Overwrited Virtual Functions
    virtual void PRE_PROCESS();
    virtual void DECISION_PROCESS();
    virtual void POST_PROCESS();
    virtual void INTIALIZE_NML();

    void generate_output(ALV_DATA *alv_data);

    // auxiliary Input NML Channels
//    NML *HDL32DATA_CHANNEL;     // NML Channel for hdl32 lidar data
//    LIDAR32_MSG *pLidar32Data;

//    NML *VLP16DATA_CHANNEL;     // NML Channel for two VLP16 lidar data
//    LIDAR16_MSG *pLidar16Data;  // NML Message from the Channel

//    NML *LIDAR4DATA_CHANNEL;    // NML Channel for 4-laser lidar data
//    LIDAR4_MSG *pLidar4Data;

//    NML *LIDAR1DATA_CHANNEL;    // NML Channel for single-laser lidar data
//    LIDAR1_MSG *pLidar1Data;

//    NML *MONITORCMD_CHANNEL;    // NML Channel for cmd
//    MONITORCONTROL_CMD_MSG *monitor_cmd;

//    NML *OBSTACLE_CHANNEL;      // NML Channel for obstacle detection result map
//    POSITIVEINFO_MSG *Obstacle_data;	// NML Data for obstacle detection result map

//    NML *LOCALDEM_CHANNEL;              // NML Channel for Local DEM Map
//    LOCALDEM_MSG *LocalDEM_data;        // NML Data for Local DEM Map

    // flags
    bool bRecvData_32;          // receive HDL 32 data
    bool bRecvData_16;          // receive VLP 16 data
    bool bRecvData_4;           // receive 4-laser lidar data
    bool bRecvData_1;           // receive single-laser lidar data
    bool bObsGenerate;          // whether the obstacle result is generated
    bool bLocalDEMGenerate;     // whether the localDEM is ready
    bool bRecvMonitor;          // receive monitor command

private:
    // Add custom variables and functions here.
    NML *CMD_CHANNEL;
    NML *STAT_CHANNEL;
};
#endif 	// NEGATIVEDETECTOR_HH

