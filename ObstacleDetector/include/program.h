/* Created  :   Ye Yuwen
 * Date     :   2018-09-21
 * Usage    :   Definition of some program parameters, such as lidar parser mode,
 *              etc.
*/
#ifndef PROGRAM_H
#define PROGRAM_H

/* VLP16 work mode definition:
 *  1.  STRONGEST, each laser returns the strongest value;
 *  2.  LAST, each laser returns the newest value;
 *  3.  DUAL, each laser returns both values, making twice the size of raw data
*/
//#define DUAL_MODE

static const int FRAME_FUSION_NUM = 1;

//#define OFF_LINE              //to use offline function
//#define SAVE_PTS_FILE         // output point cloud coordinates into text file

#endif // PROGRAM_H
