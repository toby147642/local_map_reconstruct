/* Created  :   Linhui
 * Date     :   2016-05-16
 * Usage    :   Definition of main loop function
*/
#include <cstdlib>             // exit()
#include <csignal>             // SIGINT, signal()
#include <sys/time.h>
#include <cstdio>
#include <iostream>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <sys/time.h>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include "rcs.hh"               // Common RCS definitions
#include "ObstacleDetector.h"	// definition of OBSTACLEDETECTOR_MODULE
#include "alv_data.h"
#include "detect_positive_obstacle.h"
#include "detect_negative_obstacle.h"
#include "program.h"

using namespace std;
using namespace cv;

struct PointXYZI
{
  PointXYZI(float x0 = 0, float y0 = 0, float z0 = 0, float intensity0 = 0, bool idValid = false):
    x(x0), y(y0), z(z0), intensity(intensity0), valid(idValid){}
  float x;
  float y;
  float z;
  float intensity;
  bool valid;
};

vector<PointXYZI> laserCloud[32];

void readPointCloud(string filename)
{
  // clear
  for(int i = 0; i < 32; i++)
    laserCloud[i].clear();
  // read
  ifstream infile;
  infile.open(filename);
  stringstream sline;
  string line;

  if(!infile)
  {
      cerr<<"***Error: can't open hdl32 para file \""<<filename<<"\""<<endl;
      return;
  }

  float x, y, z, intensity;

  sline.str("");
  sline.clear();
  line.clear();
  while(getline(infile, line))
  {
      sline.str("");
      sline.clear();
      sline << line;
      sline >> x >> y >> z >> z >> intensity;
      laserCloud[(int)intensity].push_back(PointXYZI(x*100.0, y*100.0, z*100.0, intensity, true));
  }
  infile.close();
}

void dataInterface(alv_Point3f **pointcloud, const int beamNum, const int MAXPointPerBeam)
{
    for(int beam=0; beam<beamNum; beam++)
    {
        int CNT = MAXPointPerBeam < laserCloud[beam].size() ? MAXPointPerBeam : laserCloud[beam].size();
        for(int cnt=0; cnt < CNT; cnt++)
        {
            alv_Point3f *p = reinterpret_cast<alv_Point3f*>(pointcloud) + beam * MAXPointPerBeam + cnt;
            p->x = laserCloud[beam][cnt].x;
            p->y = laserCloud[beam][cnt].y;
            p->z = laserCloud[beam][cnt].z;
            p->intensity = laserCloud[beam][cnt].intensity;
            p->valid = true;
        }
    }
}


/* main function
 *
 * loop work flow:
 *  1.pre_process
 *  2.decision_process, receive cmd from monitor to determine continue, quit, save data, etc.
 *  3.positive & negative obstacle detection
 *  4.post_process, write negative obstacle message to nml buffer
 *  5.timer.wait(), to wait until 100 ms, thus making loop at 10 Hz.
*/

// offline mode
int main()
{
    set_rcs_print_destination(RCS_PRINT_TO_STDOUT);
    nml_start();

    RCS_TIMER timer(0.1);
    OBSTACLEDETECTOR_MODULE ObstacleDetector;

    ALV_DATA *alv_data = new ALV_DATA;
    NEGATIVE_DETECTOR *negative_detector = NEGATIVE_DETECTOR::get_instance();
    POSITIVE_DETECTOR *positive_detector = POSITIVE_DETECTOR::get_instance();

    // setup and init para table
    if(!alv_data->setup())
    {
        cerr<<"alv_data->setup() failed! "<<endl;
        exit(-1);
    }

    //  go into loop
    struct timeval t_start, t_end;
    struct timeval t_start_1, t_end_1;
    double time_use;
    for(int i = 2; i< 186; i++)
    {
        cout<<"frame: "<<i;
        gettimeofday(&t_start, NULL);
        alv_data->cleanup();    // DO NOT FORGET TO CLEANUP !!!

        // parse data
        stringstream ss;
        string lidar32filename;
        string lidar16filename_L;
        string lidar16filename_R;

        const string subFolder = "Negative/场景1"; // Positive
        ss << "/root/LOAM_DataBag/ObjDetectionData/" << subFolder << "/selected/Fusion_32-" << i << ".txt";
        ss >> lidar32filename;
        ss.str("");
        ss.clear();

        ss << "/root/LOAM_DataBag/ObjDetectionData/" << subFolder << "/selected/Fusion_16Left-" << i << ".txt";
        ss >> lidar16filename_L;
        ss.str("");
        ss.clear();

        ss << "/root/LOAM_DataBag/ObjDetectionData/" << subFolder << "/selected/Fusion_16Right-" << i << ".txt";
        ss >> lidar16filename_R;
        ss.str("");
        ss.clear();

        readPointCloud(lidar32filename);
        dataInterface(reinterpret_cast<alv_Point3f**>(alv_data->lidar32_pointcloud), 32, HDL32_BEAM_POINTSIZE);
        readPointCloud(lidar16filename_L);
        dataInterface(reinterpret_cast<alv_Point3f**>(alv_data->lidar16_pointcloud_L), 16, VLP16_BEAM_POINTSIZE);
        readPointCloud(lidar16filename_R);
        dataInterface(reinterpret_cast<alv_Point3f**>(alv_data->lidar16_pointcloud_R), 16, VLP16_BEAM_POINTSIZE);

        // process
        gettimeofday(&t_start_1, NULL);
        positive_detector->detect(alv_data);
        gettimeofday(&t_end_1, NULL);
        time_use = (t_end_1.tv_sec-t_start_1.tv_sec)*1000.0 + (t_end_1.tv_usec - t_start_1.tv_usec)/1000.0;
        cout<<", positive = "<< time_use<<" ms";

        gettimeofday(&t_start_1, NULL);
        negative_detector->detect(alv_data);
        gettimeofday(&t_end_1, NULL);
        time_use = (t_end_1.tv_sec-t_start_1.tv_sec)*1000.0 + (t_end_1.tv_usec - t_start_1.tv_usec)/1000.0;
        cout<<", negative = "<< time_use<<" ms";

        //alv_data->show_cloud();
        alv_data->show_result();
        cv::waitKey(1);

        // timer wait
        alv_data->cleanup();
        gettimeofday(&t_end, NULL);
        time_use = (t_end.tv_sec-t_start.tv_sec)*1000.0 + (t_end.tv_usec - t_start.tv_usec)/1000.0;
        cout<<", consume time = "<< time_use<<" ms"<<endl;
    }

    // free memory
    delete alv_data;
    return 0;
}
