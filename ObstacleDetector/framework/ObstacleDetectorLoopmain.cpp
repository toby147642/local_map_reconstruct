/* Created  :   Ye Yuwen
 * Date     :   2018-09-21
 * Usage    :   Definition of main loop function
*/
#include <cstdlib>             // exit()
#include <csignal>             // SIGINT, signal()
#include <sys/time.h>
#include <cstdio>

#define OFF_LINE               //to use offline function

#ifdef OFF_LINE
#include <unistd.h>            // io library to list filenames in some directories
#include <dirent.h>
#endif

#include <iostream>
#include <sstream>
#include <iomanip>
#include <sys/time.h>
#include <stdio.h>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include "alv_data.h"
#include "detect_positive_obstacle.h"
//#include "detect_negative_obstacle.h"
#include "program.h"
#include "negativeobstacle.h"
//#define SAVE_PTS_FILE

//#define TEST_DATA


using namespace std;
using namespace cv;


/* signal handler for ^C in linux terminal
*/

/* main function
 *
 * loop work flow:
 *  1.pre_process
 *  2.decision_process, receive cmd from monitor to determine continue, quit, save data, etc.
 *  3.positive & negative obstacle detection
 *  4.post_process, write negative obstacle message to nml buffer
 *  5.timer.wait(), to wait until 100 ms, thus making loop at 10 Hz.
*/
#ifndef OFF_LINE
int main()
{
    set_rcs_print_destination(RCS_PRINT_TO_STDOUT);
    nml_start();

    RCS_TIMER timer(0.1);
    OBSTACLEDETECTOR_MODULE ObstacleDetector;

    // set the SIGINT handler
    signal(SIGINT, ObstacleDetectorLoop_quit);

    ALV_DATA *alv_data = new ALV_DATA;
    NEGATIVE_DETECTOR *negative_detector = NEGATIVE_DETECTOR::get_instance();
    POSITIVE_DETECTOR *positive_detector = POSITIVE_DETECTOR::get_instance();

    // setup and init para table
    if(!alv_data->setup())
    {
        cerr<<"alv_data->setup() failed!"<<endl;
        exit(-1);
    }

    int frameNo = 0;
    struct timeval t_start, t_end;
    double time_use;
    while(!ObstacleDetectorLoop_done)
    {
        cout<<"Frame #"<<setfill('0')<<setw(6)<<frameNo;
        gettimeofday(&t_start, NULL);
        alv_data->cleanup();    // DO NOT FORGET TO CLEANUP !!!

        // parse data
        ObstacleDetector.PRE_PROCESS();       // read data from network
        ObstacleDetector.DECISION_PROCESS();  // monitor can pass cmd in here, for instance, to terminate process
        if(ObstacleDetector.bRecvData_32)
            alv_data->parse_32data_online(ObstacleDetector.pLidar32Data);
        if(ObstacleDetector.bRecvData_16)
            alv_data->parse_16data_online(ObstacleDetector.pLidar16Data);
        if(ObstacleDetector.bRecvData_4)
            alv_data->parse_4data_online(ObstacleDetector.pLidar4Data);
        if(ObstacleDetector.bRecvData_1)
            alv_data->parse_1data_online(ObstacleDetector.pLidar1Data);

        // process
        positive_detector->detect(alv_data);
        negative_detector->detect(alv_data);
//        alv_data->show_cloud();
        alv_data->show_result();
        ObstacleDetector.generate_output(alv_data);

        // timer wait
        frameNo++;
        alv_data->cleanup();

        gettimeofday(&t_end, NULL);
        time_use = (t_end.tv_sec-t_start.tv_sec)*1000.0 + (t_end.tv_usec - t_start.tv_usec)/1000.0;
        cout<<", consume time = "<< time_use<<" ms"<<endl;

        // output result
        ObstacleDetector.POST_PROCESS();
        timer.wait();
    }

    // free memory
    delete alv_data;
    nml_cleanup();
    return 0;
}



#else
// offline mode
int main()
{

    ALV_DATA *alv_data = new ALV_DATA;
    negativeObstacle *negative_detector = negativeObstacle::get_instance();
    POSITIVE_DETECTOR *positive_detector = POSITIVE_DETECTOR::get_instance();   //new a new instance

    // setup and init para table
    if(!alv_data->setup())
    {
        cerr<<"alv_data->setup() failed! "<<endl;
        exit(-1);
    }

    // read file name index
    string base_dir = "/home/yyw/dataSet_0809";
    string folder = "0809_1645"; // 0809_1524 0809_1536/ 0809_1538/ 0809_1551 0809_1618 0809_1640/ 0809_1641/ 0809_1645/ 0913_0000
    string HDL32Dir = base_dir+"/LIDAR32_DATA/"+folder;
    DIR *dir;
    struct dirent *ptr;
    vector<string> frameNo;
    if ((dir=opendir(HDL32Dir.c_str())) == NULL)
    {
        perror("Open dir error...");
        exit(1);
    }
    while ((ptr=readdir(dir)) != NULL)
    {
        if(strcmp(ptr->d_name,".")==0 || strcmp(ptr->d_name,"..")==0)    //current dir OR parrent dir
            continue;
        else if(ptr->d_type == 8)   //file
        {
            string name(ptr->d_name);
#ifndef TEST_DATA
            name.erase(0,6);        // erase the prefix "Lidar_"
#endif
            int pos = name.find(".bin");
            name.erase(pos, 4);     // erase the subfix ".bin"
            frameNo.push_back(name);
            //cout << "name:  "<<name<<endl;
        }
        else
            continue;
    }
    closedir(dir);
    sort(frameNo.begin(), frameNo.end());   // sort

    //  go into loop
    struct timeval t_start, t_end;
    double time_use;
    unsigned int i = 0;

    while( i < frameNo.size())
    {
        i ++;
        cout<<"Frame No."<<i;
        gettimeofday(&t_start, NULL);
        alv_data->cleanup();    // DO NOT FORGET TO CLEANUP !!!

        // parse data
        stringstream ss;
        string lidar32filename;
        string imgfliename;

#ifndef TEST_DATA
        ss << base_dir << "/LIDAR32_DATA/" << folder << "/Lidar_"<<frameNo[i]<<".bin";
#else
        ss << base_dir << "/LIDAR32_DATA/" << folder << "/"<<frameNo[i]<<".bin";
#endif
        ss >> lidar32filename;
        ss.str("");
        ss.clear();

        ss << base_dir << "/IMG_DATA/" << folder << "/" << "ImgColor_" <<frameNo[i]<<".jpg";
        ss >> imgfliename;
        ss.str("");
        ss.clear();

        alv_data->parse_32data_offline(lidar32filename);    //loading 32 lidar data

        Mat colorimg = imread(imgfliename);
        if(!colorimg.empty())
        {
            imshow("color", colorimg);
            waitKey(1);
        }

        // process
        positive_detector->detect(alv_data);
        //negative_detector->detect(alv_data);

        // save my data

#ifdef SAVE_PTS_FILE
        string lidar32asc;
        ss.str("");
        ss.clear();
        ss << base_dir << "/ASC/" << folder << "/"<< frameNo[i] <<".asc";
        ss >> lidar32asc;
        alv_data->save_32pt_txt_dis_cnt_beam(lidar32asc);
#endif
        alv_data->show_result();

        // timer wait
        gettimeofday(&t_end, NULL);
        time_use = (t_end.tv_sec-t_start.tv_sec)*1000.0 + (t_end.tv_usec - t_start.tv_usec)/1000.0;
        cout<<", consume time = "<< time_use<<" ms"<<endl;

        if(frameNo[i] == "660630"){
            for(int i=347; i<358; i++){
                cout << "angle: " << i <<"  ";
                for(int j=6; j<10; j++)
                    cout<<alv_data->beam_dis_max_mm[i][j]<<"  ";
                cout << endl;
            }
            waitKey(-1);
        }
        cout << frameNo[i] <<endl;
    }

    // free memory
    delete alv_data;
    return 0;
}
#endif
