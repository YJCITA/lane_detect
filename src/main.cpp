/*TODO
 * improve edge linking
 * remove blobs whose axis direction doesnt point towards vanishing pt
 * Parallelisation
 * lane prediction
*/
#include <opencv2/core/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <vector>
#include <algorithm>
#include <math.h>
#include <time.h>

#include "lane_detect.h"
using namespace std;
using namespace cv;

void Run(string path)
{
    Mat frame;
    VideoCapture cap(path); // open the video file for reading

    if( !cap.isOpened() )  // if not success, exit program
        cout << "Cannot open the video file" << endl;

    //cap.set(CV_CAP_PROP_POS_MSEC, 300); //start the video at 300ms
    double fps = cap.get(CV_CAP_PROP_FPS); //get the frames per seconds of the video
    cout << "Input video's Frame per seconds : " << fps << endl;

    cap.read(frame);
    LaneDetect lane_detect;
	lane_detect.Init(frame);

    while(1){
         // read a new frame from video
        if(!cap.read(frame)) {
			//if not success, break loop
            cout << "Cannot read the frame from video file" << endl;
            break;
        }

        cvtColor(frame, frame, CV_BGR2GRAY);
        lane_detect.DetectLane(frame);
		
		// 按键事件，空格暂停，其他跳出循环
		if (cvWaitKey(10) == 32){
			bool is_quite = false;
			while(!is_quite)
				if(cvWaitKey(10) == 32)
					is_quite = true;
		}
    }
}

int main()
{
    Run("/home/yj/bak/data/lane/data_develop/2/rec_20170518_055838.mp4");
    destroyAllWindows();
}
