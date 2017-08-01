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

// using namespace std;
// using namespace cv;

class LaneDetect
{
public:
    LaneDetect();
	
	void Init(cv::Mat startFrame);
	
	void DetectLane(cv::Mat &nxt);
	
    void UpdateSensitivity();

    void MarkLane();

    void RemoveInvalidBlob();


private:
	
public:
    cv::Mat m_currFrame; //stores the upcoming frame
    cv::Mat m_frame_midstep;      //stores intermediate results
    cv::Mat m_frame_laneBlobs;     //stores the final lane segments
    
    int m_ROIrows;
    int m_vertical_left;
    int m_vertical_right;
    int m_vertical_top;
    int m_smallLaneArea;
    int m_longLane;
    int  m_vanishingPt;
    float m_maxLaneWidth;

    //to store various blob properties
    int m_minSize;
    int m_ratio;


};//end of class LaneDetect

