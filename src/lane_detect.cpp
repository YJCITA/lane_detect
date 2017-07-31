#include "lane_detect.h"
// using namespace std;
// using namespace cv;

LaneDetect::LaneDetect()
{
}

void LaneDetect::Init(cv::Mat startFrame)
{
// 	m_currFrame = startFrame;    //if image has to be processed at original size
// 	m_currFrame = cv::Mat(320, 480, CV_8UC1, 0.0);     //initialised the image size to 320x480
	// -YJ-
	m_currFrame = cv::Mat(640, 960, CV_8UC1, 0.0);     //initialised the image size 
	resize(startFrame, m_currFrame, m_currFrame.size());    // resize the input to required size

	m_frame_temp1 = cv::Mat(m_currFrame.rows, m_currFrame.cols, CV_8UC1,0.0);//stores possible lane markings
	m_frame_temp2 = cv::Mat(m_currFrame.rows, m_currFrame.cols, CV_8UC1,0.0);//stores finally selected lane marks

	m_vanishingPt = m_currFrame.rows/2 + 80;                       //for simplicity right now
	m_ROIrows = m_currFrame.rows - m_vanishingPt;                 //rows in region of interest
// 	m_minSize = 0.00015 * (m_currFrame.cols*m_currFrame.rows);    //min size of any region to be selected as lane
	m_minSize = 0.00003 * (m_currFrame.cols*m_currFrame.rows);    //min size of any region to be selected as lane
	m_maxLaneWidth = 0.025 * m_currFrame.cols;                     //approximate max lane width based on image size
	m_smallLaneArea = 7 * m_minSize;
	m_longLane = 0.3 * m_currFrame.rows;
	m_ratio = 4;

	//these mark the possible ROI for vertical lane segments and to filter vehicle glare
	m_vertical_left  = 2*m_currFrame.cols/5;
	m_vertical_right = 3*m_currFrame.cols/5;
	m_vertical_top   = 2*m_currFrame.rows/3;
	
// 	namedWindow("currframe", 2);
	cv::namedWindow("lane", 2);
	cv::namedWindow("midstep", 2);
	cv::namedWindow("laneBlobs", 2);
}


void LaneDetect::DetectLane(cv::Mat &nxt)
{
// 	m_currFrame = nxt;                        //if processing is to be done at original size
	resize(nxt ,m_currFrame, m_currFrame.size()); //resizing the input image for faster processing
	
	//medianBlur(m_currFrame, m_currFrame,5 );
	// UpdateSensitivity();
	//ROI = bottom half 
	// TODO: 根据外参数设定
	for(int i = m_vanishingPt; i < m_currFrame.rows; i++){
		for(int j=0; j < m_currFrame.cols; j++){
			m_frame_temp1.at<uchar>(i,j) = 0;
			m_frame_temp2.at<uchar>(i,j) = 0;
		}
	}

// 	imshow("currframe", m_currFrame);
	RemoveBlob();
	
}


void LaneDetect::UpdateSensitivity()
{
	int total = 0, average = 0;
	for(int i= m_vanishingPt; i<m_currFrame.rows; i++)
		for(int j= 0 ; j<m_currFrame.cols; j++)
			total += m_currFrame.at<uchar>(i,j);
		
	average = total/(m_ROIrows*m_currFrame.cols);
	std::cout<<"average : "<<average<<std::endl;
}

void LaneDetect::MarkLane()
{
	int diff = 0, diffL = 0, diffR = 0;
	int laneWidth = 0;
	int diffThreshTop = 0;
    int diffThreshLow = 0;
	for(int i = m_vanishingPt; i < m_currFrame.rows; i++){
		//IF COLOUR IMAGE IS GIVEN then additional check can be done
		// lane markings RGB values will be nearly same to each other(i.e without any hue)

		//min lane width is taken to be 5
		laneWidth = 5 + m_maxLaneWidth*(i - m_vanishingPt)/m_ROIrows;
		for(int j=laneWidth; j<m_currFrame.cols- laneWidth; j++){
			diffL = m_currFrame.at<uchar>(i,j) - m_currFrame.at<uchar>(i,j-laneWidth);
			diffR = m_currFrame.at<uchar>(i,j) - m_currFrame.at<uchar>(i,j+laneWidth);
			diff  =  diffL + diffR - abs(diffL-diffR);

			//1 right bit shifts to make it 0.5 times
			diffThreshLow = m_currFrame.at<uchar>(i,j)>>1;
			diffThreshTop = 1.2*m_currFrame.at<uchar>(i,j);

			//both left and right differences can be made to contribute
			//at least by certain threshold (which is >0 right now)
			//total minimum Diff should be atleast more than 5 to avoid noise
			if (diffL > 0 && diffR > 0 && diff > 4)
				if(diff>=diffThreshLow /*&& diff<= diffThreshTop*/ )
					m_frame_temp1.at<uchar>(i,j)=255;
		}
	}

}

void LaneDetect::RemoveBlob()
{
	float contour_area;
    float blob_angle_deg;
    float bounding_width;
    float bounding_length;
	cv::Size2f sz;
    std::vector< std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::RotatedRect rotated_rect;
	
	
	MarkLane();
	// find all contours in the binary image
	m_frame_temp1.copyTo(m_binary_image);
	findContours(m_binary_image, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

	// for removing invalid blobs
	if (!contours.empty()){
		for (size_t i=0; i<contours.size(); ++i){
			//====conditions for removing contours====//
			contour_area = contourArea(contours[i]) ;
			//blob size should not be less than lower threshold
			if(contour_area > m_minSize){
				rotated_rect = minAreaRect(contours[i]);
				sz = rotated_rect.size;
				bounding_width = sz.width;
				bounding_length = sz.height;

				//openCV selects length and width based on their orientation
				//so angle needs to be adjusted accordingly
				blob_angle_deg = rotated_rect.angle;
				if (bounding_width < bounding_length)
					blob_angle_deg = 90 + blob_angle_deg;

				//if such big line has been detected then it has to be a (curved or a normal)lane
				if(bounding_length>m_longLane || bounding_width >m_longLane){
					cv::drawContours(m_currFrame, contours,i, cv::Scalar(255), CV_FILLED, 8);
					cv::drawContours(m_frame_temp2, contours,i, cv::Scalar(255), CV_FILLED, 8);
				}else if ((blob_angle_deg <-10 || blob_angle_deg >-10 ) && ((blob_angle_deg > -70 && blob_angle_deg < 70 ) ||
							(rotated_rect.center.y > m_vertical_top && rotated_rect.center.x > m_vertical_left && rotated_rect.center.x < m_vertical_right)))
				{
					//angle of orientation of blob should not be near horizontal or vertical
					//vertical blobs are allowed only near center-bottom region, where centre lane mark is present
					//length:width >= m_ratio for valid line segments
					//if area is very small then m_ratio limits are compensated
					if ((bounding_length/bounding_width)>=m_ratio || (bounding_width/bounding_length)>=m_ratio
							||(contour_area< m_smallLaneArea &&  ((contour_area/(bounding_width*bounding_length)) > .75) &&
								((bounding_length/bounding_width)>=2 || (bounding_width/bounding_length)>=2)))
					{
						cv::drawContours(m_currFrame, contours,i, cv::Scalar(255), CV_FILLED, 8);
						cv::drawContours(m_frame_temp2, contours,i, cv::Scalar(255), CV_FILLED, 8);
					}
				}
			}
		}
	}
	cv::imshow("lane",m_currFrame);
	cv::imshow("midstep", m_frame_temp1);
	cv::imshow("laneBlobs", m_frame_temp2);
}





