#include "lane_detect.h"

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

	m_frame_midstep = cv::Mat(m_currFrame.rows, m_currFrame.cols, CV_8UC1, 0.0);//stores possible lane markings
	m_frame_laneBlobs = cv::Mat(m_currFrame.rows, m_currFrame.cols, CV_8UC1, 0.0);//stores finally selected lane marks

	m_vanishingPt = 0; //m_currFrame.rows/2 + 80;                       //for simplicity right now
	m_ROIrows = m_currFrame.rows - m_vanishingPt;                 //rows in region of interest
// 	m_minSize = 0.00015 * (m_currFrame.cols*m_currFrame.rows);    //min size of any region to be selected as lane
	m_minSize = 0.00003 * (m_currFrame.cols*m_currFrame.rows);    //min size of any region to be selected as lane
// 	m_maxLaneWidth = 0.025 * m_currFrame.cols;                     //approximate max lane width based on image size
	m_maxLaneWidth = 0.05 * m_currFrame.cols;                     //approximate max lane width based on image size
	m_smallLaneArea = 7 * m_minSize;
	m_longLane = 0.3 * m_currFrame.rows;
	m_ratio = 3;

	//these mark the possible ROI for vertical lane segments and to filter vehicle glare
	m_vertical_left  = 2/5*m_currFrame.cols;
	m_vertical_right = 3/5*m_currFrame.cols;
	m_vertical_top   = 2/3*m_currFrame.rows;
	
	cv::namedWindow("currframe", 2);
	cv::resizeWindow("currframe", 640, 480);
// 	cv::namedWindow("lane", 2);
	cv::namedWindow("midstep", 2);
	cv::resizeWindow("midstep", 640, 480);
	cv::namedWindow("laneBlobs", 2);
	cv::resizeWindow("laneBlobs", 640, 480);
}

void LaneDetect::InitOnIPM(cv::Size frame_size)
{
	m_frame_midstep = cv::Mat(frame_size.height, frame_size.width, CV_8UC1, 0.0);//stores possible lane markings
	m_frame_laneBlobs = cv::Mat(frame_size.height, frame_size.width, CV_8UC1, 0.0);//stores finally selected lane marks

	m_vanishingPt = 0; //frame_size.height/2 + 80;                       //for simplicity right now
	m_ROIrows = frame_size.height - m_vanishingPt;                 //rows in region of interest
	m_minSize = 0.00003 * (frame_size.width*frame_size.height);    //min size of any region to be selected as lane
	m_maxLaneWidth = 0.01 * frame_size.width;                     //approximate max lane width based on image size
	m_smallLaneArea = 7 * m_minSize;
	m_longLane = 0.3 * frame_size.height;
	m_ratio = 5;

	//these mark the possible ROI for vertical lane segments and to filter vehicle glare
	m_vertical_left  = 0;
	m_vertical_right = frame_size.width;
	m_vertical_top   = 0;
	
	cv::namedWindow("currframe", 2);
	cv::resizeWindow("currframe", frame_size.width, frame_size.height);
	cv::namedWindow("midstep", 2);
	cv::resizeWindow("midstep", frame_size.width, frame_size.height);
	cv::namedWindow("laneBlobs", 2);
	cv::resizeWindow("laneBlobs", frame_size.width, frame_size.height);
}


void LaneDetect::DetectLane(cv::Mat &nxt)
{
	m_currFrame = nxt;                        //if processing is to be done at original size
// 	resize(nxt ,m_currFrame, m_currFrame.size()); //resizing the input image for faster processing
	
	// UpdateSensitivity();
	// TODO: 根据外参数设定
	for(int i = m_vanishingPt; i < m_currFrame.rows; i++){
		for(int j=0; j < m_currFrame.cols; j++){
			m_frame_midstep.at<uchar>(i,j) = 0; // black
			m_frame_laneBlobs.at<uchar>(i,j) = 0;
		}
	}
	cv::imshow("currframe",m_currFrame);
	
	MarkLane();
	RemoveInvalidBlob();
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

// 检测每一行的灰度变化
void LaneDetect::MarkLane()
{
	for(int row_index = m_vanishingPt; row_index < m_currFrame.rows; row_index++){
		// IF COLOUR IMAGE IS GIVEN then additional check can be done
		// lane markings RGB values will be nearly same to each other(row_index.e without any hue)
		// min lane width is taken to be 5
// 		int laneWidth = 5 + m_maxLaneWidth*(row_index - m_vanishingPt)/m_ROIrows;
		int laneWidth = m_maxLaneWidth;
		for(int col_index = laneWidth; col_index < m_currFrame.cols - laneWidth; col_index++){
			int diffL = m_currFrame.at<uchar>(row_index, col_index) - m_currFrame.at<uchar>(row_index, col_index - laneWidth);
			int diffR = m_currFrame.at<uchar>(row_index, col_index) - m_currFrame.at<uchar>(row_index, col_index + laneWidth);
			int diff  =  diffL + diffR - abs(diffL - diffR);

			// both left and right differences can be made to contribute at least by certain threshold (which is > 0 right now)
			// total minimum Diff should be atleast more than 5 to avoid noise
			int diffThreshLow = 0.4*m_currFrame.at<uchar>(row_index, col_index);
			int diffThreshTop = 1.2*m_currFrame.at<uchar>(row_index, col_index);
			if (diffL > 0 && diffR > 0 && diff > 4)
				if(diff >= diffThreshLow /*&& diff <= diffThreshTop*/ )
					m_frame_midstep.at<uchar>(row_index, col_index) = 255;
		}
	}
}

void LaneDetect::RemoveInvalidBlob()
{
	float contour_area;
    float blob_angle_deg;
    float bounding_width;
    float bounding_length;
	cv::Size2f sz;
    std::vector< std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::RotatedRect rotated_rect;
	
	// find all contours in the binary image
	cv::Mat binary_image; //used for blob removal
	m_frame_midstep.copyTo(binary_image);
	cv::findContours(binary_image, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

	// for removing invalid blobs
	if (!contours.empty()){
		for (size_t i=0; i<contours.size(); ++i){
			//====conditions for removing contours====//
			contour_area = cv::contourArea(contours[i]) ;
			// 1.blob size should be bigger than lower threshold
			if(contour_area > m_minSize){
				rotated_rect = cv::minAreaRect(contours[i]);
				sz = rotated_rect.size;
				bounding_width = sz.width;
				bounding_length = sz.height;

				//openCV selects length and width based on their orientation
				//so angle needs to be adjusted accordingly
				blob_angle_deg = rotated_rect.angle;
				if (bounding_width < bounding_length)
					blob_angle_deg = 90 + blob_angle_deg;

				//if such big line has been detected then it has to be a (curved or a normal)lane
				bool is_near_center_bottom = (rotated_rect.center.y > m_vertical_top && rotated_rect.center.x > m_vertical_left && rotated_rect.center.x < m_vertical_right);
				if(bounding_length>m_longLane || bounding_width >m_longLane){
					cv::drawContours(m_currFrame, contours, i, cv::Scalar(255), CV_FILLED, 8);
					cv::drawContours(m_frame_laneBlobs, contours, i, cv::Scalar(255), CV_FILLED, 8);
// 				}else if ( abs(blob_angle_deg) > 10 && (abs(blob_angle_deg) < 80 || is_near_center_bottom) ){
				}else if ( abs(blob_angle_deg) > 45 ){
					//angle of orientation of blob should not be near horizontal or vertical
					//vertical blobs are allowed only near center-bottom region, where centre lane mark is present
					//length:width >= m_ratio for valid line segments
					//if area is very small then m_ratio limits are compensated
					double L_div_W = bounding_length/bounding_width;  // length/width
					double W_div_L = bounding_width/bounding_length;
					double aera_ratio = contour_area/(bounding_width*bounding_length);
					if (L_div_W >= m_ratio || W_div_L >= m_ratio
						||( contour_area < m_smallLaneArea && (aera_ratio > 0.75) &&(L_div_W >= 2 || W_div_L >= 2) )
					){
						cv::drawContours(m_currFrame, contours, i, cv::Scalar(255), CV_FILLED, 8);
						cv::drawContours(m_frame_laneBlobs, contours, i, cv::Scalar(255), CV_FILLED, 8);
					}
				}
			}
		}
	}
// 	cv::imshow("lane",m_currFrame);
	cv::imshow("midstep", m_frame_midstep);
	cv::imshow("laneBlobs", m_frame_laneBlobs);
}





