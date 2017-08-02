#include <iostream>  
#include "ipm.h"

IPM::IPM()
{
	m_init_ok = false;
}

IPM::~IPM()
{
}

void IPM::Init(const CameraPara& camera_para, const IPMPara& ipm_para )
{
	m_camera_para = camera_para;
	m_ipm_para = ipm_para;
	
	float fu = camera_para.fu;
    float fv =  camera_para.fv; // focal length in y
    float cu =  camera_para.cu; // optical center coordinates in image frame (origin is (0,0) at top left)
    float cv =  camera_para.cv;
	float image_width = camera_para.image_width; // width of images
    float image_height = camera_para.image_height; // width of images
    
    float roll = camera_para.roll;   // roll angle in radians (+ve clockwise)
    float pitch = camera_para.pitch;  // pitch angle in radians (+ve upwards)
    float yaw = camera_para.yaw;    // yaw angle in radians (+ve clockwise)
    float height = camera_para.height; // height of camera above ground (mm)
	
	float x_min = ipm_para.x_limits[0];
	float x_max = ipm_para.x_limits[1];
	float y_min = ipm_para.y_limits[0];
	float y_max = ipm_para.y_limits[1];
	
	Eigen::Matrix3d intrinsic_para;
	intrinsic_para <<  fu,  0,  cu,
						0,  fv, cv,
						0,   0,  1;
	Eigen::Matrix3d R_roll;
	R_roll << 1,        0,        0,
			  0,   cos(roll),  sin(roll),
			  0,   -sin(roll),  cos(roll);
			  
	Eigen::Matrix3d R_pitch;
	R_pitch << cos(pitch),  0,  -sin(pitch),
                      0,        1,     0,
                  sin(pitch),  0,  cos(pitch);
				  
	Eigen::Matrix3d R_yaw;
	R_yaw << cos(yaw),   sin(yaw),   0,
			-sin(yaw),   cos(yaw),   0,
				0,           0,      1;
	
	Eigen::Matrix3d R_att;
	R_att = R_roll*R_pitch*R_yaw;
					
	// 相机->图像坐标系  				
	Eigen::Matrix3d R_camera2iamge;
	R_camera2iamge << 0,  1,  0,
					  0,  0,  1,
					  1,  0,  0;
					  
	m_R_w2i = intrinsic_para*R_camera2iamge*R_att;
	m_R_i2w = m_R_w2i.inverse();
	m_init_ok = true;
}

// in_points： 单位m
bool IPM::TransformGround2Image(const cv::Mat& in_points, cv::Mat* out_points)
{
	if(!m_init_ok)
		return false;
	
    *out_points = cv::Mat::zeros(2, in_points.cols, CV_32FC1);
    for (int col = 0; col < in_points.cols; col++) {
		float x = in_points.at<float>(0, col);
		float y = in_points.at<float>(1, col);
		float h = m_camera_para.height;
        Eigen::Vector3d p_xyz;
		p_xyz << x, y, h;
		
		Eigen::Vector3d uv_t = m_R_w2i*p_xyz;
		Eigen::Vector3d uv_new = uv_t/uv_t(2);
		
		out_points->at<float>(0, col) = (float)uv_new(0);
		out_points->at<float>(1, col) = (float)uv_new(1);
    }
    return true;
}


int IPM::CalGround2ImageTransMatrix(const IPMPara& ipm_para, cv::Mat& H_trans)
{
	// calc uv limits (x, y)
	// 0----1
	// |    |
	// 3----2
    cv::Mat xy_limits_pts = cv::Mat::zeros(2, 4, CV_32FC1);
    xy_limits_pts.at<float>(0, 0) = ipm_para.x_limits[1];
    xy_limits_pts.at<float>(0, 1) = ipm_para.x_limits[1];
    xy_limits_pts.at<float>(0, 2) = ipm_para.x_limits[0];
    xy_limits_pts.at<float>(0, 3) = ipm_para.x_limits[0];

    xy_limits_pts.at<float>(1, 0) = ipm_para.y_limits[0];
    xy_limits_pts.at<float>(1, 1) = ipm_para.y_limits[1];
    xy_limits_pts.at<float>(1, 2) = ipm_para.y_limits[1];
    xy_limits_pts.at<float>(1, 3) = ipm_para.y_limits[0];

    cv::Mat uv_limits = cv::Mat::zeros(2, 4, CV_32FC1);
    if(!TransformGround2Image(xy_limits_pts, &uv_limits))
		return -1;
	
	std::vector<cv::Point2f> p_trans_src;
    for (int col = 0; col < uv_limits.cols; col++) {
		int uu = (int)(uv_limits.at<float>(0, col));
        int vv = (int)(uv_limits.at<float>(1, col));
        p_trans_src.push_back(cv::Point2f(uu, vv));
    }
    
    std::vector<cv::Point2f> p_trans_dst;
    p_trans_dst.push_back(cv::Point2f(0, 0));
    p_trans_dst.push_back(cv::Point2f(ipm_para.width - 1, 0));
    p_trans_dst.push_back(cv::Point2f(ipm_para.width - 1, ipm_para.height - 1));
    p_trans_dst.push_back(cv::Point2f(0, ipm_para.height - 1));
	
    H_trans = cv::getPerspectiveTransform(p_trans_src, p_trans_dst);
	return 1;
}
