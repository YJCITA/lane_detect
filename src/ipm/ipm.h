/// @brief  mapping between bird-view image and perspective image
///        coordinate system : north(x) -> east(y) -> ground (z)

#ifndef  IPM_H
#define  IPM_H

#include "opencv2/opencv.hpp"
#include <eigen3/Eigen/Dense>

// @brief parameters for camera
struct CameraPara {
    float fu; // focal length in x
    float fv; // focal length in y
    float cu; // optical center coordinates in image frame (origin is (0,0) at top left)
    float cv;

    float height; // height of camera above ground (mm)
    float pitch;  // pitch angle in radians (+ve upwards)
    float yaw;    // yaw angle in radians (+ve clockwise)
    float roll;   // roll angle in radians (+ve clockwise)

    float image_width; // width of images
    float image_height; // height of images
    const CameraPara& operator = (const CameraPara& lhs)
    {
        fu = lhs.fu;
        fv = lhs.fv;
        cu = lhs.cu;
        cv = lhs.cv;
        height = lhs.height;
        pitch = lhs.pitch;
        yaw = lhs.yaw;
        roll = lhs.roll;
        image_width = lhs.image_width;
        image_height = lhs.image_height;
        return *this;
    }
};

struct IPMPara
{
    // min and max x-value on ground in world coordinates (north direction)
    double x_limits[2];
    // min and max y-value on ground in world coordinates (east direction)
    double y_limits[2];

    // conversion between mm in world coordinate on the ground
    // in x-direction and pixel in image (north res)
    float x_scale;

    // conversion between mm in world coordinate on the ground
    // in y-direction and pixel in image (east res)
    float y_scale;

    // width in pixel
    int width;

    // height in pixel
    int height;

    // portion of image height to add to y-coordinate of vanishing point
    float vp_portion;

    // min and max u-value on ground in image plane (width direction)
    double u_limits[2];

    // min and max v-value on ground in image plane (height direction)
    double v_limits[2];

    // u/v-value with size(2, width*height) in image plane
    cv::Mat uv_grid;
};

class IPM
{
public:
    IPM();
	
    ~IPM();
	
    void Init(const CameraPara& camera_para, const IPMPara& ipm_para);
	
	int CalGround2ImageTransMatrix(const IPMPara& ipm_para, cv::Mat& H_trans);
	
	bool TransformGround2Image(const cv::Mat& in_points, cv::Mat* out_points);
private:

private:
	bool m_init_ok;
	
	// rotation from world to image
	Eigen::Matrix3d m_R_w2i;
	Eigen::Matrix3d m_R_i2w;
	
    CameraPara m_camera_para;
	IPMPara m_ipm_para;
	
    cv::Mat m_persp_matrix;
    cv::Mat m_ground2image_matrix; // 3 X 3
    cv::Mat m_image2ground_matrix; // 3 X 3
    cv::Mat m_vp; // vanishing point
};

#endif  // IPM_H
