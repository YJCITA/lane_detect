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
#include "../common/base/log_level.h"
#include "./ipm/ipm.h"

DEFINE_string(image_name, "./1.jpg", "image_name");
DEFINE_double(fu, 1482.0, "fu");
DEFINE_double(fv, 1475.874, "fv");
DEFINE_double(cu, 685.044, "cu");
DEFINE_double(cv, 360.02380, "cv");
DEFINE_double(camera_height, 1.25, "camera height mm");    // ??? mm
DEFINE_double(pitch, 1.5, "pitch angle (degree)"); // -1.8
DEFINE_double(yaw, 0.5, "yaw angle (degree)");
DEFINE_int32(image_width, 1280, "image width");
DEFINE_int32(image_height, 720, "image height");
DEFINE_double(x_start_offset, 5.0, "x start offset");
DEFINE_double(x_end_offset, 50.0, "x start offset");
DEFINE_double(y_start_offset, -5.0, "y start offset");
DEFINE_double(y_end_offset, 5.0, "y end offset");
DEFINE_double(ipm_width, 400, "ipm iamge width");
DEFINE_double(ipm_height, 600, "ipm iamge height");

DEFINE_int32(frame_start_index, 0, "frame_start_index");

// 初始化ipm参数
void InitIPM(CameraPara& camera_para, IPMPara& ipm_para)
{
    camera_para.fu = FLAGS_fu;
    camera_para.fv = FLAGS_fv;
    camera_para.cu = FLAGS_cu;
    camera_para.cv = FLAGS_cv;
    camera_para.height = FLAGS_camera_height; // m
    camera_para.pitch = FLAGS_pitch * CV_PI / 180;
    camera_para.yaw = FLAGS_yaw * CV_PI / 180;
    camera_para.image_width = FLAGS_image_width;
    camera_para.image_height = FLAGS_image_height;

    ipm_para.x_limits[0] = FLAGS_x_start_offset;
    ipm_para.x_limits[1] = FLAGS_x_end_offset;
    ipm_para.y_limits[0] = FLAGS_y_start_offset;
    ipm_para.y_limits[1] = FLAGS_y_end_offset;
	ipm_para.width = FLAGS_ipm_width;
	ipm_para.height = FLAGS_ipm_height;
	
	
	// 图像能看到最靠近车道的路面是有安装高度，外参，镜头视场角决定(目前结果有点问题)
// 	float camera_vertial_FOV = atan((camera_para.image_width/2)/camera_para.fv)*2;
// 	float x_limits_min = FLAGS_camera_height/(camera_vertial_FOV/2 - camera_para.pitch);
// 	ipm_para.x_limits[0] = x_limits_min;
	
}

void Run(std::string path)
{
    // read video init
    cv::VideoCapture cap(path); // open the video file for reading
    if( !cap.isOpened() ){  // if not success, exit program
        std::cout << "Cannot open the video file" << std::endl;
		return;
	}
    //cap.set(CV_CAP_PROP_POS_MSEC, 300); //start the video at 300ms
//     double fps = cap.get(CV_CAP_PROP_FPS); //get the frames per seconds of the video
//     std::cout << "Input video's Frame per seconds : " << fps << std::endl;
	// 第position帧 开始读取数据
	double position = FLAGS_frame_start_index; //1900 ;
	cap.set(CV_CAP_PROP_POS_FRAMES, FLAGS_frame_start_index);
	
	// init ipm
	CameraPara camera_para;
	IPMPara ipm_para;
	InitIPM( camera_para, ipm_para);
	IPM m_ipm;
	m_ipm.Init(camera_para, ipm_para);
	cv::Mat g_H_trans;
	if(!m_ipm.CalGround2ImageTransMatrix(ipm_para, g_H_trans)){
		printf("init ipm error !!!\n");
		return;
	}

	// lane detect
	cv::Mat video_frame;
    cap.read(video_frame);
	LaneDetect lane_detect;
	lane_detect.InitOnIPM(cv::Size(ipm_para.width, ipm_para.height));
	
	int frame_index = FLAGS_frame_start_index - 1;
    while(1){
         // read a new video_frame from video
        if(!cap.read(video_frame)) {
            std::cout << "Cannot read the video_frame from video file" << std::endl;
            break;
        }
		cvtColor(video_frame, video_frame, CV_BGR2GRAY);
		
		// IPM
		cv::Mat ipm_image;
		cv::warpPerspective(video_frame, ipm_image, g_H_trans, cv::Size(FLAGS_ipm_width, FLAGS_ipm_height));
// 		cv::imshow("ipm", ipm_image);
		
		// lane detect
		std::cout<<"frame_index: "<< ++frame_index <<std::endl;
        lane_detect.DetectLane(ipm_image);

		// 按键事件，空格暂停，其他跳出循环
		if (cvWaitKey(10) == 32){
			bool is_quite = false;
			while(!is_quite)
				if(cvWaitKey(10) == 32)
					is_quite = true;
		}
    }
}


int main(int argc, char *argv[])
{
	//init
	// parse gflags
    google::ParseCommandLineFlags(&argc, &argv, true);
	FLAGS_log_dir = "./build/";
	FLAGS_v = 6;
	google::SetStderrLogging(6); //设置级别高于 google::INFO 的日志同时输出到屏幕
	FLAGS_colorlogtostderr = true;    //设置输出到屏幕的日志显示相应颜色
	FLAGS_logbufsecs = 0;        //缓冲日志输出，默认为30秒，此处改为立即输出
	google::InstallFailureSignalHandler();      //捕捉 core dumped
		
    Run("/home/yj/bak/data/lane/data_develop/2/rec_20170518_055838.mp4");
    cv::destroyAllWindows();
}
