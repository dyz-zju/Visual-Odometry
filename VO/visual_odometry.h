#ifndef VISUAL_ODOMETRY_H_
#define VISUAL_ODOMETRY_H_


#include <vector>
#include <opencv2\opencv.hpp>

#include "camera.h"


class VisualOdometry
{

public:

	enum FrameStage
	{
		STAGE_FIRST_FRAME, // First frame
		STAGE_SECOND_FRAME, //Sencond frame
		STAGE_DEFAULT_FRAME //默认帧
	};


	VisualOdometry(Camera* cam);

	virtual ~VisualOdometry();

	void addImage(const cv::Mat& img, int frame_id);

	cv::Mat getCurrentR() { return cur_R_; }

	cv::Mat getCurrentT() { return cur_T_; }

protected:

	virtual bool processFirstFrame();

	virtual bool processSecondFrame();

	virtual bool processFrame(int frame_id);
	// 计算绝对尺度
	double getAbsoluteScale(int frame_id);

	void featureDetection(cv::Mat image, std::vector<cv::Point2f> &px_vec);

	void featureTracking(cv::Mat image_ref, cv::Mat image_cur, 
		std::vector<cv::Point2f>& px_ref, std::vector<cv::Point2f>& px_cur, std::vector<double>& disparities);

protected:

	FrameStage frame_stage_;
	Camera* _cam;
	cv::Mat new_frame;
	cv::Mat last_frame;

	cv::Mat cur_R_;
	cv::Mat cur_T_;

	std::vector<cv::Point2f> px_ref_;
	std::vector<cv::Point2f> px_cur_;
	std::vector<double> disparities_;

	double focal_;
	cv::Point2d pp_; // camera centre point
};


#endif // VISUAL_ODOMETRY_H_