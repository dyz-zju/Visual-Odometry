#include "visual_odometry.h"

#include <string>
#include <fstream>

const int kMinNumFeature = 2000;

VisualOdometry::VisualOdometry(Camera* cam) : _cam(cam)
{
	focal_ = _cam->fx();
	pp_ = cv::Point2d(_cam->cx(), _cam->cy());
	frame_stage_ = STAGE_FIRST_FRAME;

}

VisualOdometry::~VisualOdometry() {}

void VisualOdometry::addImage(const cv::Mat& img, int frame_id){

	cv::Mat img_gray;

	if (img.empty() || img.cols != _cam->width() || img.rows != _cam->height())
		throw std::runtime_error("Frame: provided image has not the same size as the camera model .");
	
	if (img.type() == CV_8UC4){
		cv::cvtColor(img, img_gray, CV_BGRA2GRAY);

	}

	if (img.type() == CV_8UC3){
		cv::cvtColor(img, img_gray, CV_BGR2GRAY);
	}

	if (img.type() == CV_8UC1){
		img_gray = img.clone();
	}

	new_frame = img_gray;
	bool res = true; //���״̬

	if (frame_stage_ == STAGE_DEFAULT_FRAME)
		res = processFrame(frame_id);

	else if (frame_stage_ == STAGE_SECOND_FRAME)
		res = processSecondFrame();
	
	else if (frame_stage_ == STAGE_FIRST_FRAME)
		res = processFirstFrame();

	last_frame = new_frame;
}

bool VisualOdometry::processFirstFrame(){

	// �Ե�ǰ֡�����������
	featureDetection(new_frame, px_ref_);

	frame_stage_ = STAGE_SECOND_FRAME;
	
	return true;
}

bool VisualOdometry::processSecondFrame(){

	// ͨ����������ȷ���ڶ�֡�е��������
	featureTracking(last_frame, new_frame, px_ref_, px_cur_, disparities_);

	// �����ʼλ��
	cv::Mat Essential, R, t, mask;

	Essential = cv::findEssentialMat(px_cur_, px_ref_, focal_, pp_, cv::RANSAC, 0.999, 1.0, mask);
	
	cv::recoverPose(Essential, px_cur_, px_ref_, R, t, focal_, pp_, mask);

	cur_R_ = R.clone();
	cur_T_ = t.clone();

	frame_stage_ = STAGE_DEFAULT_FRAME;

	px_ref_ = px_cur_;

	return true;

}

bool VisualOdometry::processFrame(int frame_id){

	// ��ʼ�߶�
	double scale = 1.00;
	cv::Mat Essential, R, t, mask;

	featureTracking(last_frame, new_frame, px_ref_, px_cur_, disparities_);

	Essential = cv::findEssentialMat(px_cur_, px_ref_, focal_, pp_, cv::RANSAC, 0.999, 1.0, mask);

	cv::recoverPose(Essential, px_cur_, px_ref_, R, t, focal_, pp_, mask);
	// �õ���ǰ֡��ʵ�ʳ߶�
	//scale = getAbsoluteScale(frame_id); 

	if (scale > 0.1){

		cur_T_ = cur_T_ + scale*(cur_R_ * t);
		cur_R_ = R*cur_R_;
	}
	// ���������������С�ڸ�����ֵ�����������������
	if (px_ref_.size() < kMinNumFeature){

		featureDetection(new_frame, px_ref_);
		featureTracking(last_frame, new_frame, px_ref_, px_cur_, disparities_);
	}

	px_ref_ = px_cur_;

	return true;
}

double VisualOdometry::getAbsoluteScale(int frame_id){

	std::string line;
	int i = 0;
	
	std::ifstream ground_truth("E:/SLAMDataset/kitti-dataset/data_odometry_poses/dataset/poses/00.txt");

	double x = 0, y = 0, z = 0;
	double x_pre, y_pre, z_pre;

	// ��ȡ��ǰ֡��ʵλ����ǰһ֡����ʵλ�õľ�����Ϊ�߶�ֵ
	if (ground_truth.is_open()){

		while ((std::getline(ground_truth, line)) && (i <= frame_id)){

			z_pre = z;
			x_pre = x;
			y_pre = y;
			std::istringstream in(line);
			for (int j = 0; j < 12; ++j){
				in >> z;
				if (j == 7) y = z;
				if (j == 3) x = z;
			}
			++i;
		}
		ground_truth.close();
	}

	else{
		std::cerr << "Unable to Open file";
		return 0;
	}

	return sqrt((x - x_pre)*(x - x_pre) + (y - y_pre)*(y - y_pre) + (z - z_pre)*(z - z_pre));
}

void VisualOdometry::featureDetection(cv::Mat image, std::vector<cv::Point2f>& px_vec){

	std::vector<cv::KeyPoint> keypoints;

	int fast_threshold = 20;
	bool non_max_suppression = true;

	cv::FAST(image, keypoints, fast_threshold, non_max_suppression);
	cv::KeyPoint::convert(keypoints, px_vec);

}

void VisualOdometry::featureTracking(cv::Mat image_ref, cv::Mat image_cur,
	std::vector<cv::Point2f>& px_ref, std::vector<cv::Point2f>& px_cur, std::vector<double>& disparities)
{
	const double klt_win_size = 21.0;
	const int    klt_max_iter = 30;
	const double klt_eps = 0.001;

	std::vector<uchar> status;
	std::vector<float> error;
	std::vector<float> min_eig_vec;

	cv::TermCriteria termcrit(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, klt_max_iter, klt_eps);

	cv::calcOpticalFlowPyrLK(image_ref, image_cur, px_ref, px_cur, status, error,
		cv::Size2i(klt_win_size, klt_win_size), 4, termcrit, 0);

	std::vector<cv::Point2f>::iterator px_ref_it = px_ref.begin();
	std::vector<cv::Point2f>::iterator px_cur_it = px_cur.begin();
	disparities.clear(); 
	disparities.reserve(px_cur.size());

	for (size_t i = 0; px_ref_it != px_ref.end(); ++i){

		if (!status[i]){

			px_ref_it = px_ref.erase(px_ref_it);
			px_cur_it = px_cur.erase(px_cur_it);
			continue;
		}
		disparities.push_back(norm(cv::Point2d(px_ref_it->x - px_cur_it->x, px_ref_it->y - px_cur_it->y)));
		++px_ref_it;
		++px_cur_it;
	}
}