#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>

cv::Mat frame_to_mat(rs2::video_frame frame) {
	return cv::Mat(cv::Size(frame.get_width(), frame.get_height()), CV_8UC3, (void*)frame.get_data(), cv::Mat::AUTO_STEP);
}

void create_mask_from_depth(cv::Mat& depth, int thresh, cv::ThresholdTypes type) {
	auto gen_element = [](int erosion_size){
		return getStructuringElement(cv::MORPH_RECT,
				cv::Size(erosion_size + 1, erosion_size + 1),
				cv::Point(erosion_size, erosion_size));
	};

	const int erosion_size = 3;
	auto erode_less = gen_element(erosion_size);
	auto erode_more = gen_element(erosion_size * 2);

	threshold(depth, depth, thresh, 255, type);
	dilate(depth, depth, erode_less);
	erode(depth, depth, erode_more);
}

cv::Mat grab_cuts(rs2::frame bw_depth, cv::Mat color_mat) {
	auto near = frame_to_mat(bw_depth);
	cvtColor(near, near, cv::COLOR_BGR2GRAY);
	create_mask_from_depth(near, 180, cv::THRESH_BINARY);

	auto far = frame_to_mat(bw_depth);
	cvtColor(far, far, cv::COLOR_BGR2GRAY);
	far.setTo(255, far == 0); 
	create_mask_from_depth(far, 100, cv::THRESH_BINARY_INV);

	cv::Mat mask;
	mask.create(near.size(), CV_8UC1); 
	mask.setTo(cv::Scalar::all(cv::GC_BGD)); 
	mask.setTo(cv::GC_PR_BGD, far == 0);
	mask.setTo(cv::GC_FGD, near == 255); 

	cv::Mat bgModel, fgModel; 
	grabCut(color_mat, mask, cv::Rect(), bgModel, fgModel, 1, cv::GC_INIT_WITH_MASK);

	cv::Mat3b foreground = cv::Mat3b::zeros(color_mat.rows, color_mat.cols);
	color_mat.copyTo(foreground, (mask == cv::GC_FGD) | (mask == cv::GC_PR_FGD));

	return foreground;
}
