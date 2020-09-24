#include <fstream>
#include <librealsense2/rs.hpp>
#include <librealsense2/hpp/rs_options.hpp>
#include <librealsense2/rs_advanced_mode.hpp>
#include <opencv2/opencv.hpp>
#include <pcl-1.8/pcl/visualization/cloud_viewer.h>
#include <pcl-1.8/pcl/point_types.h>
#include <pcl-1.8/pcl/io/ply_io.h>
#include <pcl-1.8/pcl/io/pcd_io.h>
#include "include/opencv.hpp"

class Scanning {
	rs2::pipeline pipe;
	rs2::config cfg;
	rs2::pipeline_profile selection;
	rs2::device dev;
	rs2::frameset aligned_set;
	const int width = 640;
	const int height = 480;
	float fx, fy, cx, cy;

	public:
	Scanning() {
		cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8, 30);
		cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, 30);

		selection = pipe.start(cfg);
		dev = selection.get_device();

		auto advanced_mode_dev = dev.as<rs400::advanced_mode>();
		std::ifstream t("./HighResHighAccuracyPreset.json");
		std::string str((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());
		advanced_mode_dev.load_json(str);

		auto intrinsics = pipe.get_active_profile().get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics();
		fx = intrinsics.fx;
		fy = intrinsics.fy;
		cx = intrinsics.ppx;
		cy = intrinsics.ppy;
	}

	void run() {
		rs2::colorizer colorize;
		rs2::align align_to(RS2_STREAM_COLOR);

		pcl::visualization::CloudViewer viewer("Realsense Vewer");
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

		cloud->width = width;
		cloud->height = height;
		cloud->is_dense = false;
		cloud->points.resize (width * height);

		for (int i = 0; i < 10; i++) pipe.wait_for_frames();
		while(true) {
			rs2::frameset data = pipe.wait_for_frames();
			rs2::frameset aligned_set = align_to.process(data);

			auto color_mat = frame_to_mat(aligned_set.get_color_frame());

			rs2::frame depth = aligned_set.get_depth_frame();
			colorize.set_option(RS2_OPTION_COLOR_SCHEME, 2);
			rs2::frame bw_depth = depth.apply_filter(colorize);

			auto foreground = grab_cuts(bw_depth, color_mat);

			for(int i = 0; i < width; i++) {
				for(int j = 0;j < height; j++) {
					float z = aligned_set.get_depth_frame().get_distance(i, j);
					float x = (i - cx) * z / fx;
					float y = (j - cy) * z / fy;

					if(z > 0.f && z < 0.5f) {
						cloud->points[i * height + j].x = x;
						cloud->points[i * height + j].y = y;
						cloud->points[i * height + j].z = z;

						cloud->points[i * height + j].b = color_mat.at<cv::Vec3b>(j, i)[2];
						cloud->points[i * height + j].g = color_mat.at<cv::Vec3b>(j, i)[1];
						cloud->points[i * height + j].r = color_mat.at<cv::Vec3b>(j, i)[0];
					} else {
						cloud->points[i * height + j].x = 0;
						cloud->points[i * height + j].y = 0;
						cloud->points[i * height + j].z = 0;

						cloud->points[i * height + j].b = 0;
						cloud->points[i * height + j].g = 0;
						cloud->points[i * height + j].r = 0;
					}
				}
			}

			viewer.showCloud(cloud);

			if(cv::waitKey(1) == 'q' || viewer.wasStopped()) {
				pcl::PCDWriter pcd_writer;
				pcl::PLYWriter ply_writer;
				pcd_writer.write<pcl::PointXYZRGB>("scannedItem.pcd", *cloud, false);
				ply_writer.write<pcl::PointXYZRGB>("scannedItem.ply", *cloud, false);
				break;
			}
		}
	}
};

int main(void) {
	Scanning *s = new Scanning();
	s->run();

	return 0;
}
