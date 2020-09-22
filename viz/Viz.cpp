#include <librealsense2/rs.hpp>
#include <librealsense2/hpp/rs_options.hpp>
#include <librealsense2/rs_advanced_mode.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/viz.hpp>
#include <pcl-1.8/pcl/filters/passthrough.h>
#include <pcl-1.8/pcl/visualization/cloud_viewer.h>

const int WIDTH = 640;
const int HEIGHT = 480;

class Viz {
	private :
		cv::Mat depth;
		cv::Mat color;
		rs2::pipeline pipe;
		rs2::frameset frames;
		rs2::colorizer color_map;
		rs2::config cfg;
		rs2::pipeline_profile selection;
		rs2::device dev;

		void get_color_frame(){
			rs2::video_frame color_frame = frames.get_color_frame();
			color = cv::Mat(cv::Size(color_frame.get_width(), color_frame.get_height()), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
		}
	public : 
		Viz() {
			cfg.enable_stream(RS2_STREAM_COLOR, WIDTH, HEIGHT, RS2_FORMAT_BGR8, 30);
			cfg.enable_stream(RS2_STREAM_DEPTH, WIDTH, HEIGHT, RS2_FORMAT_Z16, 30);

			selection = pipe.start(cfg);
			dev = selection.get_device();
		}

		void run(){
			float fx, fy, cx, cy;

			auto profile = pipe.get_active_profile().get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics();
			fx = profile.fx;
			fy = profile.fy;
			cx = profile.ppx;
			cy = profile.ppy;

			pcl::visualization::CloudViewer viewer("Realsense Vewer");
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
			cloud->width = WIDTH;
			cloud->height = HEIGHT;
			cloud->is_dense = false;
			cloud->points.resize (WIDTH * HEIGHT);

			while (cv::waitKey(1)!='q' || !viewer.wasStopped()){
				frames = pipe.wait_for_frames();
				//get_depth_frame();
				get_color_frame();

				for (int i = 0; i < HEIGHT; i++){
					for (int j = 0; j < WIDTH; j++){
						float z = frames.get_depth_frame().get_distance(j, i);
						float x = (j - cx) * z / fx;
						float y = (i - cy) * z / fy;

						if(z < 0.5f && z > 0.f) {
							cloud->points[i * WIDTH + j].x = x;
							cloud->points[i * WIDTH + j].y = y;
							cloud->points[i * WIDTH + j].z = z;

							cloud->points[i * WIDTH + j].b = color.at<cv::Vec3b>(i, j)[0];
							cloud->points[i * WIDTH + j].g = color.at<cv::Vec3b>(i, j)[1];
							cloud->points[i * WIDTH + j].r = color.at<cv::Vec3b>(i, j)[2];
						}
					}
				}
				viewer.showCloud(cloud);
				//cv::imshow("Depth", depth);
				//cv::imshow("Color", color);
			}

			return;
		}
};

int main(){
	Viz viz;
	viz.run();

	return 0;
}

