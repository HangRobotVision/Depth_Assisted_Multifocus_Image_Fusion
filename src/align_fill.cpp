#include "align_fill.h"
 
#include <climits>
#include <iostream>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp" 

 extern unsigned char depth_color_table[USHRT_MAX + 1];

int AlignDepthWithColor(const cv::Mat& src_depth, cv::Mat& aligned_depth)
{
    // initialize the intrinsic and extrinsic parameters of the depth sensor 
    // of Kinect and Pentax color camera
	// Pentax
    static double pentax_fx = 810.1948;	static double pentax_fy = 814.2451;
	static double pentax_u0 = 329.0910;	static double pentax_v0 = 244.0002;
	// Kinect
	static double kinect_fx = 583.9147;	static double kinect_fy = 586.7294;
	static double kinect_u0 = 328.9980;	static double kinect_v0 = 251.6837;
	// extrinsic parameters 
	static cv::Mat rotation_matrix = (cv::Mat_<double>(3, 3) << 0.9997,		0.0200,		0.0158,
												        -0.0200,	0.9998,		0.0035,
														-0.0157,	-0.0038,	0.9999);
	static cv::Mat translation_matrix = (cv::Mat_<double>(3, 1) << 10, 53.8973, 40.4947); // 1.6944, 53.8973, 40.4947 
                                                                                             // 10, 53.8973, 40.4947
	int depth_map_height = src_depth.rows;
	int depth_map_width = src_depth.cols;

	cv::Mat tmp_depth_for_color = cv::Mat::zeros(depth_map_height, depth_map_width, CV_16UC1);

	for (int row_idx = 0; row_idx < depth_map_height; ++row_idx)
	{
		const ushort* ptr_raw_depth_img = src_depth.ptr<ushort>(row_idx);

		for (int col_idx = 0; col_idx < depth_map_width; ++col_idx)
		{
			double raw_depth_value = static_cast<double>(ptr_raw_depth_img[col_idx]);
			const double z_kinect = raw_depth_value;
			const double x_kinect = (col_idx - kinect_u0) * z_kinect / kinect_fx;
			const double y_kinect = (row_idx - kinect_v0) * z_kinect / kinect_fy;

			cv::Mat xyz_kinect = (cv::Mat_<double>(3, 1) << x_kinect, y_kinect, z_kinect);

			cv::Mat xyz_pentax = rotation_matrix * xyz_kinect + translation_matrix;

			const double z_pentax = xyz_pentax.at<double>(2, 0);
			const double x_pentax = xyz_pentax.at<double>(0, 0);
			const double y_pentax = xyz_pentax.at<double>(1, 0);

			double tmp_pentax_col_idx = (x_pentax / z_pentax) * pentax_fx + pentax_u0;
			double tmp_pentax_row_idx = (y_pentax / z_pentax) * pentax_fy + pentax_v0;

			int pentax_col_idx = static_cast<int>(tmp_pentax_col_idx + 0.5);
			int pentax_row_idx = static_cast<int>(tmp_pentax_row_idx + 0.5);
			if (pentax_col_idx < 0)	pentax_col_idx = 0;
			if (pentax_col_idx >= depth_map_width) pentax_col_idx = (depth_map_width - 1);

			if (pentax_row_idx < 0)	pentax_row_idx = 0;
			if (pentax_row_idx >= depth_map_height) pentax_row_idx = (depth_map_height - 1);

			// obtain depth value for color
			ushort* ptr_tmp_depth_for_color = tmp_depth_for_color.ptr<ushort>(pentax_row_idx);
			ptr_tmp_depth_for_color[pentax_col_idx] = z_pentax;
		}
	}

	cv::Mat tmp_dilated_depth;
	cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
	cv::dilate(tmp_depth_for_color, tmp_dilated_depth, element);
	
	FillDepthHoles(tmp_dilated_depth, aligned_depth);

	// Get the depth map to be shown
	cv::Mat depth_for_color_show = cv::Mat::zeros(depth_map_height, depth_map_width, CV_8UC1);
	cv::Mat filled_depth_show = cv::Mat::zeros(depth_map_height, depth_map_width, CV_8UC1);

	for (int row_idx = 0; row_idx < depth_map_height; ++row_idx)
	{
		uchar* ptr_depth_for_color_show = depth_for_color_show.ptr<uchar>(row_idx);
		ushort* ptr_depth_for_color = tmp_depth_for_color.ptr<ushort>(row_idx);

		uchar* ptr_filled_depth_show = filled_depth_show.ptr<uchar>(row_idx);
		ushort* ptr_filled_depth = aligned_depth.ptr<ushort>(row_idx);

		for (int col_idx = 0; col_idx < depth_map_width; ++col_idx)
		{
			ushort depth_for_color_val = ptr_depth_for_color[col_idx];
			ptr_depth_for_color_show[col_idx] = depth_color_table[depth_for_color_val] & 0x000000ff;

			ushort filled_depth_val = ptr_filled_depth[col_idx];
			ptr_filled_depth_show[col_idx] = depth_color_table[filled_depth_val] & 0x000000ff;
		}
	}

	cv::imwrite("mapped.jpg", depth_for_color_show);
	cv::imwrite("filled.jpg", filled_depth_show);

	return 0;
}


int FillDepthHoles(const cv::Mat& src_depth, cv::Mat& filled_depth)
{
	src_depth.copyTo(filled_depth);
	double lambda = 0.25;
	double k = 25.0;

	int col_offset = 1;
	int row_offset = 3;

	for (int col = col_offset; col < src_depth.cols; ++col) 
	{
		for (int row = row_offset; row < src_depth.rows; ++row) {
			// centered around the up up pixel		
			ushort* ptr_filled_depth = filled_depth.ptr<ushort>(row);
			ushort* ptr_temp_up_three = filled_depth.ptr<ushort>(row - 3);
			ushort* ptr_temp_up_two = filled_depth.ptr<ushort>(row - 2);
			ushort* ptr_temp_up_one = filled_depth.ptr<ushort>(row - 1);

			if (ptr_filled_depth[col] < 10) {
				double ni = static_cast<double>(ptr_temp_up_three[col] - ptr_temp_up_two[col]);
				double si = static_cast<double>(ptr_temp_up_one[col] - ptr_temp_up_two[col]);
				double wi = static_cast<double>(ptr_temp_up_two[col - 1] - ptr_temp_up_two[col]);
				double ei = static_cast<double>(ptr_temp_up_two[col + 1] - ptr_temp_up_two[col]);

				double cn = exp(-(ni * ni) / (k * k));
				double cs = exp(-(si * si) / (k * k));
				double ce = exp(-(ei * ei) / (k * k));
				double cw = exp(-(wi * wi) / (k * k));

				ptr_filled_depth[col] = static_cast<ushort>( ptr_temp_up_two[col] + lambda * (cn * ni + cs * si + ce * ei + cw * wi) + 0.5 );
			}
		}
	}

	return 0;
}