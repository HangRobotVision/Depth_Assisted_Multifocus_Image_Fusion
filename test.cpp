#include "global.h"

// System
#include <iostream>
#include <fstream>

// OpenCV
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "align_fill.h"
#include "segment.h"
#include "select_combine.h"

//#define RUN_MY_MODIFIED_PROGRAM 1

//usage: ./segment depth_data.xml multi_focus.avi

int main(int argc, char* argv[])
{
// check input parameters
	if(argc != 3)
	{
		std::cout << "Invalid parameters" << std::endl;
		return -1;
	}

// initialize the look up table for visualizing depth map 
    InitializeDepthColorTable();

// load depth map data from *.xml
	cv::Mat depth;
	cv::FileStorage depth_data(argv[1], cv::FileStorage::READ);
	depth_data["data"] >> depth;
	//cv::flip(depth, depth, 1);
	CV_Assert( depth.type() == CV_64FC1 );
	depth.convertTo(depth, CV_16UC1);

// align depth map with color image
	cv::Mat aligned_depth;
	AlignDepthWithColor(depth, aligned_depth);

// create the depth map segmentation class
	const double coc_diameter = 0.019; // diameter of the circle of confusion
	const double aperture_value = 4.0;
	const double focal_length = 24;	
	GraphBasedImageSeg* ptr_graph_based_seger = new GraphBasedImageSeg(coc_diameter, aperture_value, focal_length);
	
	// segment
	aligned_depth.convertTo(aligned_depth, CV_64F);
	cv::Mat dst_color;
	int small_thresh = 10; // small components removing
	std::vector<cv::Mat> segmented_regions;
	int regions = ptr_graph_based_seger->GraphSegment(aligned_depth, small_thresh, segmented_regions, dst_color);
	printf("Segmented regions: %d\n", regions);
	cv::imwrite("segmentation_result.jpg", dst_color);

// construct all_in_focus image
	cv::Mat all_in_focus_img;
	int ret = ConstructAllInFocusImage(segmented_regions, argv[2], all_in_focus_img);
	if(-1 == ret)
	{
		std::cout << "ConstructAllInFocusImage error" << std::endl;
		goto CLEAN_UP;
	}
	else
	{
		cv::imwrite("all_in_focus.jpg", all_in_focus_img);
	}

CLEAN_UP:
	delete ptr_graph_based_seger;

	return 0;
}