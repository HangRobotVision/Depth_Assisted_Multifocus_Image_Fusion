#ifndef CONSTRUCT_ALL_IN_FOCUS_H_
#define CONSTRUCT_ALL_IN_FOCUS_H_

#include <vector>
#include "opencv2/core/core.hpp"
/*
    dir2/foo2.h.
    C system files.
    C++ system files.
    Other libraries' .h files.
    Your project's .h files.
*/

// calibration


// 
/* ************************************************************************* */
/**
* @brief:                       construct an all-in-focus image based on segmented regions 
* @param  segmented_regions:    segmented regions 
* @param  video_file_name:		name of multi-focus video
* @param  all_in_focus_img:		constructed all in focus image
* @return:                      0, success; -1 failure
*/
int ConstructAllInFocusImage(const std::vector<cv::Mat>& segmented_regions,  
                             const std::string video_file_name, 
                             cv::Mat& all_in_focus_img);


/* ************************************************************************* */
/**
* @brief:                       calculate normalized variance value of a segmented region 
* @param  region_img:           image 
* @return:                      normalized variance value
*/
float CalculateNormalizedVariance(const cv::Mat& region_img);

#endif