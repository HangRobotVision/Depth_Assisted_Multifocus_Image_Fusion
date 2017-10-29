#ifndef DEPTH_MAP_PREPROCESS_H_
#define DEPTH_MAP_PREPROCESS_H_

#include "opencv2/core/core.hpp"

/* ************************************************************************* */
/**
* @brief:                       align depth map with color image 
* @param  src_depth:            original depth map
* @param  aligned_depth:        aligned depth map 
* @return:                      0 success; 1 failure
*/
int AlignDepthWithColor(const cv::Mat& src_depth, cv::Mat& aligned_depth);


/* ************************************************************************* */
/**
* @brief:                       fill depth holes based on anisotropic diffusion method  
* @param  src_depth:            original depth map
* @param  filled_depth:         filled depth map 
* @return:                      0 success; 1 failure
*/
int FillDepthHoles(const cv::Mat& src_depth, cv::Mat& filled_depth);


#endif