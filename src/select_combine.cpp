#include "select_combine.h"

#include <iostream>

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui.hpp"

int ConstructAllInFocusImage(const std::vector<cv::Mat>& segmented_regions,  
                             const std::string video_file_name, 
                             cv::Mat& all_in_focus_img)
{
    const int region_size = segmented_regions.size();

    std::cout << "region_size: " << region_size << std::endl;

    cv::VideoCapture multi_focus_video(video_file_name);
    if(!multi_focus_video.isOpened())
    {
        std::cout << "Can not open " << video_file_name << std::endl;
    }

    cv::Mat multi_focus_img, multi_focus_gray_img;
    cv::Mat img_for_calc_nv;
    std::vector<float> max_nv_vector(region_size, 0.0f); 
    std::vector<cv::Mat> clearest_img_vector(region_size);
    
    for (;;) 
	{
		multi_focus_video >> multi_focus_img;
		if (multi_focus_img.empty()) 
            break;

		cv::cvtColor(multi_focus_img, multi_focus_gray_img, cv::COLOR_BGR2GRAY);

		for (int i = 0; i < region_size; ++i)
		{
			multi_focus_gray_img.copyTo(img_for_calc_nv, segmented_regions[i]);

			float cur_normalized_variance = CalculateNormalizedVariance(img_for_calc_nv);

			if (cur_normalized_variance > max_nv_vector[i])
			{
				max_nv_vector[i] = cur_normalized_variance;
				//multi_focus_img.copyTo(all_in_focus_img, segmented_regions[i]);
                multi_focus_img.copyTo(clearest_img_vector[i], segmented_regions[i]);
            }

            img_for_calc_nv.setTo(0);

		}
	}

    for(int i = 0; i < region_size; ++i)
    {
        clearest_img_vector[i].copyTo(all_in_focus_img, segmented_regions[i]);
		cv::imshow("all_in_focus", all_in_focus_img);
		cv::waitKey(0);
    }

    return 0;
}


float CalculateNormalizedVariance(const cv::Mat& region_img)
{
	// calculate total non-zero pixels and mean intensity in region_img
	int total_pixels = 0;
	int total_val = 0;
	for (int i = 0; i < region_img.rows; ++i)
	{
		const uchar* ptr_region_img = region_img.ptr<uchar>(i);
		for (int j = 0; j < region_img.cols; ++j)
		{
			uchar cur_location_val = ptr_region_img[j];
			if (0 != cur_location_val)
			{
				++total_pixels;
				total_val += cur_location_val;
			}
		}
	}

	float mean_intensity = static_cast<float>(total_val) / static_cast<float>(total_pixels);

	// calculate normalized variance
	float tmp_normalized_variance = 0.0f;
	for (int i = 0; i < region_img.rows; ++i)
	{
		const uchar* ptr_region_img = region_img.ptr<uchar>(i);
		for (int j = 0; j < region_img.cols; ++j)
		{
			uchar cur_location_val = ptr_region_img[j];
			if (0 != cur_location_val)
			{
				float tmp_val = static_cast<float>(ptr_region_img[j]) - mean_intensity;
				tmp_val *= tmp_val;
				tmp_normalized_variance += tmp_val;
			}
		}
	}

	return tmp_normalized_variance / (total_pixels * mean_intensity);
}
