/**
* @file graph_based_segmentation.cpp
* @brief Implement graph based image segmentation for depth map, mainly based on
*        Efficient graph-based image segmentation. International Journal of Computer Vision, 59(2), 167-181.
*        Original code released by author is at http://cs.brown.edu/~pff/segment/
* @date  Nov. 6, 2016
* @author Hang Liu
*/

#include "segment.h"

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <climits>
#include <cassert>
#include <cmath>
#include <vector>
#include <fstream>
#include <iostream>

extern unsigned char depth_color_table[USHRT_MAX + 1];

#define ADD_MY_LIMIT 1

/* ************************************************************************* */
GraphBasedImageSeg::GraphBasedImageSeg(const double coc_diameter, const double aperture_value, const double focal_length)
{
	this->fai = coc_diameter;
	this->F = aperture_value;
	this->f = focal_length;
}

/* ************************************************************************* */
GraphBasedImageSeg::~GraphBasedImageSeg()
{
	
}

/* ************************************************************************* */
FrontBackDOF GraphBasedImageSeg::GetFrontBackDof(const double distance)
{
	FrontBackDOF front_back_dof;
	front_back_dof.front_dof = F * fai * pow(distance, 2) / 
							   (pow(f, 2) + F * fai * distance);
	front_back_dof.back_dof =  F * fai * pow(distance, 2) / 
							   (pow(f, 2) - F * fai * distance);
	front_back_dof.total_dof = front_back_dof.front_dof + front_back_dof.back_dof;

	return front_back_dof;
}

inline double GraphBasedImageSeg::Dissim(const cv::Mat& depth, const int x1, const int y1, 
										 const int x2, const int y2)
{
	assert(CV_64FC1 == depth.type());

	const double* ptr_depth_1 = depth.ptr<double>(y1);
	double value_1 = ptr_depth_1[x1];
	const double* ptr_depth_2 = depth.ptr<double>(y2);
	double value_2 = ptr_depth_2[x2];
	double weight = abs(value_1 - value_2);

	return weight;
}

/* ************************************************************************* */
int GraphBasedImageSeg::GraphSegment(const cv::Mat& depth_map, const int small_thresh, 
									 std::vector<cv::Mat>& regions, cv::Mat& dst)
{
	int width = depth_map.cols;
	int height = depth_map.rows;

	Edge *edges = new Edge[width * height * 4];
	int num = 0;

	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {

			//const double* ptr_depth_map = depth_map.ptr<double>(y);
			//double depth_val = ptr_depth_map[x];

			if (x < width - 1) {
				edges[num].a = y * width + x;
				edges[num].b = y * width + (x + 1);
				//edges[num].w = depth_val;
				edges[num].w = Dissim(depth_map, x, y, x + 1, y);
				num++;
			}
			if (y < height - 1) {
				edges[num].a = y * width + x;
				edges[num].b = (y + 1) * width + x;
				//edges[num].w = depth_val;
				edges[num].w = Dissim(depth_map, x, y, x, y + 1);
				num++;
			}
			if ((x < width - 1) && (y < height - 1)) {
				edges[num].a = y * width + x;
				edges[num].b = (y + 1) * width + (x + 1);
				//edges[num].w = depth_val;
				edges[num].w = Dissim(depth_map, x, y, x + 1, y + 1);
				num++;
			}
			if ((x < width - 1) && (y > 0)) {
				edges[num].a = y * width + x;
				edges[num].b = (y - 1) * width + (x + 1);
				//edges[num].w = depth_val;
				edges[num].w = Dissim(depth_map, x, y, x + 1, y - 1);
				num++;
			}
		}
	}

	// segment the graphs
	//int64 time_1 = cv::getTickCount();
	double c = 0;
	DisJoint* d = SegGraph(depth_map, width * height, num, edges);
	//int64 time_2 = cv::getTickCount();
	//std::cout << "true seg time: " << (time_2 - time_1) / cv::getTickFrequency() << std::endl;

	// small component merging
	for (int i = 0; i < num; i++) 
	{
		int a = d->find(edges[i].a);
		int b = d->find(edges[i].b);
		if ((a != b) && ((d->size(a) < small_thresh) || (d->size(b) < small_thresh))) 
		{
			d->join(a, b);
		}
	}

	delete[] edges;

	// random-color palette
	// 3-by-wh 2D array
	uchar **randClr;
	randClr = new uchar*[width * height];
	for (int i = 0; i < width * height; i++) {
		randClr[i] = new uchar[3];
		memset(randClr[i], 0, sizeof(uchar) * 3);
	}

	// palette color set
	for (int i = 0; i < width * height; i++) {
		//uchar random_color = (uchar)rand();
		for (int j = 0; j < 3; j++) {
			randClr[i][j] = (uchar)rand();
		}
	}

	// color assignment to components
	cv::Mat comp_matrix = cv::Mat::zeros(height, width, CV_32SC1);
	dst = cv::Mat::zeros(height, width, CV_8UC3);
	std::vector<int> comp_values;
	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			int comp = d->find(y * width + x);

		   	std::vector<int>::iterator iter = find(comp_values.begin(), comp_values.end(), comp);
			if(comp_values.end() == iter)
			{
				comp_values.push_back(comp);
			}

			comp_matrix.at<int>(y, x) = comp;
			// assign color
			for (int k = 0; k < 3; k++) {
				dst.at<cv::Vec3b>(y, x)[k] = randClr[comp][k];
			}
		}
	}

	regions.resize(comp_values.size());
	for(int idx = 0; idx < comp_values.size(); ++idx)
	{
		regions[idx] = (comp_matrix == comp_values[idx]);
		cv::flip(regions[idx], regions[idx], 1);
		//cv::imshow("region", regions[idx]);
		//cv::waitKey(0);
	}

	delete d;

	return d->num_sets();
}

/* ************************************************************************* */
bool Comparison(const Edge& a, const Edge& b)
{
	return a.w < b.w;		
}

/* ************************************************************************* */
DisJoint *GraphBasedImageSeg::SegGraph(const cv::Mat& depth_map, const int num_vertices, 
					   const int num_edges, Edge* edges)
{
	int height = depth_map.rows;
	int width = depth_map.cols;

	std::sort(edges, edges + num_edges, Comparison);

	DisJoint *d = new DisJoint(num_vertices);

	// stores the maximum and minimum depth value of each region
	double* component_max = new double[num_vertices];
	double* component_min = new double[num_vertices];

	for (int i = 0; i < num_vertices; i++) 
	{
		int row = i / width;
		int col = i % width;

		const double* ptr_depth_map = depth_map.ptr<double>(row);
		double depth_value = ptr_depth_map[col];

		component_max[i] = depth_value;
		component_min[i] = depth_value;
	}

	for (int i = 0; i < num_edges; i++) 
	{
		Edge* pedge = &edges[i];
		//const Edge* pedge = edges + i;

		int a = d->find(pedge->a);
		int b = d->find(pedge->b);

		if (a != b) 
		{
			// Depth of field constraint
			double a_min = component_min[a];
			double a_max = component_max[a];
			double b_min = component_min[b];
			double b_max = component_max[b];

			double minimum = std::min(a_min, b_min);
			double maximum = std::max(a_max, b_max);

			double diff = (maximum - minimum);	//mm
			FrontBackDOF dof_at_minimum = GetFrontBackDof(minimum);
			FrontBackDOF dof_at_maximum = GetFrontBackDof(maximum);

			if ((diff < dof_at_minimum.back_dof) || (diff < dof_at_maximum.front_dof))
			{
				d->join(a, b);
				a = d->find(a);

				// Update min and max array
				component_min[a] = minimum;
				component_max[a] = maximum;
			}
		}

	}

	delete[] component_min;
	delete[] component_max;

	return d;
}




