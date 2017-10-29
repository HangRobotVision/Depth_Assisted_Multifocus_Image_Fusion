//-------------------------------------------------------------//
//
// * Graph Cuts Segmentation with OpenCV
//   In "Efficient Graph-Based Image Segmentation" 
//   by Pedro Felzenszwalb & Daniel P. Huttenlocher (2004)
//
//-------------------------------------------------------------//
//
// * Date: 2014.04.16
// * Coder: Mark Lee(tefactory@live.com)
// * OpenCV Version: 2.4.8
// * Original Source: http://cs.brown.edu/~pff/segment/
//
//-------------------------------------------------------------//

/* Graphcuts.h: Graph process and segmentation */
// #include <iostream>
// #include <cstdlib>
// #include <algorithm>
// #include <cmath>
#include "disjoint.h"

#include "opencv2/core/core.hpp"

#include <limits.h>

typedef struct FrontBackDOF
{
	double front_dof;
	double back_dof;
	double total_dof;
} FrontBackDOF;

typedef struct 
{
	double w;
	int a, b;
} Edge;

/* ************************************************************************* */
/**
* @brief:			compare edges based on edge weight
* @param  a:		edge a
* @param  b:		edge b
* @return:  		true: a.w < b.w	false: a.w > b.w
*/
bool Comparison(const Edge& a, const Edge& b);

class GraphBasedImageSeg{
public:
	GraphBasedImageSeg(const double coc_diameter = 0.019, const double aperture_value = 4.0, const double focal_length = 24.0);
	~GraphBasedImageSeg();

	/* ************************************************************************* */
	/**
	* @brief:  					this function implements the graph based image segmentation method
	* @param  depth_map:		original depth map to be segmented
	* @param  small_thresh:		determine the least pixels of each specific region
	* @param  regions:			array of segmented regions	
	* @param  dst: 				colorized segmentation result  
	* @return:					number of segmented regions
	*/
	int GraphSegment(const cv::Mat& depth_map, const int small_thresh, std::vector<cv::Mat>& regions,
					 cv::Mat& dst);
private:
	/* ************************************************************************* */
	/**
	* @brief:		 	this function calculated difference of two pixel values 
	* @param  depth:	original depth image
	* @param  x1:	 	X coordinate of pixel 1
	* @param  y1:      	Y coordinate of pixel 1
	* @param  x2:     	X coordinate of pixel 2
	* @param  y2:     	Y coordinate of pixel 2
	* @return:		 	calculated difference value 	
	*/ 
	double Dissim(const cv::Mat& depth, const int x1, const int y1, 
				  const int x2, const int y2);

	/* ************************************************************************* */
	/**
	* @brief: 			   this function calculates front and back depth of field based on the given
	*		 			   depth value
	* @param depth_value:  depth value
	* @return			   calculated front and back depth of field in depth_value
	*/
	FrontBackDOF GetFrontBackDof(const double depth_value);

	/* ************************************************************************* */
	/**
	* @brief: 				graph based depth map segmentation method based on edge graphs
	* @param  depth_map: 	original depth to be segmented
	* @param  num_vertices: number of vertices of edge graphs (equals depth_map.rows * depth_map.cols)
	* @param  num_edges: 	number of edges of edge graphs(about num_vertices * 4)
	* @param  edges: 		edge graph	
	* @return: 				segmented regions represented in linking disjoints
	*/
	DisJoint *SegGraph(const cv::Mat& depth_map, const int num_vertices, 
					   const int num_edges, Edge* edges);

	/* ************************************************************************* */
	/**
	* @brief: 				 the initialized table contains log-scaled depth value that correspond to the value from 0 to USHORT_MAX
	* @param  depth_map: 	 original depth to be segmented
	* @param  num_vertices:  number of vertices of edge graphs (equals depth_map.rows * depth_map.cols)
	* @param  num_edges: 	 number of edges of edge graphs(about num_vertices * 4)
	* @param  edges: 		 edge graph	
	* @return: 				 segmented regions represented in linking disjoints
	*/
	//int InitializeDepthColorTable(void);

	/* ************************************************************************* */
	/**
	* @brief:				obtain the log-scaled depth value for depth_value  				
	* @param  depth_value: 	depth value
	* @return: 				log-scaled depth value
	*/
	//unsigned char GetIntensity(const int depth_value);

private:
	// diameter of the circle of confusion
	double fai; 
	double F;
	double f;	
};







