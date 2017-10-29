#include "global.h"

#include <cmath>
#include <memory.h>

#define USHORT_MAX				0xffff	 
#define UCHAR_MAX					0xff
#define	MIN_DEPTH					400     // minimum reliable depth value of Kinect
#define MAX_DEPTH					16383   // maximum reliable depth value of Kinect
#define UNKNOWN_DEPTH				0
#define UNKNOWN_DEPTH_COLOR         0x00000000      
#define NEAREST_COLOR				0x00000000
#define TOO_FAR_COLOR               0x000000FF
#define MINIMUM(a,b)                (((a) < (b)) ? (a) : (b))

/* ************************************************************************* */
/**
* @brief:                       get scaled depth value
* @param  depth:                the depth value that to be scaled                       
* @return:                      scaled detph value
*/
unsigned char GetIntensity(int depth);

// look-up table for generating log-scaled depth map for visualization
unsigned char depth_color_table[USHORT_MAX + 1];

int InitializeDepthColorTable(void)
{
	memset(depth_color_table, 0, USHORT_MAX + 1);
	// set color for unknown depth
	depth_color_table[UNKNOWN_DEPTH] = UNKNOWN_DEPTH_COLOR;
	
	unsigned short min_reliable_depth = MIN_DEPTH;
	unsigned short max_reliable_depth = MAX_DEPTH;

	for (int depth = UNKNOWN_DEPTH + 1; depth < min_reliable_depth; depth++)
	{
		depth_color_table[depth] = NEAREST_COLOR;
	}

	for (unsigned short depth = min_reliable_depth; depth <= max_reliable_depth; depth++)
	{
		unsigned char intensity = GetIntensity(depth);
		depth_color_table[depth] = 255 - intensity;
	}

	return 0;
}

unsigned char GetIntensity(int depth)
{
	// Validate arguments
	if (depth < MIN_DEPTH || depth > MAX_DEPTH)
	{
		return UCHAR_MAX;
	}

	// Use a logarithmic scale that shows more detail for nearer depths.
	// The constants in this formula were chosen such that values between
	// MIN_DEPTH and MAX_DEPTH will map to the full range of possible
	// byte values.
	const float depthRangeScale = 500.0f;
	const int intensityRangeScale = 74;
	return (unsigned char)(~(unsigned char)MINIMUM(
		UCHAR_MAX,
		log((double)(depth - MIN_DEPTH) / depthRangeScale + 1) * intensityRangeScale));
}