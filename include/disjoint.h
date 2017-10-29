//-------------------------------------------------------------//
/**
* @file disjoint.h
* @brief Locate and merge regions during segmentation
* @date Nov. 6th 2016
* @author Hang Liu
*/

/* ************************************************************************* */
/**
* @brief Struct that describes the properties of specific region
* Rank	This Locates which region of the pixel in x belong to
* p		Location of the given pixel
* size	The region id of pixel in location x
*/
typedef struct {
	int rank;
	int p;
	int size;
}DisjElem;

class DisJoint{
public:
	DisJoint(int elements);
	~DisJoint();

	/* ************************************************************************* */
	/**
	* @brief	This Locates which region of the pixel in x belong to
	* @param x	Location of the given pixel
	* @return	The region id of pixel in location x
	*/
	int find(int x);

	/* ************************************************************************* */
	/**
	* @brief	Merge two regions
	* @param x	One region id
	* @param y  Another region id 
	*/
	void join(int x, int y);

	/* ************************************************************************* */
	/**
	* @brief	 Get the size of specific region
	* @param  x	 Region id
	* @return    Region size
	*/
	int size(int x) const { return elts[x].size; }

	/* ************************************************************************* */
	/**
	* @brief	   Get the total regions of image
	* @param  num  Region id
	* @return	   Region size
	*/
	int num_sets() const { return num; }

public:
	// Organize vertices of an image
	DisjElem *elts;

	// Number of regions of an image
	int num;
};