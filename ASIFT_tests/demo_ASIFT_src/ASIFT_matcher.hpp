#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <vector>
using namespace std;

#ifdef _OPENMP
#include <omp.h>
#endif

#include "demo_lib_sift.h"
#include "io_png/io_png.h"

#include "library.h"
#include "frot.h"
#include "fproj.h"
#include "compute_asift_keypoints.h"
#include "compute_asift_matches.h"

# define IM_X 800
# define IM_Y 600

// struct image
// {
// 	vector<float> img;
// 	int width;
// 	int height;
// } image;

typedef vector< vector< keypointslist > > asift_keypoints;

//ASIFT wrapper
class ASIFT_matcher
{
public:
	ASIFT_matcher();
	// virtual ~ASIFT_matcher();

	bool addReference(const char* image, unsigned int num_tilts=1);
	bool match(const char* image, unsigned int num_tilts =1);
	bool match(vector<float>& image, unsigned int w, unsigned int h, unsigned int num_tilts =1);
	void print() const; //Debugging function
	bool computeROI(int& x, int& y, unsigned int& h, unsigned int& w) const; //Compute the bounding rectangle of the keypoints
	bool computeCenter(int& cx, int& cy) const;
	bool distFilter(int threshold); //Filter keypoint which are far (Euclidian distance) from the center.


	unsigned int getNbRef() const{ return _nb_refs;}
	const vector< vector< float > >& getRefImgs() const{ return _im_refs;}
	const vector< pair<int,int> >& getSizeRef() const{ return _size_refs;}
	const vector<float>& getZoomRef() const{ return _zoom_refs;}
	const vector < unsigned int >& getNbMatchs() const{ return _num_matchings;}
	unsigned int getNbMatch() const;
	const vector< matchingslist >& getMatch() const{ return _matchings;}
	vector< matchingslist >& getMatch(){ return _matchings;}
	const siftPar& getSiftPar(){ return _siftParam;}
	void setSiftPar(const siftPar &newSiftPar){ _siftParam = newSiftPar;}
	void setResizeImg(bool resize_imgs){ _resize_imgs=resize_imgs;}
	void showDebug(bool showDebug){ _showDebug=showDebug;}

protected:

	//Reference Images
	// vector< image > _im_refs;
	unsigned int _nb_refs;// = 0; //Number of reference images
	vector< vector< float > > _im_refs; //Reference images used for matching
	vector< pair<int,int> > _size_refs; //Width/Height
	vector<float> _zoom_refs; //Zoom coeffs

	//ASIFT Keypoints
	vector< int > _num_keys; //Number of keypoint/reference
	vector< int > _num_tilts; //Number of tilts/reference (Speed VS Precision)
	vector< asift_keypoints > _keys; //Keypoints

	//Matchs
	vector < unsigned int > _num_matchings; //Number of match/reference
	vector< matchingslist > _matchings; //Matchs

	siftPar _siftParam;	//SIFT parameters

	//Flags
	bool _resize_imgs;// = false; //Resize images to IM_X/IM_Y ?
	bool _showDebug;// = 0; //Show debugging messages ?
};