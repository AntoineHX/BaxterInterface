#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <vector>
#include <sstream>

#ifdef _OPENMP
#include <omp.h>
#endif

#include "demo_lib_sift.h"
// #include "io_png/io_png.h"

#include "library.h"
#include "frot.h"
#include "fproj.h"
#include "compute_asift_keypoints.h"
#include "compute_asift_matches.h"

#include "CImg.h" //Need ImageMagick package

# define IM_X 800
# define IM_Y 600

using namespace std;

typedef vector< vector< keypointslist > > asift_keypoints;

//ASIFT wrapper
class ASIFT_matcher
{
public:
	ASIFT_matcher();
	ASIFT_matcher(const char* ref_path);
	ASIFT_matcher(const ASIFT_matcher& matcher) { *this = matcher;}
	// virtual ~ASIFT_matcher();

	bool addReference(const char* image_path, unsigned int num_tilts=1);
	unsigned int match(const char* image_path, unsigned int num_tilts =1);
	unsigned int match(vector<float>& image, unsigned int w, unsigned int h, unsigned int num_tilts =1);
	bool computeROI(int& x, int& y, unsigned int& h, unsigned int& w) const; //Compute the bounding rectangle of the keypoints
	bool computeCenter(int& cx, int& cy) const;
	bool distFilter(int threshold); //Filter keypoint which are far (Euclidian distance) from the center.

	bool saveReferences(const char* ref_path) const;
	bool loadReferences(const char* ref_path);

	ASIFT_matcher& operator=(const ASIFT_matcher& m);

	unsigned int getNbRef() const{ return _nb_refs;}
	const vector< vector< float > >& getRefImgs() const{ return _im_refs;}
	const vector< pair<int,int> >& getSizeRef() const{ return _size_refs;}
	const vector<float>& getZoomRef() const{ return _zoom_refs;}
	const std::vector<int>& getNumKeys() const{ return _num_keys;}
	const std::vector<int>& getNumTilts() const{ return _num_tilts;}
	const std::vector< asift_keypoints >& getKeys() const{ return _keys;}
	const vector < unsigned int >& getNbMatchs() const{ return _num_matchings;}
	unsigned int getNbMatch() const{ return _total_num_matchings;}
	const vector< matchingslist >& getMatch() const{ return _matchings;}
	vector< matchingslist >& getMatch(){ return _matchings;}
	const siftPar& getSiftPar() const{ return _siftParam;}
	void setSiftPar(const siftPar &newSiftPar){ _siftParam = newSiftPar;}
	bool isResizingImg() const{ return _resize_imgs;}
	void setResizeImg(bool resize_imgs){ _resize_imgs=resize_imgs;}
	bool isShowingDebug() const{ return _showDebug;}
	void showDebug(bool showDebug){ _showDebug=showDebug;}

	void print() const; //Debugging function

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
	unsigned int _total_num_matchings;
	vector < unsigned int > _num_matchings; //Number of match/reference
	vector< matchingslist > _matchings; //Matchs

	siftPar _siftParam;	//SIFT parameters

	//Flags
	bool _resize_imgs;// = false; //Resize images to IM_X/IM_Y ?
	bool _showDebug;// = 0; //Show debugging messages ?
};
