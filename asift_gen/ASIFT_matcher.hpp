/*
 * Image matching using Affine-SIFT algorithm.
 * Allow to find matching keypoints, filter, compute ROI & center of an object in an image, after having built the references (with at least 1 image).
 * @author : antoine.harle@etu.upmc.Fr
 * Reference: J.M. Morel and G.Yu, ASIFT: A New Framework for Fully Affine Invariant Image 
 *            Comparison, SIAM Journal on Imaging Sciences, vol. 2, issue 2, pp. 438-469, 2009. 
 * Reference: ASIFT online demo (You can try ASIFT with your own images online.) 
 *			  http://www.ipol.im/pub/algo/my_affine_sift/
 */


#ifndef ASIFTMATCHER_HPP
#define ASIFTMATCHER_HPP

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

class ASIFT_matcher
{
public:
	ASIFT_matcher();//Default constructor.
	ASIFT_matcher(const char* ref_path); //Constuctor from keypoints references (.txt).
	ASIFT_matcher(const ASIFT_matcher& matcher) { *this = matcher;} //Copy constructor.
	// virtual ~ASIFT_matcher();

	bool addReference(const char* image_path, unsigned int num_tilts =1); //Add a reference image.
	bool addReference(const vector<float>& image, unsigned int w, unsigned int h, unsigned int num_tilts =1); //Add a reference image.
	unsigned int match(const char* image_path, unsigned int num_tilts =1); //Perform matching between an image and the references.
	unsigned int match(const vector<float>& image, unsigned int w, unsigned int h, unsigned int num_tilts =1); //Perform matching between an image and the references.
	bool computeROI(int& x, int& y, unsigned int& h, unsigned int& w) const; //Compute the bounding rectangle of the mathcing keypoints.
	bool computeCenter(int& cx, int& cy) const; //Compute the centroid of the matching keypoints.
	bool distFilter(float threshold =2); //Perform a standard deviation filtering on the matching keypoints.

	bool saveReferences(const char* ref_path) const; //Save reference data necessary for the matching.
	bool loadReferences(const char* ref_path); //Load reference data necessary for the matching.

	ASIFT_matcher& operator=(const ASIFT_matcher& m); //Assignation operator.

	//Setter/Getter
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
	void resizeImg(bool resize_imgs){ _resize_imgs=resize_imgs;}
	bool isShowingDebug() const{ return _showDebug;}
	void showDebug(bool showDebug){ _showDebug=showDebug;}
	bool isShowingInfo() const{ return _showInfo;}
	void showInfo(bool showInfo){ _showInfo=showInfo;}

	void print() const; //Debugging function : print content form the ASIFT matcher.

protected:

	//Reference Images
	unsigned int _nb_refs;//Number of reference images
	vector< vector< float > > _im_refs; //Reference images used for matching
	vector< pair<int,int> > _size_refs; //Width/Height
	vector<float> _zoom_refs; //Zoom coeffs

	//Reference ASIFT Keypoints
	vector< int > _num_keys; //Number of keypoint/reference
	vector< int > _num_tilts; //Number of tilts/reference (Speed VS Precision)
	vector< asift_keypoints > _keys; //Reference keypoints

	//Matchs
	unsigned int _total_num_matchings; //Number of matching keypoints.
	vector < unsigned int > _num_matchings; //Number of match/reference
	vector< matchingslist > _matchings; //Matching keypoints

	siftPar _siftParam;	//SIFT parameters

	//Flags
	bool _resize_imgs;//Resize images to IM_X/IM_Y ?
	bool _showDebug;//Show debugging messages ?
	bool _showInfo; //Show info messages
};

#endif