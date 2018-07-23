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

class ASIFT_matcher
{
public:
	ASIFT_matcher();
	virtual ~ASIFT_matcher();

	bool addReference(const char* image, int num_tilts);
	bool match(const char* image, int num_tilts);
	
protected:
	//QUESCEQUESAI
	int _verb = 0;

	//Reference Images
	// vector< image > _im_refs;
	unsigned int _nb_refs = 0;
	vector< vector< float > > _im_refs;
	vector< pair<int,int> > _size_refs; //Width/Height

	//ASIFT Keypoints
	vector< int > _num_keys;
	vector< int > _num_tilts; //Speed VS Precision
	vector< vector< vector< keypointslist > > > _keys;

	//Matchs
	vector < int > _num_matchings;
	vector< matchingslist > _matchings;

	siftPar _siftParam;	

	//Flags
	bool resize_imgs = false;
};