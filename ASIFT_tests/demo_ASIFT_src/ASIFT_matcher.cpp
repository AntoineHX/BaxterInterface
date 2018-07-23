#include "ASIFT_matcher.hpp"

ASIFT_matcher::ASIFT_matcher()
{
	default_sift_parameters(_siftParam);
}

ASIFT_matcher::~ASIFT_matcher()
{

}

bool ASIFT_matcher::addReference(const char* image, int num_tilts = 1)
{
	///// Read input
	float * iarr1;
    size_t w1, h1;
    if (NULL == (iarr1 = read_png_f32_gray(image, &w1, &h1))) {
        std::cerr << "Unable to load image file " << image << std::endl;
        return false;
    }
    std::vector<float> ipixels1(iarr1, iarr1 + w1 * h1);
	free(iarr1); /*memcheck*/

    ///// Resize the images to area wS*hW in remaining the apsect-ratio	
	///// Resize if the resize flag is not set or if the flag is set unequal to 0
	float wS = IM_X;
	float hS = IM_Y;
	
	float zoom1=0;	
	int wS1=0, hS1=0;
	vector<float> ipixels1_zoom;

	if(resize_imgs)
	{
		cout << "WARNING: The input image is resized to " << wS << "x" << hS << " for ASIFT. " << endl 
		<< "         But the results will be normalized to the original image size." << endl << endl;
		
		float InitSigma_aa = 1.6;
		
		float fproj_p, fproj_bg;
		char fproj_i;
		float *fproj_x4, *fproj_y4;
		int fproj_o;

		fproj_o = 3;
		fproj_p = 0;
		fproj_i = 0;
		fproj_bg = 0;
		fproj_x4 = 0;
		fproj_y4 = 0;
				
		float areaS = wS * hS;

		// Resize image 1 
		float area1 = w1 * h1;
		zoom1 = sqrt(area1/areaS);
		
		wS1 = (int) (w1 / zoom1);
		hS1 = (int) (h1 / zoom1);
		
		int fproj_sx = wS1;
		int fproj_sy = hS1;     
		
		float fproj_x1 = 0;
		float fproj_y1 = 0;
		float fproj_x2 = wS1;
		float fproj_y2 = 0;
		float fproj_x3 = 0;	     
		float fproj_y3 = hS1;
		
		/* Anti-aliasing filtering along vertical direction */
		if ( zoom1 > 1 )
		{
			float sigma_aa = InitSigma_aa * zoom1 / 2;
			GaussianBlur1D(ipixels1,w1,h1,sigma_aa,1);
			GaussianBlur1D(ipixels1,w1,h1,sigma_aa,0);
		}
			
		// simulate a tilt: subsample the image along the vertical axis by a factor of t.
		ipixels1_zoom.resize(wS1*hS1);
		fproj (ipixels1, ipixels1_zoom, w1, h1, &fproj_sx, &fproj_sy, &fproj_bg, &fproj_o, &fproj_p, 
			   &fproj_i , fproj_x1 , fproj_y1 , fproj_x2 , fproj_y2 , fproj_x3 , fproj_y3, fproj_x4, fproj_y4); 
	}
	else
	{
		ipixels1_zoom.resize(w1*h1);	
		ipixels1_zoom = ipixels1;
		wS1 = w1;
		hS1 = h1;
		zoom1 = 1;
	}

	// image new_ref;
	// new_ref.img = ipixels1_zoom;
	// new_ref.width = wS1;
	// new_ref.height = hS1;


	///// Compute ASIFT keypoints
	vector< vector< keypointslist > > keys;
	int num_keys = 0;

	time_t tstart, tend;
	tstart = time(0);

	num_keys = compute_asift_keypoints(ipixels1_zoom, wS1, hS1, num_tilts, _verb, keys, _siftParam);

	tend = time(0);

	//Save data
	_im_refs.push_back(ipixels1_zoom);
	_size_refs.push_back(make_pair(wS1,hS1));

	_num_keys.push_back(num_keys);
	_num_tilts.push_back(num_tilts);
	_keys.push_back(keys);

	_nb_refs++;

	cout<<"Reference built in "<< difftime(tend, tstart) << " seconds." << endl;
	cout<<"		"<< num_keys <<" ASIFT keypoints found in "<< image << endl;

	return true;
}

bool ASIFT_matcher::match(const char* image, int num_tilts = 1)
{
	if(_nb_refs<=0)
	{
		cout<<"ASIFT_matcher Error : Trying to match without reference"<<endl;
		return false;
	}

	///// Read input
	float * iarr1;
    size_t w1, h1;
    if (NULL == (iarr1 = read_png_f32_gray(image, &w1, &h1))) {
        std::cerr << "Unable to load image file " << image << std::endl;
        return 1;
    }
    std::vector<float> ipixels1(iarr1, iarr1 + w1 * h1);
	free(iarr1); /*memcheck*/

    ///// Resize the images to area wS*hW in remaining the apsect-ratio	
	///// Resize if the resize flag is not set or if the flag is set unequal to 0
	float wS = IM_X;
	float hS = IM_Y;
	
	float zoom1=0;	
	int wS1=0, hS1=0;
	vector<float> ipixels1_zoom;

	if(resize_imgs)
	{
		cout << "WARNING: The input image is resized to " << wS << "x" << hS << " for ASIFT. " << endl 
		<< "         But the results will be normalized to the original image size." << endl << endl;
		
		float InitSigma_aa = 1.6;
		
		float fproj_p, fproj_bg;
		char fproj_i;
		float *fproj_x4, *fproj_y4;
		int fproj_o;

		fproj_o = 3;
		fproj_p = 0;
		fproj_i = 0;
		fproj_bg = 0;
		fproj_x4 = 0;
		fproj_y4 = 0;
				
		float areaS = wS * hS;

		// Resize image 1 
		float area1 = w1 * h1;
		zoom1 = sqrt(area1/areaS);
		
		wS1 = (int) (w1 / zoom1);
		hS1 = (int) (h1 / zoom1);
		
		int fproj_sx = wS1;
		int fproj_sy = hS1;     
		
		float fproj_x1 = 0;
		float fproj_y1 = 0;
		float fproj_x2 = wS1;
		float fproj_y2 = 0;
		float fproj_x3 = 0;	     
		float fproj_y3 = hS1;
		
		/* Anti-aliasing filtering along vertical direction */
		if ( zoom1 > 1 )
		{
			float sigma_aa = InitSigma_aa * zoom1 / 2;
			GaussianBlur1D(ipixels1,w1,h1,sigma_aa,1);
			GaussianBlur1D(ipixels1,w1,h1,sigma_aa,0);
		}
			
		// simulate a tilt: subsample the image along the vertical axis by a factor of t.
		ipixels1_zoom.resize(wS1*hS1);
		fproj (ipixels1, ipixels1_zoom, w1, h1, &fproj_sx, &fproj_sy, &fproj_bg, &fproj_o, &fproj_p, 
			   &fproj_i , fproj_x1 , fproj_y1 , fproj_x2 , fproj_y2 , fproj_x3 , fproj_y3, fproj_x4, fproj_y4); 
	}
	else
	{
		ipixels1_zoom.resize(w1*h1);	
		ipixels1_zoom = ipixels1;
		wS1 = w1;
		hS1 = h1;
		zoom1 = 1;
	}

	///// Compute ASIFT keypoints
	vector< vector< keypointslist > > keys;
	int num_keys = 0;

	time_t tstart, tend;
	tstart = time(0);

	num_keys = compute_asift_keypoints(ipixels1_zoom, wS1, hS1, num_tilts, _verb, keys, _siftParam);

	tend = time(0);
	cout << "Keypoints computation accomplished in " << difftime(tend, tstart) << " seconds." << endl;

	//// Match ASIFT keypoints
	int num_matchings;
	matchingslist matchings;	

	_num_matchings.clear();
	_matchings.clear();

	for(unsigned int i = 0; i<_nb_refs;i++)
	{
		cout << "Matching the keypoints..." << endl;
		tstart = time(0);
		num_matchings = compute_asift_matches(num_tilts, _num_tilts[i], wS1, hS1, _size_refs[i].first, _size_refs[i].second, _verb, keys, _keys[i], matchings, _siftParam);
		tend = time(0);
		cout << "Keypoints matching accomplished in " << difftime(tend, tstart) << " seconds." << endl;

		_num_matchings.push_back(num_matchings);
		_matchings.push_back(matchings);
	}

	return true;
}