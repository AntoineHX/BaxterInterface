#include "ASIFT_matcher.hpp"

ASIFT_matcher::ASIFT_matcher(): _nb_refs(0), _total_num_matchings(0), _resize_imgs(false), _showDebug(false)
{
	default_sift_parameters(_siftParam);
}

// ASIFT_matcher::~ASIFT_matcher()
// {

// }

//Return true if successfull
bool ASIFT_matcher::addReference(const char* image_path, unsigned int num_tilts)
{
	///// Read input
	// float * iarr1;
 //    size_t w1, h1;
 //    if (NULL == (iarr1 = read_png_f32_gray(image_path, &w1, &h1))) {
 //        std::cerr << "Unable to load image file " << image_path << std::endl;
 //        return false;
 //    }
 //    std::vector<float> ipixels1(iarr1, iarr1 + w1 * h1);
	// free(iarr1); /*memcheck*/

	// cout<<"Size : "<<w1<<"/"<<h1<<" - "<<ipixels1.size()<<endl;

	cimg_library::CImg<float> image;
	try
	{
		image.assign(image_path);
	}
	catch(cimg_library::CImgIOException)
	{
		std::cerr << "Unable to load image file " << image_path << std::endl;
		return false;
	}
	//Convert to grayscale
	cimg_library::CImg<float> gray(image.width(), image.height(), 1, 1, 0);
	cimg_forXY(image,x,y) { 
    // Separation of channels
    int R = (int)image(x,y,0,0);
    int G = (int)image(x,y,0,1);
    int B = (int)image(x,y,0,2);
    // Arithmetic addition of channels for gray
    // int grayValue = (int)(0.33*R + 0.33*G + 0.33*B);
    // Real weighted addition of channels for gray
    int grayValueWeight = (int)(0.299*R + 0.587*G + 0.114*B);
    // saving píxel values into image information
    // gray(x,y,0,0) = grayValue;
    gray(x,y,0,0) = grayValueWeight;
	}

	vector<float> ipixels1;
    size_t w1=gray.width(), h1=gray.height();
    ipixels1.assign(gray.begin(), gray.end());

    ///// Resize the images to area wS*hW in remaining the apsect-ratio	
	///// Resize if the resize flag is not set or if the flag is set unequal to 0
	float wS = IM_X;
	float hS = IM_Y;
	
	float zoom1=0;	
	int wS1=0, hS1=0;
	vector<float> ipixels1_zoom;

	if(_resize_imgs)
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
	asift_keypoints keys;
	// vector< vector< keypointslist > >* keys = new vector< vector< keypointslist > >;
	int num_keys = 0;

	time_t tstart, tend;
	tstart = time(0);

	num_keys = compute_asift_keypoints(ipixels1_zoom, wS1, hS1, num_tilts, _showDebug, keys, _siftParam);

	tend = time(0);

	//Save data
	_im_refs.push_back(ipixels1_zoom);
	_size_refs.push_back(make_pair(wS1,hS1));
	_zoom_refs.push_back(zoom1);

	_num_keys.push_back(num_keys);
	_num_tilts.push_back(num_tilts);
	_keys.push_back(keys);

	_nb_refs++;

	cout<<"Reference built in "<< difftime(tend, tstart) << " seconds." << endl;
	cout<<"	"<< num_keys <<" ASIFT keypoints found in "<< image_path << endl;

	return true;
}

//Return number of match
unsigned int ASIFT_matcher::match(const char* image_path, unsigned int num_tilts)
{
	if(_nb_refs<=0)
	{
		cout<<"ASIFT_matcher Error : Trying to match without reference"<<endl;
		return 0;
	}

	///// Read input
	// float * iarr1;
 //    size_t w1, h1;
 //    if (NULL == (iarr1 = read_png_f32_gray(image_path, &w1, &h1))) {
 //        std::cerr << "Unable to load image file " << image_path << std::endl;
 //        return 1;
 //    }
 //    std::vector<float> ipixels1(iarr1, iarr1 + w1 * h1);
	// free(iarr1); /*memcheck*/

    cimg_library::CImg<float> image;
	try
	{
		image.assign(image_path);
	}
	catch(cimg_library::CImgIOException)
	{
		std::cerr << "Unable to load image file " << image_path << std::endl;
		return 0;
	}
	//Convert to grayscale
	cimg_library::CImg<float> gray(image.width(), image.height(), 1, 1, 0);
	cimg_forXY(image,x,y) { 
    // Separation of channels
    int R = (int)image(x,y,0,0);
    int G = (int)image(x,y,0,1);
    int B = (int)image(x,y,0,2);
    // Arithmetic addition of channels for gray
    // int grayValue = (int)(0.33*R + 0.33*G + 0.33*B);
    // Real weighted addition of channels for gray
    int grayValueWeight = (int)(0.299*R + 0.587*G + 0.114*B);
    // saving píxel values into image information
    // gray(x,y,0,0) = grayValue;
    gray(x,y,0,0) = grayValueWeight;
	}
	vector<float> ipixels1;
    size_t w1=gray.width(), h1=gray.height();
    ipixels1.assign(gray.begin(), gray.end());

    ///// Resize the images to area wS*hW in remaining the apsect-ratio	
	///// Resize if the resize flag is not set or if the flag is set unequal to 0
	float wS = IM_X;
	float hS = IM_Y;
	
	float zoom1=0;	
	int wS1=0, hS1=0;
	vector<float> ipixels1_zoom;

	if(_resize_imgs)
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
	asift_keypoints keys;
	int num_keys = 0;

	time_t tstart, tend;
	tstart = time(0);

	num_keys = compute_asift_keypoints(ipixels1_zoom, wS1, hS1, num_tilts, _showDebug, keys, _siftParam);

	tend = time(0);
	cout<< "Keypoints computation accomplished in " << difftime(tend, tstart) << " seconds." << endl;
	cout<<"	"<< num_keys <<" ASIFT keypoints found in "<< image_path << endl;

	//// Match ASIFT keypoints
	_total_num_matchings=0;

	for(unsigned int i = 0; i<_nb_refs;i++)
	{
		int num_matchings = 0;
		matchingslist matchings;

		cout << "Matching the keypoints..." << endl;
		tstart = time(0);
		try
		{
			num_matchings = compute_asift_matches(num_tilts, _num_tilts[i], wS1, hS1, _size_refs[i].first, _size_refs[i].second, _showDebug, keys, _keys[i], matchings, _siftParam);
		}
		catch(const bad_alloc& ba)
		{
			cerr<<"ERROR: ASIFT_matcher::match - ";
			cerr << ba.what() << endl;
		}
		// cout<< _keys[i].size()<< " " << _keys[i][0].size() <<" "<< _keys[i][0][0].size()<<endl;

		tend = time(0);
		cout << "Keypoints matching accomplished in " << difftime(tend, tstart) << " seconds." << endl;

		_num_matchings.push_back(num_matchings);
		_total_num_matchings += num_matchings;
		_matchings.push_back(matchings);
	}

	return _total_num_matchings;
}

//Return number of match
unsigned int ASIFT_matcher::match(vector<float>& image, unsigned int w, unsigned int h, unsigned int num_tilts)
{
	if(image.size()!=w*h)
	{
		cerr<<"Error : Input image size doesn't correspond with parameters"<<endl;
		return 0;
	}

	///// Compute ASIFT keypoints
	asift_keypoints keys;
	int num_keys = 0;

	time_t tstart, tend;
	tstart = time(0);

	num_keys = compute_asift_keypoints(image, w, h, num_tilts, _showDebug, keys, _siftParam);

	tend = time(0);
	cout<< "Keypoints computation accomplished in " << difftime(tend, tstart) << " seconds." << endl;
	cout<<"	"<< num_keys <<" ASIFT keypoints found in Input image"<< endl;

	//// Match ASIFT keypoints
	_total_num_matchings=0;

	for(unsigned int i = 0; i<_nb_refs;i++)
	{
		int num_matchings = 0;
		matchingslist matchings;

		cout << "Matching the keypoints..." << endl;
		tstart = time(0);
		try
		{
			num_matchings = compute_asift_matches(num_tilts, _num_tilts[i], w, h, _size_refs[i].first, _size_refs[i].second, _showDebug, keys, _keys[i], matchings, _siftParam);
		}
		catch(const bad_alloc& ba)
		{
			cerr<<"ERROR: ASIFT_matcher::match - ";
			cerr << ba.what() << endl;
		}
		// cout<< _keys[i].size()<< " " << _keys[i][0].size() <<" "<< _keys[i][0][0].size()<<endl;

		tend = time(0);
		cout << "Keypoints matching accomplished in " << difftime(tend, tstart) << " seconds." << endl;

		_num_matchings.push_back(num_matchings);
		_total_num_matchings += num_matchings;
		_matchings.push_back(matchings);
	}

	return _total_num_matchings;
}

//Return true if successfull
bool ASIFT_matcher::computeROI(int& x, int& y, unsigned int& h, unsigned int& w) const
{
	if(getNbMatch()==0)
	{
		cerr<<"Error : cannot compute ROI without matchs"<<endl;
		return false;
	}

	pair<int,int> upLe, doRi; //UpLeft / DownRight
	//Initialisation
	for(unsigned int i=0;i<_matchings.size();i++)
	{
		if(getNbMatchs()[i]!=0)
		{
			upLe = make_pair(_matchings[i][0].first.x,_matchings[i][0].first.y);
			doRi = make_pair(_matchings[i][0].first.x,_matchings[i][0].first.y);
		}
	}

	//Compute ROI
	for(unsigned int i=0;i<_matchings.size();i++)
	{
		for(unsigned int j=0;j<_matchings[i].size();j++)
		{
			keypoint kp = _matchings[i][j].first;
			if(kp.x<upLe.first)
				upLe.first = kp.x;
			if(kp.y<upLe.second)
				upLe.second=kp.y;
			if(kp.x>doRi.first)
				doRi.first=kp.x;
			if(kp.y>doRi.second)
				doRi.second=kp.y;
		}
	}
	x=upLe.first; //Système de coordonée ? (devrait etre bon)
	y=upLe.second;
	h=doRi.second-y;
	w=doRi.first-x;

	return true;
}

//Return true if successfull
bool ASIFT_matcher::computeCenter(int& cx, int& cy) const
{
	if(getNbMatch()==0)
	{
		cerr<<"Error : cannot compute Center without matchs"<<endl;
		return false;
	}

	unsigned int total_kp =0;
	cx=0;cy=0;
	for(unsigned int i=0;i<_matchings.size();i++)
	{
		for(unsigned int j=0;j<_matchings[i].size();j++)
		{
			keypoint kp = _matchings[i][j].first;
			cx+=kp.x;
			cy+=kp.y;
		}
		total_kp += _matchings[i].size();
	}
	cx/=total_kp;
	cy/=total_kp;

	return true;
}

//Filter keypoint which are far (Euclidian distance) from the center.
//Not optimized
//threshold : 1-68%/2-95%/3-99%
//Return true if successfull 
bool ASIFT_matcher::distFilter(int threshold=2)
{
	cout<<"filtering keypoint..."<<endl;
	if(getNbMatch()==0)
	{
		cerr<<"Error : cannot filter points without matchs"<<endl;
		return false;
	}

	//Compute standard deviation
	int cx, cy;
	unsigned int total_kp =0;
	unsigned int euc_dist, dist_avg =0, dist_var=0, std_dev;
	vector< vector< int > > kp_euc_dist;

	if(computeCenter(cx,cy))
	{
		// cout<<"Center : "<<cx<<" / "<<cy<<endl;

		//Compute means/average distance to center + euclidian distances to center for each keypoint
		for(unsigned int i=0;i<_matchings.size();i++)
		{
			vector<int> temp_euc_dist;
			for(unsigned int j=0;j<_matchings[i].size();j++)
			{
				keypoint kp = _matchings[i][j].first;
				euc_dist =sqrt((kp.x-cx)*(kp.x-cx)+(kp.y-cy)+(kp.y-cy));
				dist_avg+=euc_dist;
				temp_euc_dist.push_back(euc_dist);
			}
			total_kp += _matchings[i].size();
			kp_euc_dist.push_back(temp_euc_dist);
		}
		dist_avg/=total_kp;
		// cout<<"Dist avg: "<<dist_avg<<endl;

		//Compute variance
		for(unsigned int i=0;i<_matchings.size();i++)
		{
			for(unsigned int j=0;j<_matchings[i].size();j++)
			{
				euc_dist =kp_euc_dist[i][j];
				dist_var+=(euc_dist-dist_avg)*(euc_dist-dist_avg);
			}
		}
		dist_var/=total_kp;

		//Compute standard deviation
		std_dev=sqrt(dist_var);
		// cout<<"Standard Deviation : "<<std_dev<<endl;

		//Filter
		vector< matchingslist > filtered_match;

		for(unsigned int i=0;i<_matchings.size();i++)
		{
			matchingslist new_match;
			for(unsigned int j=0;j<_matchings[i].size();j++)
			{
				euc_dist =kp_euc_dist[i][j];

				if(euc_dist<dist_avg+threshold*std_dev) //Filtering Condition
				{
					new_match.push_back(_matchings[i][j]);
				}
			}
			filtered_match.push_back(new_match);
			_num_matchings[i]=new_match.size();
		}

		//Save filtered matchs
		_matchings = filtered_match;

		return true;
	}
	return false;
}

//A tester
bool ASIFT_matcher::saveReferences(const char* ref_path) const
{
	// Write all the keypoints (row, col, scale, orientation, desciptor (128 integers))
	std::ofstream file_key1(ref_path);
	if (file_key1.is_open())
	{
		file_key1<<_nb_refs<<"  "<<std::endl;
		for(unsigned int i=0; i<_keys.size();i++)
		{
			asift_keypoints kps =_keys[i];
			// Follow the same convention of David Lowe: 
			// the first line contains the number of keypoints and the length of the desciptors (128) 
			// Added number of tilts
			file_key1 << _num_keys[i] << "  " << VecLength << "  " <<std::endl; //<< _num_tilts[i] << "  " << std::endl;
			for (int tt = 0; tt < (int) kps.size(); tt++)
			{
				for (int rr = 0; rr < (int) kps[tt].size(); rr++)
				{
					keypointslist::iterator ptr = kps[tt][rr].begin();
					for(int i=0; i < (int) kps[tt][rr].size(); i++, ptr++)	
					{
						file_key1 << _zoom_refs[i]*ptr->x << "  " << _zoom_refs[i]*ptr->y << "  " << _zoom_refs[i]*ptr->scale << "  " << ptr->angle;
						
						for (int ii = 0; ii < (int) VecLength; ii++)
						{
							file_key1 << "  " << ptr->vec[ii];
						}
						
						file_key1 << std::endl;
					}
				}	
			}
			file_key1<<std::endl;
		}
	}
	else 
	{
		std::cerr << "Unable to open the file :"<<ref_path;
		return false;
	}

	file_key1.close();
	return true;
}

//A finir
bool ASIFT_matcher::loadReferences(const char* ref_path)
{
	std::ifstream ref_file(ref_path);
	std::string line, tmp;
	std::stringstream iss;
	if (ref_file.is_open())
	{
		std::getline(ref_file, line);
		std::string::size_type sz;
		_nb_refs = std::stoi(line, &sz); //C++11
		// _nb_refs = atoi(line.c_str());
		_keys = std::vector<asift_keypoints>(_nb_refs);
		_num_keys = std::vector< int >(_nb_refs);
		_num_tilts = std::vector< int >(_nb_refs,1);
		_zoom_refs = std::vector<float>(_nb_refs,1);
		for(unsigned int i = 0; i<_nb_refs;i++)
		{
			std::getline(ref_file, line);
			std::stringstream iss(line);

			std::getline(iss,tmp,' ');
			_num_keys[i]=atoi(tmp.c_str());

			std::getline(iss,tmp,' ');
			if(VecLength!=atoi(tmp.c_str()))
			{
				std::cout<<"Error VecLength doesn't correspond..."<<std::endl;
				return false;
			}

			// std::getline(iss,tmp,' ');
			// _num_tilts[i]=atoi(tmp.c_str());

			keypointslist list;
			for(unsigned int j =0; j<(unsigned int)_num_keys[j];j++)
			{
				keypoint nkp;

				std::getline(ref_file, line);
				std::stringstream iss(line);

				std::getline(iss,tmp,' ');
				// std::stof(nb_ref, nkp.x); //C++11
				nkp.x=atof(tmp.c_str());

				std::getline(iss,tmp,' ');
				nkp.y=atof(tmp.c_str());

				std::getline(iss,tmp,' ');
				nkp.scale=atof(tmp.c_str());

				std::getline(iss,tmp,' ');
				nkp.angle=atof(tmp.c_str());

				for(unsigned int k=0; k<(int) VecLength; k++)
				{
					std::getline(iss,tmp,' ');
					nkp.vec[k]=atof(tmp.c_str());
				}

				list.push_back(nkp);
			}
			std::vector< keypointslist > vkps(1,list);
			asift_keypoints akps(1,vkps);
			_keys[i]=akps;
		}
	}
	else 
	{
		std::cerr << "Unable to open the file :"<<ref_path;
		return false;
	}

	ref_file.close();
	return true;
}

//Debugging function
void ASIFT_matcher::print() const
{
	for(unsigned int i=0; i< _keys.size();i++)
	{
		cout<<"Ref size:"<<i<<" - size :"<<_keys[i].size()<<endl;
		for(unsigned int j=0; j<_keys[i].size();j++)
		{
			cout<<"	"<<j<<" - size :"<<_keys[i][j].size()<<endl;
			for(unsigned int k=0; k<_keys[i][j].size();k++)
			{
				cout<<"		"<<k<<" - size :"<<_keys[i][j][k].size()<<endl;
				float sx=0,sy=0,ss=0,sa=0, sv=0;
				for(unsigned int l=0; l<_keys[i][j][k].size();l++)
				{
					sx+=_keys[i][j][k][l].x;
					sy+=_keys[i][j][k][l].y;
					ss+=_keys[i][j][k][l].scale;
					sa+=_keys[i][j][k][l].angle;
					for(unsigned int v=0;v<VecLength;v++)
					{
						sv+=_keys[i][j][k][l].vec[v];
					}
				}
				cout<<" 		"<<sx<<"-"<<sy<<"-"<<ss<<"-"<<sa<<"-"<<sv<<endl;
			}
		}
	}
}

// unsigned int ASIFT_matcher::getNbMatch() const
// {
// 	unsigned int res = 0;
// 	for (unsigned int i=0;i<_num_matchings.size();i++)
// 	{
// 		res+=_num_matchings[i];
// 	} 
// 	return res;
// }