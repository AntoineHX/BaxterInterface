#include "ASIFT_matcher.hpp"

int main(int argc, char **argv)
{			
	if ((argc <2) || (argc > 5)) {
        std::cerr << " ******************************************************************************* " << std::endl
				  << " ***************************  ASIFT image matching  **************************** " << std::endl
				  << " ******************************************************************************* " << std::endl
				  << "Usage: " << argv[0] << " imgIn.png [Tilt number option] [Filter option] [Resize option] " << std::endl
									      << "- imgIn.png: input image. " << std::endl
									      << "- [Tilt number option: 1..(32+ ?)] : 7: Recommended / 1: no tilt. " << std::endl
									      << "- [Filter option: 0..3]. Standard deviation filter coeff (1-68%/2-95%/3-99%). 0: no filtering (default). " << std::endl 
										  << "- [Resize option: 0/1]. 1: input images resize to 800x600 (default). 0: no resize. " << std::endl 
   				  << " ******************************************************************************* " << std::endl
				  << " *********************  Jean-Michel Morel, Guoshen Yu, 2010 ******************** " << std::endl
				  << " ******************************************************************************* " << std::endl;
        return 1;
    }
	
    char* output_img = "./results/res.png";
    char* output_match = "./results/matching.txt";
    char* output_references = "./results/references.txt";

    //REFERENCE DEFINITION
    unsigned int nb_ref =2;
	std::string refData[] = {
      "references/000.png", 
      "references/train_image_001.jpg"};
    //REFERE?CE DEFINITION

    cimg_library::CImg<float> image;
	try
	{
		image =cimg_library::CImg<float>(argv[1]);
	}
	catch(cimg_library::CImgIOException)
	{
		std::cerr << "Unable to load image file " << argv[1] << std::endl;
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
    int w1=gray.width(), h1=gray.height();
    ipixels1.assign(gray.begin(), gray.end());

    // cout<<"Image size : "<<ipixels1.size()<<" - "<<image.spectrum()<<" - "<<w1<<" / "<<h1<<endl;
	///// Resize the images to area wS*hW in remaining the apsect-ratio	
	///// Resize if the resize flag is not set or if the flag is set unequal to 0
	float wS = IM_X;
	float hS = IM_Y;
	
	float zoom1=0;	
	int wS1=0, hS1=0;
	vector<float> ipixels1_zoom;	
		
	int flag_resize = 1;
	if (argc == 5)
	{	
		flag_resize = atoi(argv[4]);
	}
	
	if (flag_resize != 0)
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

    int tilt_ref = 7, tilt_input = 1;
    int nb_match;

    if(argc>2)
    {
    	tilt_ref = atoi(argv[2]);
    	tilt_input = atoi(argv[2]);
    }

    ASIFT_matcher matcher;
    matcher.resizeImg(flag_resize);
    // matcher.showInfo(false);

    time_t tstart, tend;
	tstart = time(0);

	for(unsigned int i=0; i<nb_ref;i++)
	{
		matcher.addReference(refData[i].c_str(), tilt_ref);
	}

	std::cout<<"Saving..."<<std::endl;
	matcher.saveReferences(output_references);
	// matcher.print();
	// std::cout<<"Loading..."<<std::endl;
	// matcher.loadReferences(output_references);
	// std::cout<<"Saving..."<<std::endl;
	// matcher.saveReferences("./results/references2.txt");
	// matcher.print();

	nb_match = matcher.match(ipixels1_zoom, wS1, hS1, tilt_input);

	if(argc>3 && atoi(argv[3])>0)
		matcher.distFilter(atoi(argv[3]));

	tend = time(0);

    vector<unsigned int> NbMatch = matcher.getNbMatchs();
    cout<<"Computation Time : "<<difftime(tend, tstart)<<" seconds." << endl;
    cout<<"Nb matching reference : "<<NbMatch.size()<<endl;
    for(unsigned int i=0; i<NbMatch.size(); i++)
    {
    	cout<<"  Ref "<<i<<" : "<<NbMatch[i]<<endl;
    }

	int x,y,cx,cy;
	unsigned int h,w;
	
	float *opixelsASIFT = new float[w1*h1];
	/////////////////////////////////////////////////////////////////// Copy image to output
	for(int j = 0; j < (int) h1; j++)
		for(int i = 0; i < (int) w1; i++)  opixelsASIFT[j*w1+i] = ipixels1[j*w1+i];	

	//////////////////////////////////////////////////////////////////// Draw matches
	int point_size = 2;
	for(unsigned int j=0; j<matcher.getMatch().size();j++)
	{
		matchingslist::iterator ptrH = matcher.getMatch()[j].begin();
		for(int i=0; i < (int) matcher.getMatch()[j].size(); i++, ptrH++)
		{		
			draw_square(opixelsASIFT, (int) (zoom1*ptrH->first.x), (int) (zoom1*ptrH->first.y), point_size, point_size, 255, w1, h1);
		}
	}

	//////////////////////////////////////////////////////////////////// Draw ROI	
	if(matcher.computeROI(x,y,h,w))
		draw_square(opixelsASIFT, zoom1*x, zoom1*y, zoom1*w, zoom1*h, 255, w1, h1);
	
	//////////////////////////////////////////////////////////////////// Draw Center
	if(matcher.computeCenter(cx,cy))
	{
		draw_square(opixelsASIFT, zoom1*(cx-6), zoom1*(cy-6), zoom1*12, zoom1*12, 160, w1, h1);
		draw_line(opixelsASIFT, zoom1*cx, zoom1*(cy-6), zoom1*cx, zoom1*(cy+6), 255, w1, h1);
		draw_line(opixelsASIFT, zoom1*(cx-6), zoom1*cy, zoom1*(cx+6), zoom1*cy, 255, w1, h1);
	}

	///////////////////////////////////////////////////////////////// Save imgOut	
	// write_png_f32(output_img, opixelsASIFT, w1, h1, 1);
	image.assign(opixelsASIFT,w1,h1);
	image.save(output_img);
	
	delete[] opixelsASIFT; /*memcheck*/
	
	////// Write the coordinates of the matched points (row1, col1, row2, col2)
	std::ofstream file(output_match);
	if (file.is_open())
	{		
		// Write the number of matchings and number of references in the first line
		file << nb_match <<"  "<<matcher.getNbRef() <<std::endl;

		for(unsigned int i =0; i<matcher.getNbRef();i++)
		{
			//Write reference indice before match
			file<<i<<std::endl;
			matchingslist matchings = matcher.getMatch()[i];
			matchingslist::iterator ptr = matchings.begin();
			for(int i=0; i < (int) matchings.size(); i++, ptr++)		
			{
				file << zoom1*ptr->first.x << "  " << zoom1*ptr->first.y << "  " <<  matcher.getZoomRef()[i]*ptr->second.x << 
				"  " <<  matcher.getZoomRef()[i]*ptr->second.y << std::endl;
			}
		}	
	}
	else 
	{
		std::cerr << "Unable to open the file matchings."; 
	}

	file.close();

    return 0;
}
