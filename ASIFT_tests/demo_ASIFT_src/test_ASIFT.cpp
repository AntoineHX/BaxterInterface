#include "ASIFT_matcher.hpp"

int main(int argc, char **argv)
{			
	if ((argc <2) || (argc > 5)) {
        std::cerr << " ******************************************************************************* " << std::endl
				  << " ***************************  ASIFT image matching  **************************** " << std::endl
				  << " ******************************************************************************* " << std::endl
				  << "Usage: " << argv[0] << " imgIn.png [Tilt number option] [Filter option] [Resize option] " << std::endl
									      << "- imgIn.png: input image (in PNG format). " << std::endl
									      << "- [Tilt number option: 1..(32+ ?)] : 7: Recommended / 1: no tilt. " << std::endl
									      << "- [Filter option: 0..3]. Standard deviation filter coeff (1-68%/2-95%/3-99%). 0: no filtering. " << std::endl 
										  << "- [Resize option: 0/1]. 1: input images resize to 800x600 (default). 0: no resize. " << std::endl 
   				  << " ******************************************************************************* " << std::endl
				  << " *********************  Jean-Michel Morel, Guoshen Yu, 2010 ******************** " << std::endl
				  << " ******************************************************************************* " << std::endl;
        return 1;
    }
	
	//////////////////////////////////////////////// Input
	float * iarr1;
    size_t w1, h1;
    if (NULL == (iarr1 = read_png_f32_gray(argv[1], &w1, &h1))) {
        std::cerr << "Unable to load image file " << argv[1] << std::endl;
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
		
	int flag_resize = 1;
	if (argc == 5)
	{	
		flag_resize = atoi(argv[4]);
	}
	
	if (flag_resize != 0)
	{
		cout << "WARNING: The input images is resized to " << wS << "x" << hS << " for ASIFT. " << endl 
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


	std::string refData[] = {
      "book_training/train_image_000.png", 
      "book_training/train_image_001.png"};

    ASIFT_matcher matcher;
    matcher.setResizeImg(flag_resize);

    time_t tstart, tend;
	tstart = time(0);

    // matcher.print();
    // matcher.match(refData[3].c_str(), 4);
    if(argc>2)
    {
		matcher.addReference(refData[0].c_str(), atoi(argv[2]));
		matcher.addReference(refData[1].c_str(), atoi(argv[2]));
		matcher.match(ipixels1_zoom, wS1, hS1, atoi(argv[2]));
    }
	else
	{
		matcher.addReference(refData[0].c_str(), 7);
    	matcher.addReference(refData[1].c_str(), 7);
		matcher.match(ipixels1_zoom, wS1, hS1);
	}

	if(argc>3 && atoi(argv[3])>0)
		matcher.distFilter(atoi(argv[3]));

	tend = time(0);

    vector<unsigned int> NbMatch = matcher.getNbMatchs();
    cout<<"Computation Time : "<<difftime(tend, tstart)<<" seconds." << endl;
    cout<<"Nb matching reference : "<<NbMatch.size()<<endl;
    for(unsigned int i=0; i<NbMatch.size(); i++)
    {
    	cout<<"	"<<NbMatch[i]<<endl;
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
	write_png_f32("./results/res.png", opixelsASIFT, w1, h1, 1);
	
	delete[] opixelsASIFT; /*memcheck*/
	
    return 0;
}
