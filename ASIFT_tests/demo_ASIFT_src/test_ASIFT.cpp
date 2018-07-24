#include "ASIFT_matcher.hpp"

int main(int argc, char **argv)
{			
	if ((argc !=2) && (argc != 3) && (argc != 4)) {
        std::cerr << " ******************************************************************************* " << std::endl
				  << " ***************************  ASIFT image matching  **************************** " << std::endl
				  << " ******************************************************************************* " << std::endl
				  << "Usage: " << argv[0] << " imgIn.png [Tilt number option] [Resize option: 0/1] " << std::endl
									      << "- imgIn.png: input image (in PNG format). " << std::endl
									      << "- [Tilt number option: 1..n]. " << std::endl 
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
	if (argc == 4)
	{	
		flag_resize = atoi(argv[3]);
	}
	
	if ((argc==2) || (argc == 3) || (flag_resize != 0))
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
    matcher.addReference(refData[0].c_str(), 7);
    matcher.addReference(refData[1].c_str(), 7);
    // matcher.print();
    // matcher.match(refData[3].c_str(), 4);
    if(argc>2)
    	matcher.match(ipixels1_zoom, wS1, hS1, atoi(argv[2]));
	else
		matcher.match(ipixels1_zoom, wS1, hS1);

    vector<unsigned int> NbMatch = matcher.getNbMatchs();
    cout<<"Nb match : "<<NbMatch.size()<<endl;
    for(unsigned int i=0; i<NbMatch.size(); i++)
    {
    	cout<<"	"<<NbMatch[i]<<endl;
    }
	
	int x =0,y=0;
	unsigned int h=1,w=1;
	matcher.computeROI(x,y,h,w, zoom1); //ATTENTION ne pas oublie le resize -> besoin du coef zoom
	
	
	float *opixelsASIFT = new float[w1*h1];
	/////////////////////////////////////////////////////////////////// Copy image to output
	for(int j = 0; j < (int) h1; j++)
		for(int i = 0; i < (int) w1; i++)  opixelsASIFT[j*w1+i] = ipixels1[j*w1+i];				
	//////////////////////////////////////////////////////////////////// Draw ROI	
	draw_square(opixelsASIFT, x, y, w, h, 255, w1, h1);
	///////////////////////////////////////////////////////////////// Save imgOut	
	write_png_f32("./results/res.png", opixelsASIFT, w1, h1, 1);
	
	delete[] opixelsASIFT; /*memcheck*/
	
	// /////////// Output image containing line matches (the two images are concatenated one aside the other)
	// int woH =  w1+w2+band_w;
	// int hoH = MAX(h1,h2);
	
	// float *opixelsASIFT_H = new float[woH*hoH];
	
	// for(int j = 0; j < (int) hoH; j++)
	// 	for(int i = 0; i < (int) woH; i++)  opixelsASIFT_H[j*woH+i] = 255;
	
	// /////////////////////////////////////////////////////////////////// Copy both images to output
	// for(int j = 0; j < (int) h1; j++)
	// 	for(int i = 0; i < (int) w1; i++)  opixelsASIFT_H[j*woH+i] = ipixels1[j*w1+i];				
	
	// for(int j = 0; j < (int) h2; j++)
	// 	for(int i = 0; i < (int) w2; i++)  opixelsASIFT_H[j*woH + w1 + band_w + i] = ipixels2[j*w2 + i];	

	
	// //////////////////////////////////////////////////////////////////// Draw matches
	// matchingslist::iterator ptrH = matchings.begin();
	// for(int i=0; i < (int) matchings.size(); i++, ptrH++)
	// {		
	// 	draw_line(opixelsASIFT_H, (int) (zoom1*ptrH->first.x), (int) (zoom1*ptrH->first.y), 
	// 			  (int) (zoom2*ptrH->second.x) + w1 + band_w, (int) (zoom2*ptrH->second.y), 255.0f, woH, hoH);		
	// }
	
	// ///////////////////////////////////////////////////////////////// Save imgOut	
	// write_png_f32(argv[4], opixelsASIFT_H, woH, hoH, 1);
	
	// delete[] opixelsASIFT_H; /*memcheck*/
	
	// ////// Write the coordinates of the matched points (row1, col1, row2, col2) to the file argv[5]
	// std::ofstream file(argv[5]);
	// if (file.is_open())
	// {		
	// 	// Write the number of matchings in the first line
	// 	file << num_matchings << std::endl;
		
	// 	matchingslist::iterator ptr = matchings.begin();
	// 	for(int i=0; i < (int) matchings.size(); i++, ptr++)		
	// 	{
	// 		file << zoom1*ptr->first.x << "  " << zoom1*ptr->first.y << "  " <<  zoom2*ptr->second.x << 
	// 		"  " <<  zoom2*ptr->second.y << std::endl;
	// 	}		
	// }
	// else 
	// {
	// 	std::cerr << "Unable to open the file matchings."; 
	// }

	// file.close();


	
	// // Write all the keypoints (row, col, scale, orientation, desciptor (128 integers)) to 
	// // the file argv[6] (so that the users can match the keypoints with their own matching algorithm if they wish to)
	// // keypoints in the 1st image
	// std::ofstream file_key1(argv[6]);
	// if (file_key1.is_open())
	// {
	// 	// Follow the same convention of David Lowe: 
	// 	// the first line contains the number of keypoints and the length of the desciptors (128)
	// 	file_key1 << num_keys1 << "  " << VecLength << "  " << std::endl;
	// 	for (int tt = 0; tt < (int) keys1.size(); tt++)
	// 	{
	// 		for (int rr = 0; rr < (int) keys1[tt].size(); rr++)
	// 		{
	// 			keypointslist::iterator ptr = keys1[tt][rr].begin();
	// 			for(int i=0; i < (int) keys1[tt][rr].size(); i++, ptr++)	
	// 			{
	// 				file_key1 << zoom1*ptr->x << "  " << zoom1*ptr->y << "  " << zoom1*ptr->scale << "  " << ptr->angle;
					
	// 				for (int ii = 0; ii < (int) VecLength; ii++)
	// 				{
	// 					file_key1 << "  " << ptr->vec[ii];
	// 				}
					
	// 				file_key1 << std::endl;
	// 			}
	// 		}	
	// 	}
	// }
	// else 
	// {
	// 	std::cerr << "Unable to open the file keys1."; 
	// }

	// file_key1.close();
	
	// ////// keypoints in the 2nd image
	// std::ofstream file_key2(argv[7]);
	// if (file_key2.is_open())
	// {
	// 	// Follow the same convention of David Lowe: 
	// 	// the first line contains the number of keypoints and the length of the desciptors (128)
	// 	file_key2 << num_keys2 << "  " << VecLength << "  " << std::endl;
	// 	for (int tt = 0; tt < (int) keys2.size(); tt++)
	// 	{
	// 		for (int rr = 0; rr < (int) keys2[tt].size(); rr++)
	// 		{
	// 			keypointslist::iterator ptr = keys2[tt][rr].begin();
	// 			for(int i=0; i < (int) keys2[tt][rr].size(); i++, ptr++)	
	// 			{
	// 				file_key2 << zoom2*ptr->x << "  " << zoom2*ptr->y << "  " << zoom2*ptr->scale << "  " << ptr->angle;
					
	// 				for (int ii = 0; ii < (int) VecLength; ii++)
	// 				{
	// 					file_key2 << "  " << ptr->vec[ii];
	// 				}					
	// 				file_key2 << std::endl;
	// 			}
	// 		}	
	// 	}
	// }
	// else 
	// {
	// 	std::cerr << "Unable to open the file keys2."; 
	// }
	// file_key2.close();
	
    return 0;
}
