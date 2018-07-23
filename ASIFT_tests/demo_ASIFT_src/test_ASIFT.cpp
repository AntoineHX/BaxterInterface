#include "ASIFT_matcher.hpp"

int main(int argc, char **argv)
{			
	
    if ((argc != 8) && (argc != 9)) {
        std::cerr << " ******************************************************************************* " << std::endl
				  << " ***************************  ASIFT image matching  **************************** " << std::endl
				  << " ******************************************************************************* " << std::endl
				  << "Usage: " << argv[0] << " imgIn1.png imgIn2.png imgOutVert.png imgOutHori.png " << std::endl
										  << "           matchings.txt keys1.txt keys2.txt [Resize option: 0/1] " << std::endl
									      << "- imgIn1.png, imgIn2.png: input images (in PNG format). " << std::endl
										  << "- imgOutVert.png, imgOutHori.png: output images (vertical/horizontal concatenated, " << std::endl
				                          << "  in PNG format.) The detected matchings are connected by write lines." << std::endl
										  << "- matchings.txt: coordinates of matched points (col1, row1, col2, row2). " << std::endl
										  << "- keys1.txt keys2.txt: ASIFT keypoints of the two images." << std::endl
										  << "- [optional 0/1]. 1: input images resize to 800x600 (default). 0: no resize. " << std::endl 
   				  << " ******************************************************************************* " << std::endl
				  << " *********************  Jean-Michel Morel, Guoshen Yu, 2010 ******************** " << std::endl
				  << " ******************************************************************************* " << std::endl;
        return 1;
    }
	
    ASIFT_matcher matcher;
    matcher.addReference(argv[1], 1);
    matcher.match(argv[2], 1);

	
	
	// ///////////////// Output image containing line matches (the two images are concatenated one above the other)
	// int band_w = 20; // insert a black band of width band_w between the two images for better visibility
	
	// int wo =  MAX(w1,w2);
	// int ho = h1+h2+band_w;
	
	// float *opixelsASIFT = new float[wo*ho];
	
	// for(int j = 0; j < (int) ho; j++)
	// 	for(int i = 0; i < (int) wo; i++)  opixelsASIFT[j*wo+i] = 255;		
	
	// /////////////////////////////////////////////////////////////////// Copy both images to output
	// for(int j = 0; j < (int) h1; j++)
	// 	for(int i = 0; i < (int) w1; i++)  opixelsASIFT[j*wo+i] = ipixels1[j*w1+i];				
	
	// for(int j = 0; j < (int) h2; j++)
	// 	for(int i = 0; i < (int) (int)w2; i++)  opixelsASIFT[(h1 + band_w + j)*wo + i] = ipixels2[j*w2 + i];	
	
	// //////////////////////////////////////////////////////////////////// Draw matches
	// matchingslist::iterator ptr = matchings.begin();
	// for(int i=0; i < (int) matchings.size(); i++, ptr++)
	// {		
	// 	draw_line(opixelsASIFT, (int) (zoom1*ptr->first.x), (int) (zoom1*ptr->first.y), 
	// 			  (int) (zoom2*ptr->second.x), (int) (zoom2*ptr->second.y) + h1 + band_w, 255.0f, wo, ho);		
	// }
			
	// ///////////////////////////////////////////////////////////////// Save imgOut	
	// write_png_f32(argv[3], opixelsASIFT, wo, ho, 1);
	
	// delete[] opixelsASIFT; /*memcheck*/
	
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
