/* Applied Video Analysis of Sequences (AVSA)
 *
 *	LAB2: Blob detection & classification
 *	Lab2.0: Sample Opencv project
 *
 *
 * Authors: José M. Martínez (josem.martinez@uam.es), Paula Moral (paula.moral@uam.es), Juan C. San Miguel (juancarlos.sanmiguel@uam.es)
 */

#include "blobs.hpp"

/**
 *	Draws blobs with different rectangles on the image 'frame'. All the input arguments must be
 *  initialized when using this function.
 *
 * \param frame Input image
 * \param pBlobList List to store the blobs found
 * \param labelled - true write label and color bb, false does not wirite label nor color bb
 *
 * \return Image containing the draw blobs. If no blobs have to be painted
 *  or arguments are wrong, the function returns a copy of the original "frame".
 *
 */
using namespace cv;
using namespace std;


Mat Draw_tracking(cv::Mat result, std::vector<Point> &measurement_vect){
			
	Scalar red = Scalar (0, 0, 255);
	Scalar blue = Scalar (255, 0, 0);
	Scalar green = Scalar (255, 255, 0);

	Mat final_img;
	result.copyTo(final_img);

	int radius= 20;
	//cout << "drawing: "<< measurement_vect <<endl;
	
					for (int i = 0; i < measurement_vect.size () - 1; i++) {
						//cout << "drawing: "<< measurement_vect[i] <<endl;

						/*
						
				        circle (result, measurement_vect[i], 5, red, 2);
						line (result, measurement_vect[i], measurement_vect[i+1], red, 1);
						putText(result, "measurement_vect: ", 
									cv::Point(10, 10),
									FONT_HERSHEY_SIMPLEX, 
									0.5,
									red);
						*/
					}

	return final_img;
}

 Mat paintBlobImage(cv::Mat frame, std::vector<cvBlob> bloblist, bool labelled)
{
	cv::Mat blobImage;
	//check input conditions and return original if any is not satisfied
	//...
	frame.copyTo(blobImage);
	//required variables to paint
	//...
	//paint each blob of the list
	for(int i = 0; i < bloblist.size(); i++)
	{
		cvBlob blob = bloblist[i]; //get ith blob
		//...
		Scalar color;
		std::string label="";
		switch(blob.label){
		case PERSON:
			color = Scalar(255,0,0); // RED
			label="PERSON";
			break;
		case CAR:
					color = Scalar(0,255,0); // GREEN
					label="CAR";
					break;
		case OBJECT:
					color = Scalar(0,0,255); // BLUE
					label="OBJECT";
					break;
		default:
			color = Scalar(255, 255, 255); // UNKNOWN
			label="UNKOWN";
		}

		Point p1 = Point(blob.x, blob.y);
		Point p2 = Point(blob.x+blob.w, blob.y+blob.h);
		// Rectangle: Draws a simple, thick, or filled up-right rectangle using p1 and p2, two opossite corners
		//		  input, 	p1, p2,  color, thickness, line type, shift
		rectangle(blobImage, p1, p2, color, 1, 8, 0); 
		if (labelled)
			{
			rectangle(blobImage, p1, p2, color, 1, 8, 0);
			putText(blobImage, label, p1, FONT_HERSHEY_SIMPLEX, 0.5, color);
			}
			else
				rectangle(blobImage, p1, p2, Scalar(255, 255, 255), 1, 8, 0);
	}

	//destroy all resources (if required)
	//...

	//return the image to show
	return blobImage;
}


/**
 *	Blob extraction from 1-channel image (binary). The extraction is performed based
 *	on the analysis of the connected components. All the input arguments must be 
 *  initialized when using this function.
 *
 * \param fgmask Foreground/Background segmentation mask (1-channel binary image) 
 * \param bloblist List with found blobs
 *
 * \return Operation code (negative if not succesfull operation) 
 */

int extractBlobs(cv::Mat fgmask, std::vector<cvBlob> &bloblist, int connectivity)
{	
	//check input conditions and return -1 if any is not satisfied
	
    if((connectivity!=8) & (connectivity!=4))
	{
		cout<<"Bad connectivity input!!"<<endl;
		return -1;
	}

	Mat Output; // image to be updated each time a blob is detected (blob cleared)
	//fgmask.convertTo(Output,CV_32SC1);
	fgmask.copyTo(Output);
	
	//clear blob list (to fill with this function)
	bloblist.clear();
			
	// void creation of a unqie blob in the center
	cvBlob blob;
	//blob=initBlob(1, fgmask.cols/2, fgmask.rows/2, fgmask.cols/4, fgmask.rows/4);
	//bloblist.push_back(blob);

	int IDcount = 0;

	for(int i=0; i< Output.rows;++i){
		for(int j=0; j<Output.cols;++j){
				if(Output.at<uchar>(i,j)==255){
				        Rect Selection;
					IDcount +=1;
					// Floodfill reune las imagenes completas, documentacion dice que: seedpoint -> starting point
					// Entonces Point detecta un punto del fgmask u floodFill le coloca el valor de IDcount
					cv::floodFill(Output,Point(j,i),IDcount, &Selection,0,0,connectivity);
					// Add ID+BLOB to blob
					blob=initBlob(IDcount, Selection.x, Selection.y, Selection.width, Selection.height);
					// Store BLOB on bloblist
					bloblist.push_back(blob);			
				}
		}
	}
	//return OK code
	return 1;
}


int removeSmallBlobs(std::vector<cvBlob> bloblist_in, std::vector<cvBlob> &bloblist_out, int min_width, int min_height)
{
	//check input conditions and return -1 if any is not satisfied
	//clear blob list (to fill with this function)
	bloblist_out.clear();

	for(int i = 0; i < bloblist_in.size(); i++)
	{
		cvBlob blob_in = bloblist_in[i]; //get ith blob

		if ((blob_in.w > min_width) | (blob_in.h > min_height )){
		bloblist_out.push_back(blob_in); // void implementation (does not remove)
		}
	}
	//destroy all resources
	//...
	
	//return OK code
	return 1;
}

 /**
  *	Blob classification between the available classes in 'Blob.hpp' (see CLASS typedef). All the input arguments must be
  *  initialized when using this function.
  *
  * \param frame Input image
  * \param fgmask Foreground/Background segmentation mask (1-channel binary image)
  * \param bloblist List with found blobs
  *
  * \return Operation code (negative if not succesfull operation)
  */

 // ASPECT RATIO MODELS
#define MEAN_PERSON 0.3950
#define STD_PERSON 0.1887

#define MEAN_CAR 1.4736
#define STD_CAR 0.2329

#define MEAN_OBJECT 1.2111
#define STD_OBJECT 0.4470

// end ASPECT RATIO MODELS

// distances
float ED(float val1, float val2)
{
	return sqrt(pow(val1-val2,2));
}

float WED(float val1, float val2, float std)
{
	return sqrt(pow(val1-val2,2)/pow(std,2));
}
//end distances
 int classifyBlobs(std::vector<cvBlob> &bloblist)
 {
 	//check input conditions and return -1 if any is not satisfied
 	//...

 	//required variables for classification
 	//...
    double feat_i = 0;
 	//classify each blob of the list
 	for(int i = 0; i < bloblist.size(); i++)
 	{
 		cvBlob blob = bloblist[i]; //get i-th blob
 		// is the ratio of its width to its height 
 		feat_i = (double)blob.w/blob.h;
 		
 		if ( (WED(feat_i, MEAN_PERSON, STD_PERSON) < STD_PERSON) || (ED(feat_i, MEAN_PERSON) < STD_PERSON )){
 		         blob.label  = CLASS(PERSON);
 		         //cout << "PERSON!!"<<CLASS(PERSON)<<endl;
 		         bloblist[i] = blob;
 		         
 		}else if ( ( WED(feat_i, MEAN_CAR, STD_CAR) < STD_CAR) || (ED(feat_i, MEAN_CAR) < STD_CAR  )){
 		         blob.label  = CLASS(CAR);
 		         //cout << "CAR!!" <<CLASS(CAR)<<endl;
 		         bloblist[i] = blob;
 		         
 		}else if ( ( WED(feat_i, MEAN_OBJECT, STD_OBJECT) < STD_OBJECT) || (ED(feat_i, MEAN_OBJECT) < STD_OBJECT  )){
 		
 		         blob.label  = CLASS(OBJECT);
 		         //cout << "OBJECT!!" <<CLASS(OBJECT)<<endl;
 		         bloblist[i] = blob;
 		}
 	
 	}
 	

 	//destroy all resources
 	//...

 	//return OK code
 	return 1;
 }

//stationary blob extraction function
 /**
  *	Stationary FG detection
  *
  * \param fgmask Foreground/Background segmentation mask (1-channel binary image)
  * \param fgmask_history Foreground history counter image (1-channel integer image)
  * \param sfgmask Foreground/Background segmentation mask (1-channel binary image)
  *
  * \return Operation code (negative if not succesfull operation)
  *
  *
  * Based on: Stationary foreground detection for video-surveillance based on foreground and motion history images, D.Ortego, J.C.SanMiguel, AVSS2013
  *
  */

#define FPS 25 //check in video - not really critical
#define SECS_STATIONARY 0 // to set
#define I_COST 0 // to set // increment cost for stationarity detection
#define D_COST 0 // to set // decrement cost for stationarity detection
#define STAT_TH 0.0 // to set

 int extractStationaryFG (Mat fgmask, Mat &fgmask_history, Mat &sfgmask)
 {

	 int numframes4static=(int)(FPS*SECS_STATIONARY);

	 // update fgmask_counter
	 for (int i=0; i<fgmask.rows;i++)
		 for(int j=0; j<fgmask.cols;j++)
		 {
			// ...
			 fgmask_history.at<float>(i,j) = 0; // void implementation (no history)
		 }//

	// update sfgmask
	for (int i=0; i<fgmask.rows;i++)
		 for(int j=0; j<fgmask.cols;j++)
			 {
				 sfgmask.at<uchar>(i,j)=0;// void implementation (no stationary fg)
			 }
 return 1;
 }
 
 int biggest_blob(std::vector<cvBlob> &bloblist) 
 	{
 	int biggest_blob = 0;
 	for(int i = 0; i < bloblist.size(); i++)
 	        {
 		cvBlob blob = bloblist[i]; //get i-th blob
 		if ((blob.w*blob.h > biggest_blob)){
		         biggest_blob = i;			                        
		}
 		
 	}                        
 	return biggest_blob;
 
 
 }



