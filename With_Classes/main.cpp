/* Applied Video Analysis of Sequences (AVSA)
 *
 *	LAB2: Blob detection & classification
 *	Lab2.0: Sample Opencv project
 * 
 *
 * Authors: José M. Martínez (josem.martinez@uam.es), Paula Moral (paula.moral@uam.es), Juan C. San Miguel (juancarlos.sanmiguel@uam.es)
 */

//system libraries C/C++
#include <stdio.h>
#include <iostream>
#include <sstream>

//opencv libraries
#include <opencv2/opencv.hpp>
#include <opencv2/video/background_segm.hpp>

//Header ShowManyImages
#include "ShowManyImages.hpp"

//include for blob-related functions
#include "blobs.hpp"
#include "KalmanFilter_ipcv.hpp"

#include "opencv2/video/tracking.hpp"


//namespaces
using namespace cv; //avoid using 'cv' to declare OpenCV functions and variables (cv::Mat or Mat)
using namespace std;

#define MIN_WIDTH 10
#define MIN_HEIGHT 10


std::vector<Point> measurement_vect;
std::vector<Point> state_pre_vect;
std::vector<Point> state_post_vect;

//main function
int main(int argc, char ** argv) 
{
	//Point meas;
	Mat frame; // current Frame
	Mat fgmask; // foreground mask
	Mat fgmask_filtered;
	Point2f meas;
	
	int radius= 20;
	Mat tracking_Result; // to add lines and circles
	std::vector<cvBlob> bloblist; // list for blobs
	std::vector<cvBlob> bloblistFiltered; // list for blobs

	double t, acum_t; //variables for execution time
	int t_freq = getTickFrequency();
	int index;

		string dataset_path = "/home/sebasmos/AVSA2020datasets/AVSA_Lab3_datasets"; //SET THIS DIRECTORY according to your download
		string dataset_cat[1] = {"dataset_lab3"};
		string baseline_seq[1] = {"lab3.1/singleball.mp4"};
		string image_path = "";

		string project_root_path = "/home/sebasmos/AVSA2020results"; //SET THIS DIRECTORY according to your project
		string project_name = "kalman"; //SET THIS DIRECTORY according to your project
		string results_path = project_root_path+"/"+project_name+"/results";

		// create directory to store results
		/*
		string makedir_cmd = "mkdir "+project_root_path+"/"+project_name;
		system(makedir_cmd.c_str());
		makedir_cmd = "mkdir "+results_path;
		system(makedir_cmd.c_str());*/

		int NumCat = sizeof(dataset_cat)/sizeof(dataset_cat[0]); //number of categories (have faith ... it works! ;) ... each string size is 32 -at leat for the current values-)

		//Loop for all categories
		for (int c=0; c<NumCat; c++ )
		{
			// create directory to store results for category
			string makedir_cmd = "mkdir "+results_path + "/" + dataset_cat[c];
			system(makedir_cmd.c_str());

			int NumSeq = sizeof(baseline_seq)/sizeof(baseline_seq[0]);  //number of sequences per category ((have faith ... it works! ;) ... each string size is 32 -at leat for the current values-)

			//Loop for all sequence of each category
			for (int s=0; s<NumSeq; s++ )
			{
			VideoCapture cap;//reader to grab videoframes

			//Compose full path of images
			string inputvideo = dataset_path + "/" + dataset_cat[c] + "/" + baseline_seq[s] + image_path;
			cout << "Accessing sequence at " << inputvideo << endl;

			//open the video file to check if it exists
			cap.open(inputvideo);
			if (!cap.isOpened()) {
				cout << "Could not open video file " << inputvideo << endl;
			return -1;
			}

			// create directory to store results for sequence
			string makedir_cmd = "mkdir "+results_path + "/" + dataset_cat[c] + "/" + baseline_seq[s];
			system(makedir_cmd.c_str());

			//MOG2 approach

            //Ptr<BackgroundSubtractorMOG2>  pMOG2 = cv::createBackgroundSubtractorMOG2();
			
			// Set threshold
			//pMOG2->setVarThreshold(16);
			//pMOG2->setHistory(50);

			//main loop
			Mat img; // current Frame

			int it=1;
			acum_t=0;

			int tipo = 0;

			//----------------------------------------------------------------------------------------------------------
			// KALMAN INIT
			//----------------------------------------------------------------------------------------------------------
			
			Point state_post, state_pre;

			KalmanFilter_ipcv::Tracking avsa_bgs; //construct object of the bgs class
			
			//Tracking::Tracking();
			avsa_bgs.Kalman_initialization(0,0,0);

			//----------------------------------------------------------------------------------------------------------
			// END KALMAN INIT
			//----------------------------------------------------------------------------------------------------------
			
			
			for (;;) {
				//get frame
				cap >> img;

				//check if we achieved the end of the file (e.g. img.data is empty)
				if (!img.data)
					break;

				//Time measurement
				t = (double)getTickCount();

				//apply algs: Get current img & store in frame
				img.copyTo(frame);				
				
				// Compute fgmask
				double learningrate=0.001;
				//pMOG2->apply(frame, fgmask, learningrate); 
				//----------------------------------------------------------------------------------------------------------
				// APPLY FILTER
				//----------------------------------------------------------------------------------------------------------
				// https://docs.opencv.org/3.4/d4/d86/group__imgproc__filter.html#gac342a1bb6eabf6f55c803b09268e36dc 
				//Mat kernel = getStructuringElement(MORPH_RECT, Size(3,3)); // https://docs.opencv.org/3.4/df/d5e/samples_2cpp_2tutorial_code_2ImgProc_2Morphology_1_8cpp-example.html#a16 
				
                //morphologyEx(fgmask, fgmask_filtered, MORPH_OPEN , kernel);
				
				//----------------------------------------------------------------------------------------------------------
				// END OF FILTER
				//----------------------------------------------------------------------------------------------------------

				int connectivity = 8; // 4 or 8

				// Extract the blobs in fgmask
				extractBlobs(fgmask, bloblist, connectivity);
				//extractBlobs(fgmask_filtered, bloblist, connectivity);

				//cout << "Old number of blobs : "<< bloblist.size() << endl;			
				
				removeSmallBlobs(bloblist, bloblistFiltered, MIN_WIDTH, MIN_HEIGHT);
				//cout << "Filtered Number of blobs : " << bloblistFiltered.size() << endl;
				//cout << "Num blobs removed=" << bloblist.size()- bloblistFiltered.size() << endl;

				//cout << "Filtered number of blobs : "<< bloblistFiltered.size() << endl;			
				//----------------------------------------------------------------------------------------------------------
				// DETECT THE CENTER OF THE BIGGEST BLOB
				//----------------------------------------------------------------------------------------------------------
				int val = biggest_blob(bloblistFiltered);				
				//------------------------END OF BIGGEST BLOB DETECTION-----------------------------------------------------

				frame.copyTo(tracking_Result);

				// Make prediction when no blob is detected, meaning that if bloblist is empty, no correction is needed.
				if (bloblistFiltered.empty()){

				}else{
					//cout << "index of biggest blob index: " <<  val << endl;
				
				}			
		
				//Time measurement
				t = (double)getTickCount() - t;
        		        //if (_CONSOLE_DEBUG) cout << "proc. time = " << 1000*t/t_freq << " milliseconds."<< endl;
				acum_t=+t;

				//SHOW RESULTS
				//get the frame number and write it on the current frame

				string title= project_name + " | Frame - FgM - Tracking | Blobs - Classes - Stat Classes | BlobsFil - ClassesFil - Stat ClassesFil | ("+dataset_cat[c] + "/" + baseline_seq[s] + ")";

				//ShowManyImages(title, 6, frame, fgmask, fgmask,
						//fgmask, fgmask, fgmask);
				//ShowManyImages(title, 6, frame, fgmask, fgmask,
				//		paintBlobImage(frame,bloblistFiltered, true), paintBlobImage(frame,bloblistFiltered, true), paintBlobImage(frame,bloblistFiltered, true));
				
				imshow("frame: ",frame);
				//imshow("result: ",Draw_tracking(tracking_Result,  meas,state_pre,state_post));
				//exit if ESC key is pressed
				if(waitKey(300) == 27) break;
				//---------------------------------------------------------
				char key = waitKey(5); // waits to display frame
				if (key == 'q')
					break;
				//---------------------------------------------------------
				it++;
			} //main loop

	cout << it-1 << "frames processed in " << 1000*acum_t/t_freq << " milliseconds."<< endl;


	//release all resources

	cap.release();
	destroyAllWindows();
	waitKey(0); // (should stop till any key is pressed .. doesn't!!!!!)
}
}
return 0;
}


/**
 * REFERENCES
 * https://www.myzhar.com/blog/tutorials/tutorial-opencv-ball-tracker-using-kalman-filter/ 
 * https://stackoverflow.com/questions/18403918/opencv-kalman-filter-prediction-without-new-observtion 
 * 
 * 
 */
