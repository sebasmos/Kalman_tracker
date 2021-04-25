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

#include "opencv2/video/tracking.hpp"


//namespaces
using namespace cv; //avoid using 'cv' to declare OpenCV functions and variables (cv::Mat or Mat)
using namespace std;

#define MIN_WIDTH 10
#define MIN_HEIGHT 10

cv::KalmanFilter KF;
Mat_<float> measurement(2,1); 
int type = 0;

void Kalman_initialization (int type, float x, float y)
{
	if (type == 0) { // velocity
		KF.init(4, 2, 0);
		// x,y - biggest blob center
		measurement = Mat_<float>::zeros(2,1);
		/*
		measurement.at<float>(0, 0) = x;
		measurement.at<float>(0, 0) = y;
		*/
		/*
		// TO CHECK
		KF.statePre.setTo(0);
		KF.statePre.at<float>(0, 0) = x;
		KF.statePre.at<float>(1, 0) = y;
		// TO CHECK
		KF.statePost.setTo(0);
		KF.statePost.at<float>(0, 0) = x;
		KF.statePost.at<float>(1, 0) = y; 

		*/
		//----------------------------------------------------------------------------------------------------------
		// INITIALIZE KF MATRIXES
		//----------------------------------------------------------------------------------------------------------
 		// A MATRIX - TRANSITION MATRIX
		KF.transitionMatrix = (Mat_<float>(4,4) <<  1, 1, 0, 0, 
													0, 1, 0, 0,
													0, 0, 1, 1, 
													0, 0, 0, 1); 
		// Q MATRIX - COVARIANCE MATRIX
		KF.processNoiseCov = (Mat_<float>(4,4) <<
			25, 0, 0, 0,
			0, 10, 0, 0,
			0, 0, 25, 0,
			0, 0, 0, 10); 
		// H MATRIX - MEASUREMENT MATRIX
		KF.measurementMatrix = (Mat_<float>(2,4) << 1, 0, 0, 0, 
													0, 0, 1, 0); 
		// R MATRIX - MEASUREMENT NOISE COVARIANCE MATRIX
		KF.measurementNoiseCov = (Mat_<float>(2,2) <<
												25, 0,
												0, 25); 
		// PROCESS NOISE COVARIANCE P 
		KF.errorCovPost = (Mat_<float>(4,4) <<
												100000,0,0,0,
												0,100000,0,0,
												0,0,100000,0,
												0,0,0,100000
												);// P Covariance 
	}

	else if (type == 1){//acceleration
		KF.init (6,2,0);
		// x,y - biggest blob center
		measurement = Mat_<float>::zeros(2,1); // Position x,y
		measurement.at<float>(0, 0) = x;
		measurement.at<float>(0, 0) = y;
		// TO CHECK
		KF.statePre.setTo(0);
		KF.statePre.at<float>(0, 0) = x;
		KF.statePre.at<float>(1, 0) = y;
		// TO CHECK
		KF.statePost.setTo(0);
		KF.statePost.at<float>(0, 0) = x;
		KF.statePost.at<float>(1, 0) = y;
		 
		KF.transitionMatrix = (Mat_<float>(6,6) <<
			1, 0, 1, 0, 0.5, 0,
			0, 1, 0, 1, 0, 0.5,
			0, 0, 1, 0, 1, 0,
			0, 0, 0, 1, 0, 1,
			0, 0, 0, 0, 1, 0,
			0, 0, 0, 0, 0, 1); // A matrix, transition matrix 
		KF.processNoiseCov = (Mat_<float>(6,6) <<
				25, 0, 0, 0, 0 , 0,
				0, 25, 0, 0, 0, 0,
				0, 0, 10, 0, 0, 0,
				0, 0, 0, 10, 0, 0,
				0, 0, 0, 0, 1, 0,
				0, 0, 0, 0, 0, 1); // Q matrix, noise covariance matrix
		KF.measurementMatrix = (Mat_<float>(2,6) <<
			1, 0, 0, 0, 0, 0,
			0, 1, 0, 0, 0, 0); // H matrix, measurement matrix
		KF.measurementNoiseCov = (Mat_<float>(2,2) <<
			25, 0,
			0, 25); // R matrix, measurement noise covariance matrix
		KF.errorCovPost = (Mat_<float>(6,6) <<
												100000,0,0,0,0,0,
												0,100000,0,0,0,0,
												0,0,100000,0,0,0,
												0,0,0,100000,0,0,
												0,0,0,0,100000,0,
												0,0,0,0,0,100000
												);// P Covariance 
		
	}
    //setIdentity(KF.errorCovPost, Scalar::all(1)); // P : posteriori error estimate covariance matrix (P(k)): P(k)=(I-K(k)*H)*P'(k)

}
	
Point kalmanPredict() 
{
    Mat prediction = KF.predict();
	//cout <<"pred: "<<prediction<<endl;
    Point predictPt(prediction.at<float>(0),prediction.at<float>(2)); // TO CHECK: indexes for vx,vy, and x,y
    return predictPt;
}

Point kalmanCorrect(float x, float y)
{
    measurement(0) = x;
    measurement(1) = y;
    Mat estimated = KF.correct(measurement);
    Point statePt(estimated.at<float>(0),estimated.at<float>(2));
    return statePt;
}



//main function
int main(int argc, char ** argv) 
{
	Mat frame; // current Frame
	Mat fgmask; // foreground mask
	Mat fgmask_filtered;
	std::vector<cvBlob> bloblist; // list for blobs
	std::vector<cvBlob> bloblistFiltered; // list for blobs

	// STATIONARY BLOBS
	Mat fgmask_history; // STATIONARY foreground mask
	Mat sfgmask; // STATIONARY foreground mask
	std::vector<cvBlob> sbloblist; // list for STATIONARY blobs     
	std::vector<cvBlob> sbloblistFiltered; // list for STATIONARY blobs

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
		string makedir_cmd = "mkdir "+project_root_path+"/"+project_name;
		system(makedir_cmd.c_str());
		makedir_cmd = "mkdir "+results_path;
		system(makedir_cmd.c_str());

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

            Ptr<BackgroundSubtractorMOG2>  pMOG2 = cv::createBackgroundSubtractorMOG2();
			
			// Set threshold
			pMOG2->setVarThreshold(16);
			pMOG2->setHistory(50);

			//main loop
			Mat img; // current Frame

			int it=1;
			acum_t=0;

			//----------------------------------------------------------------------------------------------------------
			// KALMAN INIT
			//----------------------------------------------------------------------------------------------------------
			
			Point state_post, state_pre;
			Kalman_initialization(0,0,0);

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
				pMOG2->apply(frame, fgmask, learningrate); 
				//----------------------------------------------------------------------------------------------------------
				// APPLY FILTER
				//----------------------------------------------------------------------------------------------------------
				// https://docs.opencv.org/3.4/d4/d86/group__imgproc__filter.html#gac342a1bb6eabf6f55c803b09268e36dc 
				Mat kernel = getStructuringElement(MORPH_RECT, Size(3,3)); // https://docs.opencv.org/3.4/df/d5e/samples_2cpp_2tutorial_code_2ImgProc_2Morphology_1_8cpp-example.html#a16 
				
                morphologyEx(fgmask, fgmask_filtered, MORPH_OPEN , kernel);
				
				//----------------------------------------------------------------------------------------------------------
				// END OF FILTER
				//----------------------------------------------------------------------------------------------------------

				int connectivity = 8; // 4 or 8

				// Extract the blobs in fgmask

				extractBlobs(fgmask_filtered, bloblist, connectivity);

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
				float meas = 0;
								// Plotting
				Scalar red = Scalar (0, 0, 255);
				Scalar blue = Scalar (255, 0, 0);
				Scalar green = Scalar (255, 255, 0);
				int radius= 20;
				Mat result;
				frame.copyTo (result);

				// Make prediction when no blob is detected, meaning that if bloblist is empty, no correction is needed.
				if (bloblistFiltered.empty()){
					state_pre = kalmanPredict();
					cout << "kalman prediction (state_pre): " << state_pre.x << " " << state_pre.y << endl;
					
					cout << "No blobs detected: "<< bloblistFiltered.size() << endl;

					//------------------------DRAWING------------------------------------------------------

					vector<Point> state_pre_vect;
					state_pre_vect.push_back(state_pre);
					for (Point point : state_pre_vect) {
						circle (result,point, 5, blue, 2);
						line (result, Point (point.x - radius / 2, point.y), Point (point.x + radius / 2, point.y), blue, 1);
						putText(result, "state_pre_vect: ", 
									cv::Point(10, 40),
									FONT_HERSHEY_SIMPLEX, 
									0.5,
									blue);
					}
				}else{
					//cout << "index of biggest blob index: " <<  val << endl;
					Point meas = Point(bloblistFiltered[val].x + bloblistFiltered[val].w/2,  bloblistFiltered[val].y + bloblistFiltered[val].h/2); 
					int x = bloblistFiltered[val].x + bloblistFiltered[val].w/2;
					int y = bloblistFiltered[val].y + bloblistFiltered[val].h/2;
					
					cout<< "Measurement: " << x << " " <<y <<endl;
					// PREDICTION
					state_pre = kalmanPredict();
					cout << "kalman prediction (state_pre): " << state_pre.x << " " << state_pre.y << endl;
					measurement(0) = x;
					measurement(1) = y;
					// CORRECTION
					state_post = kalmanCorrect(x, y);
					cout << "kalman corrected state (state_post): " << state_post.x << " " << state_post.y << endl;
					//Mat estimated = KF.correct(measurement);
					//Point statePt(estimated.at<float>(0),estimated.at<float>(1));
					//cout << "kalman corrected state: " << statePt << endl;

					//------------------------DRAWING------------------------------------------------------
					vector<Point> measurement_vect;
					vector<Point> state_pre_vect;
					vector<Point> state_post_vect;

					measurement_vect.push_back(meas);
					state_pre_vect.push_back(state_pre);
					state_post_vect.push_back(state_post);
										//line (result, state_pre, state_pre, red, 1);
					for (Point point : measurement_vect) {
						circle (result,point, 5, red, 2);
						line (result, Point (point.x - radius / 2, point.y), Point (point.x + radius / 2, point.y), red, 1);
						putText(result, "measurement_vect: ", 
									cv::Point(10, 10),
									FONT_HERSHEY_SIMPLEX, 
									0.5,
									red);
					}
					for (Point point : state_pre_vect) {
						circle (result,point, 5, blue, 2);
						line (result, Point (point.x - radius / 2, point.y), Point (point.x + radius / 2, point.y), blue, 1);
						putText(result, "state_pre_vect: ", 
									cv::Point(10, 40),
									FONT_HERSHEY_SIMPLEX, 
									0.5,
									blue);
					}
					for (Point point : state_post_vect) {
						circle (result,point, 5, blue, 2);
						line (result, Point (point.x - radius / 2, point.y), Point (point.x + radius / 2, point.y), green, 1);
						putText(result, "state_post_vect: ", 
									cv::Point(10, 70),
									FONT_HERSHEY_SIMPLEX, 
									0.5,
									green);
					}
						
					
				}				
				imshow("Tracing: ", result);
				// STATIONARY BLOBS
				if (it==1)
					{
					sfgmask = Mat::zeros(Size(fgmask.cols, fgmask.rows), CV_8UC1);
					fgmask_history = Mat::zeros(Size(fgmask.cols, fgmask.rows), CV_32FC1);
					}
					
				// Extract the STATIC blobs in fgmask
				extractStationaryFG(fgmask, fgmask_history, sfgmask);
				extractBlobs(sfgmask, sbloblist, connectivity);
				//		cout << "Num STATIONARY blobs extracted=" << sbloblist.size() << endl;
				
			
				removeSmallBlobs(sbloblist, sbloblistFiltered, MIN_WIDTH, MIN_HEIGHT);
				//		cout << "Num STATIONARY small blobs removed=" << sbloblist.size()-sbloblistFiltered.size() << endl;


				// Clasify the blobs in fgmask
				classifyBlobs(sbloblistFiltered);
			
		
				//Time measurement
				t = (double)getTickCount() - t;
        		        //if (_CONSOLE_DEBUG) cout << "proc. time = " << 1000*t/t_freq << " milliseconds."<< endl;
				acum_t=+t;

				//SHOW RESULTS
				//get the frame number and write it on the current frame

				string title= project_name + " | Frame - FgM - Stat FgM | Blobs - Classes - Stat Classes | BlobsFil - ClassesFil - Stat ClassesFil | ("+dataset_cat[c] + "/" + baseline_seq[s] + ")";

				ShowManyImages(title, 6, frame, fgmask, sfgmask,
						paintBlobImage(frame,bloblistFiltered, true), paintBlobImage(frame,bloblistFiltered, true), paintBlobImage(frame,sbloblistFiltered, true));
				
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