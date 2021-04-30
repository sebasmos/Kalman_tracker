/* Applied Video Analysis of Sequences (AVSA)
 *
 *	LAB3: Kalman Filtering for object tracking
 * 
 *
 * Authors: Sebastian Ordonez and Mary Chris Go
 */

#include "KalmanFilter_ipcv.hpp"
#include <opencv2/opencv.hpp>

using namespace KalmanFilter_ipcv;

using namespace cv; 

cv::KalmanFilter KF;


void Tracking::Kalman_initialization (int type, float x, float y)
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

 


 /*
 
 
 */