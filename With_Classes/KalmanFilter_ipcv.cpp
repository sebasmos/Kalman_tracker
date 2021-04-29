/* Applied Video Analysis of Sequences (AVSA)
 *
 *	LAB3: Kalman Filtering for object tracking
 * 
 *
 * Authors: Sebastian Ordonez and Mary Chris Go
 */

#include "KalmanFilter_ipcv.hpp"

using namespace cv; 

void KalmanFilter_ipcv::Kalman_initialization (int type, float x, float y)
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

// prediction of the next step 
// https://answers.opencv.org/question/113657/kalman-filter-assertion-failed-matrix-types/

void KalmanFilter::PredictionOfNextStep () 
{
	Mat prediction = KF.predict();

	if (type == TypeOfFilter::constantVelocity) 
        Point predictPt(prediction.at<float> (1), prediction.at<float> (2));
        predictv.push_back(predictPt); // inserts new element at the end of vector and increases size of vector by one.
	

	else if (type == TypeOfFilter::constantAcceleration)
		Point predictPt(prediction.at<float> (1), prediction.at<float> (3)); 
		predictv.push_back(predictPt);

	RealTrajectory.push_back (predictv.back ());
	
}

// correction of the next step
void KalmanFilter::Update ()  (const cv::Point& measurementPoints)
{
	measuredTrajectory.push_back (measurementPoints);
	Mat estimated = KF.correct() ((Mat_<float> (x,x) << (float)measurementPoints.x, (float)measurementPoints.y)); // insert values

	if (type == TypeOfFilter::constantVelocity) 
        Point statePt(estimated.at<float>(1), estimated.at<float>(2)); 
        kalmanv.push_back(statePt);

	else if (type == TypeOfFilter::constantAcceleration)
		Point statePt(estimated.at<float>(1), estimated.at<float>(3)); 
        kalmanv.push_back(statePt);

	RealTrajectory.back (kalmanv.back ()); // no need for push_back since have an estimation, we just overwrite the prediction with the right measurements 
	
}

// corrected state
// https://sites.google.com/site/timecontroll/tutorials/extended-kalman-filtering-with-opencv 
void KalmanFilter::Initialization (const cv::Points& initialPoints)
{
	measuredTrajectory.push_back (initialPoints)
	if (type == TypeOfFilter::constantVelocity) 
    	KF.statePost.at<float> (1) = initialPosition.x;
    	KF.statePost.at<float> (2) = initialPosition.y;

	else if (type == TypeOfFilter::constantAcceleration)
    	KF.statePost.at<float> (1) = initialPosition.x;
    	KF.statePost.at<float> (3) = initialPosition.y;
                 
}
/*
void initKalman_vel_model(float x, float y)
{
    // Instantate Kalman Filter with
    // 4 dynamic parameters and 2 measurement parameters,
    // where my measurement is: 2D location of object,
    // and dynamic is: 2D location and 2D velocity.
    KF.init(4, 2, 0);

    measurement = Mat_<float>::zeros(2,1);
    measurement.at<float>(0, 0) = x;
    measurement.at<float>(0, 0) = y;


    KF.statePre.setTo(0);
    KF.statePre.at<float>(0, 0) = x;
    KF.statePre.at<float>(1, 0) = y;

    KF.st

*/

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


Mat Draw_tracking(cv::Mat result, cv::Point_<float> measurement, cv::Point_<float> state_pre, cv::Point_ <float> state_post  ){

	//Scalar red = Scalar (0, 0, 255);

	Scalar blue = Scalar (255, 0, 0);

	Scalar green = Scalar (255, 255, 0);

	Mat final_img;

	result.copyTo(final_img);

	// Store results in vectors for plotting purposes.
	measurement_vect.push_back(measurement);
	state_pre_vect.push_back(state_pre);
	state_post_vect.push_back(state_post);

	cout << "kalman prediction (state_pre): " << state_pre.x << " " << state_pre.y << endl;
	cout << "kalman corrected state (state_post): " << state_post.x << " " << state_post.y << endl;
					

	//int radius= 5;
		for (size_t i = 0; i < state_pre_vect.size () - 1; i++) {

		circle (final_img, state_pre_vect[i], 5, blue, 2);
		line (final_img, state_pre_vect[i], state_pre_vect[i+1], blue, 1);
		putText(final_img, "state_pre_vect: ", 
									cv::Point(10, 15),
									FONT_HERSHEY_SIMPLEX, 
									0.5,
									blue);
	}
		for (size_t i = 0; i < state_post_vect.size () - 1; i++) {

		circle (final_img, state_post_vect[i], 5, green, 2);
		line (final_img, state_post_vect[i], state_pre_vect[i+1], green, 1);
		putText(final_img, "state_post_vect: ", 
									cv::Point(10, 45),
									FONT_HERSHEY_SIMPLEX, 
									0.5,
									green);
	}
	//-------------------plot measurement for comparaison------------
	/*
	
	for (Point point : measurement_vect) {

		line (final_img, Point (point.x - radius / 2, point.y), Point (point.x + radius / 2, point.y), red, 1);

		line (final_img, Point (point.x, point.y - radius / 2), Point (point.x, point.y + radius / 2), red, 1);

	}

	for (size_t i = 0; i < measurement_vect.size () - 1; i++) {

		//cout << measurement_vect[i]<< endl;
		//circle (final_img, measurement_vect[i], 5, red, 2);
		line (final_img, measurement_vect[i], measurement_vect[i+1], red, 1);
		putText(final_img, "measurement_vect: ", 
									cv::Point(10, 10),
									FONT_HERSHEY_SIMPLEX, 
									0.5,
									red);
	}
	*/
		return final_img;

}



// for getting the predicted trajectory
std::vector<Point> KalmanFilter_ipcv::ObtainPredictedTrajectory () const
{
	return predictv;
}
// for getting the estimated trajectory
std::vector<Point> KalmanFilter_ipcv::ObtainEstimatedTrajectory () const
{
	return kalmanv;
}
// for getting measured trajectory
std::vector<Point> KalmanFilter_ipcv::ObtainMeasuredTrajectory () const
{
	return measuredTrajectory;
}
// for getting the final trajectory
std::vector<Point> KalmanFilter_ipcv::ObtainRealTrajectory () const
{
	return RealTrajectory;
}

