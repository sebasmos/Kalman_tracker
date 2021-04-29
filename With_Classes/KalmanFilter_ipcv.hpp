/* Applied Video Analysis of Sequences (AVSA)
 *
 *	LAB3: Kalman Filtering for object tracking
 * 
 *
 * Authors: Sebastian Ordonez and Mary Chris Go
 */

#ifndef KALMANFILTER_H_INCLUDE
#define KALMANFILTER_H_INCLUDE

#include "opencv2/opencv.hpp"

// add namespace
class KalmanFilter_ipcv{
public:
    // Constructor
    void Kalman_initialization (int type, float x, float y);
    Point kalmanPredict() ;
    Point kalmanCorrect(float x, float y);
    	
    /*
    void PredictionOfNextStep ();
    void Update (const cv::Point& measurementPoints);
    void KalmanFilter::Initialization (const cv::Points& initialPoints)
      */
    std::vector<cv::Point> ObtainMeasuredTrajectory () const;
	std::vector<cv::Point> ObtainPredictedTrajectory () const;
	std::vector<cv::Point> ObtainEstimatedTrajectory () const;
	std::vector<cv::Point> ObtainRealTrajectory () const;
  

private:
    cv::KalmanFilter KF;
    /*
    std::vector<cv::Point> measuredTrajectory;
	std::vector<cv::Point> predictv;
	std::vector<cv::Point> kalmanv;
	std::vector<cv::Point> RealTrajectory;
    */

    std::vector<Point> measurement_vect;
    std::vector<Point> state_pre_vect;
    std::vector<Point> state_post_vect;

    Mat_<float> measurement;//measurement(2,1); 
    int type = 0;

};
#endif