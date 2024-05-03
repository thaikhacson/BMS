#ifndef __KALMAN_H_
#define __KALMAN_H_

	// class SimpleKalmanFilter
	void SimpleKalmanFilter(float mea_e, float est_e, float q, int sensor);
	float updateEstimate(float mea, int sensor);
	void setMeasurementError(float mea_e, int sensor);
	void setEstimateError(float est_e, int sensor);
	void setProcessNoise(float q, int sensor);
	float getKalmanGain(int sensor);
	float getEstimateError(int sensor);

#endif /*__KALMAN_H*/