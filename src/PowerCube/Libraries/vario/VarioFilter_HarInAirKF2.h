// VarioFilter_HarInAirKF2.h
//

//
// Reference: https://github.com/har-in-air/ESP32_IMU_BARO_GPS_VARIO/blob/master/offline/kf/compare_kf2_kf3_kf4.ipynb
//
//
//

#ifndef __VARIL_FILTER_HARINAIRKF2_H__
#define __VARIL_FILTER_HARINAIRKF2_H__

#include "VarioFilter.h"


////////////////////////////////////////////////////////////////////////////////////////////
//

class VarioFilter_HarInAirKF2 : public IVarioFilter
{
public:
	VarioFilter_HarInAirKF2();

public:
	int						begin(float zVariance = 400.0, float zAccelVariance = 1000.0, float altitude = 0);
	void					end();

	// IVarioFilter
	void					update(float altitude, float va, float* altitudeFiltered, float* vv);
	void					reset(float altitude);
	
protected:
	void					predict(/*float zAccelVariance,*/ float dt);

private:
	// State being tracked
	float					z_;  // position
	float					v_;  // velocity

	// 2x2 State Covariance matrix
	float					Pzz_;
	float					Pzv_;
	float					Pvz_;
	float					Pvv_;


	float					zAccelVariance_;  // dynamic acceleration variance
	float					zVariance_; //  z measurement noise variance fixed

	uint32_t				t_;
};

#endif // __VARIL_FILTER_HARINAIRKF2_H__
