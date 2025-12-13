// VarioFilter_HarInAirKF2.cpp
//

#include "VarioFilter_HarInAirKF2.h"
#include "utils.h"

#define KALMAN_UPDATE_FREQ          (25)




///////////////////////////////////////////////////////////////////////////////////////////////
//

VarioFilter_HarInAirKF2::VarioFilter_HarInAirKF2()
{
}


int VarioFilter_HarInAirKF2::begin(float zVariance, float zAccelVariance, float altitude)
{
	// init values
	zAccelVariance_ = zAccelVariance;
	zVariance_ = zVariance;

	//
	reset(altitude);

	return 0;
}

void VarioFilter_HarInAirKF2::reset(float altitude)
{
	z_ = altitude;
	v_ = 0.0f; // vInitial;
	t_ = getTick();

	Pzz_ = 1500.0f;
	Pzv_ = 0.0f;
	Pvz_ = Pzv_;
	Pvv_ = 1500.0f;
}

void VarioFilter_HarInAirKF2::update(float altitude, float va, float* altitudeFilteredPtr, float* varioPtr)
{
	//
	#if USE_CM
	altitude = altitude * 100.0f; // m --> cm
	#endif

	// delta time
	#if 0
	uint32_t lastTick = getTick();
	float dt = ((float)(lastTick - t_)) / 1000.0;
	t_ = lastTick;
	#else
	float dt = 1.0 / KALMAN_UPDATE_FREQ; // 25Hz
	#endif

	// predict
	predict(dt);

	// Update step.
	float y = altitude - z_;  // Innovation.
	float sInv = 1.0f / (Pzz_ + zVariance_);  // Innovation precision.
	float kz = Pzz_ * sInv;  // Kalman gain
	float kv = Pzv_ * sInv;

	// Update state estimate.
	z_ += kz * y;
	v_ += kv * y;

	#if USE_CM
	*altitudeFilteredPtr = z_ / 100.0f; // cm --> m
	*varioPtr = v_ / 100.0f; // cm/s --> m/s
	#else
	*altitudeFilteredPtr = z_;
	*varioPtr = v_;
	#endif

	// Update state covariance.
	Pvv_ -= Pzv_ * kv;
	Pzv_ -= Pzv_ * kz;
	Pvz_  = Pzv_;
	Pzz_ -= Pzz_ * kz;
}

void VarioFilter_HarInAirKF2::predict(/*float zAccelVariance,*/ float dt)
{
	/*zAccelVariance_ = zAccelVariance;*/

	// predict state estimate.
	z_ += v_ * dt;

	// predict state covariance. The last term mixes in acceleration noise.
	Pzz_ += dt*Pzv_ + dt*Pvz_ + dt*dt*Pvv_ + zAccelVariance_*dt*dt*dt*dt/4.0f;
	Pzv_ +=                        dt*Pvv_ + zAccelVariance_*dt*dt*dt/2.0f;
	Pvz_ = Pzv_;
	Pvv_ +=                                + zAccelVariance_*dt*dt;
}
