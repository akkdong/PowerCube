/*
 * VarioFilter.cpp
 *
 *
 */

#include "Variometer.h"


#define VFILTER_HARINAIR_KF2     1
#define VFILTER_HARINAIR_KF3     2
#define VFILTER_HARINAIR_KF4d    3
#define VFILTER_ROBIN_KF         4

#define USE_KALMAN_FILTER        VFILTER_HARINAIR_KF3

#if USE_KALMAN_FILTER == VFILTER_HARINAIR_KF2
#include "vario/VarioFilter_HarInAirKF2.h"
#elif USE_KALMAN_FILTER == VFILTER_HARINAIR_KF3
#include "vario/VarioFilter_HarInAirKF3.h"
#elif USE_KALMAN_FILTER == VFILTER_HARINAIR_KF4d
#include "vario/VarioFilter_HarInAirKF4d.h"
#elif USE_KALMAN_FILTER == VFILTER_ROBIN_KF
#include "vario/VarioFilter_RobinKF.h"
#else
#error "Invalid vario kalman-filter"
#endif

#if USE_KALMAN_FILTER == VFILTER_HARINAIR_KF2
VarioFilter_HarInAirKF2     varioFilter;
#elif USE_KALMAN_FILTER == VFILTER_HARINAIR_KF3
VarioFilter_HarInAirKF3     varioFilter;
#elif USE_KALMAN_FILTER == VFILTER_HARINAIR_KF4d
VarioFilter_HarInAirKF4d    varioFilter;
#elif USE_KALMAN_FILTER == VFILTER_ROBIN_KF
VarioFilter_RobinKF         varioFilter;
#endif





//
//
//

IVarioFilter *CreateVarioFilter()
{
	#if USE_KALMAN_FILTER == VFILTER_HARINAIR_KF2
	varioFilter.begin(40.0f, 1000000.0f, 0);
	#elif USE_KALMAN_FILTER == VFILTER_HARINAIR_KF3
	varioFilter.begin(500.0f, 6000.0f, 1.0f, 0);
	#elif USE_KALMAN_FILTER == VFILTER_HARINAIR_KF4d

	// injects additional uncertainty depending on magnitude of acceleration
	// helps respond quickly to large accelerations while heavily filtering
	// in low acceleration situations.  Range : 0.5 - 1.5
	#define KF_ACCEL_VARIANCE_DEFAULT   100     // 50 ~ 150
	#define KF_ADAPT_DEFAULT            10     // 50 ~ 150

	varioFilter.begin(KF_ACCEL_VARIANCE_DEFAULT * 1000.0f, KF_ADAPT_DEFAULT / 100.0f, 0, 0, 0);
	#elif USE_KALMAN_FILTER == VFILTER_ROBIN_KF
	varioFilter.Configure(30.0f, 4.0f, altitude);
	#endif

	return &varioFilter;
}
