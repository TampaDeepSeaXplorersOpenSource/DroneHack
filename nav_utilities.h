/*
 * nav_utilities.h
 *
 *  Created on: Oct 29, 2016
 *      Author: cbegian
 */

#ifndef NAV_UTILITIES_H_
#define NAV_UTILITIES_H_

#include <stdbool.h>
#include "nav_typedefs.h"

#define DEGREES_PER_RADIAN	57.2958
#define	CLOSE_ENOUGH_METERS	3.0

double wpt_TC( Waypoint fromWPT, Waypoint toWPT );
double wpt_distance( Waypoint first, Waypoint second );

bool navigateToWPT( Waypoint startWPT, Waypoint* destWPTPtr,
					long* batteryDurationPtr, int surfacingInterval );

Waypoint readGPS();

double getSpeed();

Timestamp compute_WPT_ETA( Waypoint wpt, Waypoint fix );
Drift calculate_drift( Waypoint anticipatedWPT, Waypoint fix,
					   int trueCourse, int surfacingInterval );

Direction mag_hdg_to_wpt( Waypoint wpt, Waypoint fix, int surfacingInterval,
							int magVar );

bool isCloseEnough( Waypoint fix, Waypoint wpt );


#endif /* NAV_UTILITIES_H_ */
