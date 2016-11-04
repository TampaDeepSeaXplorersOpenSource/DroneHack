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
#define	CLOSE_ENOUGH_METERS	1.0
#define MAX_DISTANCE_ON_SURFACE	3.0

#define PI	3.14159265
#define	FOURTHPI (PI / 4)
#define DEGREES_TO_RADIANS (PI / 180)
#define RADIANS_TO_DEGREES (180.0 / PI)

//Ellipsoid name, Equatorial Radius, square of eccentricity

#define ELLIPSOID_NAME 			"WGS-84"
#define	EQUATORIAL_RADIUS		6378137
#define ECCENTRICITY_SQUARED	0.00669438

double wpt_TC( Waypoint fromWPT, Waypoint toWPT );
double wpt_distance( Waypoint first, Waypoint second );

bool navigateToWPT( Waypoint startWPT, Waypoint* destWPTPtr,
					long* batteryDurationPtr, int surfacingInterval );

Waypoint readGPS();

double getSpeed();

bool createRouteSegment( Waypoint lastPosition, Waypoint toWPT,
						 int surfacingInterval,
							RouteSegment *routeSegmentPtr,
							double speedInMetersPerSecond );

Waypoint computeSegmentEnd( Waypoint fromWPT, Direction trueCourse,
							double segmentDistance );

Timestamp compute_WPT_ETA( Waypoint wpt, Waypoint fix );
Drift calculate_drift( Waypoint anticipatedWPT, Waypoint fix,
					   int trueCourse, int surfacingInterval );

Direction mag_hdg_to_wpt( Waypoint wpt, Waypoint fix, int surfacingInterval,
							int magVar );

bool isCloseEnough( Waypoint fix, Waypoint wpt );

Waypoint latLongToUTM( double latInDegrees, double lonInDegrees,
					   char *dateTimeString );

char UTMLetterDesignator( double latitude );

#endif /* NAV_UTILITIES_H_ */


