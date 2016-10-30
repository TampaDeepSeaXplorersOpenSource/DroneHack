/*
 * nav_typedefs.h
 *
 *  Created on: Oct 28, 2016
 *      Author: cbegian
 */

#ifndef NAV_TYPEDEFS_H_
#define NAV_TYPEDEFS_H_

#include <time.h>

typedef time_t Timestamp;

typedef short Direction;

typedef struct {
	/* speed of current */
	double speed;

	/* direction of current */
	double direction;

	/* Number of degrees E (-) or W (+) to correct for drift. */
	double correctionAngle;
} Drift;

typedef struct {

	/* These UTM values are in meters */
	unsigned int utm_east;
	unsigned int utm_north;
	Timestamp timestamp;		/* If from GPS, time of GPS reading, else 0 */
} Waypoint;

typedef struct {
	Waypoint originationWPT;	/* "from" WPT */
	Waypoint destinationWPT;	/* "to" WPT */
	Direction	TC;				/* True Course 0-359 */
	Direction	MH;				/* Magnetic Heading */
	long	ETE;				/* Estimated Time Enroute (in SECONDS) */
	Timestamp ETA;				/* Estimated Time of Arrival */
	Drift	drift;				/* Last computed drift and correction angle. */
} RouteSegment;

#endif /* NAV_TYPEDEFS_H_ */
