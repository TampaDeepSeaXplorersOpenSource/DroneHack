/*
 * nav_utilities.c
 *
 *  Created on: Oct 29, 2016
 *      Author: cbegian
 */
#include <stdlib.h>
#include <math.h>
#include "nav_utilities.h"

double wpt_distance( Waypoint first, Waypoint second )
{
	int eastingDiff;
	int northingDiff;
	double distance;

	/* Using Pythagorean Theorem, compute distance between two UTM points. */
	eastingDiff = abs( first.utm_east - second.utm_east );
	northingDiff = abs( first.utm_north - second.utm_north );
	distance = sqrt( pow( eastingDiff, 2.0 ) + pow( northingDiff, 2.0 ) );

	return distance;
}

double wpt_TC( Waypoint fromWPT, Waypoint toWPT )
{
	int quadrantOffset[ 4 ] = { 0, 90, 180, 270 };
	double tcInRadians;
	double ratio;
	double tcInDegrees;

	/* First, compute True Course (TC) from the first WPT to the second. */

	int eastingDiff = toWPT.utm_east - fromWPT.utm_east;
	int northingDiff = toWPT.utm_north - fromWPT.utm_north;

	/* Determine where (which quadrant) second WPT is, in relation to first. */
	int quadrant = 0; /* start by assuming 2nd WPT is NE of 1st(quadrant I) */

	if ( eastingDiff < 0 )
	{
		/* First WPT is W of second. */
		if ( northingDiff < 0 )
		{
			/* First WPT is SW of second. */
			quadrant = 2; /* quadrant III */
		}
		else
		{
			/* First WPT is NW of second. */
			quadrant = 3; /* quadrant IV */
		}
	}
	else
	{
		/* First WPT is E of second. */
		if ( northingDiff < 0 ) {
			/* First WPT is SE of second. */
			quadrant = 1; /* quadrant II */
		}
	}

	/* TC is the arctan() of the ratio of the northingDiff and eastingDiff. */
	/* For quadrants I and III, eastingDiff is the numerator of the ratio. */
	/* Otherwise, northingDiff is the numerator. */
	if ( ( quadrant == 0 ) || ( quadrant == 2) )
	{
		/* swap numerator and denominator of ratio */
		int temp = northingDiff;
		northingDiff = eastingDiff;
		eastingDiff = temp;
	}

	ratio = northingDiff / eastingDiff;

	tcInRadians = atan( ratio );

	/* Convert radians to degrees. */
	tcInDegrees = tcInRadians * DEGREES_PER_RADIAN;

	/* Add the angle in degrees to the beginning of the quadrant. */
	tcInDegrees += quadrantOffset[ quadrant ];

	/**** WARNING! the TC returned is in decimal degrees. It may be */
	/* < 0 or > 359. Caller must correct for both of these before using. */
	return tcInDegrees;
}

bool navigateToWPT( Waypoint startWPT, Waypoint* destWPTPtr,
					long* batteryDurationPtr, int surfacingInterval )
{
	bool okToContinue = true;
	bool arrivedAtWPT = false;
	RouteSegment routeSegment;
	Waypoint toWPT = *destWPTPtr;
	Waypoint lastPosition = startWPT;

	/****
	 * Keep creating and executing route segments until we either reach the
	 * destination WPT, or we determine that we will not have enough battery
	 * remaining to make it home if we were to execute the next route segment.
	 */
	while ( ( !arrivedAtWPT ) && ( okToContinue ) )
	{
		/****
		 * Create the next route segment. If we would not have sufficient
		 * battery to make it home after traveling this segment, do not
		 * execute the segment. Set flag to indicate we do not have enough
		 * battery to continue the mission.
		 */
		okToContinue = createRouteSegment(lastPosition, toWPT, &routeSegment);
		if ( okToContinue )
		{
			/****
			 * PSEUDOCODE
			 *
			 * Execute the route segment. Note that when we complete
			 * the segment, we must surface and get a GPS reading. We
			 * may or may not be where we expected. Either way, the GPS
			 * reading will serve as our starting point for the next segment.
			 * However, if we ended the segment "close enough" to our target
			 * WPT, then we have "arrived" at the WPT. Return our current
			 * location in *destWPTPtr to be used as a starting point for
			 * further navigation.
			 */
			executeSegment( routeSegment );

			/* Surface to get a GPS fix. */
			surface();

			/* Get GPS fix. */
			lastPosition = readGPS();

			/* Are we "close enough" to destination WPT? */
			arrivedAtWPT = isCloseEnough( lastPosition, *destWPTPtr );
		}

		/****
		 * Save our last known position for use in further navigation.
		 * Notice that this MIGHT NOT be exactly at the destination
		 * WPT. Therefore, return current position as destination.
		 */
		*destWPTPtr = lastPosition;
	}

	return okToContinue;
}

Waypoint readGPS()
{
	Waypoint gpsFix = { 0, 0, 0 };

	/****
	 * PSEUDOCODE
	 *
	 * Read the current location from the GPS, and return it as a Waypoint
	 * value.
	 *
	 * Set the timestamp field to the current GPS time. The system time
	 * will have been previously set to GPS UTC time (when we got our initial
	 * GPS fix.
	 */

	return gpsFix;
}


double getSpeed()
{
	double speed = 0.25; 	/* Speed in meters/sec */

	/* Returns the current estimated turtle speed. */

	return speed;
}

Timestamp compute_WPT_ETA( Waypoint wpt, Waypoint fix )
{
	double distance;
	double travelTime;		/* Number of seconds it will take to reach wpt */
	Timestamp eta = 0;

	/* Get the distance between where we are (fix) and our destination (wpt) */
	distance = wpt_distance( wpt, fix );

	/* Divide the distance (in meters) by our speed in meters/sec */
	travelTime = distance / getSpeed();

	/* ETA is current time + estimated travel time. */
	eta = travelTime + time(NULL);

	return eta;
}

Drift calculate_drift( Waypoint anticipatedWPT, Waypoint fix,
					   int trueCourse, int surfacingInterval )

{
	Drift drift = { 0.0, 0.0, 0.0 };	/* Initialize to zero. */

	/* Need to determine how far away we are (the fix) from where we should */
	/* be (anticipatedWPT). That will give us the length of the drift vector */
	drift.speed = wpt_distance( anticipatedWPT, fix ) / surfacingInterval;

	/* Get the direction of the drift vector. */
	/* direction is TRUE (not magnetic) */
	drift.direction = wpt_TC( anticipatedWPT, fix );

	/***
	 * Drift correction angle (DCA) formula:
	 * DCA = asin( (speed of current / speed of turtle)
	 * 				* sin(direction of current - true course) )
	 */
	drift.correctionAngle = asin( ( drift.speed / getSpeed() ) *
									sin( drift.direction - trueCourse ) );

	return drift;
}

Direction mag_hdg_to_wpt( Waypoint wpt, Waypoint fix, int surfacingInterval,
						  int magVar )
{
	Direction magneticHeading = 0.0;
	Drift drift;

	double trueCourseInDegrees;
	double trueHeadingInDegrees;
	double magHeadingInDegrees;

	/* UTM coordinates allow us to simplify computation of new heading. */
	/* First, compute True Course (TC) from our position (fix) to the */
	/* waypoint (wpt). */

	trueCourseInDegrees = wpt_TC( wpt, fix );

	/* We now have the true course (TC) from fix to WPT. If we simply turn */
	/* until our compass points in that direction, we will miss the WPT, due */
	/* to magnetic variation, drift, and compass error. We can compensate */
	/* for some of these. */

	/* Compensate for drift. */
	drift = calculate_drift( wpt, fix, trueCourseInDegrees, surfacingInterval );
	trueHeadingInDegrees = trueCourseInDegrees + drift.correctionAngle;

	/* Compensate for magnetic variation (magVar) */
	/* We subtract an easterly variation, and add a westerly variation. */
	/* We store easterly variations as negative values, so adding works for */
	/* both easterly and westerly variations. */
	magHeadingInDegrees = trueHeadingInDegrees + magVar;

	/* If we compensated for compass errors, it would go here. We would then */
	/* have a compass heading (CH) to steer. */

	/* Correct any negative result. */
	if ( magHeadingInDegrees < 0.0 )
	{
		magHeadingInDegrees += 360.0;
	}

	/* Round off result. */
	magneticHeading = round( magHeadingInDegrees );

	/* Keep the result between 0 and 359 degrees */
	magneticHeading %= 360;

	/* Return the Magnetic Heading (MH) needed to steer directly to WPT */
	return magneticHeading;
}

bool isCloseEnough( Waypoint fix, Waypoint wpt )
{
	double distance;
	bool closeEnough = false;

	/* Get the distance bewtween the fix and our goal (the wpt). */
	distance = wpt_distance( fix, wpt );

	/* Determine if we are "close enough" to our goal. */
	if ( distance <= CLOSE_ENOUGH_METERS )
	{
		closeEnough = true;
	}
	return closeEnough;
}
