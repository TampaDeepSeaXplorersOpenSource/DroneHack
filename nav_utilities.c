/*
 * nav_utilities.c
 *
 *  Created on: Oct 29, 2016
 *      Author: cbegian
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
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
		//okToContinue = createRouteSegment(lastPosition, toWPT, &routeSegment);
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
			//executeSegment( routeSegment );

			/* Surface to get a GPS fix. */
			//surface();

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
	Waypoint gpsFix;

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

	/* Presume for now that GPS sends back a character sequence. DAVE: change
	 * this to whatever format you want to return the GPS data in.
	 */
	char gpsData[ 80 ];

	double latInDegrees;
	double lonInDegrees;
	char *dateTimeString;

	/* Read GPS position. */
	/* readTheGPS( gpsData ); */

	char *token = NULL;

	/* Parse the GPS data. Each field is separated by a comma. */
	/* 0,lat,lon,altitude,UTC_yymmddHHMMSS.mmm,ttf,#sats,kts,course */

	/* First token is "0" (the mode). Read past it. */
	token = strtok( gpsData, "," );

	/* The lat/lon coordinates are returned as:
	 * Latitude: DDMM.MMMM
	 * Longitude: DDDMM.MMMM
	 *
	 * Need to convert these to decimal degrees before conversion to UTM.
	 */

	/* Get latitude */
	token = strtok( NULL, "," );
	sscanf( token, "%lf", &latInDegrees );

	/* Get longitude */
	token = strtok( NULL, "," );
	sscanf( token, "%lf", &lonInDegrees );

	/* Get timestamp */
	dateTimeString = strtok( NULL, "," );

	/* The rest of the tokens are ignored. */

	/* Convert lat/lon to UTM coordinates. */
	gpsFix = latLongToUTM( latInDegrees, lonInDegrees, dateTimeString );

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


Waypoint latLongToUTM( double latInDegrees, double lonInDegrees,
						char *dateTimeString )
{
	/****
	 *
	 * Converts lat/long to UTM coords.  Equations from USGS Bulletin 1532
	 * East Longitudes are positive, West longitudes are negative.
	 * North latitudes are positive, South latitudes are negative
	 * Lat and Long are in decimal degrees.
	 */

	double a = EQUATORIAL_RADIUS;
	double eccSquared = ECCENTRICITY_SQUARED;
	double k0 = 0.9996;

	double eccPrimeSquared;
	double N, T, C, A, M;

	/* Make sure the longitude is between -180.00 .. 179.9 */
	double tempLongitude = ( lonInDegrees + 180 ) - ((int)( ( lonInDegrees + 180 ) / 360 ) * 360) - 180;

	double latitudeRadius = latInDegrees * DEGREES_TO_RADIANS;
	double longitudeRadius = tempLongitude * DEGREES_TO_RADIANS;
	double longitudeOriginRadius;
	int    zoneNumber;

	double wholeDegrees;
	double fractionalDegrees;
	struct tm timeStruct;
	int year;
	int month;
	int day;
	int hours;
	int minutes;
	int seconds;

	Waypoint utmWaypoint;

	latInDegrees /= 100.0;
	wholeDegrees = trunc( latInDegrees );

	/* Get the minutes. */
	fractionalDegrees = (latInDegrees - wholeDegrees) * 100.0;

	latInDegrees = wholeDegrees + ( fractionalDegrees / 60.0 );

	lonInDegrees /= 100.0;
	wholeDegrees = trunc( lonInDegrees );

	/* Get the minutes. */
	fractionalDegrees = (lonInDegrees - wholeDegrees) * 100.0;

	lonInDegrees = wholeDegrees + ( fractionalDegrees / 60.0 );

	sscanf( dateTimeString, "%2d%2d%2d%2d%2d%2d", &year, &month, &day,
			                                      &hours, &minutes, &seconds );


	timeStruct.tm_year = year + 100;
	timeStruct.tm_mon = month;
	timeStruct.tm_mday = day;
	timeStruct.tm_hour = hours;
	timeStruct.tm_min = minutes;
	timeStruct.tm_sec = seconds;

	utmWaypoint.timestamp = mktime( &timeStruct );

	zoneNumber = ((int)( ( tempLongitude + 180 ) / 6 )) + 1;

	if( (latInDegrees >= 56.0 && latInDegrees < 64.0) &&
		 (tempLongitude >= 3.0 && tempLongitude < 12.0 ) )
	{
		zoneNumber = 32;
	}

  // Special zones for Svalbard
	if( latInDegrees >= 72.0 && latInDegrees < 84.0 )
	{
	  if (      tempLongitude >= 0.0  && tempLongitude <  9.0 ) zoneNumber = 31;
	  else if ( tempLongitude >= 9.0  && tempLongitude < 21.0 ) zoneNumber = 33;
	  else if ( tempLongitude >= 21.0 && tempLongitude < 33.0 ) zoneNumber = 35;
	  else if ( tempLongitude >= 33.0 && tempLongitude < 42.0 ) zoneNumber = 37;
	 }
	longitudeOriginRadius = ((zoneNumber - 1)*6) - 180 + 3;  //+3 puts origin in middle of zone
	longitudeOriginRadius = longitudeOriginRadius * DEGREES_TO_RADIANS;

	//compute the UTM Zone from the latitude and longitude
	sprintf( utmWaypoint.zone, "%d%c", zoneNumber, UTMLetterDesignator( latInDegrees ) );

	eccPrimeSquared = (eccSquared)/(1-eccSquared);

	N = a/sqrt(1-eccSquared*sin(latitudeRadius)*sin(latitudeRadius));
	T = tan(latitudeRadius)*tan(latitudeRadius);
	C = eccPrimeSquared*cos(latitudeRadius)*cos(latitudeRadius);
	A = cos(latitudeRadius)*(longitudeRadius-longitudeOriginRadius);

	M = a*((1	- eccSquared/4		- 3*eccSquared*eccSquared/64	- 5*eccSquared*eccSquared*eccSquared/256)*latitudeRadius
				- (3*eccSquared/8	+ 3*eccSquared*eccSquared/32	+ 45*eccSquared*eccSquared*eccSquared/1024)*sin(2*latitudeRadius)
									+ (15*eccSquared*eccSquared/256 + 45*eccSquared*eccSquared*eccSquared/1024)*sin(4*latitudeRadius)
									- (35*eccSquared*eccSquared*eccSquared/3072)*sin(6*latitudeRadius));

	utmWaypoint.utm_east = (double)(k0*N*(A+(1-T+C)*A*A*A/6
					+ (5-18*T+T*T+72*C-58*eccPrimeSquared)*A*A*A*A*A/120)
					+ 500000.0);

	utmWaypoint.utm_north = (double)(k0*(M+N*tan(latitudeRadius)*(A*A/2+(5-T+9*C+4*C*C)*A*A*A*A/24
				 + (61-58*T+T*T+600*C-330*eccPrimeSquared)*A*A*A*A*A*A/720)));
	if ( latInDegrees < 0 )
	{
		utmWaypoint.utm_north += 10000000.0; //10000000 meter offset for southern hemisphere
    }

	return utmWaypoint;
}

char UTMLetterDesignator(double Lat)
{
//This routine determines the correct UTM letter designator for the given latitude
//returns 'Z' if latitude is outside the UTM limits of 84N to 80S
	//Written by Chuck Gantz- chuck.gantz@globalstar.com
	char LetterDesignator;

	if((84 >= Lat) && (Lat >= 72)) LetterDesignator = 'X';
	else if((72 > Lat) && (Lat >= 64)) LetterDesignator = 'W';
	else if((64 > Lat) && (Lat >= 56)) LetterDesignator = 'V';
	else if((56 > Lat) && (Lat >= 48)) LetterDesignator = 'U';
	else if((48 > Lat) && (Lat >= 40)) LetterDesignator = 'T';
	else if((40 > Lat) && (Lat >= 32)) LetterDesignator = 'S';
	else if((32 > Lat) && (Lat >= 24)) LetterDesignator = 'R';
	else if((24 > Lat) && (Lat >= 16)) LetterDesignator = 'Q';
	else if((16 > Lat) && (Lat >= 8)) LetterDesignator = 'P';
	else if(( 8 > Lat) && (Lat >= 0)) LetterDesignator = 'N';
	else if(( 0 > Lat) && (Lat >= -8)) LetterDesignator = 'M';
	else if((-8> Lat) && (Lat >= -16)) LetterDesignator = 'L';
	else if((-16 > Lat) && (Lat >= -24)) LetterDesignator = 'K';
	else if((-24 > Lat) && (Lat >= -32)) LetterDesignator = 'J';
	else if((-32 > Lat) && (Lat >= -40)) LetterDesignator = 'H';
	else if((-40 > Lat) && (Lat >= -48)) LetterDesignator = 'G';
	else if((-48 > Lat) && (Lat >= -56)) LetterDesignator = 'F';
	else if((-56 > Lat) && (Lat >= -64)) LetterDesignator = 'E';
	else if((-64 > Lat) && (Lat >= -72)) LetterDesignator = 'D';
	else if((-72 > Lat) && (Lat >= -80)) LetterDesignator = 'C';
	else LetterDesignator = 'Z'; //This is here as an error flag to show that the Latitude is outside the UTM limits

	return LetterDesignator;
}
