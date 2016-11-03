/*
 * TurtleController.c
 *
 *  Created on: Oct 28, 2016
 *      Author: cbegian
 */

#include <stdio.h>
#include <stdbool.h>

#include "battery.h"
#include "nav_typedefs.h"
#include "nav_utilities.h"

#define	MISSION_FILE	"mission.txt"

int main( int argc, char **argv )
{
	/* Turtle Controller */

	Waypoint wpt;
	Waypoint home;
	int numberOfWPTs = 0;
	int currentWPT = 0;	/* first WPT is #0 */
	bool abortMission = false;
	float magVar = 0.0;
	double noCurrentSpeed; /* estimated speed when water is calm (no current)*/
	long batteryDuration = 0;
	int surfacingInterval;
	double latInDegrees;
	double lonInDegrees;
	Waypoint lastPosition;

#if 0
	latInDegrees = 27.954159;
	lonInDegrees = -82.529316;
	char dateTimeString[] = "121102141516.172";
	wpt = latLongToUTM( latInDegrees, lonInDegrees, dateTimeString );


	printf( "LAT = %lf, LON = %lf\n", wpt.utm_east, wpt.utm_north );
	printf( "Zone = %s, EASTING = %lf, NORTHING = %lf\n", wpt.utm_zone,
														wpt.utm_east, upt.utm_north );

	return(0);
#endif

	FILE *missionFilePtr = fopen( MISSION_FILE, "r" );

	/* First line of mission file is the magnetic variation. */
	(void)fscanf( missionFilePtr, "%f\n", &magVar );

	/* Second line is the estimated speed in a "no current" area. */
	(void)fscanf( missionFilePtr, "%lf\n", &noCurrentSpeed );

	/* Third line is the time interval between surfacing (in seconds). */
	(void)fscanf( missionFilePtr, "%d\n", &surfacingInterval );

	/* Fourth line is number of waypoints (WPTs) in file. */
	(void)fscanf( missionFilePtr, "%d\n", &numberOfWPTs );

	/**********
	 * Take the initial GPS reading. This is our launch point. This is where we
	 * will go when we finish the mission or abort- "Return To Home" (RTH).
	 */
	 home = readGPS();
	 lastPosition = home;

	 /****
	  * PSEUDOCODE
	  *
	  * The time on our first GPS reading can be used to set the current system
	  * time.
	  *
	  * set_pi_system_time( home.timestamp );
	  */

	 /******
	  * Get the initial amount of time remaining in the battery. This figure
	  * will decrease as the mission proceeds. Should we reach a time when
	  * proceeding on the next segment of the journey (i.e. proceeding to our
	  * next point of surfacing) would place us at a point where the time to
	  * RTH exceeds batteryDuration, we need to abort the mission, and RTH.
	  */
	 batteryDuration = batteryDurationRemaining();

	 while ( ( currentWPT < numberOfWPTs ) && ( !abortMission ) )
	 {
		 /* Read the next waypoint in the mission. */
		 (void)fscanf( missionFilePtr, "%lf %lf\n", &latInDegrees, &lonInDegrees );

		 /* convert to UTM. The time value is unused, so insert dummy value. */
		 wpt = latLongToUTM( latInDegrees, lonInDegrees, "120101000000.000" );

		 /* Go to the next WPT. Return boolean value indicates if we abort. */
		 abortMission = navigateToWPT( lastPosition, &wpt, &batteryDuration,
				 	 	 	 	 	   surfacingInterval );

		 /***********
		  * If we made it to the waypoint without aborting, increment the
		  * current waypoint counter.
		  */
		 if ( !abortMission )
		 {
			 ++currentWPT;
		 }
	 }

	 /***
	  * Mission is complete (or aborted). Return To Home.
	  * For now, ignore return value from this call.
	  */
	 (void) navigateToWPT( lastPosition, &home, &batteryDuration,
			 	 	 	 	 surfacingInterval );

	 return 0;
}
