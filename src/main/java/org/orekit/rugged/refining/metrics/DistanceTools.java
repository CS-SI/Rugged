package org.orekit.rugged.refining.metrics;

import org.hipparchus.geometry.euclidean.threed.Vector3D;

/**
 * class for computing geodetic distance
 * @author Jonathan Guinet
 */
public class DistanceTools {

	/**
	 * Earth radius in cms
	 */
	public static final double EARTH_RADIUS = 637100000d;

	public DistanceTools() {

	}

	/** Choice the method for computing geodetic distance between two points
     * @param xRad1 Longitude of first geodetic point
     * @param xRad2 Latitude of first geodetic point
     * @param yRad1 Longitude of second geodetic point
     * @param yRad2 Latitude of second geodetic point
     * @return distance in meters
     */
     public static double computeDistance(double xRad1, double yRad1, double xRad2, double yRad2, boolean computeAngular) {

         double distance=0.0;
         if (computeAngular == true) {
             distance = computeDistanceAngular(xRad1, yRad1, xRad2, yRad2);
         } else {
             distance = computeDistanceInMeter(xRad1, yRad1, xRad2, yRad2);
         }
         return distance;
     }
     
     /** Compute a geodetic distance in meters between point (xRad1, yRad1) and point (xRad2, yRad2)
	 * @param xRad1 Longitude of first geodetic point
     * @param xRad2 Latitude of first geodetic point
     * @param yRad1 Longitude of second geodetic point
     * @param yRad2 Latitude of second geodetic point
     * @return distance in meters
	 */
	 public static double computeDistanceInMeter(double xRad1, double yRad1, double xRad2, double yRad2) {

		  // get vectors on unit sphere from angular coordinates
		  final Vector3D p1 = new Vector3D(yRad1, xRad1); // 
		  final Vector3D p2 = new Vector3D(yRad2, xRad2);

		  final double distance = EARTH_RADIUS /100* Vector3D.angle(p1, p2);
		  return distance;

	 }
	 
	 /** Compute an angular distance between two geodetic points
	  * @param xRad1 Longitude of first geodetic point
	  * @param xRad2 Latitude of first geodetic point
	  * @param yRad1 Longitude of second geodetic point
	  * @param yRad2 Latitude of second geodetic point
	  * @return distance in meters
	  */
	 public static double computeDistanceAngular(double xRad1, double yRad1, double xRad2, double yRad2) {

	     final double lonDiff = xRad1 - xRad2;
         final double latDiff = yRad1 - yRad2;
              
         final double distance = Math.sqrt(lonDiff * lonDiff + latDiff * latDiff);
         return distance;
	 }
	
	




}
