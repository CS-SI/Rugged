package org.orekit.rugged.refining.metrics;

import org.hipparchus.util.FastMath;
import org.hipparchus.geometry.euclidean.threed.Vector3D;

public class DistanceTools {

	/**
	 * Earth radius in cms
	 */
	public static final double EARTH_RADIUS = 637100000d;

	public DistanceTools() {

	}

	
	private static boolean checkDiff(double lon1, double lat1, double lon2, double lat2)
	{
	if(FastMath.abs(lon1 - lon2) < 1e-15 &&  FastMath.abs(lat1 - lat2) < 1e-15 )
		return false;
	return true;
	}
	/**
	 * Convert the given degrees value in meters
	 * 
	 * @param distanceDeg
	 * @return the given degrees value in meters
	 */
	public static double computeDistanceDeg(double lonDeg, double latDeg, double distanceDeg) {
		double lonDeg1 = lonDeg;
		double latDeg1 = latDeg;
		double lonDeg2 = lonDeg + distanceDeg;
		double latDeg2 = latDeg;
		return computeDistance(lonDeg1, latDeg1, lonDeg2, latDeg2);
	}

	/**
	 * Compute distance between point (lonDeg1, latDeg1) and point (lonDeg2,
	 * latDeg2) in meters
	 * 
	 * @param lonDeg1
	 * @param latDeg1
	 * @param lonDeg2
	 * @param latDeg2
	 * @return distance between point (lonDeg1, latDeg1) and point (lonDeg2,
	 *         latDeg2)
	 */
	public static double computeDistance(double lonDeg1, double latDeg1, double lonDeg2, double latDeg2) {
		double returned;
		if(!checkDiff(lonDeg1,latDeg1,lonDeg2,latDeg2) )
				return 0.0;
		//System.out.format("******** %n");
		//System.out.format("%8.14f %8.14f %n",lonDeg1,latDeg1);
		//System.out.format("%3.14e %n",lonDeg2-lonDeg1);
		//System.out.format("%8.14f %8.14f %n",lonDeg2,latDeg2);
		//System.out.format("%3.14e  %n",latDeg2-latDeg1);
		
		double xRad1 = FastMath.toRadians(lonDeg1);
		double xRad2 = FastMath.toRadians(lonDeg2);
		double yRad1 = FastMath.toRadians(latDeg1);
		double yRad2 = FastMath.toRadians(latDeg2);
		returned = computeDistanceRad(xRad1, yRad1, xRad2, yRad2);
		return returned;
	}

	/**
	 * distance in meters between point (xRad1, yRad1) and point (xRad2, yRad2)
	 * 
	 * @param xRad1
	 * @param xRad2
	 * @param yRad1
	 * @param yRad2
	 * @return
	 */
	  public static double computeDistanceRad(double xRad1, double yRad1, double xRad2, double yRad2) {

		  // get vectors on unit sphere from angular coordinates
		  Vector3D p1 = new Vector3D(yRad1, xRad1); // 
		  Vector3D p2 = new Vector3D(yRad2, xRad2);

		  return EARTH_RADIUS /100* Vector3D.angle(p1, p2);

	  	}
	
	
	




}
