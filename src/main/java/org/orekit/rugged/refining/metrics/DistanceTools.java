/* Copyright 2013-2017 CS Systèmes d'Information
 * Licensed to CS Systèmes d'Information (CS) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.
 * CS licenses this file to You under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with
 * the License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package org.orekit.rugged.refining.metrics;

import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.util.FastMath;

/**
 * Class for computing geodetic distance.
 * TODO GP voir avec Luc si pas d'outil dans orekit ???
 * @author Jonathan Guinet
 * @author Guylaine Prat
 * @since 2.0
 */
public class DistanceTools {

    /** Earth radius in cms.  */
	// TODO GP constant for earth ???
    public static final double EARTH_RADIUS = 637100000d;

    /** Private constructor for utility class.
     */
    private DistanceTools() {
    }

    /** Choice the method for computing geodetic distance between two points.
     * @param long1 Longitude of first geodetic point (rad)
     * @param lat1 Latitude of first geodetic point (rad)
     * @param long2 Longitude of second geodetic point (rad)
     * @param lat2 Latitude of second geodetic point (rad)
     * @param computeAngular if true, distance will be angular
     * @return distance in meters or radians if flag computeAngular is true
     */
    public static double computeDistance(final double long1, final double lat1,
                                         final double long2, final double lat2,
                                         final boolean computeAngular) {

        if (computeAngular == true) {
            return computeDistanceAngular(long1, lat1, long2, lat2);
        } else {
            return computeDistanceInMeter(long1, lat1, long2, lat2);
        }
    }

    /** Compute a geodetic distance in meters between point (long1, lat1) and point (long2, lat2).
     * @param long1 Longitude of first geodetic point (rad)
     * @param lat1 Latitude of first geodetic point (rad)
     * @param long2 Longitude of second geodetic point (rad)
     * @param lat2 Latitude of second geodetic point (rad)
     * @return distance in meters
     */
    public static double computeDistanceInMeter(final double long1, final double lat1,
                                                final double long2, final double lat2) {
    	
        // get vectors on unit sphere from angular coordinates
        final Vector3D p1 = new Vector3D(lat1, long1); //
        final Vector3D p2 = new Vector3D(lat2, long2);

        final double distance = EARTH_RADIUS / 100 * Vector3D.angle(p1, p2);
        return distance;
    }

    /** Compute an angular distance between two geodetic points.
     * @param long1 Longitude of first geodetic point (rad)
     * @param lat1 Latitude of first geodetic point (rad)
     * @param long2 Longitude of second geodetic point (rad)
     * @param lat2 Latitude of second geodetic point (rad)
     * @return angular distance in radians
     */
    public static double computeDistanceAngular(final double long1, final double lat1,
                                                final double long2, final double lat2) {

        final double lonDiff = long1 - long2;
        final double latDiff = lat1 - lat2;
        return FastMath.hypot(lonDiff, latDiff);
    }
}
