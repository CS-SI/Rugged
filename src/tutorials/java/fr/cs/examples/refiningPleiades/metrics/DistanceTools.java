/* Copyright 2013-2020 CS GROUP
 * Licensed to CS GROUP (CS) under one or more
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
package fr.cs.examples.refiningPleiades.metrics;

import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.util.FastMath;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.utils.Constants;

/**
 * Class for computing geodetic and anguler distance between two geodetic points.
 * @author Jonathan Guinet
 * @author Guylaine Prat
 * @since 2.0
 */
public class DistanceTools {

    /** Private constructor for utility class.
     */
    private DistanceTools() {
    }

    /** Choice the method for computing geodetic distance between two points.
     * @param geoPoint1 first geodetic point (rad)
     * @param geoPoint2 second geodetic point (rad)
     * @param computeAngular if true, distance will be angular, otherwise will be in meters
     * @return distance in meters or radians if flag computeAngular is true
     */
    public static double computeDistance(final GeodeticPoint geoPoint1, final GeodeticPoint geoPoint2,
                                         final boolean computeAngular) {

        if (computeAngular == true) {
            return computeDistanceAngular(geoPoint1, geoPoint2);
        } else {
            return computeDistanceInMeter(geoPoint1, geoPoint2);
        }
    }

    /** Compute a geodetic distance in meters between two geodetic points..
     * @param geoPoint1 first geodetic point (rad)
     * @param geoPoint2 second geodetic point (rad)
     * @return distance in meters
     */
    public static double computeDistanceInMeter(final GeodeticPoint geoPoint1, final GeodeticPoint geoPoint2) {

        // get vectors on unit sphere from angular coordinates
        final Vector3D p1 = new Vector3D(geoPoint1.getLatitude(), geoPoint1.getLongitude()); //
        final Vector3D p2 = new Vector3D(geoPoint2.getLatitude(), geoPoint2.getLongitude());
        final double distance =  Constants.WGS84_EARTH_EQUATORIAL_RADIUS * Vector3D.angle(p1, p2);
        return distance;
    }

    /** Compute an angular distance between two geodetic points.
     * @param geoPoint1 first geodetic point (rad)
     * @param geoPoint2 second geodetic point (rad)
     * @return angular distance in radians
     */
    public static double computeDistanceAngular(final GeodeticPoint geoPoint1, final GeodeticPoint geoPoint2) {

        final double lonDiff = geoPoint1.getLongitude() - geoPoint2.getLongitude();
        final double latDiff = geoPoint1.getLatitude() - geoPoint2.getLatitude();
        return FastMath.hypot(lonDiff, latDiff);
    }
}
