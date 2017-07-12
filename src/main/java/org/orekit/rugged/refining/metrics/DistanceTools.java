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
 * class for computing geodetic distance.
 * @author Jonathan Guinet
 */
public class DistanceTools {

    /**
     * Earth radius in cms.
     */
    public static final double EARTH_RADIUS = 637100000d;

    /** Private constructor for utility class.
     */
    private DistanceTools() {
    }

    /** Choice the method for computing geodetic distance between two points.
     * @param xRad1 Longitude of first geodetic point
     * @param xRad2 Latitude of first geodetic point
     * @param yRad1 Longitude of second geodetic point
     * @param yRad2 Latitude of second geodetic point
     * @param computeAngular if true, distance will be angular
     * @return distance in meters
     */
    public static double computeDistance(final double xRad1, final double yRad1,
                                         final double xRad2, final double yRad2,
                                         final boolean computeAngular) {

        if (computeAngular == true) {
            return computeDistanceAngular(xRad1, yRad1, xRad2, yRad2);
        } else {
            return computeDistanceInMeter(xRad1, yRad1, xRad2, yRad2);
        }

    }

    /** Compute a geodetic distance in meters between point (xRad1, yRad1) and point (xRad2, yRad2).
     * @param xRad1 Longitude of first geodetic point
     * @param xRad2 Latitude of first geodetic point
     * @param yRad1 Longitude of second geodetic point
     * @param yRad2 Latitude of second geodetic point
     * @return distance in meters
     */
    public static double computeDistanceInMeter(final double xRad1, final double yRad1,
                                                final double xRad2, final double yRad2) {
        // get vectors on unit sphere from angular coordinates
        final Vector3D p1 = new Vector3D(yRad1, xRad1); //
        final Vector3D p2 = new Vector3D(yRad2, xRad2);

        final double distance = EARTH_RADIUS / 100 * Vector3D.angle(p1, p2);
        return distance;

    }

    /** Compute an angular distance between two geodetic points.
     * @param xRad1 Longitude of first geodetic point
     * @param xRad2 Latitude of first geodetic point
     * @param yRad1 Longitude of second geodetic point
     * @param yRad2 Latitude of second geodetic point
     * @return distance in meters
     */
    public static double computeDistanceAngular(final double xRad1, final double yRad1,
                                                final double xRad2, final double yRad2) {

        final double lonDiff = xRad1 - xRad2;
        final double latDiff = yRad1 - yRad2;

        return FastMath.hypot(lonDiff, latDiff);

    }

}
