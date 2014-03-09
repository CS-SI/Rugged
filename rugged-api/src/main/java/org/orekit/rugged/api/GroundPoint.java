/* Copyright 2013-2014 CS Systèmes d'Information
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
package org.orekit.rugged.api;

import java.io.Serializable;

import org.apache.commons.math3.util.FastMath;
import org.apache.commons.math3.util.MathUtils;

/** Container for ground point.
 * <p>
 * This class is a stripped-down version of Orekit {@link org.orekit.bodies.GeodeticPoint},
 * which is distributed under the terms of the Apache License V2.
 * </p>
 * <p>
 * Instances of this class are guaranteed to be immutable.
 * </p>
 * @author Luc Maisonobe
 */
public class GroundPoint implements Serializable {

    /** Serializable UID. */
    private static final long serialVersionUID = 20140309L;

    /** Latitude of the point (rad). */
    private final double latitude;

    /** Longitude of the point (rad). */
    private final double longitude;

    /** Altitude of the point (m). */
    private final double altitude;

    /**
     * Build a new instance. The angular coordinates will be normalized so that
     * the latitude is between ±π/2 and the longitude is between ±π.
     *
     * @param latitude latitude of the point
     * @param longitude longitude of the point
     * @param altitude altitude of the point
     */
    public GroundPoint(final double latitude, final double longitude, final double altitude) {
        double lat = MathUtils.normalizeAngle(latitude, FastMath.PI / 2);
        double lon = MathUtils.normalizeAngle(longitude, 0);
        if (lat > FastMath.PI / 2.0) {
            // latitude is beyond the pole -> add 180 to longitude
            lat = FastMath.PI - lat;
            lon = MathUtils.normalizeAngle(longitude + FastMath.PI, 0);

        }
        this.latitude  = lat;
        this.longitude = lon;
        this.altitude  = altitude;
    }

    /** Get the latitude.
     * @return latitude, an angular value in the range [-π/2, π/2]
     */
    public double getLatitude() {
        return latitude;
    }

    /** Get the longitude.
     * @return longitude, an angular value in the range [-π, π]
     */
    public double getLongitude() {
        return longitude;
    }

    /** Get the altitude.
     * @return altitude
     */
    public double getAltitude() {
        return altitude;
    }

}
