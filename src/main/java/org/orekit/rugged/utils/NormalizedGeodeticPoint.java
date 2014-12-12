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
package org.orekit.rugged.utils;

import org.apache.commons.math3.util.MathUtils;
import org.orekit.bodies.GeodeticPoint;

/** Geodetic point whose longitude can be selected with respect to the 2π boundary.
 * @author Luc Maisonobe
 */
public class NormalizedGeodeticPoint extends GeodeticPoint {

    /** Serializable UID. */
    private static final long serialVersionUID = 20141016l;

    /** Normalized longitude. */
    private final double normalizedLongitude;

    /**
     * Build a new instance. The angular coordinates will be normalized
     * to ensure     that the latitude is between ±π/2 and the longitude
     * is between lc-π and lc+π.
     *
     * @param latitude latitude of the point
     * @param longitude longitude of the point
     * @param altitude altitude of the point
     * @param centralLongitude central longitude lc
     */
    public NormalizedGeodeticPoint(final double latitude, final double longitude,
                                   final double altitude, final double centralLongitude) {
        super(latitude, longitude, altitude);
        this.normalizedLongitude = MathUtils.normalizeAngle(longitude, centralLongitude);
    }

    /** Get the longitude.
     * @return longitude, an angular value in the range [lc-π, lc+π],
     * where l₀ was selected at construction
     */
    @Override
    public double getLongitude() {
        return normalizedLongitude;
    }

    /** {@inheritDoc} */
    @Override
    public boolean equals(final Object object) {
        // we override the method just to make it clear that we INTENTIONALLY
        // consider normalized point are just similar to regular points
        return super.equals(object);
    }

    /** {@inheritDoc} */
    @Override
    public int hashCode() {
        // we override the method just to make it clear that we INTENTIONALLY
        // consider normalized point are just similar to regular points
        return super.hashCode();
    }

}
