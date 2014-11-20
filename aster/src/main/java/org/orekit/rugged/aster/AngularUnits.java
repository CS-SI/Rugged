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
package org.orekit.rugged.aster;

import org.apache.commons.math3.util.FastMath;

/** Enumerate for angular units.
 * @see <a href="http://www.remotesensing.org/geotiff/spec/geotiff6.html#6.3.1.4">GeoTIFF specification, section 6.3.1.4</a>
 * @author Luc Maisonobe
 */
enum AngularUnits {

    // CHECKSTYLE: stop JavadocVariable check
    RADIAN(9101) {
        /** {@inheritDoc} */
        @Override
        public double toRadians(final double raw) {
            return raw;
        }
    },

    DEGREE(9102) {
        /** {@inheritDoc} */
        @Override
        public double toRadians(final double raw) {
            return FastMath.toRadians(raw);
        }
    },

    ARC_MINUTE(9103) {
        /** {@inheritDoc} */
        @Override
        public double toRadians(final double raw) {
            return FastMath.toRadians(raw / 60.0);
        }
    },

    ARC_SECOND(9104) {
        /** {@inheritDoc} */
        @Override
        public double toRadians(final double raw) {
            return FastMath.toRadians(raw / 3600.0);
        }
    },

    GRAD(9105) {
        /** {@inheritDoc} */
        @Override
        public double toRadians(final double raw) {
            return FastMath.PI * raw / 200.0;
        }
    },

    GON(9106) {
        /** {@inheritDoc} */
        @Override
        public double toRadians(final double raw) {
            return GRAD.toRadians(raw);
        }
    },

    DMS(9107) {
        /** {@inheritDoc} */
        @Override
        public double toRadians(final double raw) {
            throw new UnsupportedOperationException();
        }
    },

    DMS_HEMISPHERE(9108) {
        /** {@inheritDoc} */
        @Override
        public double toRadians(final double raw) {
            throw new UnsupportedOperationException();
        }
    };

    // CHECKSTYLE: resume JavadocVariable check

    /** Units ID. */
    private final int id;

    /** Simple constructor.
     * @param id key id
     */
    private AngularUnits(final int id) {
        this.id = id;
    }

    /** Convert an angle to radians.
     * @param raw angle, in instance units
     * @return angle in radians
     */
    public abstract double toRadians(final double raw);

    /** Get the units corresponding to an id.
     * @param id type id
     * @return the units corresponding to the id
     * @exception IllegalArgumentException if the id does not correspond to known units
     */
    public static AngularUnits getUnits(final int id) {
        for (AngularUnits units : values()) {
            if (units.id == id) {
                return units;
            }
        }
        throw new IllegalArgumentException();
    }

}
