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
package org.orekit.rugged.geotiff;

/** Enumerate for angular units.
 * @see <a href="http://www.remotesensing.org/geotiff/spec/geotiff6.html#6.3.1.4">GeoTIFF specification, section 6.3.1.4</a>
 * @author Luc Maisonobe
 */
enum AngulerUnits {

    // CHECKSTYLE: stop JavadocVariable check
    RADIAN(9101),
    DEGREE(9102),
    ARC_MINUTE(9103),
    ARC_SECOND(9104),
    GRAD(9105),
    GON(9106),
    DMS(9107),
    DMS_HEMISPHERE(9108);
    // CHECKSTYLE: resume JavadocVariable check

    /** Units ID. */
    private final int id;

    /** Simple constructor.
     * @param id key id
     */
    private AngulerUnits(final int id) {
        this.id = id;
    }

    /** Get the units corresponding to an id.
     * @param id type id
     * @return the units corresponding to the id
     * @exception IllegalArgumentException if the id does not correspond to known units
     */
    public static AngulerUnits getUnits(final int id) {
        for (AngulerUnits units : values()) {
            if (units.id == id) {
                return units;
            }
        }
        throw new IllegalArgumentException();
    }

}
