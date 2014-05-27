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


/** Enumerate for raster types.
 * @see <a href="http://www.remotesensing.org/geotiff/spec/geotiff6.html#6.3.1.2">GeoTIFF specification, section 6.3.1.2</a>
 * @author Luc Maisonobe
 */
enum RasterType {

    // CHECKSTYLE: stop JavadocVariable check
    UNDEFINED(0),
    RASTER_PIXEL_IS_AREA(1),
    RASTER_PIXEL_IS_POINT(2);
    // CHECKSTYLE: resume JavadocVariable check

    /** Type ID. */
    private final int id;

    /** Simple constructor.
     * @param id key id
     */
    private RasterType(final int id) {
        this.id   = id;
    }

    /** Get the type corresponding to an id.
     * @param id type id
     * @return the type corresponding to the id
     * @exception IllegalArgumentException if the id does not correspond to a known type
     */
    public static RasterType getType(final int id) {
        for (RasterType type : values()) {
            if (type.id == id) {
                return type;
            }
        }
        throw new IllegalArgumentException();
    }

}
