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


/** Enumerate for GeoTIFF keys.
 * @see <a href="http://www.remotesensing.org/geotiff/spec/geotiff6.html#6.2">GeoTIFF specification, section 6.2</a>
 * @author Luc Maisonobe
 */
enum GeoKey {

    // CHECKSTYLE: stop JavadocVariable check
    GT_MODEL_TYPE(1024),
    GT_RASTER_TYPE(1025),
    GT_CITATION(1026),

    GEOGRAPHIC_TYPE(2048),
    GEOG_CITATION(2049),
    GEOG_GEODETICDATUM(2050),
    GEOG_PRIME_MERIDIAN(2051),
    GEOG_PRIME_MERIDIAN_LONG(2061),
    GEOG_LINEAR_UNITS(2052),
    GEOG_LINEAR_UNIT_SIZE(2053),
    GEOG_ANGULAR_UNITS(2054),
    GEOG_ANGULAR_UNIT_SIZE(2055),
    GEOG_ELLIPSOID(2056),
    GEOG_SEMI_MAJOR_AXIS(2057),
    GEOG_SEMI_MINOR_AXIS(2058),
    GEOG_INV_FLATTENING(2059),
    GEOG_AZIMUTH_UNITS(2060),

    PROJECTED_CS_TYPE(3072),
    PCS_CITATION(3073),
    PROJECTION(3074),
    PROJ_COORD_TRANS(3075),
    PROJ_LINEAR_UNITS(3076),
    PROJ_LINEAR_UNIT_SIZE(3077),
    PROJ_STD_PARALLEL_1(3078),
    PROJ_STD_PARALLEL_2(3079),
    PROJ_NAT_ORIGIN_LONG(3080),
    PROJ_NAT_ORIGIN_LAT(3081),
    PROJ_FALSE_EASTING(3082),
    PROJ_FALSE_NORTHING(3083),
    PROJ_FALSE_ORIGIN_LONG(3084),
    PROJ_FALSE_ORIGIN_LAT(3085),
    PROJ_FALSE_ORIGIN_EASTING(3086),
    PROJ_FALSE_ORIGIN_NORTHING(3087),
    PROJ_CENTER_LONG(3088),
    PROJ_CENTER_LAT(3089),
    PROJ_CENTER_EASTING(3090),
    PROJ_FALSE_ORIGIN_NORTHING_ALTERNATE(3091),
    PROJ_SCALE_AT_NAT_ORIGIN(3092),
    PROJ_SCALE_AT_CENTER(3093),
    PROJ_AZIMUTH_ANGLE(3094),
    PROJ_STRAIGHT_VERT_POLE_LONG(3095),

    VERTICAL_CS_TYPE(4096),
    VERTICAL_CITATION(4097),
    VERTICAL_DATUM(4098),
    VERTICAL_UNITS(4099);
    // CHECKSTYLE: resume JavadocVariable check

    /** Key ID. */
    private final int id;

    /** Simple constructor.
     * @param id key id
     */
    private GeoKey(final int id) {
        this.id   = id;
    }

    /** Get the key corresponding to an id.
     * @param id key id
     * @return the key corresponding to the id
     * @exception IllegalArgumentException if the id does not correspond to a known key
     */
    public static GeoKey getKey(final int id) {
        for (GeoKey key : values()) {
            if (key.id == id) {
                return key;
            }
        }
        throw new IllegalArgumentException();
    }

}
