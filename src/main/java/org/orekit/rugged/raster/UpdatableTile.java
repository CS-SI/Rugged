/* Copyright 2013-2025 CS GROUP
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
package org.orekit.rugged.raster;

/** Interface representing one tile of a raster Digital Elevation Model.
 * @author Luc Maisonobe
 * @author Guylaine Prat
 */
public interface UpdatableTile {

    /** Set the tile global geometry.
     * @param minLatitude minimum latitude (rad)
     * @param minLongitude minimum longitude (rad)
     * @param latitudeStep step in latitude (size of one raster element) (rad)
     * @param longitudeStep step in longitude (size of one raster element) (rad)
     * @param latitudeRows number of latitude rows
     * @param longitudeColumns number of longitude columns
     */
    void setGeometry(double minLatitude, double minLongitude,
                     double latitudeStep, double longitudeStep,
                     int latitudeRows, int longitudeColumns);

    /** Set the elevation for one raster element.
     * <p>
     * BEWARE! The order of the indices follows geodetic conventions, i.e.
     * the latitude is given first and longitude afterwards, so the first
     * index specifies a <em>row</em> index with zero at South and max value
     * at North, and the second index specifies a <em>column</em> index
     * with zero at West and max value at East. This is <em>not</em> the
     * same as some raster conventions (as our row index increases from South
     * to North) and this is also not the same as Cartesian coordinates as
     * our ordinate index appears before our abscissa index).
     * </p>
     * @param latitudeIndex index of latitude (row index)
     * @param longitudeIndex index of longitude (column index)
     * @param elevation elevation (m)
     */
    void setElevation(int latitudeIndex, int longitudeIndex, double elevation);

}
