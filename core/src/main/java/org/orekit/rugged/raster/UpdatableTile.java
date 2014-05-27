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
package org.orekit.rugged.raster;

import org.orekit.rugged.api.RuggedException;

/** Interface representing one tile of a raster Digital Elevation Model.
 * @author Luc Maisonobe
 */
public interface UpdatableTile {

    /** Set the tile global geometry.
     * @param minLatitude minimum latitude
     * @param minLongitude minimum longitude
     * @param latitudeStep step in latitude (size of one raster element)
     * @param longitudeStep step in longitude (size of one raster element)
     * @param latitudeRows number of latitude rows
     * @param longitudeColumns number of longitude columns
     * @exception RuggedException if tile is empty (zero rows or columns)
     */
    void setGeometry(double minLatitude, double minLongitude,
                     double latitudeStep, double longitudeStep,
                     int latitudeRows, int longitudeColumns)
        throws RuggedException;

    /** Set the elevation for one raster element.
     * @param latitudeIndex index of latitude (row index)
     * @param longitudeIndex index of longitude (column index)
     * @param elevation elevation (m)
     * @exception RuggedException if indices are out of bound
     */
    void setElevation(int latitudeIndex, int longitudeIndex, double elevation)
        throws RuggedException;

}
