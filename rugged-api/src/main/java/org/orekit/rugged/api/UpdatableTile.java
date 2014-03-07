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

/** Interface representing one tile of a raster Digital Elevation Model.
 * @author Luc Maisonobe
 */
public interface UpdatableTile {

    /** Set the tile global geometry.
     * @param referenceLatitude reference latitude
     * @param referenceLongitude reference longitude
     * @param latitudeStep step in latitude (size of one raster element)
     * @param longitudeStep step in longitude (size of one raster element)
     * @param latitudeRows number of latitude rows
     * @param longitudeColumns number of longitude columns
     */
    void setGeometry(double referenceLatitude, double referenceLongitude,
                     double latitudeStep, double longitudeStep,
                     int latitudeRows, int longitudeColumns);

    /** Set the elevation for one raster element.
     * @param latitudeIndex index of latitude (row index)
     * @param longitudeIndex index of longitude (column index)
     * @param elevation elevation (m)
     * @exception IllegalArgumentException if indices are out of bound
     */
    void setElevation(int latitudeIndex, int longitudeIndex, double elevation)
        throws IllegalArgumentException;

}
