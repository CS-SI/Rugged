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

/** Main interface to Rugged library.
 * @author Luc Maisonobe
 */
public interface Rugged {

    /** Set up the tiles management.
     * @param updater updater used to load Digital Elevation Model tiles
     * @param maxCachedTiles maximum number of tiles stored in the cache
     */
    void setUpTilesManagement(TileUpdater updater, int maxCachedTiles);

    /** Direct localization of a sensor line.
     * @param sensorName name of the sensor
     * @param lineNumber number of the line to localize on ground
     * @param algorithm algorithm to use for line-of-sight/DEM intersection computation
     * @return ground position of all pixels of the specified sensor line
     * @exception RuggedException if line cannot be localized
     */
    GroundPoint[] directLocalization(String sensorName, int lineNumber,
                                     LineOfSightAlgorithm algorithm)
        throws RuggedException;

    /** Inverse localization of a ground point.
     * @param sensorName name of the sensor
     * @param ground point to localize
     * @param algorithm algorithm to use for line-of-sight/DEM intersection computation
     * @return sensor pixel seeing ground point
     * @exception RuggedException if line cannot be localized
     */
    SensorPixel inverseLocalization(String sensorName, GroundPoint groundPoint,
                                    LineOfSightAlgorithm algorithm)
        throws RuggedException;

}
