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

import java.io.File;
import java.util.List;

/** Main interface to Rugged library.
 * @author Luc Maisonobe
 */
public interface Rugged {

    /** Enumerate for ellipsoid. */
    enum Ellipsoid {
        GRS80, WGS84, IERS96, IERS2003
    }

    /** Enumerate for inertial frames. */
    enum InertialFrame {
        GCRF, EME2000, MOD, TOD, VEIS1950
    }

    /** Enumerate for body rotating frames. */
    enum BodyRotatingFrame {
        ITRF, GTOD
    }

    /** Set up general context.
     * <p>
     * This method is the first one that must be called, otherwise the
     * other methods will fail due to uninitialized context.
     * </p>
     * @param orekitDataDir top directory for Orekit data
     * @param referenceDate reference date from which all other dates
     * are computed
     * @param ellipsoid reference ellipsoid
     * @param inertialFrameName inertial frame
     * @param bodyRotatingFrame body rotating frame
     * @param positionsVelocities satellite position and velocity
     * @param pvInterpolationOrder order to use for position/velocity interpolation
     * @param quaternions satellite quaternions
     * @param aInterpolationOrder order to use for attitude interpolation
     * @exception RuggedException if data needed for some frame cannot be loaded
     */
    void setGeneralContext(File orekitDataDir, String referenceDate, Ellipsoid ellipsoid,
                           InertialFrame inertialFrame, BodyRotatingFrame bodyRotatingFrame,
                           List<SatellitePV> positionsVelocities, int pvInterpolationOrder,
                           List<SatelliteQ> quaternions, int aInterpolationOrder)
        throws RuggedException;

    /** Set up the tiles management.
     * @param updater updater used to load Digital Elevation Model tiles
     * @param maxCachedTiles maximum number of tiles stored in the cache
     */
    void setUpTilesManagement(TileUpdater updater, int maxCachedTiles);

    /** Set up sensor model.
     * @param sensorName name of the sensor.
     * @param linesOfSigth lines of sight for each pixels
     * @param datationModel model to use for dating sensor lines
     */
    void setSensor(String sensorName, List<PixelLOS> linesOfSigth, LineDatation datationModel);

    /** Direct localization of a sensor line.
     * @param sensorName name of the sensor
     * @param lineNumber number of the line to localize on ground
     * @return ground position of all pixels of the specified sensor line
     * @exception RuggedException if line cannot be localized,
     * if {@link #setGeneralContext(File, InertialFrame, BodyRotatingFrame, Ellipsoid)} has
     * not been called beforehand, or if {@link #setOrbitAndAttitude(List, List)} has not
     * been called beforehand, or sensor is unknown
     */
    GroundPoint[] directLocalization(String sensorName, int lineNumber)
        throws RuggedException;

    /** Inverse localization of a ground point.
     * @param sensorName name of the sensor
     * @param ground point to localize
     * @return sensor pixel seeing ground point
     * @exception RuggedException if line cannot be localized,
     * if {@link #setGeneralContext(File, InertialFrame, BodyRotatingFrame, Ellipsoid)} has
     * not been called beforehand, or if {@link #setOrbitAndAttitude(List, List)} has not
     * been called beforehand, or sensor is unknown
     */
    SensorPixel inverseLocalization(String sensorName, GroundPoint groundPoint)
        throws RuggedException;

}
