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

        /** Constant for GRS 80 ellipsoid. */
        GRS80,

        /** Constant for WGS 84 ellipsoid. */
        WGS84,

        /** Constant for IERS 96 ellipsoid. */
        IERS96,

        /** Constant for IERS 2003 ellipsoid. */
        IERS2003

    }

    /** Enumerate for inertial frames. */
    enum InertialFrame {

        /** Constant for Geocentric Celestial Reference Frame. */
        GCRF,

        /** Constant for Earth Mean Equator 2000 frame (aka J2000). */
        EME2000,

        /** Constant for Mean Of Date frame, with IERS 96 conventions (Lieske precession). */
        MOD,

        /** Constant for True Of Date frame, with IERS 96 conventions (Wahr nutation). */
        TOD,

        /** Constant for Veis 1950 frame. */
        VEIS1950

    }

    /** Enumerate for body rotating frames. */
    enum BodyRotatingFrame {

        /** Constant for International Terrestrial Reference Frame. */
        ITRF,

        /** Constant for Geocentric True Of Date frame. */
        GTOD

    }

    /** Enumerate for Digital Elevation Model intersection. */
    enum Algorithm {

        /** Fast algorithm due to Bernardt Duvenhage.
         * <p>
         * The algorithm is described in the 2009 paper:
         * <a href="http://researchspace.csir.co.za/dspace/bitstream/10204/3041/1/Duvenhage_2009.pdf">Using
         * An Implicit Min/Max KD-Tree for Doing Efficient Terrain Line of Sight Calculations</a>.
         * </p>
         */
        DUVENHAGE,

        /** Basic, <em>very slow</em> algorithm, designed only for tests and validation purposes.
         * <p>
         * The algorithm simply computes entry and exit points at high and low altitudes,
         * and scans all Digital Elevation Models in the sub-tiles defined by these two
         * corner points. It is not designed for operational use.
         * </p>
         */
        BASIC_SLOW_EXHAUSTIVE_SCAN_FOR_TESTS_ONLY,

        /** Dummy algorithm that simply ignores the Digital Elevation Model.
         * <p>
         * Intersections are computed only with respect to the reference ellipsoid.
         * </p>
         */
        IGNORE_DEM_USE_ELLIPSOID

    }

    /** Set up general context.
     * <p>
     * This method is the first one that must be called, otherwise the
     * other methods will fail due to uninitialized context.
     * </p>
     * @param orekitDataDir top directory for Orekit data
     * @param referenceDate reference date from which all other dates are computed
     * @param algorithmID identifier of algorithm to use for Digital Elevation Model intersection
     * @param ellipsoidID identifier of reference ellipsoid
     * @param inertialFrameID identifier of inertial frame
     * @param bodyRotatingFrameID identifier of body rotating frame
     * @param positionsVelocities satellite position and velocity
     * @param pvInterpolationOrder order to use for position/velocity interpolation
     * @param quaternions satellite quaternions
     * @param aInterpolationOrder order to use for attitude interpolation
     * @exception RuggedException if data needed for some frame cannot be loaded
     */
    void setGeneralContext(File orekitDataDir, String referenceDate,
                           Algorithm algorithmID, Ellipsoid ellipsoidID,
                           InertialFrame inertialFrameID, BodyRotatingFrame bodyRotatingFrameID,
                           List<SatellitePV> positionsVelocities, int pvInterpolationOrder,
                           List<SatelliteQ> quaternions, int aInterpolationOrder)
        throws RuggedException;

    /** Set up the tiles management.
     * @param updater updater used to load Digital Elevation Model tiles
     * @param maxCachedTiles maximum number of tiles stored in the cache
     */
    void setUpTilesManagement(TileUpdater updater, int maxCachedTiles);

    /** Set up line sensor model.
     * @param name name of the line sensor.
     * @param pixels lines of sight for each pixels
     * @param datationModel model to use for dating sensor lines
     */
    void setLineSensor(String name, List<PixelLOS> pixels, LineDatation datationModel);

    /** Direct localization of a sensor line.
     * @param name name of the line sensor
     * @param lineNumber number of the line to localize on ground
     * @return ground position of all pixels of the specified sensor line
     * @exception RuggedException if line cannot be localized,
     * if {@link #setGeneralContext(File, InertialFrame, BodyRotatingFrame, Ellipsoid)} has
     * not been called beforehand, or if {@link #setOrbitAndAttitude(List, List)} has not
     * been called beforehand, or sensor is unknown
     */
    GroundPoint[] directLocalization(String name, double lineNumber)
        throws RuggedException;

    /** Inverse localization of a ground point.
     * @param name name of the line  sensor
     * @param groundPoint ground point to localize
     * @return sensor pixel seeing ground point
     * @exception RuggedException if line cannot be localized,
     * if {@link #setGeneralContext(File, InertialFrame, BodyRotatingFrame, Ellipsoid)} has
     * not been called beforehand, or if {@link #setOrbitAndAttitude(List, List)} has not
     * been called beforehand, or sensor is unknown
     */
    SensorPixel inverseLocalization(String name, GroundPoint groundPoint)
        throws RuggedException;

}
