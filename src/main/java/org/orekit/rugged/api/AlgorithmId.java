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


/** Enumerate for Digital Elevation Model intersection.
 * @author Luc Maisonobe
 */
public enum AlgorithmId {

    /** Fast algorithm due to Bernardt Duvenhage.
     * <p>
     * The algorithm is described in the 2009 paper:
     * <a href="http://researchspace.csir.co.za/dspace/bitstream/10204/3041/1/Duvenhage_2009.pdf">Using
     * An Implicit Min/Max KD-Tree for Doing Efficient Terrain Line of Sight Calculations</a>.
     * </p>
     */
    DUVENHAGE,

    /** Fast algorithm due to Bernardt Duvenhage.
     * <p>
     * The algorithm is described in the 2009 paper:
     * <a href="http://researchspace.csir.co.za/dspace/bitstream/10204/3041/1/Duvenhage_2009.pdf">Using
     * An Implicit Min/Max KD-Tree for Doing Efficient Terrain Line of Sight Calculations</a>.
     * </p>
     * <p>
     * This version of the duvenhage's algorithm considers the body to be flat, i.e. lines computed
     * from entry/exit points in the Digital Elevation Model are considered to be straight lines
     * also in geodetic coordinates. The sagitta resulting from real ellipsoid curvature is therefore
     * <em>not</em> corrected in this case. As this computation is not costly (a few percents overhead),
     * the full {@link #DUVENHAGE} is recommended instead of this one. This choice is mainly intended
     * for comparison purposes with other systems.
     * </p>
     */
    DUVENHAGE_FLAT_BODY,

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
