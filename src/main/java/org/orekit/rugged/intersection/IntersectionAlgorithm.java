/* Copyright 2013-2019 CS Systèmes d'Information
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
package org.orekit.rugged.intersection;

import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.rugged.utils.ExtendedEllipsoid;
import org.orekit.rugged.utils.NormalizedGeodeticPoint;

/** Interface for Digital Elevation Model intersection algorithm.
 * @author Luc Maisonobe
 */
public interface IntersectionAlgorithm {

    /** Compute intersection of line with Digital Elevation Model.
     * @param ellipsoid reference ellipsoid
     * @param position pixel position in ellipsoid frame
     * @param los pixel line-of-sight in ellipsoid frame
     * @return point at which the line first enters ground
     */
    NormalizedGeodeticPoint intersection(ExtendedEllipsoid ellipsoid, Vector3D position, Vector3D los);

    /** Refine intersection of line with Digital Elevation Model.
     * <p>
     * This method is used to refine an intersection when a close guess is
     * already known. The intersection is typically looked for by a direct
     * {@link org.orekit.rugged.raster.Tile#cellIntersection(GeodeticPoint,
     * Vector3D, int, int) cell intersection} in the tile which already
     * contains the close guess, or any similar very fast algorithm.
     * </p>
     * @param ellipsoid reference ellipsoid
     * @param position pixel position in ellipsoid frame
     * @param los pixel line-of-sight in ellipsoid frame
     * @param closeGuess guess close to the real intersection
     * @return point at which the line first enters ground
     */
    NormalizedGeodeticPoint refineIntersection(ExtendedEllipsoid ellipsoid, Vector3D position, Vector3D los,
                                               NormalizedGeodeticPoint closeGuess);

    /** Get elevation at a given ground point.
     * @param latitude ground point latitude
     * @param longitude ground point longitude
     * @return elevation at specified point
     */
    double getElevation(double latitude, double longitude);

}
