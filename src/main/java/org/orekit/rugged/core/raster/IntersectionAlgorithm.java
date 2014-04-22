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
package org.orekit.rugged.core.raster;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.rugged.api.RuggedException;
import org.orekit.rugged.core.ExtendedEllipsoid;

/** Interface for Digital Elevation Model intersection algorithm.
 * @author Luc Maisonobe
 */
public interface IntersectionAlgorithm {

    /** Compute intersection of line with Digital Elevation Model.
     * @param ellipsoid reference ellipsoid
     * @param position pixel position in ellipsoid frame
     * @param los pixel line-of-sight in ellipsoid frame
     * @return point at which the line first enters ground
     * @exception RuggedException if intersection cannot be found
     */
    GeodeticPoint intersection(ExtendedEllipsoid ellipsoid, Vector3D position, Vector3D los)
        throws RuggedException;

}
