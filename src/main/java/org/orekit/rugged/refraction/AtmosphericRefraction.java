/* Copyright 2013-2016 CS Systèmes d'Information
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
package org.orekit.rugged.refraction;


import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.orekit.rugged.errors.RuggedException;
import org.orekit.rugged.intersection.IntersectionAlgorithm;
import org.orekit.rugged.utils.NormalizedGeodeticPoint;

/**
 * Interface for atmospheric refraction model.
 * @author Sergio Esteves
 */
public interface AtmosphericRefraction {

    /** Apply correction to the intersected point with an atmospheric refraction model.
     * @param satPos satellite position
     * @param satLos satellite line of sight
     * @param rawIntersection intersection point before refraction correction
     * @param algorithm intersection algorithm
     * @return corrected point with the effect of atmospheric refraction
     * @throws RuggedException if there is no refraction data at altitude of rawIntersection or see
     * {@link org.orekit.rugged.utils.ExtendedEllipsoid#pointAtAltitude(Vector3D, Vector3D, double)} or see
     * {@link IntersectionAlgorithm#refineIntersection(ExtendedEllipsoid, Vector3D, Vector3D, NormalizedGeodeticPoint)}
     */
    NormalizedGeodeticPoint applyCorrection(Vector3D satPos, Vector3D satLos, NormalizedGeodeticPoint rawIntersection,
                                             IntersectionAlgorithm algorithm)
        throws RuggedException;

}
