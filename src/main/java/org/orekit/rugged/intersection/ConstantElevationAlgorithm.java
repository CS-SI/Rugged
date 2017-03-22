/* Copyright 2013-2017 CS Systèmes d'Information
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
import org.orekit.errors.OrekitException;
import org.orekit.rugged.api.AlgorithmId;
import org.orekit.rugged.errors.DumpManager;
import org.orekit.rugged.errors.RuggedException;
import org.orekit.rugged.utils.ExtendedEllipsoid;
import org.orekit.rugged.utils.NormalizedGeodeticPoint;

/** Intersection ignoring Digital Elevation Model.
 * <p>
 * This dummy implementation simply uses the ellipsoid itself.
 * </p>
 * @author Luc Maisonobe
 */
public class ConstantElevationAlgorithm implements IntersectionAlgorithm {

    /** Constant elevation over ellipsoid. */
    private final double constantElevation;

    /** Simple constructor.
     * @param constantElevation constant elevation over ellipsoid
     */
    public ConstantElevationAlgorithm(final double constantElevation) {
        this.constantElevation = constantElevation;
    }

    /** {@inheritDoc} */
    @Override
    public NormalizedGeodeticPoint intersection(final ExtendedEllipsoid ellipsoid,
                                                final Vector3D position, final Vector3D los)
        throws RuggedException {
        try {
            DumpManager.dumpAlgorithm(AlgorithmId.CONSTANT_ELEVATION_OVER_ELLIPSOID, constantElevation);
            final Vector3D      p  = ellipsoid.pointAtAltitude(position, los, constantElevation);
            final GeodeticPoint gp = ellipsoid.transform(p, ellipsoid.getFrame(), null);
            return new NormalizedGeodeticPoint(gp.getLatitude(), gp.getLongitude(), gp.getAltitude(), 0.0);
        } catch (OrekitException oe) {
            throw new RuggedException(oe, oe.getSpecifier(), oe.getParts());
        }
    }

    /** {@inheritDoc} */
    @Override
    public NormalizedGeodeticPoint refineIntersection(final ExtendedEllipsoid ellipsoid,
                                                      final Vector3D position, final Vector3D los,
                                                      final NormalizedGeodeticPoint closeGuess)
        throws RuggedException {
        try {
            DumpManager.dumpAlgorithm(AlgorithmId.CONSTANT_ELEVATION_OVER_ELLIPSOID, constantElevation);
            final Vector3D      p  = ellipsoid.pointAtAltitude(position, los, constantElevation);
            final GeodeticPoint gp = ellipsoid.transform(p, ellipsoid.getFrame(), null);
            return new NormalizedGeodeticPoint(gp.getLatitude(), gp.getLongitude(), gp.getAltitude(),
                                               closeGuess.getLongitude());
        } catch (OrekitException oe) {
            throw new RuggedException(oe, oe.getSpecifier(), oe.getParts());
        }
    }

    /** {@inheritDoc}
     * <p>
     * As this algorithm uses a constant elevation,
     * this method always returns the same value.
     * </p>
     */
    @Override
    public double getElevation(final double latitude, final double longitude) {
        DumpManager.dumpAlgorithm(AlgorithmId.CONSTANT_ELEVATION_OVER_ELLIPSOID, constantElevation);
        return constantElevation;
    }

}
