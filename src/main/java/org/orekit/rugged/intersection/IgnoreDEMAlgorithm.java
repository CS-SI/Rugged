/* Copyright 2013-2020 CS GROUP
 * Licensed to CS GROUP (CS) under one or more
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
import org.orekit.rugged.api.AlgorithmId;
import org.orekit.rugged.errors.DumpManager;
import org.orekit.rugged.utils.ExtendedEllipsoid;
import org.orekit.rugged.utils.NormalizedGeodeticPoint;

/** Intersection ignoring Digital Elevation Model.
 * <p>
 * This dummy implementation simply uses the ellipsoid itself.
 * </p>
 * @author Luc Maisonobe
 */
public class IgnoreDEMAlgorithm implements IntersectionAlgorithm {

    /** Simple constructor.
     */
    public IgnoreDEMAlgorithm() {
    }

    /** {@inheritDoc} */
    @Override
    public NormalizedGeodeticPoint intersection(final ExtendedEllipsoid ellipsoid,
                                                final Vector3D position, final Vector3D los) {
        DumpManager.dumpAlgorithm(AlgorithmId.IGNORE_DEM_USE_ELLIPSOID);
        return ellipsoid.pointOnGround(position, los, 0.0);
    }

    /** {@inheritDoc} */
    @Override
    public NormalizedGeodeticPoint refineIntersection(final ExtendedEllipsoid ellipsoid,
                                                      final Vector3D position, final Vector3D los,
                                                      final NormalizedGeodeticPoint closeGuess) {
        DumpManager.dumpAlgorithm(AlgorithmId.IGNORE_DEM_USE_ELLIPSOID);
        return intersection(ellipsoid, position, los);
    }

    /** {@inheritDoc}
     * <p>
     * As this algorithm ignored the Digital Elevation Model,
     * this method always returns 0.0.
     * </p>
     */
    @Override
    public double getElevation(final double latitude, final double longitude) {
        DumpManager.dumpAlgorithm(AlgorithmId.IGNORE_DEM_USE_ELLIPSOID);
        return 0.0;
    }

}
