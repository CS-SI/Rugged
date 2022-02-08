/* Copyright 2013-2022 CS GROUP
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
 * @author Guylaine Prat
 */
public class IgnoreDEMAlgorithm implements IntersectionAlgorithm {

    /** Algorithm Id.
     * @since 2.2 */
    private final AlgorithmId algorithmId;

    /** Simple constructor.
     */
    public IgnoreDEMAlgorithm() {
        this.algorithmId = AlgorithmId.IGNORE_DEM_USE_ELLIPSOID;
    }

    /** {@inheritDoc} */
    @Override
    public NormalizedGeodeticPoint intersection(final ExtendedEllipsoid ellipsoid,
                                                final Vector3D position, final Vector3D los) {
        DumpManager.dumpAlgorithm(this.algorithmId);
        return ellipsoid.pointOnGround(position, los, 0.0);
    }

    /** {@inheritDoc} */
    @Override
    public NormalizedGeodeticPoint refineIntersection(final ExtendedEllipsoid ellipsoid,
                                                      final Vector3D position, final Vector3D los,
                                                      final NormalizedGeodeticPoint closeGuess) {
        DumpManager.dumpAlgorithm(this.algorithmId);
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
        DumpManager.dumpAlgorithm(this.algorithmId);
        return 0.0;
    }

    /** {@inheritDoc} */
    @Override
    public AlgorithmId getAlgorithmId() {
        return this.algorithmId;
    }
}
