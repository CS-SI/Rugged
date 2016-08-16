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
package org.orekit.rugged.los;

import java.util.stream.Stream;

import org.hipparchus.analysis.differentiation.DerivativeStructure;
import org.hipparchus.geometry.euclidean.threed.FieldRotation;
import org.hipparchus.geometry.euclidean.threed.FieldVector3D;
import org.hipparchus.geometry.euclidean.threed.Rotation;
import org.hipparchus.geometry.euclidean.threed.RotationConvention;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.util.FastMath;
import org.orekit.errors.OrekitException;
import org.orekit.rugged.errors.RuggedException;
import org.orekit.rugged.utils.ExtendedParameterDriver;
import org.orekit.utils.ParameterDriver;
import org.orekit.utils.ParameterObserver;

/** {@link TimeIndependentLOSTransform LOS transform} based on a fixed rotation.
 * @author Luc Maisonobe
 * @see LOSBuilder
 */
public class FixedRotation implements TimeIndependentLOSTransform {

    /** Parameters scaling factor.
     * <p>
     * We use a power of 2 to avoid numeric noise introduction
     * in the multiplications/divisions sequences.
     * </p>
     */
    private final double SCALE = FastMath.scalb(1.0, -20);

    /** Rotation axis. */
    private final Vector3D axis;

    /** Underlying rotation. */
    private Rotation rotation;

    /** Underlying rotation with derivatives. */
    private FieldRotation<DerivativeStructure> rDS;

    /** Driver for rotation angle. */
    private final ExtendedParameterDriver angleDriver;

    /** Simple constructor.
     * <p>
     * The single parameter is the rotation angle.
     * </p>
     * @param name name of the rotation (used for estimated parameters identification)
     * @param axis rotation axis
     * @param angle rotation angle
     */
    public FixedRotation(final String name, final Vector3D axis, final double angle) {
        this.axis     = axis;
        this.rotation = null;
        this.rDS      = null;
        try {
            this.angleDriver = new ExtendedParameterDriver(name, angle, SCALE, 0, 2 * FastMath.PI);
            angleDriver.addObserver(new ParameterObserver() {
                @Override
                public void valueChanged(final double previousValue, final ParameterDriver driver) {
                    // reset rotations to null, they will evaluated lazily if needed
                    rotation = null;
                    rDS      = null;
                }
            });
        } catch (OrekitException oe) {
            // this should never happen
            throw RuggedException.createInternalError(oe);
        }
    }

    /** {@inheritDoc} */
    @Override
    public Stream<ExtendedParameterDriver> getExtendedParametersDrivers() {
        return Stream.of(angleDriver);
    }

    /** {@inheritDoc} */
    @Override
    public Vector3D transformLOS(final int i, final Vector3D los) {
        if (rotation == null) {
            // lazy evaluation of the rotation
            rotation = new Rotation(axis, angleDriver.getValue(), RotationConvention.VECTOR_OPERATOR);
        }
        return rotation.applyTo(los);
    }

    /** {@inheritDoc} */
    @Override
    public FieldVector3D<DerivativeStructure> transformLOS(final int i, final FieldVector3D<DerivativeStructure> los) {
        if (rDS == null) {
            // lazy evaluation of the rotation
            final int nbEstimated = angleDriver.getNbEstimated();
            final FieldVector3D<DerivativeStructure> axisDS =
                            new FieldVector3D<DerivativeStructure>(new DerivativeStructure(nbEstimated, 1, axis.getX()),
                                                                   new DerivativeStructure(nbEstimated, 1, axis.getY()),
                                                                   new DerivativeStructure(nbEstimated, 1, axis.getZ()));
            final double value = angleDriver.getValue();
            final DerivativeStructure angleDS;
            if (angleDriver.isSelected()) {
                // the angle is estimated
                angleDS = new DerivativeStructure(nbEstimated, 1, angleDriver.getIndex(), value);
            } else {
                // the angle is not estimated
                angleDS = new DerivativeStructure(nbEstimated, 1, value);
            }
            rDS = new FieldRotation<DerivativeStructure>(axisDS, angleDS, RotationConvention.VECTOR_OPERATOR);
        }
        return rDS.applyTo(los);
    }

}
