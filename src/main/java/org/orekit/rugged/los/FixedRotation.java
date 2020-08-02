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
package org.orekit.rugged.los;

import java.util.stream.Stream;

import org.hipparchus.analysis.differentiation.Derivative;
import org.hipparchus.geometry.euclidean.threed.FieldRotation;
import org.hipparchus.geometry.euclidean.threed.FieldVector3D;
import org.hipparchus.geometry.euclidean.threed.Rotation;
import org.hipparchus.geometry.euclidean.threed.RotationConvention;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.util.FastMath;
import org.orekit.rugged.utils.DerivativeGenerator;
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
    private FieldRotation<?> rDS;

    /** Driver for rotation angle. */
    private final ParameterDriver angleDriver;

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
        this.angleDriver = new ParameterDriver(name, angle, SCALE, -2 * FastMath.PI, 2 * FastMath.PI);
        angleDriver.addObserver(new ParameterObserver() {
            @Override
            public void valueChanged(final double previousValue, final ParameterDriver driver) {
                // reset rotations to null, they will be evaluated lazily if needed
                rotation = null;
                rDS      = null;
            }
        });
    }

    /** {@inheritDoc} */
    @Override
    public Stream<ParameterDriver> getParametersDrivers() {
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
    @SuppressWarnings("unchecked")
    @Override
    public <T extends Derivative<T>> FieldVector3D<T> transformLOS(final int i, final FieldVector3D<T> los,
                                                                   final DerivativeGenerator<T> generator) {
        final FieldRotation<T> rD;
        if (rDS == null || !rDS.getQ0().getField().equals(generator.getField())) {

            // lazy evaluation of the rotation
            final FieldVector3D<T> axisDS =
                            new FieldVector3D<>(generator.constant(axis.getX()),
                                                generator.constant(axis.getY()),
                                                generator.constant(axis.getZ()));
            final T angleDS = generator.variable(angleDriver);
            rD = new FieldRotation<>(axisDS, angleDS, RotationConvention.VECTOR_OPERATOR);

            // cache evaluated rotation
            rDS = rD;

        } else {
            // reuse cached value
            rD  = (FieldRotation<T>) rDS;
        }

        return rD.applyTo(los);

    }

}
