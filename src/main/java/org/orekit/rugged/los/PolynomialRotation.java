/* Copyright 2013-2018 CS Systèmes d'Information
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
import org.hipparchus.analysis.polynomials.PolynomialFunction;
import org.hipparchus.geometry.euclidean.threed.FieldRotation;
import org.hipparchus.geometry.euclidean.threed.FieldVector3D;
import org.hipparchus.geometry.euclidean.threed.Rotation;
import org.hipparchus.geometry.euclidean.threed.RotationConvention;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.util.FastMath;
import org.orekit.rugged.utils.DSGenerator;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.ParameterDriver;
import org.orekit.utils.ParameterObserver;

/** {@link LOSTransform LOS transform} based on a rotation with polynomial angle.
 * @author Luc Maisonobe
 * @see LOSBuilder
 */
public class PolynomialRotation implements LOSTransform {

    /** Parameters scaling factor.
     * <p>
     * We use a power of 2 to avoid numeric noise introduction
     * in the multiplications/divisions sequences.
     * </p>
     */
    private final double SCALE = FastMath.scalb(1.0, -20);

    /** Rotation axis. */
    private final Vector3D axis;

    /** Rotation angle polynomial. */
    private PolynomialFunction angle;

    /** Rotation axis and derivatives. */
    private FieldVector3D<DerivativeStructure> axisDS;

    /** Rotation angle polynomial and derivatives. */
    private DerivativeStructure[] angleDS;

    /** Reference date for polynomial evaluation. */
    private final AbsoluteDate referenceDate;

    /** Drivers for rotation angle polynomial coefficients. */
    private final ParameterDriver[] coefficientsDrivers;

    /** Simple constructor.
     * <p>
     * The angle of the rotation is evaluated as a polynomial in t,
     * where t is the duration in seconds between evaluation date and
     * reference date. The parameters are the polynomial coefficients,
     * with the constant term at index 0.
     * </p>
     * @param name name of the rotation (used for estimated parameters identification)
     * @param axis rotation axis
     * @param referenceDate reference date for the polynomial angle
     * @param angleCoeffs polynomial coefficients of the polynomial angle,
     * with the constant term at index 0
     */
    public PolynomialRotation(final String name,
                              final Vector3D axis,
                              final AbsoluteDate referenceDate,
                              final double... angleCoeffs) {
        this.axis                = axis;
        this.referenceDate       = referenceDate;
        this.coefficientsDrivers = new ParameterDriver[angleCoeffs.length];
        final ParameterObserver resettingObserver = new ParameterObserver() {
            @Override
            public void valueChanged(final double previousValue, final ParameterDriver driver) {
                // reset rotations to null, they will be evaluated lazily if needed
                angle   = null;
                axisDS  = null;
                angleDS = null;
            }
        };
        for (int i = 0; i < angleCoeffs.length; ++i) {
            coefficientsDrivers[i] = new ParameterDriver(name + "[" + i + "]", angleCoeffs[i], SCALE,
                    Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
            coefficientsDrivers[i].addObserver(resettingObserver);
        }
    }

    /** Simple constructor.
     * <p>
     * The angle of the rotation is evaluated as a polynomial in t,
     * where t is the duration in seconds between evaluation date and
     * reference date. The parameters are the polynomial coefficients,
     * with the constant term at index 0.
     * </p>
     * @param name name of the rotation (used for estimated parameters identification)
     * @param axis rotation axis
     * @param referenceDate reference date for the polynomial angle
     * @param angle polynomial angle
     */
    public PolynomialRotation(final String name,
                              final Vector3D axis,
                              final AbsoluteDate referenceDate,
                              final PolynomialFunction angle) {
        this(name, axis, referenceDate, angle.getCoefficients());
    }

    /** {@inheritDoc}
     * @since 2.0
     */
    @Override
    public Stream<ParameterDriver> getParametersDrivers() {
        return Stream.of(coefficientsDrivers);
    }

    /** {@inheritDoc} */
    @Override
    public Vector3D transformLOS(final int i, final Vector3D los, final AbsoluteDate date) {
        if (angle == null) {
            // lazy evaluation of the rotation
            final double[] coefficients = new double[coefficientsDrivers.length];
            for (int k = 0; k < coefficients.length; ++k) {
                coefficients[k] = coefficientsDrivers[k].getValue();
            }
            angle = new PolynomialFunction(coefficients);
        }
        return new Rotation(axis,
                            angle.value(date.durationFrom(referenceDate)),
                            RotationConvention.VECTOR_OPERATOR).applyTo(los);
    }

    /** {@inheritDoc} */
    @Override
    public FieldVector3D<DerivativeStructure> transformLOS(final int i, final FieldVector3D<DerivativeStructure> los,
                                                           final AbsoluteDate date, final DSGenerator generator) {

        if (angleDS == null) {
            // lazy evaluation of the rotation
            axisDS = new FieldVector3D<DerivativeStructure>(generator.constant(axis.getX()),
                                                            generator.constant(axis.getY()),
                                                            generator.constant(axis.getZ()));
            angleDS = new DerivativeStructure[coefficientsDrivers.length];
            for (int k = 0; k < angleDS.length; ++k) {
                angleDS[k] = generator.variable(coefficientsDrivers[k]);
            }
        }
        // evaluate polynomial, with all its partial derivatives
        final double t = date.durationFrom(referenceDate);
        DerivativeStructure alpha = axisDS.getX().getField().getZero();
        for (int k = angleDS.length - 1; k >= 0; --k) {
            alpha = alpha.multiply(t).add(angleDS[k]);
        }

        return new FieldRotation<DerivativeStructure>(axisDS,
                                                      alpha,
                                                      RotationConvention.VECTOR_OPERATOR).applyTo(los);

    }

}
