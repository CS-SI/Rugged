/* Copyright 2013-2015 CS Systèmes d'Information
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

import org.apache.commons.math3.analysis.differentiation.DerivativeStructure;
import org.apache.commons.math3.analysis.polynomials.PolynomialFunction;
import org.apache.commons.math3.geometry.euclidean.threed.FieldRotation;
import org.apache.commons.math3.geometry.euclidean.threed.FieldVector3D;
import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.orekit.rugged.errors.RuggedException;
import org.orekit.rugged.errors.RuggedMessages;
import org.orekit.rugged.utils.ParameterType;
import org.orekit.time.AbsoluteDate;

/** {@link LOSTransform LOS transform} based on a rotation with polynomial angle.
 * @author Luc Maisonobe
 * @see LOSBuilder
 */
public class PolynomialRotation implements LOSTransform {

    /** Parameters type. */
    private final ParameterType type;

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

    /** Simple constructor.
     * <p>
     * The angle of the rotation is evaluated as a polynomial in t,
     * where t is the duration in seconds between evaluation date and
     * reference date. The parameters are the polynomial coefficients,
     * with the constant term at index 0.
     * </p>
     * @param type parameters type
     * @param axis rotation axis
     * @param referenceDate reference date for the polynomial angle
     * @param angleCoeffs polynomial coefficients of the polynomial angle,
     * with the constant term at index 0
     */
    public PolynomialRotation(final ParameterType type,
                              final Vector3D axis,
                              final AbsoluteDate referenceDate,
                              final double ... angleCoeffs) {
        this(type, axis, referenceDate, new PolynomialFunction(angleCoeffs));
    }

    /** Simple constructor.
     * <p>
     * The angle of the rotation is evaluated as a polynomial in t,
     * where t is the duration in seconds between evaluation date and
     * reference date. The parameters are the polynomial coefficients,
     * with the constant term at index 0.
     * </p>
     * @param type parameters type
     * @param axis rotation axis
     * @param referenceDate reference date for the polynomial angle
     * @param angle polynomial angle
     */
    public PolynomialRotation(final ParameterType type,
                              final Vector3D axis,
                              final AbsoluteDate referenceDate,
                              final PolynomialFunction angle) {
        this.type          = type;
        this.axis          = axis;
        this.angle         = angle;
        this.referenceDate = referenceDate;
    }

    /** {@inheritDoc} */
    @Override
    public int getNbEstimatedParameters() {
        return type == ParameterType.FIXED ? 0 : (angle.degree() + 1);
    }

    /** {@inheritDoc}
     * <p>
     * The parameters are the polynomial coefficients,
     * with the constant term at index 0.
     * </p>
     */
    @Override
    public void getEstimatedParameters(final double[] parameters, final int start, final int length)
        throws RuggedException {
        checkSlice(length);
        System.arraycopy(angle.getCoefficients(), 0, length, start, length);
    }

    /** {@inheritDoc}
     * <p>
     * The parameters are the polynomial coefficients,
     * with the constant term at index 0.
     * </p>
     */
    @Override
    public void setEstimatedParameters(final double[] parameters, final int start, final int length)
        throws RuggedException {

        checkSlice(length);

        // regular rotation
        angle = new PolynomialFunction(parameters);

        // prepare components to compute rotation with derivatives
        axisDS = new FieldVector3D<DerivativeStructure>(new DerivativeStructure(parameters.length, 1, axis.getX()),
                                                        new DerivativeStructure(parameters.length, 1, axis.getY()),
                                                        new DerivativeStructure(parameters.length, 1, axis.getZ()));
        angleDS = new DerivativeStructure[length];
        for (int i = 0; i < length; ++i) {
            angleDS[i] = new DerivativeStructure(parameters.length, 1, start + i, parameters[start + i]);
        }

    }

    /** Check the number of parameters of an array slice.
     * @param length number of elements in the array slice to consider
     * @exception RuggedException if the size of the slice does not match
     * the {@link #getNbEstimatedParameters() number of estimated parameters}
     */
    private void checkSlice(final int length) throws RuggedException {
        if (getNbEstimatedParameters() != length) {
            throw new RuggedException(RuggedMessages.ESTIMATED_PARAMETERS_NUMBER_MISMATCH,
                                      getNbEstimatedParameters(), length);
        }
    }

    /** {@inheritDoc} */
    @Override
    public Vector3D transformLOS(final int i, final Vector3D los, final AbsoluteDate date) {
        return new Rotation(axis, angle.value(date.durationFrom(referenceDate))).applyTo(los);
    }

    /** {@inheritDoc} */
    @Override
    public FieldVector3D<DerivativeStructure> transformLOS(final int i, final FieldVector3D<DerivativeStructure> los,
                                                           final AbsoluteDate date) {

        // evaluate polynomial, with all its partial derivatives
        final double t = date.durationFrom(referenceDate);
        DerivativeStructure alpha = axisDS.getX().getField().getZero();
        for (int k = angleDS.length - 1; k >= 0; --k) {
            alpha = alpha.multiply(t).add(angleDS[k]);
        }

        return new FieldRotation<DerivativeStructure>(axisDS, alpha).applyTo(los);

    }

}
