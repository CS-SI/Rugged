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
import org.apache.commons.math3.geometry.euclidean.threed.FieldRotation;
import org.apache.commons.math3.geometry.euclidean.threed.FieldVector3D;
import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.orekit.rugged.errors.RuggedException;
import org.orekit.rugged.errors.RuggedMessages;
import org.orekit.rugged.utils.ParameterType;

/** {@link TimeIndependentLOSTransform LOS transform} based on a fixed rotation.
 * @author Luc Maisonobe
 * @see LOSBuilder
 */
public class FixedRotation implements TimeIndependentLOSTransform {

    /** Parameters type. */
    private final ParameterType type;

    /** Underlying rotation. */
    private Rotation rotation;

    /** Underlying rotation with derivatives. */
    private FieldRotation<DerivativeStructure> rDS;

    /** Simple constructor.
     * <p>
     * The single parameter is the rotation angle.
     * </p>
     * @param type parameter type
     * @param axis rotation axis
     * @param angle rotation angle
     */
    public FixedRotation(final ParameterType type, final Vector3D axis, final double angle) {
        this.type     = type;
        this.rotation = new Rotation(axis, angle);
        this.rDS      = null;
    }

    /** {@inheritDoc} */
    @Override
    public int getNbEstimatedParameters() {
        return type == ParameterType.FIXED ? 0 : 1;
    }

    /** {@inheritDoc}
     * <p>
     * The single parameter is the rotation angle.
     * </p>
     */
    @Override
    public void getEstimatedParameters(final double[] parameters, final int start, final int length)
        throws RuggedException {
        checkSlice(length);
        parameters[start] = rotation.getAngle();
    }

    /** {@inheritDoc}
     * <p>
     * The single parameter is the rotation angle.
     * </p>
     */
    @Override
    public void setEstimatedParameters(final double[] parameters, final int start, final int length)
        throws RuggedException {
        checkSlice(length);
        final Vector3D axis = rotation.getAxis();
        rotation = new Rotation(axis, parameters[start]);
        final FieldVector3D<DerivativeStructure> axisDS =
                new FieldVector3D<DerivativeStructure>(new DerivativeStructure(parameters.length, 1, axis.getX()),
                                                       new DerivativeStructure(parameters.length, 1, axis.getY()),
                                                       new DerivativeStructure(parameters.length, 1, axis.getZ()));
        final DerivativeStructure angleDS = new DerivativeStructure(parameters.length, 1, start, parameters[start]);
        rDS = new FieldRotation<DerivativeStructure>(axisDS, angleDS);
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
    public Vector3D transformLOS(final int i, final Vector3D los) {
        return rotation.applyTo(los);
    }

    /** {@inheritDoc} */
    @Override
    public FieldVector3D<DerivativeStructure> transformLOS(final int i, final FieldVector3D<DerivativeStructure> los) {
        return rDS.applyTo(los);
    }

}
