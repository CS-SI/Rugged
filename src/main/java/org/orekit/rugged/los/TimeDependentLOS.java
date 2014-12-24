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
package org.orekit.rugged.los;

import org.apache.commons.math3.analysis.differentiation.DerivativeStructure;
import org.apache.commons.math3.geometry.euclidean.threed.FieldVector3D;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.orekit.rugged.utils.ParametricModel;
import org.orekit.time.AbsoluteDate;

/** Interface representing a line-of-sight which depends on time.
 * @see LineSensor
 * @author Luc Maisonobe
 */
public interface TimeDependentLOS extends ParametricModel {

    /** Get the number of pixels.
     * @return number of pixels
     */
    int getNbPixels();

    /** Get the line of sight for a given date.
     * @param index los pixel index
     * @param date date
     * @return line of sight
     */
    Vector3D getLOS(int index, AbsoluteDate date);

    /** Get the line of sight and its partial derivatives for a given date.
     * <p>
     * This method is used for LOS calibration purposes. It allows to compute
     * the Jacobian matrix of the LOS with respect to the parameters, which
     * are typically polynomials coefficients representing rotation angles.
     * These polynomials can be used for example to model thermo-elastic effects.
     * </p>
     * <p>
     * Note that in order for the partial derivatives to be properly set up, the
     * {@link #setEstimatedParameters(double[], int, int) setEstimatedParameters}
     * <em>must</em> have been called at least once before this method and its
     * {@code start} parameter will be used to ensure the partial derivatives are
     * ordered in the same way in the returned vector as they were in the set
     * parameters.
     * </p>
     * @param index los pixel index
     * @param date date
     * @param parameters current estimate of the adjusted parameters
     * @return line of sight, and its first partial derivatives with respect to the parameters
     */
    FieldVector3D<DerivativeStructure> getLOS(int index, AbsoluteDate date, double[] parameters);

}
