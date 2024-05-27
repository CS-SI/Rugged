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
package org.orekit.rugged.los;

import java.util.stream.Stream;

import org.hipparchus.analysis.differentiation.Derivative;
import org.hipparchus.geometry.euclidean.threed.FieldVector3D;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.orekit.rugged.utils.DerivativeGenerator;
import org.orekit.utils.ParameterDriver;

/** Interface for lines-of-sight tranforms that do not depend on time.
 * @author Luc Maisonobe
 * @see LOSBuilder
 */
public interface TimeIndependentLOSTransform {

    /** Transform a line-of-sight.
     * @param i los pixel index
     * @param los line-of-sight to transform
     * @return transformed line-of-sight
     */
    Vector3D transformLOS(int i, Vector3D los);

    /** Transform a line-of-sight and its partial derivatives.
     * <p>
     * This method is used for LOS calibration purposes. It allows to compute
     * the Jacobian matrix of the LOS with respect to the parameters, which
     * are typically polynomials coefficients representing rotation angles.
     * These polynomials can be used for example to model thermo-elastic effects.
     * </p>
     * <p>
     * Note that in order for the partial derivatives to be properly set up, the
     * {@link org.orekit.utils.ParameterDriver#setSelected(boolean) setSelected}
     * method must have been set to {@code true} for the various parameters returned
     * by {@link #getParametersDrivers()} that should be estimated.
     * </p>
     * @param <T> derivative type
     * @param index los pixel index
     * @param los line-of-sight to transform
     * @param generator generator to use for building {@link Derivative} instances
     * @return line of sight, and its first partial derivatives with respect to the parameters
     */
    <T extends Derivative<T>> FieldVector3D<T> transformLOS(int index, FieldVector3D<T> los,
                                                            DerivativeGenerator<T> generator);

    /** Get the drivers for LOS parameters.
     * @return drivers for LOS parameters
     * @since 2.0
     */
    Stream<ParameterDriver> getParametersDrivers();

}
