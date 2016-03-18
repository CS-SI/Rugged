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

import org.apache.commons.math3.analysis.differentiation.DerivativeStructure;
import org.apache.commons.math3.geometry.euclidean.threed.FieldVector3D;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.orekit.rugged.utils.ParametricModel;
import org.orekit.time.AbsoluteDate;

/** Interface for lines-of-sight tranforms.
 * @author Luc Maisonobe
 * @see LOSBuilder
 */
public interface LOSTransform extends ParametricModel {

    /** Transform a line-of-sight.
     * @param i los pixel index
     * @param los line-of-sight to transform
     * @param date current date
     * @return transformed line-of-sight
     */
    Vector3D transformLOS(int i, Vector3D los, AbsoluteDate date);

    /** Transform a line-of-sight and its partial derivatives.
     * <p>
     * This method is used for LOS calibration purposes. It allows to compute
     * the Jacobian matrix of the LOS with respect to the parameters, which
     * are typically polynomials coefficients representing rotation angles.
     * These polynomials can be used for example to model thermo-elastic effects.
     * </p>
     * @param index los pixel index
     * @param date date
     * @param los line-of-sight to transform
     * @return line of sight, and its first partial derivatives with respect to the parameters
     */
    FieldVector3D<DerivativeStructure> transformLOS(int index, FieldVector3D<DerivativeStructure> los, AbsoluteDate date);

}
