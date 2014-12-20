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

import org.apache.commons.math3.analysis.polynomials.PolynomialFunction;
import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.orekit.time.AbsoluteDate;

/** {@link LOSTransform LOS transform} based on a rotation with polynomial angle.
 * @author Luc Maisonobe
 * @see LOSBuilder
 */
public class PolynomialRotation implements LOSTransform {

    /** Rotation axis. */
    final Vector3D axis;

    /** Rotation angle polynomial. */
    final PolynomialFunction angle;

    /** Reference date for polynomial evaluation. */
    final AbsoluteDate referenceDate;

    /** Simple constructor.
     * @param axis rotation axis
     * @param angle rotation angle as a polynomial in t, where t
     * is the duration in seconds between evaluation date and reference date
     * @param referenceDate reference date for the polynomial angle
     */
    public PolynomialRotation(final Vector3D axis,
                              final PolynomialFunction angle,
                              final AbsoluteDate referenceDate) {
        this.axis          = axis;
        this.angle         = angle;
        this.referenceDate = referenceDate;
    }

    /** {@inheritDoc} */
    @Override
    public Vector3D transformLOS(int i, Vector3D los, AbsoluteDate date) {
        return new Rotation(axis, angle.value(date.durationFrom(referenceDate))).applyTo(los);
    }
    
}