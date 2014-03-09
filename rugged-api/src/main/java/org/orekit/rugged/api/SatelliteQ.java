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
package org.orekit.rugged.api;

import java.io.Serializable;

/** Container for satellite quaternion.
 * <p>
 * Instances of this class are guaranteed to be immutable.
 * </p>
 * @author Luc Maisonobe
 */
public class SatelliteQ implements Serializable {

    /** Serializable UID. */
    private static final long serialVersionUID = 20140309L;

    /** UTC date. */
    private final String date;

    /** Scalar component. */
    private final double q0;

    /** First vectorial component. */
    private final double q1;

    /** Second vectorial component. */
    private final double q2;

    /** Third vectorial component. */
    private final double q3;

    /**
     * Build a new instance.
     *
     * @param date UTC date
     * @param q0 scalar component
     * @param q1 first vectorial component
     * @param q2 second vectorial component
     * @param q3 third vectorial component
     */
    public SatelliteQ(final String date,
                      final double q0, final double q1, final double q2, final double q3) {
        this.date = date;
        this.q0   = q0;
        this.q1   = q1;
        this.q2   = q2;
        this.q3   = q3;
    }

    /** Get the UTC date.
     * @return utc date
     */
    public String getDate() {
        return date;
    }

    /** Get the scalar component.
     * @return scalar component
     */
    public double getQ0() {
        return q0;
    }

    /** Get the first vectorial component.
     * @return first vectorial component
     */
    public double getQ1() {
        return q1;
    }

    /** Get the second vectorial component.
     * @return second vectorial component
     */
    public double getQ2() {
        return q2;
    }

    /** Get the third vectorial component.
     * @return third vectorial component
     */
    public double getQ3() {
        return q3;
    }

}
