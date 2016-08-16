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
package org.orekit.rugged.utils;

import org.orekit.errors.OrekitException;
import org.orekit.utils.ParameterDriver;

/** {@link ParameterDriver} belonging to a list.
 * @since 2.0
 * @author Luc Maisonobe
 */
public class ExtendedParameterDriver extends ParameterDriver {

    /** Total number of estimated parameters. */
    private int nbEstimated;

    /** Index of this parameter in the estimated parameters list. */
    private int index;

    /** Simple constructor.
     * <p>
     * At construction, the parameter is configured as <em>not</em> selected,
     * and the value is set to the {@code referenceValue}.
     * </p>
     * @param name name of the parameter
     * @param referenceValue reference value of the parameter
     * @param scale scaling factor to convert the parameters value to
     * non-dimensional (typically set to the expected standard deviation of the
     * parameter), it must be non-zero
     * @param minValue minimum value
     * @param maxValue maximum value
     * @exception OrekitException if scale is too close to zero
     */
    public ExtendedParameterDriver(final String name, final double referenceValue,
                                   final double scale, final double minValue,
                                   final double maxValue)
        throws OrekitException {
        super(name, referenceValue, scale, minValue, maxValue);
        nbEstimated =  0;
        index       = -1;
    }

    /** Set the total number of estimated parameters.
     * @param nbEstimated total number of estimated parameters
     */
    public void setNbEstimated(final int nbEstimated) {
        this.nbEstimated = nbEstimated;
    }

    /** Get the total number of estimated parameters.
     * @return total number of estimated parameters
     */
    public int getNbEstimated() {
        return nbEstimated;
    }

    /** Set the index of this parameter in the estimated parameters list.
     * @param index index of this parameter in the estimated parameters list
     */
    public void setIndex(final int index) {
        this.index = index;
    }

    /** Get the index of this parameter in the estimated parameters list.
     * @return index of this parameter in the estimated parameters list
     */
    public int getIndex() {
        return index;
    }

}
