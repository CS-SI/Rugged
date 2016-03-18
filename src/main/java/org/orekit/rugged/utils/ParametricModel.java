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

import org.orekit.rugged.errors.RuggedException;


/** Interface for models that have parameters.
 * <p>
 * The parameters are typically polynomial coefficients, for example
 * to model thermo-elastic deformations of the sensor or spacecraft.
 * </p>
 * @author Luc Maisonobe
 */
public interface ParametricModel {

    /** Get the number of estimated parameters.
     * @return number of estimated parameters
     */
    int getNbEstimatedParameters();

    /** Get the current values of the estimated parameters.
     * @param parameters global array where to put the parameters
     * @param start start index of the array slice to consider
     * @param length number of elements in the array slice to consider
     * @exception RuggedException if the size of the slice does not match
     * the {@link #getNbEstimatedParameters() number of estimated parameters}
     */
    void getEstimatedParameters(double[] parameters, int start, int length)
        throws RuggedException;

    /** Set new values for the estimated parameters.
     * @param parameters global array containing the parameters to set (among others)
     * @param start start index of the array slice to consider
     * @param length number of elements in the array slice to consider
     * @exception RuggedException if the size of the slice does not match
     * the {@link #getNbEstimatedParameters() number of estimated parameters}
     */
    void setEstimatedParameters(double[] parameters, int start, int length)
        throws RuggedException;

}
