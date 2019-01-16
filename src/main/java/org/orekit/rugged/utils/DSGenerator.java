/* Copyright 2013-2019 CS Systèmes d'Information
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

import java.util.List;

import org.hipparchus.analysis.differentiation.DerivativeStructure;
import org.orekit.utils.ParameterDriver;

/** Generator for {@link DerivativeStructure} instances from {@link ParameterDriver}.
 * <p>
 * Note that this interface is for Rugged library internal use only.
 * </p>
 * @author Luc Maisonobe
 * @since 2.0
 */
public interface DSGenerator {

    /** Get the parameters selected for estimation.
     * @return parameters selected for estimation
     */
    List<ParameterDriver> getSelected();

    /** Generate a constant {@link DerivativeStructure}.
     * @param value value of the constant
     * @return constant {@link DerivativeStructure}
     */
    DerivativeStructure constant(double value);

    /** Generate a {@link DerivativeStructure} representing the
     * parameter driver either as a canonical variable or a constant.
     * <p>
     * The instance created is a variable only if the parameter
     * has been selected for estimation, otherwise it is a constant.
     * </p>
     * @param driver driver for the variable
     * @return variable {@link DerivativeStructure}
     */
    DerivativeStructure variable(ParameterDriver driver);

}
