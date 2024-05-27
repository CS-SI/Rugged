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
package org.orekit.rugged.utils;


/** Class for selecting one value among two.
 * @see MinSelector
 * @see MaxSelector
 * @author Luc Maisonobe
 */
public abstract class Selector {

    /** Check if first value should be selected.
     * @param v1 first value
     * @param v2 second value
     * @return true if v1 should be selected
     */
    public abstract boolean selectFirst(double v1, double v2);

    /** Select a value.
     * @param v1 first value
     * @param v2 second value
     * @return selected value
     */
    public double select(final double v1, final double v2) {
        return selectFirst(v1, v2) ? v1 : v2;
    }

}
