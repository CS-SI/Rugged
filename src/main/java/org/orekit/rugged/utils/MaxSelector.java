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

/** Selector for max value.
 * <p>
 * This selector considers {@code Double.NaN} values correspond
 * to non-initialized data that should be ignored rather than
 * selected.
 * </p>
 * @see MinSelector
 * @author Luc Maisonobe
 */
public class MaxSelector extends Selector {

    /** Private constructor for singleton.
     */
    private MaxSelector() {
    }

    /** Get the unique instance.
     * @return unique instance of the min selector.
     */
    public static MaxSelector getInstance() {
        return LazyHolder.INSTANCE;
    }

    /** Check if first value should be selected.
     * @param v1 first value
     * @param v2 second value
     * @return true if v1 is higher than v2, or if v2 is {@code Double.NaN}
     */
    @Override
    public boolean selectFirst(final double v1, final double v2) {
        return v1 > v2 || Double.isNaN(v2);
    }

    /** Holder for the min selector singleton.
     * <p>
     * We use the Initialization On Demand Holder Idiom to store
     * the singletons, as it is both thread-safe, efficient (no
     * synchronization) and works with all versions of java.
     * </p>
     */
    private static class LazyHolder {

        /** Unique instance. */
        private static final MaxSelector INSTANCE = new MaxSelector();

        /** Private constructor.
         * <p>This class is a utility class, it should neither have a public
         * nor a default constructor. This private constructor prevents
         * the compiler from generating one automatically.</p>
         */
        private LazyHolder() {
        }

    }

}
