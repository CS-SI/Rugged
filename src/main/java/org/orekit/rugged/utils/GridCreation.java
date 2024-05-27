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

/** Utility class for grids creation.
 * @author Guylaine Prat
 * @since 2.1
 */
public final class GridCreation {

    /** Private constructor for utility class.
     * Suppress default constructor for non instantiability ...
     */
    private GridCreation() {
        super();
    }

    /** Create a linear grid between min and max value for a number n of points.
     * TBN: no checks are performed here. Must be done by the calling method.
     * @param min value for grid[0]
     * @param max value for grid[n-1]
     * @param n number of points
     * @return the linear grid
     */
    public static double[] createLinearGrid(final double min, final double max, final int n) {

        final double[] grid = new double[n];
        for (int i = 0; i < n; ++i) {
            grid[i] = ((n - 1 - i) * min + i * max) / (n - 1);
        }
        return grid;
    }
}
