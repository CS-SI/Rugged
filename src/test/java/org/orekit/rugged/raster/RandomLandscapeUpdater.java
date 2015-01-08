/* Copyright 2013-2015 CS Systèmes d'Information
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
package org.orekit.rugged.raster;

import org.apache.commons.math3.exception.MathIllegalArgumentException;
import org.apache.commons.math3.exception.util.LocalizedFormats;
import org.apache.commons.math3.random.RandomGenerator;
import org.apache.commons.math3.random.Well19937a;
import org.apache.commons.math3.util.ArithmeticUtils;
import org.apache.commons.math3.util.FastMath;
import org.orekit.rugged.errors.RuggedException;

public class RandomLandscapeUpdater implements TileUpdater {

    private double size;
    private int    n;
    private double[][] heightMap;

    public RandomLandscapeUpdater(double baseH, double initialScale, double reductionFactor,
                                  long seed, double size, int n)
        throws MathIllegalArgumentException {

        if (!ArithmeticUtils.isPowerOfTwo(n - 1)) {
            throw new MathIllegalArgumentException(LocalizedFormats.SIMPLE_MESSAGE,
                                                   "tile size must be a power of two plus one");
        }

        this.size = size;
        this.n    = n;

        // as we want to use this for testing and comparison purposes,
        // and as we don't control when tiles are generated, we generate
        // only ONCE a height map with continuity at the edges, and
        // reuse this single height map for ALL generated tiles
        heightMap = new double[n][n];
        RandomGenerator random  = new Well19937a(seed);

        // initialize corners in diamond-square algorithm
        heightMap[0][0]         = baseH;
        heightMap[0][n - 1]     = baseH;
        heightMap[n - 1][0]     = baseH;
        heightMap[n - 1][n - 1] = baseH;

        double scale = initialScale;
        for (int span = n - 1; span > 1; span = span / 2) {

            // perform squares step
            for (int i = span / 2; i < n; i += span) {
                for (int j = span / 2; j < n; j += span) {
                    double middleH = mean(i - span / 2, j - span / 2,
                                          i - span / 2, j + span / 2,
                                          i + span / 2, j - span / 2,
                                          i + span / 2, j + span / 2) +
                                     scale * (random.nextDouble() - 0.5);
                    heightMap[i][j] = middleH;
                }
            }

            // perform diamonds step
            boolean flipFlop = false;
            for (int i = 0; i < n - 1; i += span / 2) {
                for (int j = flipFlop ? 0 : span / 2; j < n - 1; j += span) {
                    double middleH = mean(i - span / 2, j,
                                          i + span / 2, j,
                                          i,            j - span / 2,
                                          i,            j + span / 2) +
                                     scale * (random.nextDouble() - 0.5);
                    heightMap[i][j] = middleH;
                    if (i == 0) {
                        heightMap[n - 1][j] = middleH;
                    }
                    if (j == 0) {
                        heightMap[i][n - 1] = middleH;
                    }
                }
                flipFlop = !flipFlop;
            }

            // reduce scale
            scale *= reductionFactor;

        }

    }

    public void updateTile(double latitude, double longitude, UpdatableTile tile)
        throws RuggedException {

        double step         = size / (n - 1);
        double minLatitude  = size * FastMath.floor(latitude  / size);
        double minLongitude = size * FastMath.floor(longitude / size);
        tile.setGeometry(minLatitude, minLongitude, step, step, n, n);
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) {
                tile.setElevation(i, j, heightMap[i][j]);
            }
        }

    }

    private double mean(int i1, int j1, int i2, int j2, int i3, int j3, int i4, int j4) {
        return (heightMap[(i1 + n) % n][(j1 + n) % n] +
                heightMap[(i2 + n) % n][(j2 + n) % n] +
                heightMap[(i3 + n) % n][(j3 + n) % n] +
                heightMap[(i4 + n) % n][(j4 + n) % n]) / 4;
    }

}
