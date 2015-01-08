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
package org.orekit.rugged.intersection.duvenhage;

import java.lang.reflect.Field;

import org.apache.commons.math3.util.FastMath;
import org.junit.Assert;
import org.junit.Test;
import org.orekit.rugged.errors.RuggedException;
import org.orekit.rugged.intersection.duvenhage.MinMaxTreeTile;
import org.orekit.rugged.intersection.duvenhage.MinMaxTreeTileFactory;

public class MinMaxTreeTileTest {

    @Test
    public void testSizeTall()
        throws RuggedException, SecurityException, NoSuchFieldException,
               IllegalArgumentException, IllegalAccessException {
        MinMaxTreeTile tile = createTile(107, 19);
        Assert.assertEquals(9, tile.getLevels());

        Field startField = MinMaxTreeTile.class.getDeclaredField("start");
        startField.setAccessible(true);
        int[] start = (int[]) startField.get(tile);
        Assert.assertEquals(   0, start[ 0]);
        Assert.assertEquals(   7, start[ 1]);
        Assert.assertEquals(  21, start[ 2]);
        Assert.assertEquals(  49, start[ 3]);
        Assert.assertEquals(  91, start[ 4]);
        Assert.assertEquals( 172, start[ 5]);
        Assert.assertEquals( 307, start[ 6]);
        Assert.assertEquals( 577, start[ 7]);
        Assert.assertEquals(1117, start[ 8]);

        Assert.assertTrue(tile.isColumnMerging(9));
        Assert.assertFalse(tile.isColumnMerging(8));
        Assert.assertTrue(tile.isColumnMerging(7));
        Assert.assertFalse(tile.isColumnMerging(6));
        Assert.assertTrue(tile.isColumnMerging(5));
        Assert.assertFalse(tile.isColumnMerging(4));
        Assert.assertTrue(tile.isColumnMerging(3));
        Assert.assertFalse(tile.isColumnMerging(2));
        Assert.assertTrue(tile.isColumnMerging(1));

        Field minTreeField = MinMaxTreeTile.class.getDeclaredField("minTree");
        minTreeField.setAccessible(true);
        Assert.assertEquals(2187, ((double[]) minTreeField.get(tile)).length);
        Field maxTreeField = MinMaxTreeTile.class.getDeclaredField("maxTree");
        maxTreeField.setAccessible(true);
        Assert.assertEquals(2187, ((double[]) maxTreeField.get(tile)).length);

    }

    @Test
    public void testSizeFat()
        throws RuggedException, SecurityException, NoSuchFieldException,
               IllegalArgumentException, IllegalAccessException {
        MinMaxTreeTile tile = createTile(4, 7);
        Assert.assertEquals(4, tile.getLevels());

        Field startField = MinMaxTreeTile.class.getDeclaredField("start");
        startField.setAccessible(true);
        int[] start = (int[]) startField.get(tile);
        Assert.assertEquals( 0, start[ 0]);
        Assert.assertEquals( 2, start[ 1]);
        Assert.assertEquals( 6, start[ 2]);
        Assert.assertEquals(14, start[ 3]);

        Assert.assertTrue(tile.isColumnMerging(4));
        Assert.assertFalse(tile.isColumnMerging(3));
        Assert.assertTrue(tile.isColumnMerging(2));
        Assert.assertFalse(tile.isColumnMerging(1));

        Field minTreeField = MinMaxTreeTile.class.getDeclaredField("minTree");
        minTreeField.setAccessible(true);
        Assert.assertEquals(30, ((double[]) minTreeField.get(tile)).length);
        Field maxTreeField = MinMaxTreeTile.class.getDeclaredField("maxTree");
        maxTreeField.setAccessible(true);
        Assert.assertEquals(30, ((double[]) maxTreeField.get(tile)).length);

    }

    @Test
    public void testSinglePixel() throws RuggedException {
        Assert.assertEquals(0, createTile(1, 1).getLevels());
    }

    @Test
    public void testMinMax() throws RuggedException {
        for (int nbRows = 1; nbRows < 25; nbRows++) {
            for (int nbColumns = 1; nbColumns < 25; nbColumns++) {

                MinMaxTreeTile tile = createTile(nbRows, nbColumns);

                for (int level = 0; level < tile.getLevels(); level++) {
                    for (int row = 0; row < nbRows; row++) {
                        for (int column = 0; column < nbColumns; column++) {

                            // reference min and max
                            int[] neighbors = neighbors(row, column, nbRows, nbColumns, tile.getLevels() - level);
                            double min = Double.POSITIVE_INFINITY;
                            double max = Double.NEGATIVE_INFINITY;
                            for (int i = neighbors[0]; i < FastMath.min(neighbors[1] + 1, nbRows); ++i) {
                                for (int j = neighbors[2]; j < FastMath.min(neighbors[3] + 1, nbColumns); ++j) {
                                    double cellValue = tile.getElevationAtIndices(i, j);
                                    min = FastMath.min(min, cellValue);
                                    max = FastMath.max(max, cellValue);
                                }
                            }

                            Assert.assertEquals(min, tile.getMinElevation(row, column, level), 1.0e-10 * min);
                            Assert.assertEquals(max, tile.getMaxElevation(row, column, level), 1.0e-10 * max);
                        }
                    }
                }
            }
        }
    }

    @Test
    public void testMergeLarge() throws RuggedException {
        MinMaxTreeTile tile = createTile(1201, 1201);
        Assert.assertEquals(21, tile.getLevels());
        Assert.assertEquals( 7, tile.getMergeLevel(703, 97, 765, 59));
    }

    @Test
    public void testMergeLevel() throws RuggedException {
        for (int nbRows = 1; nbRows < 20; nbRows++) {
            for (int nbColumns = 1; nbColumns < 20; nbColumns++) {

                MinMaxTreeTile tile = createTile(nbRows, nbColumns);

                for (int i1 = 0; i1 < nbRows; i1++) {
                    for (int j1 = 0; j1 < nbColumns; j1++) {
                        for (int i2 = 0; i2 < nbRows; i2++) {
                            for (int j2 = 0; j2 < nbColumns; j2++) {

                                int level = tile.getMergeLevel(i1, j1, i2, j2);
                                if (level > 0) {
                                    int[] neighbors1 = neighbors(i1, j1, nbRows, nbColumns, tile.getLevels() - level);
                                    int[] neighbors2 = neighbors(i2, j2, nbRows, nbColumns, tile.getLevels() - level);
                                    for (int k = 0; k < neighbors1.length; ++k) {
                                        Assert.assertTrue(neighbors1[0] == neighbors2[0] &&
                                                          neighbors1[1] == neighbors2[1] &&
                                                          neighbors1[2] == neighbors2[2] &&
                                                          neighbors1[3] == neighbors2[3]);
                                    }
                                }
                                if (level + 1 < tile.getLevels()) {
                                    int[] neighbors1 = neighbors(i1, j1, nbRows, nbColumns, tile.getLevels() - (level + 1));
                                    int[] neighbors2 = neighbors(i2, j2, nbRows, nbColumns, tile.getLevels() - (level + 1));
                                    for (int k = 0; k < neighbors1.length; ++k) {
                                        if ((neighbors1[0] == neighbors2[0] &&
                                                neighbors1[1] == neighbors2[1] &&
                                                neighbors1[2] == neighbors2[2] &&
                                                neighbors1[3] == neighbors2[3])) {
                                            tile.getMergeLevel(i1, j1, i2, j2);
                                        }
                                        Assert.assertFalse(neighbors1[0] == neighbors2[0] &&
                                                           neighbors1[1] == neighbors2[1] &&
                                                           neighbors1[2] == neighbors2[2] &&
                                                           neighbors1[3] == neighbors2[3]);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    @Test
    public void testSubTilesLimits() throws RuggedException {
        for (int nbRows = 1; nbRows < 25; nbRows++) {
            for (int nbColumns = 1; nbColumns < 25; nbColumns++) {

                MinMaxTreeTile tile = createTile(nbRows, nbColumns);

                for (int level = 0; level < tile.getLevels(); ++level) {
                    int[] neighbors = neighbors(0, 0, nbRows, nbColumns, tile.getLevels() - level);
                    int subTileRows = neighbors[1] - neighbors[0];
                    int subTileCols = neighbors[3] - neighbors[2];
                    for (int i1 = 0; i1 < nbRows; ++i1) {
                        for (int i2 = 0; i2 < nbRows; ++i2) {
                            int[] crossings = tile.getCrossedBoundaryRows(i1, i2, level);
                            int[] ref       = multiples(i1, i2, subTileRows);
                            Assert.assertArrayEquals(ref, crossings);
                        }
                    }
                    for (int j1 = 0; j1 < nbColumns; ++j1) {
                        for (int j2 = 0; j2 < nbColumns; ++j2) {
                            int[] crossings = tile.getCrossedBoundaryColumns(j1, j2, level);
                            int[] ref       = multiples(j1, j2, subTileCols);
                            Assert.assertArrayEquals(ref, crossings);
                        }
                    }
                }
            }
        }
    }

    private int[] neighbors(int row, int column, int nbRows, int nbColumns, int stages) {

        // poor man identification of neighbors cells merged together with specified cell
        // this identification is intentionally independent of the MinMaxTreeTile class,
        // for testing purposes
        int rMin  = row;
        int rN    = 1;
        int rMask = -1;
        int cMin  = column;
        int cMask = -1;
        int cN    = 1;

        boolean mergeColumns = true;
        for (int i = 0; i < stages; ++i) {
            if (mergeColumns) {
                cMask = cMask << 1;
                cMin  = cMin & cMask;
                cN    = cN * 2;
            } else {
                rMask = rMask << 1;
                rMin  = rMin & rMask;
                rN    = rN * 2;
            }
            mergeColumns = !mergeColumns;
        }

        return new int[] {
            rMin, FastMath.min(rMin + rN, nbRows),
            cMin, FastMath.min(cMin + cN, nbColumns)
        };

    }

    private int[] multiples(int k1, int k2, int n) {

        // poor man identification of rows/columns crossings
        // this identification is intentionally independent of the MinMaxTreeTile class,
        // for testing purposes

        // intentionally dumb way of counting multiples of n
        int kS = FastMath.min(k1, k2);
        int kE = FastMath.max(k1, k2) + 1;
        int count = 0;
        for (int k = kS; k < kE; ++k) {
            if (k % n == 0) {
                ++count;
            }
        }

        int[] multiples = new int[count];
        int index = 0;
        for (int k = kS; k < kE; ++k) {
            if (k % n == 0) {
                multiples[index++] = k;
            }
        }

        if (k1 > k2) {
            // revert the array
            for (int i = 0; i < count / 2; ++i) {
                int tmp = multiples[i];
                multiples[i] = multiples[count - 1 - i];
                multiples[count - 1 - i] = tmp;
            }
        }

        return multiples;

    }

    private MinMaxTreeTile createTile(int nbRows, int nbColumns) throws RuggedException {
        MinMaxTreeTile tile = new MinMaxTreeTileFactory().createTile();
        tile.setGeometry(1.0, 2.0, 0.1, 0.2, nbRows, nbColumns);
        for (int i = 0; i < nbRows; ++i) {
            for (int j = 0; j < nbColumns; ++j) {
                tile.setElevation(i, j, i + 0.01 * j);
            }
        }
        tile.tileUpdateCompleted();
        return tile;
    }

}
