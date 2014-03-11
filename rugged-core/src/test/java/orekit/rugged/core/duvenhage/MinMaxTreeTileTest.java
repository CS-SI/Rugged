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
package orekit.rugged.core.duvenhage;

import java.lang.reflect.Field;

import org.apache.commons.math3.util.FastMath;
import org.junit.Assert;
import org.junit.Test;
import org.orekit.rugged.api.RuggedException;
import org.orekit.rugged.core.duvenhage.MinMaxTreeTile;
import org.orekit.rugged.core.duvenhage.MinMaxTreeTileFactory;

public class MinMaxTreeTileTest {

    @Test
    public void testSizeTall()
        throws RuggedException, SecurityException, NoSuchFieldException,
               IllegalArgumentException, IllegalAccessException {
        MinMaxTreeTile tile = new MinMaxTreeTileFactory().createTile();
        tile.setGeometry(1.0, 2.0, 0.1, 0.2, 107, 19);
        tile.tileUpdateCompleted();
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
        MinMaxTreeTile tile = new MinMaxTreeTileFactory().createTile();
        tile.setGeometry(1.0, 2.0, 0.1, 0.2, 4, 7);
        tile.tileUpdateCompleted();
        Assert.assertEquals(4, tile.getLevels());

        Field startField = MinMaxTreeTile.class.getDeclaredField("start");
        startField.setAccessible(true);
        int[] start = (int[]) startField.get(tile);
        Assert.assertEquals( 0, start[ 0]);
        Assert.assertEquals( 2, start[ 1]);
        Assert.assertEquals( 6, start[ 2]);
        Assert.assertEquals(14, start[ 3]);

        Field minTreeField = MinMaxTreeTile.class.getDeclaredField("minTree");
        minTreeField.setAccessible(true);
        Assert.assertEquals(30, ((double[]) minTreeField.get(tile)).length);
        Field maxTreeField = MinMaxTreeTile.class.getDeclaredField("maxTree");
        maxTreeField.setAccessible(true);
        Assert.assertEquals(30, ((double[]) maxTreeField.get(tile)).length);

    }

    @Test
    public void testSinglePixel() throws RuggedException {
        MinMaxTreeTile tile = new MinMaxTreeTileFactory().createTile();
        tile.setGeometry(1.0, 2.0, 0.1, 0.2, 1, 1);
        tile.setElevation(0, 0, 2.5);
        tile.tileUpdateCompleted();
        Assert.assertEquals(0, tile.getLevels());
    }

    @Test
    public void testMinMax() throws RuggedException {
        for (int nbRows = 1; nbRows < 25; nbRows++) {
            for (int nbColumns = 1; nbColumns < 25; nbColumns++) {
                checkMinMax(nbRows, nbColumns);
            }
        }
    }

    private void checkMinMax(int nbRows, int nbColumns)
        throws RuggedException {

        MinMaxTreeTile tile = new MinMaxTreeTileFactory().createTile();
        tile.setGeometry(1.0, 2.0, 0.1, 0.2, nbRows, nbColumns);
        for (int i = 0; i < nbRows; ++i) {
            for (int j = 0; j < nbColumns; ++j) {
                tile.setElevation(i, j, i + 0.01 * j);
            }
        }
        tile.tileUpdateCompleted();

        for (int level = 0; level < tile.getLevels(); level++) {
            for (int row = 0; row < nbRows; row++) {
                for (int column = 0; column < nbColumns; column++) {

                    // reference min and max
                    int[] neighbors = neighbors(row, column, nbRows, nbColumns, tile.getLevels() - level);
                    double min = Double.POSITIVE_INFINITY;
                    double max = Double.NEGATIVE_INFINITY;
                    for (int i = neighbors[0]; i < neighbors[1]; ++i) {
                        for (int j = neighbors[2]; j < neighbors[3]; ++j) {
                            double pixelValue = tile.getElevationAtIndices(i, j);
                            min = FastMath.min(min, pixelValue);
                            max = FastMath.max(max, pixelValue);
                        }
                    }

                    Assert.assertEquals(min, tile.getMinElevation(row, column, level), 1.0e-10 * min);
                    Assert.assertEquals(max, tile.getMaxElevation(row, column, level), 1.0e-10 * max);
                }
            }
        }

    }

    private int[] neighbors(int row, int column, int nbRows, int nbColumns, int stages) {

        // poor man identification of neighbors cells merged together with specified cell
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

}
