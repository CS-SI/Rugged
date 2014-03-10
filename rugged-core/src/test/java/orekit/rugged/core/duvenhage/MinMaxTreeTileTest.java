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

import org.apache.commons.math3.random.RandomGenerator;
import org.apache.commons.math3.random.Well19937a;
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
        Assert.assertEquals(   0, start[ 8]);
        Assert.assertEquals(   7, start[ 7]);
        Assert.assertEquals(  21, start[ 6]);
        Assert.assertEquals(  49, start[ 5]);
        Assert.assertEquals(  91, start[ 4]);
        Assert.assertEquals( 172, start[ 3]);
        Assert.assertEquals( 307, start[ 2]);
        Assert.assertEquals( 577, start[ 1]);
        Assert.assertEquals(1117, start[ 0]);

        Field rowsField = MinMaxTreeTile.class.getDeclaredField("rows");
        rowsField.setAccessible(true);
        int[] rows = (int[]) rowsField.get(tile);
        Assert.assertEquals(   7, rows[ 8]);
        Assert.assertEquals(   7, rows[ 7]);
        Assert.assertEquals(  14, rows[ 6]);
        Assert.assertEquals(  14, rows[ 5]);
        Assert.assertEquals(  27, rows[ 4]);
        Assert.assertEquals(  27, rows[ 3]);
        Assert.assertEquals(  54, rows[ 2]);
        Assert.assertEquals(  54, rows[ 1]);
        Assert.assertEquals( 107, rows[ 0]);

        Field columnsField = MinMaxTreeTile.class.getDeclaredField("columns");
        columnsField.setAccessible(true);
        int[] columns = (int[]) columnsField.get(tile);
        Assert.assertEquals(  1, columns[ 8]);
        Assert.assertEquals(  2, columns[ 7]);
        Assert.assertEquals(  2, columns[ 6]);
        Assert.assertEquals(  3, columns[ 5]);
        Assert.assertEquals(  3, columns[ 4]);
        Assert.assertEquals(  5, columns[ 3]);
        Assert.assertEquals(  5, columns[ 2]);
        Assert.assertEquals( 10, columns[ 1]);
        Assert.assertEquals( 10, columns[ 0]);
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
        Assert.assertEquals( 0, start[ 3]);
        Assert.assertEquals( 2, start[ 2]);
        Assert.assertEquals( 6, start[ 1]);
        Assert.assertEquals(14, start[ 0]);

        Field rowsField = MinMaxTreeTile.class.getDeclaredField("rows");
        rowsField.setAccessible(true);
        int[] rows = (int[]) rowsField.get(tile);
        Assert.assertEquals( 1, rows[ 3]);
        Assert.assertEquals( 2, rows[ 2]);
        Assert.assertEquals( 2, rows[ 1]);
        Assert.assertEquals( 4, rows[ 0]);

        Field columnsField = MinMaxTreeTile.class.getDeclaredField("columns");
        columnsField.setAccessible(true);
        int[] columns = (int[]) columnsField.get(tile);
        Assert.assertEquals( 2, columns[ 3]);
        Assert.assertEquals( 2, columns[ 2]);
        Assert.assertEquals( 4, columns[ 1]);
        Assert.assertEquals( 4, columns[ 0]);

        Field minTreeField = MinMaxTreeTile.class.getDeclaredField("minTree");
        minTreeField.setAccessible(true);
        Assert.assertEquals(30, ((double[]) minTreeField.get(tile)).length);
        Field maxTreeField = MinMaxTreeTile.class.getDeclaredField("maxTree");
        maxTreeField.setAccessible(true);
        Assert.assertEquals(30, ((double[]) maxTreeField.get(tile)).length);
    }

    @Test
    public void testMinMax() throws RuggedException {
        RandomGenerator random = new Well19937a(0xfbbc1d1739b23555l);
        checkMinMax(4, 7, 100, random);
    }

    private void checkMinMax(int nbRows, int nbColumns, int nbChecks, RandomGenerator random)
        throws RuggedException {

        MinMaxTreeTile tile = new MinMaxTreeTileFactory().createTile();
        tile.setGeometry(1.0, 2.0, 0.1, 0.2, nbRows, nbColumns);
        for (int i = 0; i < nbRows; ++i) {
            for (int j = 0; j < nbColumns; ++j) {
                tile.setElevation(i, j, 1000 * random.nextDouble());
            }
        }
        tile.tileUpdateCompleted();

        for (int k = 0; k < nbChecks; ++k) {
            int row    = random.nextInt(nbRows);
            int column = random.nextInt(nbColumns);
            int level  = random.nextInt(tile.getLevels());

            // reference min and max
            int[] neighbors = neighbors(row, column, nbRows, nbColumns, level);
            System.out.println(row + " " + column + " (" + level + "): [" +
                               neighbors[0] + ", " + neighbors[1] + "] [" +
                               neighbors[2] + ", " + neighbors[3] + "]");
            double min = Double.POSITIVE_INFINITY;
            double max = Double.POSITIVE_INFINITY;
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

    private int[] neighbors(int row, int column, int nbRows, int nbColumns, int level) {

        // poor man identification of neighbors cells merged together with specified cell
        int rMin = 0;
        int rMax = row;
        int cMin = 0;
        int cMax = column;

        boolean mergeColumns = true;
        for (int i = 0; i <= level; ++i) {
            if (mergeColumns) {
                int split = (nbColumns + 1) / 2;
                if (column < split) {
                    cMax = split;
                } else {
                    cMin = split;
                }
                nbColumns = cMax - cMin;
            } else {
                int split = (nbRows + 1) / 2;
                if (row < split) {
                    rMax = split;
                } else {
                    rMin = split;
                }
                nbRows = rMax - rMin;
            }
            mergeColumns = !mergeColumns;
        }

        return new int[] { rMin, rMax, cMin, cMax };

    }

}
