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
package orekit.rugged.core.dem;

import org.junit.Assert;
import org.junit.Test;
import org.orekit.rugged.api.RuggedException;
import org.orekit.rugged.api.RuggedMessages;
import org.orekit.rugged.core.dem.SimpleTile;
import org.orekit.rugged.core.dem.Tile;

public class SimpleTileTest {

    @Test
    public void testEmpty() {
        SimpleTile tile = new SimpleTile();
        Assert.assertEquals(0, tile.getMinimumLatitude(), 1.0e-10);
        Assert.assertEquals(0, tile.getMinimumLongitude(), 1.0e-10);
        Assert.assertEquals(0, tile.getLatitudeStep(), 1.0e-10);
        Assert.assertEquals(0, tile.getLongitudeStep(), 1.0e-10);
        Assert.assertEquals(0, tile.getLatitudeRows());
        Assert.assertEquals(0, tile.getLongitudeColumns());
    }

    @Test
    public void testUpdate() throws RuggedException {

        SimpleTile tile = new SimpleTile();
        tile.setGeometry(1.0, 2.0, 0.1, 0.2, 100, 200);
        for (int i = 0; i < tile.getLatitudeRows(); ++i) {
            for (int j = 0; j < tile.getLongitudeColumns(); ++j) {
                tile.setElevation(i, j, 1000 * i + j);
            }
        }

        Assert.assertEquals(1.0, tile.getMinimumLatitude(), 1.0e-10);
        Assert.assertEquals(2.0, tile.getMinimumLongitude(), 1.0e-10);
        Assert.assertEquals(0.1, tile.getLatitudeStep(), 1.0e-10);
        Assert.assertEquals(0.2, tile.getLongitudeStep(), 1.0e-10);
        Assert.assertEquals(100, tile.getLatitudeRows());
        Assert.assertEquals(200, tile.getLongitudeColumns());

        Assert.assertTrue(tile.covers(  6.0, 22.0));
        Assert.assertFalse(tile.covers( 0.0, 22.0));
        Assert.assertFalse(tile.covers(12.0, 22.0));
        Assert.assertFalse(tile.covers( 6.0,  1.0));
        Assert.assertFalse(tile.covers( 6.0, 43.0));

        for (int i = 0; i < tile.getLatitudeRows(); ++i) {
            for (int j = 0; j < tile.getLongitudeColumns(); ++j) {
                Assert.assertEquals(1000 * i + j, tile.getElevationAtIndices(i, j), 1.0e-10);
            }
        }

    }

    @Test
    public void testOutOfBounds() throws RuggedException {

        SimpleTile tile = new SimpleTile();
        tile.setGeometry(1.0, 2.0, 0.1, 0.2, 100, 200);
        tile.setElevation(50, 100, 1000.0);
        checkOutOfBound( -1, 100, tile);
        checkOutOfBound(100, 100, tile);
        checkOutOfBound( 50,  -1, tile);
        checkOutOfBound( 50, 200, tile);
    }

    private void checkOutOfBound(int i, int j, Tile tile) {
        try {
            tile.setElevation(i, j, 1000.0);
        } catch (RuggedException re) {
            Assert.assertEquals(RuggedMessages.OUT_OF_TILE_INDICES, re.getSpecifier());
            Assert.assertEquals(i,                              ((Integer) re.getParts()[0]).intValue());
            Assert.assertEquals(j,                              ((Integer) re.getParts()[1]).intValue());
            Assert.assertEquals(tile.getLatitudeRows() - 1,     ((Integer) re.getParts()[2]).intValue());
            Assert.assertEquals(tile.getLongitudeColumns() - 1, ((Integer) re.getParts()[3]).intValue());
        }
    }

}
