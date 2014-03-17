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
package org.orekit.rugged.core.raster;

import org.apache.commons.math3.util.FastMath;
import org.junit.Assert;
import org.junit.Test;
import org.orekit.rugged.api.RuggedException;
import org.orekit.rugged.api.RuggedMessages;
import org.orekit.rugged.core.raster.SimpleTile;
import org.orekit.rugged.core.raster.SimpleTileFactory;
import org.orekit.rugged.core.raster.Tile;
import org.orekit.rugged.core.raster.Tile.Location;

public class SimpleTileTest {

    @Test
    public void testNotConfigured() {
        SimpleTile tile = new SimpleTileFactory().createTile();
        Assert.assertEquals(0, tile.getMinimumLatitude(), 1.0e-10);
        Assert.assertEquals(0, tile.getMinimumLongitude(), 1.0e-10);
        Assert.assertEquals(0, tile.getLatitudeStep(), 1.0e-10);
        Assert.assertEquals(0, tile.getLongitudeStep(), 1.0e-10);
        Assert.assertEquals(0, tile.getLatitudeRows());
        Assert.assertEquals(0, tile.getLongitudeColumns());
    }

    @Test
    public void testEmpty() {
        SimpleTile tile = new SimpleTileFactory().createTile();
        try {
            tile.setGeometry(1.0, 2.0, 0.1, 0.2, 0, 200);
            Assert.fail("an exception should have been thrown");
        } catch (RuggedException re) {
            Assert.assertEquals(RuggedMessages.EMPTY_TILE, re.getSpecifier());
            Assert.assertEquals(  0, ((Integer) re.getParts()[0]).intValue());
            Assert.assertEquals(200, ((Integer) re.getParts()[1]).intValue());
        }
        try {
            tile.setGeometry(1.0, 2.0, 0.1, 0.2, 100, 0);
            Assert.fail("an exception should have been thrown");
        } catch (RuggedException re) {
            Assert.assertEquals(RuggedMessages.EMPTY_TILE, re.getSpecifier());
            Assert.assertEquals(100, ((Integer) re.getParts()[0]).intValue());
            Assert.assertEquals(  0, ((Integer) re.getParts()[1]).intValue());
        }
    }

    @Test
    public void testUpdate() throws RuggedException {

        SimpleTile tile = new SimpleTileFactory().createTile();
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

        Assert.assertEquals(Location.SOUTH_WEST, tile.getLocation( 0.0,  1.0));
        Assert.assertEquals(Location.WEST,       tile.getLocation( 6.0,  1.0));
        Assert.assertEquals(Location.NORTH_WEST, tile.getLocation(12.0,  1.0));
        Assert.assertEquals(Location.SOUTH,      tile.getLocation( 0.0, 22.0));
        Assert.assertEquals(Location.IN_TILE,    tile.getLocation( 6.0, 22.0));
        Assert.assertEquals(Location.NORTH,      tile.getLocation(12.0, 22.0));
        Assert.assertEquals(Location.SOUTH_EAST, tile.getLocation( 0.0, 43.0));
        Assert.assertEquals(Location.EAST,       tile.getLocation( 6.0, 43.0));
        Assert.assertEquals(Location.NORTH_EAST, tile.getLocation(12.0, 43.0));
        for (int i = 0; i < tile.getLatitudeRows(); ++i) {
            for (int j = 0; j < tile.getLongitudeColumns(); ++j) {
                Assert.assertEquals(1000 * i + j, tile.getElevationAtIndices(i, j), 1.0e-10);
            }
        }

    }

    @Test
    public void testOutOfBoundsIndices() throws RuggedException {

        SimpleTile tile = new SimpleTileFactory().createTile();
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

    @Test
    public void testInterpolation() throws RuggedException {
        SimpleTile tile = new SimpleTileFactory().createTile();
        tile.setGeometry(0.0, 0.0, 1.0, 1.0, 50, 50);
        tile.setElevation(20, 14,  91.0);
        tile.setElevation(20, 15, 210.0);
        tile.setElevation(21, 14, 162.0);
        tile.setElevation(21, 15,  95.0);
        Assert.assertEquals(150.5, tile.interpolateElevation(20.0, 14.5), 1.0e-10);
        Assert.assertEquals(128.5, tile.interpolateElevation(21.0, 14.5), 1.0e-10);
        Assert.assertEquals(146.1, tile.interpolateElevation(20.2, 14.5), 1.0e-10);
    }

    @Test
    public void testInterpolationWithinTolerance() throws RuggedException {
        SimpleTile tile = new SimpleTileFactory().createTile();
        tile.setGeometry(0.0, 0.0, 1.0, 1.0, 2, 2);
        tile.setElevation(0, 0,  91.0);
        tile.setElevation(0, 1, 210.0);
        tile.setElevation(1, 0, 162.0);
        tile.setElevation(1, 1,  95.0);
        // the following points are 1/16 pixel out of tile
        Assert.assertEquals(151.875, tile.interpolateElevation(-0.0625,  0.5),    1.0e-10);
        Assert.assertEquals(127.125, tile.interpolateElevation( 1.0625,  0.5),    1.0e-10);
        Assert.assertEquals(124.875, tile.interpolateElevation( 0.5,    -0.0625), 1.0e-10);
        Assert.assertEquals(154.125, tile.interpolateElevation( 0.5,     1.0625), 1.0e-10);
    }

    @Test
    public void testInterpolationOutOfTolerance() throws RuggedException {
        SimpleTile tile = new SimpleTileFactory().createTile();
        tile.setGeometry(0.0, 0.0, 1.0, 1.0, 2, 2);
        tile.setElevation(0, 0,  91.0);
        tile.setElevation(0, 1, 210.0);
        tile.setElevation(1, 0, 162.0);
        tile.setElevation(1, 1,  95.0);
        // the following points are 3/16 pixel out of tile
        checkOutOfBound(-0.1875,  0.5,    tile);
        checkOutOfBound( 1.1875,  0.5,    tile);
        checkOutOfBound( 0.5,    -0.1875, tile);
        checkOutOfBound( 0.5,     1.1875, tile);
    }

    private void checkOutOfBound(double latitude, double longitude, Tile tile) {
        try {
            tile.interpolateElevation(latitude, longitude);
        } catch (RuggedException re) {
            Assert.assertEquals(RuggedMessages.OUT_OF_TILE_ANGLES, re.getSpecifier());
            Assert.assertEquals(FastMath.toDegrees(latitude),
                                ((Double) re.getParts()[0]).doubleValue(),
                                1.0e-10);
            Assert.assertEquals(FastMath.toDegrees(longitude),
                                ((Double) re.getParts()[1]).doubleValue(),
                                1.0e-10);
            Assert.assertEquals(FastMath.toDegrees(tile.getMinimumLatitude()),
                                ((Double) re.getParts()[2]).doubleValue(),
                                1.0e-10);
            Assert.assertEquals(FastMath.toDegrees(tile.getMaximumLatitude()),
                                ((Double) re.getParts()[3]).doubleValue(),
                                1.0e-10);
            Assert.assertEquals(FastMath.toDegrees(tile.getMinimumLongitude()),
                                ((Double) re.getParts()[4]).doubleValue(),
                                1.0e-10);
            Assert.assertEquals(FastMath.toDegrees(tile.getMaximumLongitude()),
                                ((Double) re.getParts()[5]).doubleValue(),
                                1.0e-10);
        }
    }

}
