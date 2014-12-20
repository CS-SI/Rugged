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
package org.orekit.rugged.raster;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.util.FastMath;
import org.junit.Assert;
import org.junit.Test;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.rugged.errors.RuggedException;
import org.orekit.rugged.errors.RuggedMessages;
import org.orekit.rugged.raster.SimpleTile;
import org.orekit.rugged.raster.SimpleTileFactory;
import org.orekit.rugged.raster.Tile;
import org.orekit.rugged.raster.Tile.Location;

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
        Assert.assertEquals(0, tile.getMinElevation(), 1.0e-10);
        Assert.assertEquals(0, tile.getMaxElevation(), 1.0e-10);
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
        tile.tileUpdateCompleted();

        Assert.assertEquals(1.0, tile.getMinimumLatitude(), 1.0e-10);
        Assert.assertEquals(2.0, tile.getMinimumLongitude(), 1.0e-10);
        Assert.assertEquals(0.1, tile.getLatitudeStep(), 1.0e-10);
        Assert.assertEquals(0.2, tile.getLongitudeStep(), 1.0e-10);
        Assert.assertEquals(100, tile.getLatitudeRows());
        Assert.assertEquals(200, tile.getLongitudeColumns());
        Assert.assertEquals(0.0, tile.getMinElevation(), 1.0e-10);
        Assert.assertEquals(99199.0, tile.getMaxElevation(), 1.0e-10);

        Assert.assertEquals(Location.SOUTH_WEST, tile.getLocation( 0.0,  1.0));
        Assert.assertEquals(Location.WEST,       tile.getLocation( 6.0,  1.0));
        Assert.assertEquals(Location.NORTH_WEST, tile.getLocation(12.0,  1.0));
        Assert.assertEquals(Location.SOUTH,      tile.getLocation( 0.0, 22.0));
        Assert.assertEquals(Location.HAS_INTERPOLATION_NEIGHBORS,    tile.getLocation( 6.0, 22.0));
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
        tile.tileUpdateCompleted();
        checkOutOfBound( -1, 100, tile);
        checkOutOfBound(100, 100, tile);
        checkOutOfBound( 50,  -1, tile);
        checkOutOfBound( 50, 200, tile);
    }

    @Test
    public void testIndexShift() throws RuggedException {

        SimpleTile tile = new SimpleTileFactory().createTile();
        tile.setGeometry(1.0, 2.0, 0.1, 0.2, 100, 200);
        tile.setElevation(50, 100, 1000.0);
        tile.tileUpdateCompleted();

        // indices correspond to cells centers
        double latCenterColumn50 = tile.getLatitudeAtIndex(50);
        double latCenterColumn51 = tile.getLatitudeAtIndex(51);
        double lonCenterRow23    = tile.getLongitudeAtIndex(23);
        double lonCenterRow24    = tile.getLongitudeAtIndex(24);

        // getLatitudeIndex shift indices 1/2 cell, so that
        // the specified latitude is always between index and index+1
        // so despite latWestColumn51 is very close to column 51 center,
        // getLatitudeIndex should return 50
        double latWestColumn51   = 0.001 * latCenterColumn50 + 0.999 * latCenterColumn51;
        int retrievedLatIndex = tile.getFloorLatitudeIndex(latWestColumn51);
        Assert.assertEquals(50, retrievedLatIndex);
        Assert.assertTrue(tile.getLatitudeAtIndex(retrievedLatIndex) < latWestColumn51);
        Assert.assertTrue(latWestColumn51 < tile.getLatitudeAtIndex(retrievedLatIndex + 1));

        // getLongitudeIndex shift indices 1/2 cell, so that
        // the specified longitude is always between index and index+1
        // so despite lonSouthRow24 is very close to row 24 center,
        // getLongitudeIndex should return 23
        double lonSouthRow24     = 0.001 * lonCenterRow23    + 0.999 * lonCenterRow24;
        int retrievedLonIndex = tile.getFloorLongitudeIndex(lonSouthRow24);
        Assert.assertEquals(23, retrievedLonIndex);
        Assert.assertTrue(tile.getLongitudeAtIndex(retrievedLonIndex) < lonSouthRow24);
        Assert.assertTrue(lonSouthRow24 < tile.getLongitudeAtIndex(retrievedLonIndex + 1));

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
        tile.tileUpdateCompleted();
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
        tile.tileUpdateCompleted();
        // the following points are 1/16 cell out of tile
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
        tile.tileUpdateCompleted();
        // the following points are 3/16 cell out of tile
        checkOutOfBound(-0.1875,  0.5,    tile);
        checkOutOfBound( 1.1875,  0.5,    tile);
        checkOutOfBound( 0.5,    -0.1875, tile);
        checkOutOfBound( 0.5,     1.1875, tile);
    }

    @Test
    public void testCellIntersection() throws RuggedException {
        SimpleTile tile = new SimpleTileFactory().createTile();
        tile.setGeometry(0.0, 0.0, 0.025, 0.025, 50, 50);
        tile.setElevation(20, 14,  91.0);
        tile.setElevation(20, 15, 210.0);
        tile.setElevation(21, 14, 162.0);
        tile.setElevation(21, 15,  95.0);
        tile.tileUpdateCompleted();
        GeodeticPoint gpA = new GeodeticPoint(tile.getLatitudeAtIndex(20)  + 0.1 * tile.getLatitudeStep(),
                                              tile.getLongitudeAtIndex(14) + 0.2 * tile.getLongitudeStep(),
                                              300.0);
        GeodeticPoint gpB = new GeodeticPoint(tile.getLatitudeAtIndex(20)  + 0.7 * tile.getLatitudeStep(),
                                              tile.getLongitudeAtIndex(14) + 0.9 * tile.getLongitudeStep(),
                                              10.0);
        GeodeticPoint gpIAB = tile.cellIntersection(gpA, los(gpA, gpB), 20, 14);
        checkInLine(gpA, gpB, gpIAB);
        checkOnTile(tile, gpIAB);
        GeodeticPoint gpIBA = tile.cellIntersection(gpB, los(gpB, gpA), 20, 14);
        checkInLine(gpA, gpB, gpIBA);
        checkOnTile(tile, gpIBA);

        Assert.assertEquals(gpIAB.getLatitude(),  gpIBA.getLatitude(),  1.0e-10);
        Assert.assertEquals(gpIAB.getLongitude(), gpIBA.getLongitude(), 1.0e-10);
        Assert.assertEquals(gpIAB.getAltitude(),  gpIBA.getAltitude(),  1.0e-10);

    }

    @Test
    public void testCellIntersection2Solutions() throws RuggedException {
        SimpleTile tile = new SimpleTileFactory().createTile();
        tile.setGeometry(0.0, 0.0, 0.025, 0.025, 50, 50);
        tile.setElevation(20, 14,  91.0);
        tile.setElevation(20, 15, 210.0);
        tile.setElevation(21, 14, 162.0);
        tile.setElevation(21, 15,  95.0);
        tile.tileUpdateCompleted();
        GeodeticPoint gpA = new GeodeticPoint(tile.getLatitudeAtIndex(20)  + 0.1 * tile.getLatitudeStep(),
                                              tile.getLongitudeAtIndex(14) + 0.2 * tile.getLongitudeStep(),
                                              120.0);
        GeodeticPoint gpB = new GeodeticPoint(tile.getLatitudeAtIndex(20)  + 0.7 * tile.getLatitudeStep(),
                                              tile.getLongitudeAtIndex(14) + 0.9 * tile.getLongitudeStep(),
                                              130.0);

        // the line from gpA to gpB should traverse the DEM twice within the tile
        // we use the points in the two different orders to retrieve both solutions
        GeodeticPoint gpIAB = tile.cellIntersection(gpA, los(gpA, gpB), 20, 14);
        checkInLine(gpA, gpB, gpIAB);
        checkOnTile(tile, gpIAB);
        GeodeticPoint gpIBA = tile.cellIntersection(gpB, los(gpB, gpA), 20, 14);
        checkInLine(gpA, gpB, gpIBA);
        checkOnTile(tile, gpIBA);

        // the two solutions are different
        Assert.assertEquals(120.231, gpIAB.getAltitude(), 1.0e-3);
        Assert.assertEquals(130.081, gpIBA.getAltitude(), 1.0e-3);

    }

    @Test
    public void testCellIntersectionNoSolutions() throws RuggedException {
        SimpleTile tile = new SimpleTileFactory().createTile();
        tile.setGeometry(0.0, 0.0, 0.025, 0.025, 50, 50);
        tile.setElevation(20, 14,  91.0);
        tile.setElevation(20, 15, 210.0);
        tile.setElevation(21, 14, 162.0);
        tile.setElevation(21, 15,  95.0);
        tile.tileUpdateCompleted();
        GeodeticPoint gpA = new GeodeticPoint(tile.getLatitudeAtIndex(20)  + 0.1 * tile.getLatitudeStep(),
                                              tile.getLongitudeAtIndex(14) + 0.2 * tile.getLongitudeStep(),
                                              180.0);
        GeodeticPoint gpB = new GeodeticPoint(tile.getLatitudeAtIndex(20)  + 0.7 * tile.getLatitudeStep(),
                                              tile.getLongitudeAtIndex(14) + 0.9 * tile.getLongitudeStep(),
                                              190.0);

        Assert.assertNull(tile.cellIntersection(gpA, los(gpA, gpB), 20, 14));

    }

    @Test
    public void testCellIntersectionLinearOnly() throws RuggedException {
        SimpleTile tile = new SimpleTileFactory().createTile();
        tile.setGeometry(0.0, 0.0, 0.025, 0.025, 50, 50);
        tile.setElevation(0, 0,  30.0);
        tile.setElevation(0, 1,  30.0);
        tile.setElevation(1, 0,  40.0);
        tile.setElevation(1, 1,  40.0);
        tile.tileUpdateCompleted();
        GeodeticPoint gpA = new GeodeticPoint(tile.getLatitudeAtIndex(0)  + 0.25 * tile.getLatitudeStep(),
                                              tile.getLongitudeAtIndex(0) + 0.50 * tile.getLongitudeStep(),
                                              50.0);
        GeodeticPoint gpB = new GeodeticPoint(tile.getLatitudeAtIndex(0)  + 0.75 * tile.getLatitudeStep(),
                                              tile.getLongitudeAtIndex(0) + 0.50 * tile.getLongitudeStep(),
                                              20.0);

        GeodeticPoint gpIAB = tile.cellIntersection(gpA, los(gpA, gpB), 0, 0);
        checkInLine(gpA, gpB, gpIAB);
        checkOnTile(tile, gpIAB);
        GeodeticPoint gpIBA = tile.cellIntersection(gpB, los(gpB, gpA), 0, 0);
        checkInLine(gpA, gpB, gpIBA);
        checkOnTile(tile, gpIBA);

        Assert.assertEquals(gpIAB.getLatitude(),  gpIBA.getLatitude(),  1.0e-10);
        Assert.assertEquals(gpIAB.getLongitude(), gpIBA.getLongitude(), 1.0e-10);
        Assert.assertEquals(gpIAB.getAltitude(),  gpIBA.getAltitude(),  1.0e-10);

    }

    @Test
    public void testCellIntersectionLinearIntersectionOutside() throws RuggedException {
        SimpleTile tile = new SimpleTileFactory().createTile();
        tile.setGeometry(0.0, 0.0, 0.025, 0.025, 50, 50);
        tile.setElevation(0, 0,  30.0);
        tile.setElevation(0, 1,  30.0);
        tile.setElevation(1, 0,  40.0);
        tile.setElevation(1, 1,  40.0);
        tile.tileUpdateCompleted();
        GeodeticPoint gpA = new GeodeticPoint(tile.getLatitudeAtIndex(0)  + 0.25 * tile.getLatitudeStep(),
                                              tile.getLongitudeAtIndex(0) + 0.50 * tile.getLongitudeStep(),
                                              45.0);
        GeodeticPoint gpB = new GeodeticPoint(tile.getLatitudeAtIndex(0)  + 0.75 * tile.getLatitudeStep(),
                                              tile.getLongitudeAtIndex(0) + 0.50 * tile.getLongitudeStep(),
                                              55.0);

       Assert.assertNull(tile.cellIntersection(gpA, los(gpA, gpB), 0, 0));

    }

    @Test
    public void testCellIntersectionLinearNoIntersection() throws RuggedException {
        SimpleTile tile = new SimpleTileFactory().createTile();
        tile.setGeometry(0.0, 0.0, 0.025, 0.025, 50, 50);
        tile.setElevation(0, 0,  30.0);
        tile.setElevation(0, 1,  30.0);
        tile.setElevation(1, 0,  40.0);
        tile.setElevation(1, 1,  40.0);
        tile.tileUpdateCompleted();
        GeodeticPoint gpA = new GeodeticPoint(tile.getLatitudeAtIndex(0)  + 0.25 * tile.getLatitudeStep(),
                                              tile.getLongitudeAtIndex(0) + 0.50 * tile.getLongitudeStep(),
                                              45.0);
        GeodeticPoint gpB = new GeodeticPoint(tile.getLatitudeAtIndex(0)  + 0.75 * tile.getLatitudeStep(),
                                              tile.getLongitudeAtIndex(0) + 0.50 * tile.getLongitudeStep(),
                                              50.0);

        Assert.assertNull(tile.cellIntersection(gpA, los(gpA, gpB), 0, 0));

    }

    @Test
    public void testCellIntersectionConstant0() throws RuggedException {
        SimpleTile tile = new SimpleTileFactory().createTile();
        tile.setGeometry(0.0, 0.0, 0.025, 0.025, 50, 50);
        tile.setElevation(0, 0,  30.0);
        tile.setElevation(0, 1,  30.0);
        tile.setElevation(1, 0,  40.0);
        tile.setElevation(1, 1,  40.0);
        tile.tileUpdateCompleted();
        GeodeticPoint gpA = new GeodeticPoint(tile.getLatitudeAtIndex(0)  + 0.25 * tile.getLatitudeStep(),
                                              tile.getLongitudeAtIndex(0) + 0.50 * tile.getLongitudeStep(),
                                              32.5);
        GeodeticPoint gpB = new GeodeticPoint(tile.getLatitudeAtIndex(0)  + 0.75 * tile.getLatitudeStep(),
                                              tile.getLongitudeAtIndex(0) + 0.50 * tile.getLongitudeStep(),
                                              37.5);

        GeodeticPoint gpIAB = tile.cellIntersection(gpA, los(gpA, gpB), 0, 0);
        checkInLine(gpA, gpB, gpIAB);
        checkOnTile(tile, gpIAB);
        GeodeticPoint gpIBA = tile.cellIntersection(gpB, los(gpB, gpA), 0, 0);
        checkInLine(gpA, gpB, gpIBA);
        checkOnTile(tile, gpIBA);

        Assert.assertEquals(gpIAB.getLatitude(),  gpA.getLatitude(),  1.0e-10);
        Assert.assertEquals(gpIAB.getLongitude(), gpA.getLongitude(), 1.0e-10);
        Assert.assertEquals(gpIAB.getAltitude(),  gpA.getAltitude(),  1.0e-10);
        Assert.assertEquals(gpIBA.getLatitude(),  gpB.getLatitude(),  1.0e-10);
        Assert.assertEquals(gpIBA.getLongitude(), gpB.getLongitude(), 1.0e-10);
        Assert.assertEquals(gpIBA.getAltitude(),  gpB.getAltitude(),  1.0e-10);

    }

    private Vector3D los(GeodeticPoint gpA, GeodeticPoint gpB) {
        // this is a crude conversion into geodetic space
        // intended *only* for the purposes of these tests
        // it considers the geodetic space *is* perfectly Cartesian
        // in the East, North, Zenith frame
        return new Vector3D(gpB.getLongitude() - gpA.getLongitude(),
                            gpB.getLatitude()  - gpA.getLatitude(),
                            gpB.getAltitude()  - gpA.getAltitude());
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

    private void checkInLine(GeodeticPoint gpA, GeodeticPoint gpB, GeodeticPoint gpI) {

        double t = (gpI.getAltitude() - gpA.getAltitude()) / (gpB.getAltitude() - gpA.getAltitude());

        Assert.assertEquals(gpI.getLatitude(),
                            gpA.getLatitude() * (1 - t) + gpB.getLatitude() * t,
                            1.0e-10);

        Assert.assertEquals(gpI.getLongitude(),
                            gpA.getLongitude() * (1 - t) + gpB.getLongitude() * t,
                            1.0e-10);

    }

    private void checkOnTile(Tile tile, GeodeticPoint gpI)
        throws RuggedException {
        Assert.assertEquals(gpI.getAltitude(),
                            tile.interpolateElevation(gpI.getLatitude(), gpI.getLongitude()),
                            1.0e-10);
    }

}
