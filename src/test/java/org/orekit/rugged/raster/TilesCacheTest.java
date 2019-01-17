/* Copyright 2013-2019 CS Systèmes d'Information
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

import java.io.File;
import java.net.URISyntaxException;

import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.random.RandomGenerator;
import org.hipparchus.random.Well19937a;
import org.hipparchus.util.FastMath;
import org.junit.Assert;
import org.junit.Test;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.data.DataProvidersManager;
import org.orekit.data.DirectoryCrawler;
import org.orekit.frames.FramesFactory;
import org.orekit.rugged.errors.RuggedException;
import org.orekit.rugged.errors.RuggedMessages;
import org.orekit.rugged.intersection.IntersectionAlgorithm;
import org.orekit.rugged.intersection.duvenhage.DuvenhageAlgorithm;
import org.orekit.rugged.utils.ExtendedEllipsoid;
import org.orekit.utils.Constants;
import org.orekit.utils.IERSConventions;

public class TilesCacheTest {

    @Test
    public void testSingleTile() {
        CountingFactory factory = new CountingFactory();
        TilesCache<SimpleTile> cache = new TilesCache<SimpleTile>(factory,
                new CheckedPatternElevationUpdater(FastMath.toRadians(3.0), 11, 10.0, 20.0), 1000);
        SimpleTile tile = cache.getTile(FastMath.toRadians(-23.2), FastMath.toRadians(137.5));
        Assert.assertEquals(1, factory.getCount());
        Assert.assertEquals(-24.0, FastMath.toDegrees(tile.getMinimumLatitude()),  1.0e-10);
        Assert.assertEquals(135.0, FastMath.toDegrees(tile.getMinimumLongitude()), 1.0e-10);
        Assert.assertEquals(  0.3, FastMath.toDegrees(tile.getLatitudeStep()),     1.0e-10);
        Assert.assertEquals(  0.3, FastMath.toDegrees(tile.getLongitudeStep()),    1.0e-10);
        Assert.assertEquals(10.0, tile.getMinElevation(), 1.0e-10);
        Assert.assertEquals(20.0, tile.getMaxElevation(), 1.0e-10);
    }

    @Test
    public void testEviction() {
        CountingFactory factory = new CountingFactory();
        TilesCache<SimpleTile> cache = new TilesCache<SimpleTile>(factory,
                new CheckedPatternElevationUpdater(FastMath.toRadians(1.0), 11, 10.0, 20.0), 12);

        // fill up the 12 tiles we can keep in cache
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 3; ++j) {
                cache.getTile(FastMath.toRadians(0.5 + j), FastMath.toRadians(0.5 + i));
            }
        }
        Assert.assertEquals(12, factory.getCount());

        // keep using the same tiles for a while
        RandomGenerator generator = new Well19937a(0xf556baa5977435c5l);
        for (int i = 0; i < 10000; ++i) {
            double lat = 3.0 * generator.nextDouble();
            double lon = 4.0 * generator.nextDouble();
            cache.getTile(FastMath.toRadians(lat), FastMath.toRadians(lon));
        }
        Assert.assertEquals(12, factory.getCount());

        // ensure the (0.0, 0.0) tile is the least recently used one
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 3; ++j) {
                cache.getTile(FastMath.toRadians(0.5 + j), FastMath.toRadians(0.5 + i));
            }
        }

        // ask for one point outside of the covered area, to evict the (0.0, 0.0) tile
        cache.getTile(FastMath.toRadians(20.5), FastMath.toRadians(30.5));
        Assert.assertEquals(13, factory.getCount());

        // ask again for one point in the evicted tile which must be reallocated
        cache.getTile(FastMath.toRadians(0.5), FastMath.toRadians(0.5));
        Assert.assertEquals(14, factory.getCount());

        // the 13th allocated tile should still be there
        cache.getTile(FastMath.toRadians(20.5), FastMath.toRadians(30.5));
        Assert.assertEquals(14, factory.getCount());

        // evict all the tiles, going to a completely different zone
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 3; ++j) {
                cache.getTile(FastMath.toRadians(40.5 + i), FastMath.toRadians(90.5 + j));
            }
        }
        Assert.assertEquals(26, factory.getCount());

    }

    @Test
    public void testExactEnd() {
        CountingFactory factory = new CountingFactory();
        TilesCache<SimpleTile> cache =
                new TilesCache<SimpleTile>(factory,
                                           new CheckedPatternElevationUpdater(0.125, 9, 10.0, 20.0),
                                           12);

        SimpleTile regularTile = cache.getTile(0.2, 0.6);
        Assert.assertEquals(1, factory.getCount());
        Assert.assertEquals(0.125,    regularTile.getMinimumLatitude(),  1.0e-10);
        Assert.assertEquals(0.5,      regularTile.getMinimumLongitude(), 1.0e-10);
        Assert.assertEquals(0.015625, regularTile.getLatitudeStep(),     1.0e-10);
        Assert.assertEquals(0.015625, regularTile.getLongitudeStep(),    1.0e-10);
        Assert.assertEquals(10.0,     regularTile.getMinElevation(),     1.0e-10);
        Assert.assertEquals(20.0,     regularTile.getMaxElevation(),     1.0e-10);

        SimpleTile tileAtEnd = cache.getTile(0.234375, 0.609375);
        Assert.assertEquals(1, factory.getCount());
        Assert.assertEquals(0.125,    tileAtEnd.getMinimumLatitude(),  1.0e-10);
        Assert.assertEquals(0.5,      tileAtEnd.getMinimumLongitude(), 1.0e-10);
        Assert.assertEquals(0.015625, tileAtEnd.getLatitudeStep(),     1.0e-10);
        Assert.assertEquals(0.015625, tileAtEnd.getLongitudeStep(),    1.0e-10);
        Assert.assertEquals(10.0,     tileAtEnd.getMinElevation(),     1.0e-10);
        Assert.assertEquals(20.0,     tileAtEnd.getMaxElevation(),     1.0e-10);

    }

    @Test
    public void testNonContiguousFill() {
        CountingFactory factory = new CountingFactory();
        TilesCache<SimpleTile> cache =
                new TilesCache<SimpleTile>(factory,
                                           new CheckedPatternElevationUpdater(FastMath.toRadians(1.0), 11, 10.0, 20.0),
                                           16);

        cache.getTile(FastMath.toRadians(1.5), FastMath.toRadians(0.5));
        cache.getTile(FastMath.toRadians(3.5), FastMath.toRadians(2.5));
        cache.getTile(FastMath.toRadians(2.5), FastMath.toRadians(3.5));
        cache.getTile(FastMath.toRadians(3.5), FastMath.toRadians(3.5));
        cache.getTile(FastMath.toRadians(1.5), FastMath.toRadians(3.5));
        cache.getTile(FastMath.toRadians(1.5), FastMath.toRadians(1.5));
        cache.getTile(FastMath.toRadians(3.5), FastMath.toRadians(1.5));
        cache.getTile(FastMath.toRadians(2.5), FastMath.toRadians(1.5));
        cache.getTile(FastMath.toRadians(0.5), FastMath.toRadians(3.5));
        cache.getTile(FastMath.toRadians(1.5), FastMath.toRadians(2.5));
        cache.getTile(FastMath.toRadians(2.5), FastMath.toRadians(2.5));
        cache.getTile(FastMath.toRadians(0.5), FastMath.toRadians(2.5));
        cache.getTile(FastMath.toRadians(3.5), FastMath.toRadians(0.5));
        cache.getTile(FastMath.toRadians(0.5), FastMath.toRadians(1.5));
        cache.getTile(FastMath.toRadians(2.5), FastMath.toRadians(0.5));
        cache.getTile(FastMath.toRadians(0.5), FastMath.toRadians(0.5));
        Assert.assertEquals(16, factory.getCount());

        // keep using the same tiles for a while
        RandomGenerator generator = new Well19937a(0x1c951160de55c9d5l);
        for (int i = 0; i < 10000; ++i) {
            double lat = 3.0 * generator.nextDouble();
            double lon = 4.0 * generator.nextDouble();
                cache.getTile(FastMath.toRadians(lat), FastMath.toRadians(lon));
        }
        Assert.assertEquals(16, factory.getCount());

        cache.getTile(FastMath.toRadians(-30.5), FastMath.toRadians(2.5));
        Assert.assertEquals(17, factory.getCount());

    }

    @Test
    public void testZipperTileCreation() throws URISyntaxException {

        String path = getClass().getClassLoader().getResource("orekit-data").toURI().getPath();
        DataProvidersManager.getInstance().addProvider(new DirectoryCrawler(new File(path)));
        ExtendedEllipsoid earth = new ExtendedEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                      Constants.WGS84_EARTH_FLATTENING,
                                      FramesFactory.getITRF(IERSConventions.IERS_2010, true));

        final int n = 1201;
        final double size = FastMath.toRadians(1.0);
        TileUpdater updater = new TileUpdater() {
            public void updateTile(double latitude, double longitude, UpdatableTile tile) {
                
                double step = size / (n - 1);
                // this geometry is incorrect:
                // the specified latitude/longitude belong to rows/columns [1, n-1]
                // and not [0, n-2].
                tile.setGeometry(size * FastMath.floor(latitude / size) - 0.5 * step,
                                 size * FastMath.floor(longitude / size) - 0.5 * step,
                                 step, step, n, n);
                for (int i = 0; i < n; ++i) {
                    for (int j = 0; j < n; ++j) {
                        tile.setElevation(i, j, ((i + j) % 2 == 0) ? (-7.0 - j*Math.random()) : (224 + i*Math.random()));
                    }
                }
            }
        };
        final IntersectionAlgorithm algorithm = new DuvenhageAlgorithm(updater, 8, false);
        try {
//            GeodeticPoint gp = algorithm.intersection(earth,
//                    new Vector3D(-3010311.9672771087, 5307094.8081077365, 1852867.7919871407),
//                    new Vector3D(-0.3953329359154183, +0.8654901360032332, +0.30763402650162286));
           GeodeticPoint gp =  algorithm.intersection(earth,
                    new Vector3D(-3010311.9672771087, 5307094.8081077365, 1852867.7919871407),
                    new Vector3D(0.3953329359154183, -0.8654901360032332, -0.30763402650162286));
            System.out.println(gp.toString());

        } catch (RuggedException re) {
            Assert.assertEquals(RuggedMessages.TILE_WITHOUT_REQUIRED_NEIGHBORS_SELECTED, re.getSpecifier());
        }
    }
}
