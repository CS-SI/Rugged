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

import org.apache.commons.math3.random.RandomGenerator;
import org.apache.commons.math3.random.Well19937a;
import org.apache.commons.math3.util.FastMath;
import org.junit.Assert;
import org.junit.Test;
import org.orekit.rugged.api.RuggedException;
import org.orekit.rugged.core.dem.SimpleTile;
import org.orekit.rugged.core.dem.TilesCache;

public class TilesCacheTest {

    @Test
    public void testSingleTile() throws RuggedException {
        CountingFactory factory = new CountingFactory();
        TilesCache<SimpleTile> cache = new TilesCache<SimpleTile>(factory,
                new ConstantElevationUpdater(FastMath.toRadians(3.0), 10, 10.0), 1000);
        SimpleTile tile = cache.getTile(FastMath.toRadians(-23.2), FastMath.toRadians(137.5));
        Assert.assertEquals(1, factory.getCount());
        Assert.assertEquals(-24.0, FastMath.toDegrees(tile.getMinimumLatitude()),  1.0e-10);
        Assert.assertEquals(135.0, FastMath.toDegrees(tile.getMinimumLongitude()), 1.0e-10);
        Assert.assertEquals(  0.3, FastMath.toDegrees(tile.getLatitudeStep()),     1.0e-10);
        Assert.assertEquals(  0.3, FastMath.toDegrees(tile.getLongitudeStep()),    1.0e-10);
    }

    @Test
    public void testEviction() throws RuggedException {
        CountingFactory factory = new CountingFactory();
        TilesCache<SimpleTile> cache = new TilesCache<SimpleTile>(factory,
                new ConstantElevationUpdater(FastMath.toRadians(1.0), 10, 10.0), 12);

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

        // ask for one point outside of the covered area, to evict the (0.0, 0.0) tile
        cache.getTile(FastMath.toRadians(20.5), FastMath.toRadians(30.5));
        Assert.assertEquals(13, factory.getCount());

        // ask again for one point in the evicted tile which must be reallocated
        cache.getTile(FastMath.toRadians(0.5), FastMath.toRadians(0.5));
        Assert.assertEquals(14, factory.getCount());

        // the 13th allocated tile should still be there
        cache.getTile(FastMath.toRadians(20.5), FastMath.toRadians(30.5));
        Assert.assertEquals(14, factory.getCount());

        // evict all the tiles, goind to a completely different zone
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 3; ++j) {
                cache.getTile(FastMath.toRadians(40.5 + i), FastMath.toRadians(90.5 + j));
            }
        }
        Assert.assertEquals(26, factory.getCount());

    }

    @Test
    public void testExactEnd() throws RuggedException {
        CountingFactory factory = new CountingFactory();
        TilesCache<SimpleTile> cache =
                new TilesCache<SimpleTile>(factory,
                                           new ConstantElevationUpdater(0.125, 8, 10.0),
                                           12);

        SimpleTile regularTile = cache.getTile(0.2, 0.6);
        Assert.assertEquals(1, factory.getCount());
        Assert.assertEquals(0.125,    regularTile.getMinimumLatitude(),  1.0e-10);
        Assert.assertEquals(0.5,      regularTile.getMinimumLongitude(), 1.0e-10);
        Assert.assertEquals(0.015625, regularTile.getLatitudeStep(),     1.0e-10);
        Assert.assertEquals(0.015625, regularTile.getLongitudeStep(),    1.0e-10);

        SimpleTile tileAtEnd = cache.getTile(0.250, 0.625);
        Assert.assertEquals(1, factory.getCount());
        Assert.assertEquals(0.125,    tileAtEnd.getMinimumLatitude(),  1.0e-10);
        Assert.assertEquals(0.5,      tileAtEnd.getMinimumLongitude(), 1.0e-10);
        Assert.assertEquals(0.015625, tileAtEnd.getLatitudeStep(),     1.0e-10);
        Assert.assertEquals(0.015625, tileAtEnd.getLongitudeStep(),    1.0e-10);

    }

    @Test
    public void testNonContiguousFill() throws RuggedException {
        CountingFactory factory = new CountingFactory();
        TilesCache<SimpleTile> cache =
                new TilesCache<SimpleTile>(factory,
                                           new ConstantElevationUpdater(FastMath.toRadians(1.0), 10, 10.0),
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

}
