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
package org.orekit.rugged.raster;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.io.UnsupportedEncodingException;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.net.URISyntaxException;
import java.util.Locale;
import java.util.Map;

import org.hipparchus.random.RandomGenerator;
import org.hipparchus.random.Well19937a;
import org.hipparchus.util.FastMath;
import org.hipparchus.util.MathUtils;
import org.junit.Assert;
import org.junit.Test;

/**
 * @author Luc Maisonobe
 * @author Guylaine Prat
 */
public class TilesCacheTest {

    @Test
    public void testSingleTile() {
        CountingFactory factory = new CountingFactory();
        TilesCache<SimpleTile> cache = new TilesCache<SimpleTile>(factory,
                new CheckedPatternElevationUpdater(FastMath.toRadians(3.0), 11, 10.0, 20.0), 1000,true);
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
                new CheckedPatternElevationUpdater(FastMath.toRadians(1.0), 11, 10.0, 20.0), 12, true);

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
                                           12, true);

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
                                           16, true);

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
    public void testDummySRTM() throws URISyntaxException, FileNotFoundException, UnsupportedEncodingException {

        // Simple SRTM with 2 elevations
        int rowCols = 1000;
        DummySRTMsimpleElevationUpdater srtmUpdater = new DummySRTMsimpleElevationUpdater(rowCols, 10.0, 20.0);
        
        double tileSizeDeg = srtmUpdater.getTileSizeDeg();
        double rasterStepDeg = srtmUpdater.getTileStepDeg();
        System.out.println("Default size: " + srtmUpdater.getTileSizeDeg() + " step " + srtmUpdater.getTileStepDeg());

        CountingFactory factory = new CountingFactory();
        TilesCache<SimpleTile> cache = new TilesCache<SimpleTile>(factory, srtmUpdater , 5, false);

        boolean isDifferentStep = false;
        double epsilonLatLon = 1.e-5;
        double latTileDeg;
        double lonTileDeg;
        double latDeg;
        double lonDeg;
        
        // ##########################################################
        // Tiles with same resolution
        // ##########################################################

        // North-East hemisphere  
        // =====================
        System.out.println("\n\n#################################");
        System.out.println("NORTH EAST hemisphere");
        latTileDeg = 47.;
        lonTileDeg = 12.3;

        System.out.println(">>>> Search lat deg = " + latTileDeg + " lon deg= " + lonTileDeg + "\n");
        SimpleTile tileNEhemisphere = cache.getTile(FastMath.toRadians(latTileDeg), FastMath.toRadians(lonTileDeg));

        // Latitude North of the tile
        System.out.println("##### Bord de tuile au niveau latitude North #####");
        latDeg = getNorthernEdgeOfTile(tileNEhemisphere);
        lonDeg = lonTileDeg;
        searchAndVerifyZipperTile(latDeg, lonDeg, Tile.Location.NORTH, tileNEhemisphere, tileSizeDeg, cache, epsilonLatLon, isDifferentStep);

        // Latitude South of the tile
        System.out.println("#### Bord de tuile au niveau latitude South #####");
        latDeg = getSouthernEdgeOfTile(tileNEhemisphere);
        lonDeg = lonTileDeg;

        searchAndVerifyZipperTile(latDeg, lonDeg, Tile.Location.SOUTH, tileNEhemisphere, tileSizeDeg, cache, epsilonLatLon, isDifferentStep);

        // Longitude West of the tile
        System.out.println("#### Bord de tuile au niveau longitude West #####");
        latDeg = latTileDeg;
        lonDeg = getWesternEdgeOfTile(tileNEhemisphere);

        searchAndVerifyZipperTile(latDeg, lonDeg, Tile.Location.WEST, tileNEhemisphere, tileSizeDeg, cache, epsilonLatLon, isDifferentStep);

        // Longitude East of the tile
        System.out.println("#### Bord de tuile au niveau longitude East #####");
        latDeg = latTileDeg;
        lonDeg = getEasternEdgeOfTile(tileNEhemisphere);

        searchAndVerifyZipperTile(latDeg, lonDeg, Tile.Location.EAST,tileNEhemisphere, tileSizeDeg, cache, epsilonLatLon, isDifferentStep);

        // Check the 4 corner zipper tiles
        check4cornersZipperTiles(tileNEhemisphere, tileSizeDeg, cache, epsilonLatLon, isDifferentStep);

        // South-East hemisphere 
        // =====================
        System.out.println("\n\n#################################");
        System.out.println("SOUTH EAST hemisphere");

        latTileDeg = -16.2;
        lonTileDeg = 22.3;
        System.out.println(">>>> Search lat deg = " + latTileDeg + " lon deg= " + lonTileDeg + "\n");
        SimpleTile tileSEhemisphere = cache.getTile(FastMath.toRadians(latTileDeg), FastMath.toRadians(lonTileDeg));

        printTileInfo(tileSEhemisphere, rasterStepDeg, rasterStepDeg);

        // Latitude North of the tile
        System.out.println("##### Bord de tuile au niveau latitude North #####");
        latDeg = getNorthernEdgeOfTile(tileSEhemisphere);
        lonDeg = lonTileDeg;

        searchAndVerifyZipperTile(latDeg, lonDeg, Tile.Location.NORTH, tileSEhemisphere, tileSizeDeg, cache, epsilonLatLon, isDifferentStep);

        // Latitude South of the tile
        System.out.println("#### Bord de tuile au niveau latitude South #####");
        latDeg = getSouthernEdgeOfTile(tileSEhemisphere);
        lonDeg = lonTileDeg;

        searchAndVerifyZipperTile(latDeg, lonDeg, Tile.Location.SOUTH, tileSEhemisphere, tileSizeDeg, cache, epsilonLatLon, isDifferentStep);

        // Longitude West of the tile
        System.out.println("#### Bord de tuile au niveau longitude West #####");
        latDeg = latTileDeg;
        lonDeg = getWesternEdgeOfTile(tileSEhemisphere);

        searchAndVerifyZipperTile(latDeg, lonDeg, Tile.Location.WEST, tileSEhemisphere, tileSizeDeg, cache, epsilonLatLon, isDifferentStep);

        // Longitude East of the tile
        System.out.println("#### Bord de tuile au niveau longitude East #####");
        latDeg = latTileDeg;
        lonDeg = getEasternEdgeOfTile(tileSEhemisphere);
        searchAndVerifyZipperTile(latDeg, lonDeg, Tile.Location.EAST, tileSEhemisphere, tileSizeDeg, cache, epsilonLatLon, isDifferentStep);

        // Check the 4 corner zipper tiles
        check4cornersZipperTiles(tileSEhemisphere, tileSizeDeg, cache, epsilonLatLon, isDifferentStep);

        // North-West hemisphere 
        // =====================
        System.out.println("\n\n#################################");
        System.out.println("NORTH WEST hemisphere");
        latTileDeg = 46.8;
        lonTileDeg = -66.5; 

        System.out.println(">>>> Search lat deg = " + latTileDeg + " lon deg= " + lonTileDeg + "\n");
        SimpleTile tileNWhemisphere = cache.getTile(FastMath.toRadians(latTileDeg), FastMath.toRadians(lonTileDeg));
        printTileInfo(tileNWhemisphere, rasterStepDeg, rasterStepDeg);

        // Latitude North of the tile
        System.out.println("##### Bord de tuile au niveau latitude North #####");
        latDeg = getNorthernEdgeOfTile(tileNWhemisphere);
        lonDeg = lonTileDeg;

        searchAndVerifyZipperTile(latDeg, lonDeg, Tile.Location.NORTH, tileNWhemisphere, tileSizeDeg, cache, epsilonLatLon, isDifferentStep);

        // Latitude South of the tile
        System.out.println("#### Bord de tuile au niveau latitude South #####");
        latDeg = getSouthernEdgeOfTile(tileNWhemisphere);
        lonDeg = lonTileDeg;

        searchAndVerifyZipperTile(latDeg, lonDeg, Tile.Location.SOUTH, tileNWhemisphere, tileSizeDeg, cache, epsilonLatLon, isDifferentStep);

        // Longitude West of the tile
        System.out.println("#### Bord de tuile au niveau longitude West #####");
        latDeg = latTileDeg;
        lonDeg = getWesternEdgeOfTile(tileNWhemisphere);

        searchAndVerifyZipperTile(latDeg, lonDeg, Tile.Location.WEST, tileNWhemisphere, tileSizeDeg, cache, epsilonLatLon, isDifferentStep);

        // Longitude East of the tile
        System.out.println("#### Bord de tuile au niveau longitude East #####");
        latDeg = latTileDeg;
        lonDeg = getEasternEdgeOfTile(tileNWhemisphere);

        searchAndVerifyZipperTile(latDeg, lonDeg, Tile.Location.EAST, tileNWhemisphere, tileSizeDeg, cache, epsilonLatLon, isDifferentStep);

        // Check the 4 corner zipper tiles
        check4cornersZipperTiles(tileNWhemisphere, tileSizeDeg, cache, epsilonLatLon, isDifferentStep);

        // South-West hemisphere 
        // =====================
        System.out.println("\n\n#################################");
        System.out.println("SOUTH WEST hemisphere");

        latTileDeg = -28.8  ;
        lonTileDeg = -58.4; 
        System.out.println(">>>> Search lat deg = " + latTileDeg + " lon deg= " + lonTileDeg + "\n");
        SimpleTile tileSWhemisphere = cache.getTile(FastMath.toRadians(latTileDeg), FastMath.toRadians(lonTileDeg));
        printTileInfo(tileSWhemisphere, rasterStepDeg, rasterStepDeg);

        // Latitude North of the tile
        System.out.println("##### Bord de tuile au niveau latitude North #####");
        latDeg = getNorthernEdgeOfTile(tileSWhemisphere);
        lonDeg = lonTileDeg;

        searchAndVerifyZipperTile(latDeg, lonDeg, Tile.Location.NORTH, tileSWhemisphere, tileSizeDeg, cache, epsilonLatLon, isDifferentStep);

        // Latitude South of the tile
        System.out.println("#### Bord de tuile au niveau latitude South #####");
        latDeg = getSouthernEdgeOfTile(tileSWhemisphere);
        lonDeg = lonTileDeg;

        searchAndVerifyZipperTile(latDeg, lonDeg, Tile.Location.SOUTH, tileSWhemisphere, tileSizeDeg, cache, epsilonLatLon, isDifferentStep);

        // Longitude West of the tile
        System.out.println("#### Bord de tuile au niveau longitude West #####");
        latDeg = latTileDeg;
        lonDeg = getWesternEdgeOfTile(tileSWhemisphere);

        searchAndVerifyZipperTile(latDeg, lonDeg, Tile.Location.WEST, tileSWhemisphere, tileSizeDeg, cache, epsilonLatLon, isDifferentStep);

        // Longitude East of the tile
        System.out.println("#### Bord de tuile au niveau longitude East #####");
        latDeg = latTileDeg;
        lonDeg = getEasternEdgeOfTile(tileSWhemisphere);

        searchAndVerifyZipperTile(latDeg, lonDeg, Tile.Location.EAST, tileSWhemisphere, tileSizeDeg, cache, epsilonLatLon, isDifferentStep);

        // Check the 4 corner zipper tiles
        check4cornersZipperTiles(tileSWhemisphere, tileSizeDeg, cache, epsilonLatLon, isDifferentStep);
        
        // Anti meridians  (180 degrees W/E)
        // =====================
        System.out.println("\n\n#################################");
        System.out.println("Anti meridien test (180 degre East)");
        // tile SRTM 72/16: 175 - 180 East / 15 - 20 South  
        latTileDeg = -18.;
        lonTileDeg = 178.;
        System.out.println(">>>> Search lat deg = " + latTileDeg + " lon deg= " + lonTileDeg + "\n");
        SimpleTile tileAntiMeridianEast = cache.getTile(FastMath.toRadians(latTileDeg), FastMath.toRadians(lonTileDeg));

        // Longitude East of the tile
        System.out.println("#### Bord de tuile au niveau longitude East #####");
        latDeg = latTileDeg;
        lonDeg = getEasternEdgeOfTile(tileAntiMeridianEast);

        searchAndVerifyZipperTile(latDeg, lonDeg, Tile.Location.EAST,tileAntiMeridianEast, tileSizeDeg, cache, epsilonLatLon, isDifferentStep);

        System.out.println("#################################");
        System.out.println("Anti meridien test (180 degre West)");
        // tile SRTM 01/16: 175 - 180 West / 15 - 20 South  
        latTileDeg = -18.;
        lonTileDeg = -178.;
        System.out.println(">>>> Search lat deg = " + latTileDeg + " lon deg= " + lonTileDeg + "\n");
        SimpleTile tileAntiMeridianWest = cache.getTile(FastMath.toRadians(latTileDeg), FastMath.toRadians(lonTileDeg));

        // Longitude West of the tile
        System.out.println("#### Bord de tuile au niveau longitude West #####");
        latDeg = latTileDeg;
        lonDeg = getWesternEdgeOfTile(tileAntiMeridianWest);

        searchAndVerifyZipperTile(latDeg, lonDeg, Tile.Location.WEST,tileAntiMeridianWest, tileSizeDeg, cache, epsilonLatLon, isDifferentStep);
        
        // ##########################################################
        // Tiles around the resolution change (around +/- 60 degrees)
        // ##########################################################

        // Cleanup
        clearFactoryMaps(CountingFactory.class);
        
        rowCols = 10;
        srtmUpdater = new DummySRTMsimpleElevationUpdater(rowCols, 10.0, 20.0);
        tileSizeDeg = srtmUpdater.getTileSizeDeg();

        factory = new CountingFactory();
        cache = new TilesCache<SimpleTile>(factory, srtmUpdater , 5, false);

        // Above 60 degrees  
        // ================
        System.out.println("\n\n#################################");
        System.out.println("Above 60 degrees ");
        latTileDeg = 59.;
        lonTileDeg = 12.3;

        System.out.println(">>>> Search lat deg = " + latTileDeg + " lon deg= " + lonTileDeg + "\n");
        SimpleTile tileAround60deg = cache.getTile(FastMath.toRadians(latTileDeg), FastMath.toRadians(lonTileDeg));
        
        // Latitude North of the tile
        System.out.println("##### Bord de tuile au niveau latitude North #####");

        latDeg = getNorthernEdgeOfTile(tileAround60deg);
        lonDeg = lonTileDeg;
        System.out.println(">>>> Bord Nord lat deg = " + latDeg + "\n");

        isDifferentStep = true;
        searchAndVerifyZipperTile(latDeg, lonDeg, Tile.Location.NORTH, tileAround60deg, tileSizeDeg, cache, epsilonLatLon, isDifferentStep);
        isDifferentStep = false;
      
        // Check the 4 corner zipper tiles
        isDifferentStep = true;
        check4cornersZipperTiles(tileAround60deg, tileSizeDeg, cache, epsilonLatLon, isDifferentStep);
        isDifferentStep = false;
        
        // Below -60 degrees  
        // =================
        System.out.println("\n\n#################################");
        System.out.println("Below -60 degrees ");
        latTileDeg = -59.;
        lonTileDeg = 12.3;

        System.out.println(">>>> Search lat deg = " + latTileDeg + " lon deg= " + lonTileDeg + "\n");
        SimpleTile tileAroundMinus60deg = cache.getTile(FastMath.toRadians(latTileDeg), FastMath.toRadians(lonTileDeg));

        // Latitude South of the tile
        System.out.println("##### Bord de tuile au niveau latitude South #####");

        latDeg = getSouthernEdgeOfTile(tileAroundMinus60deg);
        lonDeg = lonTileDeg;
        System.out.println(">>>> Bord South lat deg = " + latDeg + "\n");
        
        isDifferentStep = true;
        searchAndVerifyZipperTile(latDeg, lonDeg, Tile.Location.SOUTH, tileAroundMinus60deg, tileSizeDeg, cache, epsilonLatLon, isDifferentStep);
        isDifferentStep = false;
        
        // Check the 4 corner zipper tiles
        isDifferentStep = true;
        check4cornersZipperTiles(tileAroundMinus60deg, tileSizeDeg, cache, epsilonLatLon, isDifferentStep);
        isDifferentStep = false;
    }

    /**
     * Check the computed zipper tile 
     * @param latDeg latitude belonging to the zipper tile (deg)
     * @param lonDeg longitude belonging to the zipper tile (deg)
     * @param zipperLocation location of the zipper tile according to the current tile 
     * @param tile the current tile 
     * @param tileSizeDeg the current tile size (deg)
     * @param cache the tiles cache
     * @param epsilonLatLon epsilon to compare latitude and longitude
     * @param isDifferentStep flag to tell if zipper is at the edge of tiles with different steps
     * @return the zipper tile
     */
    private SimpleTile searchAndVerifyZipperTile(double latDeg, double lonDeg, Tile.Location zipperLocation, SimpleTile tile, final double tileSizeDeg,
            TilesCache<SimpleTile> cache, double epsilonLatLon, boolean isDifferentStep) {
 
        SimpleTile zipperTile = cache.getTile(FastMath.toRadians(latDeg), FastMath.toRadians(lonDeg));
        
        checkGeodeticPointBelongsToZipper(FastMath.toRadians(latDeg), FastMath.toRadians(lonDeg), zipperTile);
        
        if (!isDifferentStep) {
            checkZipperTile(zipperTile, zipperLocation, tile, tileSizeDeg, cache, epsilonLatLon);
        } else {
            checkZipperTileDifferentStep(zipperTile, zipperLocation, tile, tileSizeDeg, cache, epsilonLatLon);
        }
        
        return zipperTile;
    }

    private void check4cornersZipperTiles(SimpleTile tile, final double tileSizeDeg,
                                          TilesCache<SimpleTile> cache, double epsilonLatLon,
                                          boolean isDifferentStep) {
        double latDeg;
        double lonDeg;
        
        SimpleTile cornerZipperTile;
        
        // Latitude/Longitude corner NW
        System.out.println("##### Coin de tuile au niveau latitude North et longitude West #####");
        latDeg = getNorthernEdgeOfTile(tile);
        lonDeg = getWesternEdgeOfTile(tile);
        System.out.println(">>>> Search lat deg = " + latDeg + " lon deg= " + lonDeg + "\n");
        cornerZipperTile = cache.getTile(FastMath.toRadians(latDeg), FastMath.toRadians(lonDeg));
//        printTileInfo(cornerZipperTile, rasterStepDeg, rasterStepDeg);
        
        checkGeodeticPointBelongsToZipper(FastMath.toRadians(latDeg), FastMath.toRadians(lonDeg), cornerZipperTile);
        checkCornerZipperTile(cornerZipperTile, Tile.Location.NORTH_WEST, tile, tileSizeDeg, cache, epsilonLatLon, isDifferentStep);
        
        // Latitude/Longitude corner SW
        System.out.println("##### Coin de tuile au niveau latitude South et longitude West #####");
        latDeg = getSouthernEdgeOfTile(tile);
        lonDeg = getWesternEdgeOfTile(tile);
        System.out.println(">>>> Search lat deg = " + latDeg + " lon deg= " + lonDeg + "\n");
        cornerZipperTile = cache.getTile(FastMath.toRadians(latDeg), FastMath.toRadians(lonDeg));
//        printTileInfo(cornerZipperTile, rasterStepDeg, rasterStepDeg);

        checkGeodeticPointBelongsToZipper(FastMath.toRadians(latDeg), FastMath.toRadians(lonDeg), cornerZipperTile);
        checkCornerZipperTile(cornerZipperTile, Tile.Location.SOUTH_WEST, tile, tileSizeDeg, cache, epsilonLatLon, isDifferentStep);

        // Latitude/Longitude corner NE
        System.out.println("##### Coin de tuile au niveau latitude North et longitude Est #####");
        latDeg = getNorthernEdgeOfTile(tile);
        lonDeg = getEasternEdgeOfTile(tile);
        System.out.println(">>>> Search lat deg = " + latDeg + " lon deg= " + lonDeg + "\n");
        cornerZipperTile = cache.getTile(FastMath.toRadians(latDeg), FastMath.toRadians(lonDeg));
//        printTileInfo(cornerZipperTile, rasterStepDeg, rasterStepDeg);

        checkGeodeticPointBelongsToZipper(FastMath.toRadians(latDeg), FastMath.toRadians(lonDeg), cornerZipperTile);
        checkCornerZipperTile(cornerZipperTile, Tile.Location.NORTH_EAST, tile, tileSizeDeg, cache, epsilonLatLon, isDifferentStep);

        // Latitude/Longitude corner SE
        System.out.println("##### Coin de tuile au niveau latitude South et longitude Est #####");
        latDeg = getSouthernEdgeOfTile(tile);
        lonDeg = getEasternEdgeOfTile(tile);
        System.out.println(">>>> Search lat deg = " + latDeg + " lon deg= " + lonDeg + "\n");
        cornerZipperTile = cache.getTile(FastMath.toRadians(latDeg), FastMath.toRadians(lonDeg));
//        printTileInfo(cornerZipperTile, rasterStepDeg, rasterStepDeg);

        checkGeodeticPointBelongsToZipper(FastMath.toRadians(latDeg), FastMath.toRadians(lonDeg), cornerZipperTile);
        checkCornerZipperTile(cornerZipperTile, Tile.Location.SOUTH_EAST, tile, tileSizeDeg, cache, epsilonLatLon, isDifferentStep);
    }

    private void checkGeodeticPointBelongsToZipper(double latitude, double longitude, SimpleTile cornerZipperTile) {
        
        double latMin = cornerZipperTile.getMinimumLatitude();
        double lonMin = cornerZipperTile.getMinimumLongitude();
        double latMax = cornerZipperTile.getLatitudeAtIndex(cornerZipperTile.getLatitudeRows() - 1);
        double lonMax = cornerZipperTile.getLongitudeAtIndex(cornerZipperTile.getLongitudeColumns() - 1);
        
        // this test purpose is to ensure the geodetic points belongs to the zipper tile
        assertTrue((latMin <= latitude) && (latitude <= latMax));
        assertTrue((lonMin <= longitude) && (longitude <= lonMax));
        
    }

    private void printSurroundingTiles(double latDeg, double lonDeg, final double sizeDeg, final double rasterStepDeg,
            TilesCache<SimpleTile> cache, int stepPrint, int nbRowCol) throws FileNotFoundException, UnsupportedEncodingException {

        // print tile above ...
        double latUp = latDeg + sizeDeg;
        SimpleTile tileUp = cache.getTile(FastMath.toRadians(latUp), FastMath.toRadians(lonDeg));
        String tileUpName = "above_lat" + Double.toString(latUp) + "lon" + Double.toString(lonDeg) + ".txt";
        printExtractTileData(tileUpName, tileUp, stepPrint, 0, nbRowCol, 0, tileUp.getLongitudeColumns());

        // print tile below
        double latBelow = latDeg - sizeDeg;
        SimpleTile tileBelow = cache.getTile(FastMath.toRadians(latBelow), FastMath.toRadians(lonDeg));
        String tileBelowName = "below_lat" + Double.toString(latBelow) + "lon" + Double.toString(lonDeg) + ".txt";
        printExtractTileData(tileBelowName, tileBelow, stepPrint, tileBelow.getLatitudeRows() - nbRowCol, tileBelow.getLatitudeRows(), 0, tileBelow.getLongitudeColumns());

        // print tile left
        double lonLeft = lonDeg - sizeDeg;
        SimpleTile tileLeft = cache.getTile(FastMath.toRadians(latDeg), FastMath.toRadians(lonLeft));
        String tileLeftName = "left_lat" + Double.toString(latDeg) + "lon" + Double.toString(lonLeft) + ".txt";
        printExtractTileData(tileLeftName, tileLeft, stepPrint, 0, tileLeft.getLatitudeRows(),tileLeft.getLongitudeColumns() - nbRowCol, tileLeft.getLongitudeColumns());

        // print tile right
        double lonRight = lonDeg + sizeDeg;
        SimpleTile tileRight = cache.getTile(FastMath.toRadians(latDeg), FastMath.toRadians(lonRight));
        String tileRightName = "right_lat" + Double.toString(latDeg) + "lon" + Double.toString(lonRight) + ".txt";
        printExtractTileData(tileRightName, tileRight, stepPrint, 0, tileRight.getLatitudeRows(), 0, nbRowCol);

    }

    private void checkOneValueZipperTile(double latDeg, double lonDeg, SimpleTile zipperTile, 
            final int nbRows, final int nbCols,
            double epsilonLatLon, double minLat, double maxLat, double minLon, double maxLon,
            final int expectedLatIndex, final int expectedLonIndex, final double expectedElevation) {
        
        assertTrue(zipperTile.getLatitudeRows() == nbRows);
        assertTrue(zipperTile.getLongitudeColumns() == nbCols);
        assertEquals(FastMath.toDegrees(zipperTile.getMinimumLatitude()), minLat, epsilonLatLon);
        assertEquals(FastMath.toDegrees(zipperTile.getMaximumLatitude()), maxLat, epsilonLatLon);
        assertEquals(FastMath.toDegrees(zipperTile.getMinimumLongitude()), minLon, epsilonLatLon);
        assertEquals(FastMath.toDegrees(zipperTile.getMaximumLongitude()), maxLon, epsilonLatLon);
        checkLatLonIndex(latDeg, expectedLatIndex, lonDeg, expectedLonIndex,  expectedElevation, zipperTile);
    }
    
    private void checkCornerZipperTile(SimpleTile cornerZipperTile, Tile.Location location, SimpleTile originTile,
            double tileSizeDeg, TilesCache<SimpleTile> cache, double epsilonLatLon, boolean isDifferentStep) {
    
        SurroundingTiles surroundingTiles = new SurroundingTiles(originTile, tileSizeDeg, cache);

        if (location == Tile.Location.SOUTH_WEST) {

            SimpleTile tileBelow = surroundingTiles.getTileBelow();
            SimpleTile tileLeft = surroundingTiles.getTileLeft();
            
            if (! isDifferentStep) {
                // The 2 following tiles are the same if the tiles step are the same 
                SimpleTile belowLeftOfCurrent = new SurroundingTiles(tileLeft, tileSizeDeg, cache).getTileBelow();
                SimpleTile leftBelowOfCurrent = new SurroundingTiles(tileBelow, tileSizeDeg, cache).getTileLeft();

                SimpleTile belowRight = tileBelow;
                SimpleTile belowLeft1  = belowLeftOfCurrent;
                SimpleTile belowLeft2  = leftBelowOfCurrent;
                SimpleTile aboveRight = originTile;
                SimpleTile aboveLeft = tileLeft;

                checkCornerElevations(cornerZipperTile, originTile, belowLeft1, belowRight, aboveLeft, aboveRight, epsilonLatLon);
                checkCornerElevations(cornerZipperTile, originTile, belowLeft2, belowRight, aboveLeft, aboveRight, epsilonLatLon);
                
            } else {

                // Tiles at same latitude have same resolutions
                SimpleTile leftBelowOfCurrent = new SurroundingTiles(tileBelow, tileSizeDeg, cache).getTileLeft();

                SimpleTile belowRight = tileBelow;
                SimpleTile belowLeft  = leftBelowOfCurrent;
                SimpleTile aboveRight = originTile;
                SimpleTile aboveLeft = tileLeft;
                
                
                System.out.println("CORNER SOUTH WEST");
                printSurroundingCorner(cornerZipperTile,belowRight, belowLeft, aboveRight, aboveLeft);
                
               checkCornerElevationsDifferentStep(cornerZipperTile, originTile, belowLeft, belowRight, aboveLeft, aboveRight, epsilonLatLon);

            }

        } else if (location == Tile.Location.NORTH_WEST) {

            SimpleTile tileAbove = surroundingTiles.getTileAbove();
            SimpleTile tileLeft = surroundingTiles.getTileLeft();
            
            if (! isDifferentStep) {

                // The 2 following tiles are the same if the tiles step are the same 
                SimpleTile aboveLeftOfCurrent = new SurroundingTiles(tileLeft, tileSizeDeg, cache).getTileAbove();
                SimpleTile leftAboveOfCurrent = new SurroundingTiles(tileAbove, tileSizeDeg, cache).getTileLeft();

                SimpleTile belowRight = originTile;
                SimpleTile belowLeft = tileLeft;
                SimpleTile aboveRight = tileAbove;
                SimpleTile aboveLeft1 = aboveLeftOfCurrent;
                SimpleTile aboveLeft2 = leftAboveOfCurrent;

                checkCornerElevations(cornerZipperTile, originTile, belowLeft, belowRight, aboveLeft1, aboveRight, epsilonLatLon);
                checkCornerElevations(cornerZipperTile, originTile, belowLeft, belowRight, aboveLeft2, aboveRight, epsilonLatLon);
                
            } else {
                
                // Tiles at same latitude have same resolutions
                SimpleTile leftAboveOfCurrent = new SurroundingTiles(tileAbove, tileSizeDeg, cache).getTileLeft();

                SimpleTile belowRight = originTile;
                SimpleTile belowLeft = tileLeft;
                SimpleTile aboveRight = tileAbove;
                SimpleTile aboveLeft = leftAboveOfCurrent;
                
                System.out.println("CORNER NORTH WEST");
                printSurroundingCorner(cornerZipperTile,belowRight, belowLeft, aboveRight, aboveLeft);

                checkCornerElevationsDifferentStep(cornerZipperTile, originTile, belowLeft, belowRight, aboveLeft, aboveRight, epsilonLatLon);

            }
            

        } else if (location == Tile.Location.SOUTH_EAST) {
            
            SimpleTile tileBelow = surroundingTiles.getTileBelow();
            SimpleTile tileRight = surroundingTiles.getTileRight();
            if (! isDifferentStep) {

                // The 2 following tiles are the same if the tiles step are the same 
                SimpleTile belowRightOfCurrent = new SurroundingTiles(tileRight, tileSizeDeg, cache).getTileBelow();
                SimpleTile rightBelowOfCurrent = new SurroundingTiles(tileBelow, tileSizeDeg, cache).getTileRight();

                SimpleTile belowRight1  = belowRightOfCurrent;
                SimpleTile belowRight2  = rightBelowOfCurrent;
                SimpleTile belowLeft   = tileBelow;
                SimpleTile aboveRight  = tileRight;
                SimpleTile aboveLeft   = originTile;

                checkCornerElevations(cornerZipperTile, originTile, belowLeft, belowRight1, aboveLeft, aboveRight, epsilonLatLon);
                checkCornerElevations(cornerZipperTile, originTile, belowLeft, belowRight2, aboveLeft, aboveRight, epsilonLatLon);
                
            } else {
                
                // Tiles at same latitude have same resolutions
                SimpleTile belowRightOfCurrent = new SurroundingTiles(tileRight, tileSizeDeg, cache).getTileBelow();
                
                SimpleTile belowRight  = belowRightOfCurrent;
                SimpleTile belowLeft   = tileBelow;
                SimpleTile aboveRight  = tileRight;
                SimpleTile aboveLeft   = originTile;
                
                System.out.println("CORNER SOUTH EAST");
                printSurroundingCorner(cornerZipperTile,belowRight, belowLeft, aboveRight, aboveLeft);
                
                checkCornerElevationsDifferentStep(cornerZipperTile, originTile, belowLeft, belowRight, aboveLeft, aboveRight, epsilonLatLon);
            }

        } else if (location == Tile.Location.NORTH_EAST) {

            SimpleTile tileAbove = surroundingTiles.getTileAbove();
            SimpleTile tileRight = surroundingTiles.getTileRight();
            
            if (! isDifferentStep) {

            // The 2 following tiles are the same if the tiles step are the same 
            SimpleTile aboveRightOfCurrent = new SurroundingTiles(tileRight, tileSizeDeg, cache).getTileAbove();
            SimpleTile rightAboveOfCurrent = new SurroundingTiles(tileAbove, tileSizeDeg, cache).getTileRight();
          
            SimpleTile belowRight = tileRight;
            SimpleTile belowLeft = originTile;
            SimpleTile aboveRight1 = aboveRightOfCurrent;
            SimpleTile aboveRight2 = rightAboveOfCurrent;
            SimpleTile aboveLeft = tileAbove;
            
            checkCornerElevations(cornerZipperTile, originTile, belowLeft, belowRight, aboveLeft, aboveRight1, epsilonLatLon);
            checkCornerElevations(cornerZipperTile, originTile, belowLeft, belowRight, aboveLeft, aboveRight2, epsilonLatLon);
            } else {
                
                // Tiles at same latitude have same resolutions
                SimpleTile aboveRightOfCurrent = new SurroundingTiles(tileRight, tileSizeDeg, cache).getTileAbove();
                
                SimpleTile belowRight = tileRight;
                SimpleTile belowLeft = originTile;
                SimpleTile aboveRight = aboveRightOfCurrent;
                SimpleTile aboveLeft = tileAbove;
                
                System.out.println("CORNER NORTH_EAST");
                printSurroundingCorner(cornerZipperTile,belowRight, belowLeft, aboveRight, aboveLeft);
 
                checkCornerElevationsDifferentStep(cornerZipperTile, originTile, belowLeft, belowRight, aboveLeft, aboveRight, epsilonLatLon);

            }
        }
    }

    private void printSurroundingCorner(SimpleTile cornerZipperTile, SimpleTile belowRight, SimpleTile belowLeft, SimpleTile aboveRight,
            SimpleTile aboveLeft) {
        
        
        for (int i = 3; i >= 0; i--) {
            System.out.println(cornerZipperTile.getElevationAtIndices(i, 0) + " | " + cornerZipperTile.getElevationAtIndices(i, 1) + " | " + 
                               cornerZipperTile.getElevationAtIndices(i, 2) + " | " + cornerZipperTile.getElevationAtIndices(i, 3) + " | " );
            }

        System.out.println(" BELOW RIGHT");
        if (belowRight.getLongitudeColumns() > 3) {
            for (int i = belowRight.getLatitudeRows() - 1; i >= belowRight.getLatitudeRows() - 3; i--) {
                System.out.println(belowRight.getElevationAtIndices(i, 0) + " | " +
                        belowRight.getElevationAtIndices(i, 1) + " | "  +
                        belowRight.getElevationAtIndices(i, 2) + " | "  +
                        belowRight.getElevationAtIndices(i, 3) + " | "  +
                        belowRight.getElevationAtIndices(i, 4) + " | "  +
                        belowRight.getElevationAtIndices(i, 5) + " | "  +
                        belowRight.getElevationAtIndices(i, 6) + " | "  +
                        belowRight.getElevationAtIndices(i, 7) + " | "  +
                        belowRight.getElevationAtIndices(i, 8) + " | "  +
                        belowRight.getElevationAtIndices(i, 9) + " | ");
            }
        } else {
            for (int i = 2; i >= 0; i--) {
                System.out.println(belowRight.getElevationAtIndices(i, 0) + " | " +
                        belowRight.getElevationAtIndices(i, 1) + " | "  +
                        belowRight.getElevationAtIndices(i, 2) + " | " );
            }
        }
        
        System.out.println(" BELOW LEFT");
        if (belowLeft.getLongitudeColumns() > 3) {
            for (int i = belowLeft.getLatitudeRows() - 1; i >= belowLeft.getLatitudeRows() - 3; i--) {
                System.out.println(belowLeft.getElevationAtIndices(i, 0) + " | " +
                        belowLeft.getElevationAtIndices(i, 1) + " | "  +
                        belowLeft.getElevationAtIndices(i, 2) + " | "  +
                        belowLeft.getElevationAtIndices(i, 3) + " | "  +
                        belowLeft.getElevationAtIndices(i, 4) + " | "  +
                        belowLeft.getElevationAtIndices(i, 5) + " | "  +
                        belowLeft.getElevationAtIndices(i, 6) + " | "  +
                        belowLeft.getElevationAtIndices(i, 7) + " | "  +
                        belowLeft.getElevationAtIndices(i, 8) + " | "  +
                        belowLeft.getElevationAtIndices(i, 9) + " | ");
            }
        } else {
            for (int i = 2; i >= 0; i--) {
                System.out.println(belowLeft.getElevationAtIndices(i, 0) + " | " +
                        belowLeft.getElevationAtIndices(i, 1) + " | "  +
                        belowLeft.getElevationAtIndices(i, 2) + " | " );
            }

        }
        
        
        System.out.println(" ABOVE RIGHT");
        
        if (aboveRight.getLongitudeColumns() > 3) {
            for (int i = 3; i >= 0; i--) {
                System.out.println(aboveRight.getElevationAtIndices(i, 0) + " | " +
                        aboveRight.getElevationAtIndices(i, 1) + " | "  +
                        aboveRight.getElevationAtIndices(i, 2) + " | "  +
                        aboveRight.getElevationAtIndices(i, 3) + " | "  +
                        aboveRight.getElevationAtIndices(i, 4) + " | "  +
                        aboveRight.getElevationAtIndices(i, 5) + " | "  +
                        aboveRight.getElevationAtIndices(i, 6) + " | "  +
                        aboveRight.getElevationAtIndices(i, 7) + " | "  +
                        aboveRight.getElevationAtIndices(i, 8) + " | "  +
                        aboveRight.getElevationAtIndices(i, 9) + " | ");
            }

        } else {
            for (int i = 2; i >= 0; i--) {
                System.out.println(aboveRight.getElevationAtIndices(i, 0) + " | " +
                        aboveRight.getElevationAtIndices(i, 1) + " | "  +
                        aboveRight.getElevationAtIndices(i, 2) + " | " );
            }
        }

        System.out.println(" ABOVE LEFT");
        if (aboveLeft.getLongitudeColumns() > 3) {
            for (int i = 3; i >= 0; i--) {
                System.out.println(aboveLeft.getElevationAtIndices(i, 0) + " | " +
                        aboveLeft.getElevationAtIndices(i, 1) + " | "  +
                        aboveLeft.getElevationAtIndices(i, 2) + " | "  +
                        aboveLeft.getElevationAtIndices(i, 3) + " | "  +
                        aboveLeft.getElevationAtIndices(i, 4) + " | "  +
                        aboveLeft.getElevationAtIndices(i, 5) + " | "  +
                        aboveLeft.getElevationAtIndices(i, 6) + " | "  +
                        aboveLeft.getElevationAtIndices(i, 7) + " | "  +
                        aboveLeft.getElevationAtIndices(i, 8) + " | "  +
                        aboveLeft.getElevationAtIndices(i, 9) + " | ");
            }

        } else {
            for (int i = 2; i >= 0; i--) {
                System.out.println(aboveLeft.getElevationAtIndices(i, 0) + " | " +
                        aboveLeft.getElevationAtIndices(i, 1) + " | "  +
                        aboveLeft.getElevationAtIndices(i, 2) + " | " );
            }
        }
    }

    private void checkCornerElevations(SimpleTile cornerZipperTile, SimpleTile originTile, 
                                       SimpleTile belowLeft, SimpleTile belowRight, SimpleTile aboveLeft, SimpleTile aboveRight, 
                                       double epsilonLatLon) {

        // row 0 of zipper 
        double cornerZipperElevation = cornerZipperTile.getElevationAtIndices(0, 0);
        double belowLeftElevation = belowLeft.getElevationAtIndices(belowLeft.getLatitudeRows() - 2, belowLeft.getLongitudeColumns() - 2);
        assertEquals(belowLeftElevation, cornerZipperElevation, epsilonLatLon);

        cornerZipperElevation = cornerZipperTile.getElevationAtIndices(0, 1);
        belowLeftElevation = belowLeft.getElevationAtIndices(belowLeft.getLatitudeRows() - 2, belowLeft.getLongitudeColumns() - 1);
        assertEquals(belowLeftElevation, cornerZipperElevation, epsilonLatLon);

        cornerZipperElevation = cornerZipperTile.getElevationAtIndices(0, 2);
        double belowRightElevation = belowRight.getElevationAtIndices(belowRight.getLatitudeRows() - 2, 0);
        assertEquals(belowRightElevation, cornerZipperElevation, epsilonLatLon);

        cornerZipperElevation = cornerZipperTile.getElevationAtIndices(0, 3);
        belowRightElevation = belowRight.getElevationAtIndices(belowRight.getLatitudeRows() - 2, 1);
        assertEquals(belowRightElevation, cornerZipperElevation, epsilonLatLon);


        // row 1 of zipper 
        cornerZipperElevation = cornerZipperTile.getElevationAtIndices(1, 0);
        belowLeftElevation = belowLeft.getElevationAtIndices(belowLeft.getLatitudeRows() - 1, belowLeft.getLongitudeColumns() - 2);
        assertEquals(belowLeftElevation, cornerZipperElevation, epsilonLatLon);

        cornerZipperElevation = cornerZipperTile.getElevationAtIndices(1, 1);
        belowLeftElevation = belowLeft.getElevationAtIndices(belowLeft.getLatitudeRows() - 1, belowLeft.getLongitudeColumns() - 1);
        assertEquals(belowLeftElevation, cornerZipperElevation, epsilonLatLon);

        cornerZipperElevation = cornerZipperTile.getElevationAtIndices(1, 2);
        belowRightElevation = belowRight.getElevationAtIndices(belowRight.getLatitudeRows() - 1, 0);
        assertEquals(belowRightElevation, cornerZipperElevation, epsilonLatLon);

        cornerZipperElevation = cornerZipperTile.getElevationAtIndices(1, 3);
        belowRightElevation = belowRight.getElevationAtIndices(belowRight.getLatitudeRows() - 1, 1);
        assertEquals(belowRightElevation, cornerZipperElevation, epsilonLatLon);

        // row 2 of zipper 
        cornerZipperElevation = cornerZipperTile.getElevationAtIndices(2, 0);
        double aboveLeftELevation = aboveLeft.getElevationAtIndices(0, aboveLeft.getLongitudeColumns() - 2);
        assertEquals(aboveLeftELevation, cornerZipperElevation, epsilonLatLon);
        //double leftAboveOfCurrentELevation = leftAboveOfCurrent.getElevationAtIndices(0, leftAboveOfCurrent.getLongitudeColumns() - 2);
        //assertEquals(leftAboveOfCurrentELevation, cornerZipperElevation, epsilonLatLon);

        cornerZipperElevation = cornerZipperTile.getElevationAtIndices(2, 1);
        aboveLeftELevation = aboveLeft.getElevationAtIndices(0, aboveLeft.getLongitudeColumns() - 1);
        assertEquals(aboveLeftELevation, cornerZipperElevation, epsilonLatLon);
        //leftAboveOfCurrentELevation = leftAboveOfCurrent.getElevationAtIndices(0, leftAboveOfCurrent.getLongitudeColumns() - 1);
        //assertEquals(leftAboveOfCurrentELevation, cornerZipperElevation, epsilonLatLon);

        cornerZipperElevation = cornerZipperTile.getElevationAtIndices(2, 2);
        double aboveRightElevation = aboveRight.getElevationAtIndices(0, 0);
        assertEquals(aboveRightElevation, cornerZipperElevation, epsilonLatLon);

        cornerZipperElevation = cornerZipperTile.getElevationAtIndices(2, 3);
        aboveRightElevation = aboveRight.getElevationAtIndices(0, 1);
        assertEquals(aboveRightElevation, cornerZipperElevation, epsilonLatLon);

        // row 3 of zipper
        cornerZipperElevation = cornerZipperTile.getElevationAtIndices(3, 0);
        aboveLeftELevation = aboveLeft.getElevationAtIndices(1, aboveLeft.getLongitudeColumns() - 2);
        assertEquals(aboveLeftELevation, cornerZipperElevation, epsilonLatLon);
        //leftAboveOfCurrentELevation = leftAboveOfCurrent.getElevationAtIndices(1, leftAboveOfCurrent.getLongitudeColumns() - 2);
        //assertEquals(leftAboveOfCurrentELevation, cornerZipperElevation, epsilonLatLon);

        cornerZipperElevation = cornerZipperTile.getElevationAtIndices(3, 1);
        aboveLeftELevation = aboveLeft.getElevationAtIndices(1, aboveLeft.getLongitudeColumns() - 1);
        assertEquals(aboveLeftELevation, cornerZipperElevation, epsilonLatLon);
        //leftAboveOfCurrentELevation = leftAboveOfCurrent.getElevationAtIndices(1, leftAboveOfCurrent.getLongitudeColumns() - 1);
        //assertEquals(leftAboveOfCurrentELevation, cornerZipperElevation, epsilonLatLon);

        cornerZipperElevation = cornerZipperTile.getElevationAtIndices(3, 2);
        aboveRightElevation = aboveRight.getElevationAtIndices(1, 0);
        assertEquals(aboveRightElevation, cornerZipperElevation, epsilonLatLon);

        cornerZipperElevation = cornerZipperTile.getElevationAtIndices(3, 3);
        aboveRightElevation = aboveRight.getElevationAtIndices(1, 1);
        assertEquals(aboveRightElevation, cornerZipperElevation, epsilonLatLon);
    }
    
    
    private void checkCornerElevationsDifferentStep(SimpleTile cornerZipperTile, SimpleTile originTile, 
                                       SimpleTile belowLeft, SimpleTile belowRight, SimpleTile aboveLeft, SimpleTile aboveRight, 
                                       double epsilonLatLon) {
        
        
        // We assure to be inside a cell by adding a delta step (to avoid inappropriate index computation)
        double deltaLon = 0.1*cornerZipperTile.getLongitudeStep();
        double zipperLongitude0 = cornerZipperTile.getLongitudeAtIndex(0) + deltaLon;
        double zipperLongitude1 = cornerZipperTile.getLongitudeAtIndex(1) + deltaLon;
        double zipperLongitude2 = cornerZipperTile.getLongitudeAtIndex(2) + deltaLon;
        double zipperLongitude3 = cornerZipperTile.getLongitudeAtIndex(3) + deltaLon;
        
        double deltaLat = 0.1*cornerZipperTile.getLatitudeStep();
        double zipperLatitude0 = cornerZipperTile.getLatitudeAtIndex(0) + deltaLat;
        double zipperLatitude1 = cornerZipperTile.getLatitudeAtIndex(1) + deltaLat;
        double zipperLatitude2 = cornerZipperTile.getLatitudeAtIndex(2) + deltaLat;
        double zipperLatitude3 = cornerZipperTile.getLatitudeAtIndex(3) + deltaLat;

        // Longitudes for column 0 of corner zipper
        double aboveLeftDoubleLongitudeIndex0 = (zipperLongitude0 - aboveLeft.getMinimumLongitude()) / aboveLeft.getLongitudeStep();
        int aboveLeftLongitudeIndex0 = FastMath.max(0, FastMath.min(aboveLeft.getLongitudeColumns() - 1, (int) FastMath.floor(aboveLeftDoubleLongitudeIndex0)));

        double belowLeftDoubleLongitudeIndex0 = (zipperLongitude0 - belowLeft.getMinimumLongitude()) / belowLeft.getLongitudeStep();
        int belowLeftLongitudeIndex0 = FastMath.max(0, FastMath.min(belowLeft.getLongitudeColumns() - 1, (int) FastMath.floor(belowLeftDoubleLongitudeIndex0)));

        // Longitudes for column 1 of corner zipper
        double aboveLeftDoubleLongitudeIndex1 = (zipperLongitude1 - aboveLeft.getMinimumLongitude()) / aboveLeft.getLongitudeStep();
        int aboveLeftLongitudeIndex1 = FastMath.max(0, FastMath.min(aboveLeft.getLongitudeColumns() - 1, (int) FastMath.floor(aboveLeftDoubleLongitudeIndex1)));

        double belowLeftDoubleLongitudeIndex1 = (zipperLongitude1 - belowLeft.getMinimumLongitude()) / belowLeft.getLongitudeStep();
        int belowLeftLongitudeIndex1 = FastMath.max(0, FastMath.min(belowLeft.getLongitudeColumns() - 1, (int) FastMath.floor(belowLeftDoubleLongitudeIndex1)));

        // Longitudes for column 2 of corner zipper
        double aboveRightDoubleLongitudeIndex2 = (zipperLongitude2 - aboveRight.getMinimumLongitude()) / aboveRight.getLongitudeStep();
        int aboveRightLongitudeIndex2 = FastMath.max(0, FastMath.min(aboveRight.getLongitudeColumns() - 1, (int) FastMath.floor(aboveRightDoubleLongitudeIndex2)));

        double belowRightDoubleLongitudeIndex2 = (zipperLongitude2 - belowRight.getMinimumLongitude()) / belowRight.getLongitudeStep();
        int belowRightLongitudeIndex2 = FastMath.max(0, FastMath.min(belowRight.getLongitudeColumns() - 1, (int) FastMath.floor(belowRightDoubleLongitudeIndex2)));

        // Longitudes for column 3 of corner zipper
        double aboveRightDoubleLongitudeIndex3 = (zipperLongitude3 - aboveRight.getMinimumLongitude()) / aboveRight.getLongitudeStep();
        int aboveRightLongitudeIndex3 = FastMath.max(0, FastMath.min(aboveRight.getLongitudeColumns() - 1, (int) FastMath.floor(aboveRightDoubleLongitudeIndex3)));

        double belowRightDoubleLongitudeIndex3 = (zipperLongitude3 - belowRight.getMinimumLongitude()) / belowRight.getLongitudeStep();
        int belowRightLongitudeIndex3 = FastMath.max(0, FastMath.min(belowRight.getLongitudeColumns() - 1, (int) FastMath.floor(belowRightDoubleLongitudeIndex3)));

        
        // row 0 of zipper 
        double belowLeftDoubleLatitudeIndex0 = (zipperLatitude0 - belowLeft.getMinimumLatitude()) / belowLeft.getLatitudeStep();
        int belowLeftLatitudeIndex0 = FastMath.max(0, FastMath.min(belowLeft.getLatitudeRows() - 1, (int) FastMath.floor(belowLeftDoubleLatitudeIndex0)));
        
        double belowLeftElevation = belowLeft.getElevationAtIndices(belowLeftLatitudeIndex0, belowLeftLongitudeIndex0);
        double cornerZipperElevation = cornerZipperTile.getElevationAtIndices(0, 0);
        assertEquals(belowLeftElevation, cornerZipperElevation, epsilonLatLon);

        belowLeftElevation = belowLeft.getElevationAtIndices(belowLeftLatitudeIndex0, belowLeftLongitudeIndex1);
        cornerZipperElevation = cornerZipperTile.getElevationAtIndices(0, 1);
        assertEquals(belowLeftElevation, cornerZipperElevation, epsilonLatLon);

        
        double belowRightDoubleLatitudeIndex0 = (zipperLatitude0 - belowRight.getMinimumLatitude()) / belowRight.getLatitudeStep();
        int belowRightLatitudeIndex0 = FastMath.max(0, FastMath.min(belowRight.getLatitudeRows() - 1, (int) FastMath.floor(belowRightDoubleLatitudeIndex0)));

        double belowRightElevation = belowRight.getElevationAtIndices(belowRightLatitudeIndex0, belowRightLongitudeIndex2);
        cornerZipperElevation = cornerZipperTile.getElevationAtIndices(0, 2);
        assertEquals(belowRightElevation, cornerZipperElevation, epsilonLatLon);

        belowRightElevation = belowRight.getElevationAtIndices(belowRightLatitudeIndex0, belowRightLongitudeIndex3);
        cornerZipperElevation = cornerZipperTile.getElevationAtIndices(0, 3);
        assertEquals(belowRightElevation, cornerZipperElevation, epsilonLatLon);


        // row 1 of zipper 
        double belowLeftDoubleLatitudeIndex1 = (zipperLatitude1 - belowLeft.getMinimumLatitude()) / belowLeft.getLatitudeStep();
        int belowLeftLatitudeIndex1 = FastMath.max(0, FastMath.min(belowLeft.getLatitudeRows() - 1, (int) FastMath.floor(belowLeftDoubleLatitudeIndex1)));
        
        belowLeftElevation = belowLeft.getElevationAtIndices(belowLeftLatitudeIndex1, belowLeftLongitudeIndex0);
        cornerZipperElevation = cornerZipperTile.getElevationAtIndices(1, 0);
        assertEquals(belowLeftElevation, cornerZipperElevation, epsilonLatLon);

        belowLeftElevation = belowLeft.getElevationAtIndices(belowLeftLatitudeIndex1, belowLeftLongitudeIndex1);
        cornerZipperElevation = cornerZipperTile.getElevationAtIndices(1, 1);
        assertEquals(belowLeftElevation, cornerZipperElevation, epsilonLatLon);
        

        double belowRightDoubleLatitudeIndex1 = (zipperLatitude1 - belowRight.getMinimumLatitude()) / belowRight.getLatitudeStep();
        int belowRightLatitudeIndex1 = FastMath.max(0, FastMath.min(belowRight.getLatitudeRows() - 1, (int) FastMath.floor(belowRightDoubleLatitudeIndex1)));

        belowRightElevation = belowRight.getElevationAtIndices(belowRightLatitudeIndex1, belowRightLongitudeIndex2);
        cornerZipperElevation = cornerZipperTile.getElevationAtIndices(1, 2);
        assertEquals(belowRightElevation, cornerZipperElevation, epsilonLatLon);

        cornerZipperElevation = cornerZipperTile.getElevationAtIndices(1, 3);
        belowRightElevation = belowRight.getElevationAtIndices(belowRightLatitudeIndex1, belowRightLongitudeIndex3);
        assertEquals(belowRightElevation, cornerZipperElevation, epsilonLatLon);

        // row 2 of zipper 
        double aboveLeftDoubleLatitudeIndex2 = (zipperLatitude2 - aboveLeft.getMinimumLatitude()) / aboveLeft.getLatitudeStep();
        int aboveLeftLatitudeIndex2 = FastMath.max(0, FastMath.min(aboveLeft.getLatitudeRows() - 1, (int) FastMath.floor(aboveLeftDoubleLatitudeIndex2)));
        
        double aboveLeftELevation = aboveLeft.getElevationAtIndices(aboveLeftLatitudeIndex2, aboveLeftLongitudeIndex0);
        cornerZipperElevation = cornerZipperTile.getElevationAtIndices(2, 0);
        assertEquals(aboveLeftELevation, cornerZipperElevation, epsilonLatLon);

        aboveLeftELevation = aboveLeft.getElevationAtIndices(aboveLeftLatitudeIndex2, aboveLeftLongitudeIndex1);
        cornerZipperElevation = cornerZipperTile.getElevationAtIndices(2, 1);
        assertEquals(aboveLeftELevation, cornerZipperElevation, epsilonLatLon);

        
        double aboveRightDoubleLatitudeIndex2 = (zipperLatitude2 - aboveRight.getMinimumLatitude()) / aboveRight.getLatitudeStep();
        int aboveRightLatitudeIndex2 = FastMath.max(0, FastMath.min(aboveRight.getLatitudeRows() - 1, (int) FastMath.floor(aboveRightDoubleLatitudeIndex2)));

        double aboveRightElevation = aboveRight.getElevationAtIndices(aboveRightLatitudeIndex2, aboveRightLongitudeIndex2);
        cornerZipperElevation = cornerZipperTile.getElevationAtIndices(2, 2);
        assertEquals(aboveRightElevation, cornerZipperElevation, epsilonLatLon);
         
        aboveRightElevation = aboveRight.getElevationAtIndices(aboveRightLatitudeIndex2, aboveRightLongitudeIndex3);
        cornerZipperElevation = cornerZipperTile.getElevationAtIndices(2, 3);
        assertEquals(aboveRightElevation, cornerZipperElevation, epsilonLatLon);

        // row 3 of zipper
        double aboveLeftDoubleLatitudeIndex3 = (zipperLatitude3 - aboveLeft.getMinimumLatitude()) / aboveLeft.getLatitudeStep();
        int aboveLeftLatitudeIndex3 = FastMath.max(0, FastMath.min(aboveLeft.getLatitudeRows() - 1, (int) FastMath.floor(aboveLeftDoubleLatitudeIndex3)));
        
        aboveLeftELevation = aboveLeft.getElevationAtIndices(aboveLeftLatitudeIndex3, aboveLeftLongitudeIndex0);
        cornerZipperElevation = cornerZipperTile.getElevationAtIndices(3, 0);
        assertEquals(aboveLeftELevation, cornerZipperElevation, epsilonLatLon);

        aboveLeftELevation = aboveLeft.getElevationAtIndices(aboveLeftLatitudeIndex3, aboveLeftLongitudeIndex1);
        cornerZipperElevation = cornerZipperTile.getElevationAtIndices(3, 1);
        assertEquals(aboveLeftELevation, cornerZipperElevation, epsilonLatLon);

        double aboveRightDoubleLatitudeIndex3 = (zipperLatitude3 - aboveRight.getMinimumLatitude()) / aboveRight.getLatitudeStep();
        int aboveRightLatitudeIndex3 = FastMath.max(0, FastMath.min(aboveRight.getLatitudeRows() - 1, (int) FastMath.floor(aboveRightDoubleLatitudeIndex3)));

        aboveRightElevation = aboveRight.getElevationAtIndices(aboveRightLatitudeIndex3, aboveRightLongitudeIndex2);
        cornerZipperElevation = cornerZipperTile.getElevationAtIndices(3, 2);
        assertEquals(aboveRightElevation, cornerZipperElevation, epsilonLatLon);
         
        aboveRightElevation = aboveRight.getElevationAtIndices(aboveRightLatitudeIndex3, aboveRightLongitudeIndex3);
        cornerZipperElevation = cornerZipperTile.getElevationAtIndices(3, 3);
        assertEquals(aboveRightElevation, cornerZipperElevation, epsilonLatLon);
    }

    	
    private void checkZipperTile(SimpleTile zipperTile, Tile.Location zipperLocation, 
                                 SimpleTile originTile, double tileSizeDeg, 
                                 TilesCache<SimpleTile> cache, double epsilonLatLon) {

    	SurroundingTiles surroundingTiles = new SurroundingTiles(originTile, tileSizeDeg, cache);
    	
        if (zipperLocation == Tile.Location.SOUTH) {
        	SimpleTile tileBelow = surroundingTiles.getTileBelow();
            for (int jLon = 0; jLon < zipperTile.getLongitudeColumns(); jLon++) {
                // row 0 of zipper 
                double zipperElevation = zipperTile.getElevationAtIndices(0, jLon);
                double belowELevation = tileBelow.getElevationAtIndices(tileBelow.getLatitudeRows() - 2, jLon);
                assertEquals(belowELevation, zipperElevation, epsilonLatLon);
   
                // row 1 of zipper 
                zipperElevation = zipperTile.getElevationAtIndices(1, jLon);
                belowELevation = tileBelow.getElevationAtIndices(tileBelow.getLatitudeRows() - 1, jLon);
                assertEquals(belowELevation, zipperElevation, epsilonLatLon);
                
                // row 2 of zipper 
                zipperElevation = zipperTile.getElevationAtIndices(2, jLon);
                double originELevation = originTile.getElevationAtIndices(0, jLon);
                assertEquals(originELevation, zipperElevation, epsilonLatLon);

                // row 3 of zipper 
                zipperElevation = zipperTile.getElevationAtIndices(3, jLon);
                originELevation = originTile.getElevationAtIndices(1, jLon);
                assertEquals(originELevation, zipperElevation, epsilonLatLon);
            }
        } else if (zipperLocation == Tile.Location.NORTH) {
        	SimpleTile tileAbove = surroundingTiles.getTileAbove();
            for (int jLon = 0; jLon < zipperTile.getLongitudeColumns(); jLon++) {
                // row 0 of zipper 
                double zipperElevation = zipperTile.getElevationAtIndices(0, jLon);
                double originELevation = originTile.getElevationAtIndices(originTile.getLatitudeRows() - 2, jLon);
                assertEquals(originELevation, zipperElevation, epsilonLatLon);
   
                // row 1 of zipper 
                zipperElevation = zipperTile.getElevationAtIndices(1, jLon);
                originELevation = originTile.getElevationAtIndices(originTile.getLatitudeRows() - 1, jLon);
                assertEquals(originELevation, zipperElevation, epsilonLatLon);
                
                // row 2 of zipper 
                zipperElevation = zipperTile.getElevationAtIndices(2, jLon);
                double aboveELevation = tileAbove.getElevationAtIndices(0, jLon);
                assertEquals(aboveELevation, zipperElevation, epsilonLatLon);

                // row 3 of zipper 
                zipperElevation = zipperTile.getElevationAtIndices(3, jLon);
                aboveELevation = tileAbove.getElevationAtIndices(1, jLon);
                assertEquals(aboveELevation, zipperElevation, epsilonLatLon);
            }
        } else if (zipperLocation == Tile.Location.WEST) {
        	SimpleTile tileLeft = surroundingTiles.getTileLeft();
            for (int iLat = 0; iLat < zipperTile.getLatitudeRows(); iLat++) {
                // col 0 of zipper 
                double zipperElevation = zipperTile.getElevationAtIndices(iLat, 0);
                double leftELevation = tileLeft.getElevationAtIndices(iLat, tileLeft.getLongitudeColumns() - 2);
                assertEquals(leftELevation, zipperElevation, epsilonLatLon);
   
                // col 1 of zipper 
                zipperElevation = zipperTile.getElevationAtIndices(iLat, 1);
                leftELevation = tileLeft.getElevationAtIndices(iLat, tileLeft.getLongitudeColumns() - 1);
                assertEquals(leftELevation, zipperElevation, epsilonLatLon);
                
                // col 2 of zipper 
                zipperElevation = zipperTile.getElevationAtIndices(iLat, 2);
                double originELevation = originTile.getElevationAtIndices(iLat, 0);
                assertEquals(originELevation, zipperElevation, epsilonLatLon);

                // col 3 of zipper 
                zipperElevation = zipperTile.getElevationAtIndices(iLat, 3);
                originELevation = originTile.getElevationAtIndices(iLat, 1);
                assertEquals(originELevation, zipperElevation, epsilonLatLon);
            }

        } else if (zipperLocation == Tile.Location.EAST) {
        	SimpleTile tileRight = surroundingTiles.getTileRight();
            for (int iLat = 0; iLat < zipperTile.getLatitudeRows(); iLat++) {
                // col 0 of zipper 
                double zipperElevation = zipperTile.getElevationAtIndices(iLat, 0);
                double originELevation = originTile.getElevationAtIndices(iLat, originTile.getLongitudeColumns() - 2);
                assertEquals(originELevation, zipperElevation, epsilonLatLon);
   
                // col 1 of zipper 
                zipperElevation = zipperTile.getElevationAtIndices(iLat, 1);
                originELevation = originTile.getElevationAtIndices(iLat, originTile.getLongitudeColumns() - 1);
                assertEquals(originELevation, zipperElevation, epsilonLatLon);
                
                // col 2 of zipper 
                zipperElevation = zipperTile.getElevationAtIndices(iLat, 2);
                double rightELevation = tileRight.getElevationAtIndices(iLat, 0);
                assertEquals(rightELevation, zipperElevation, epsilonLatLon);

                // col 3 of zipper 
                zipperElevation = zipperTile.getElevationAtIndices(iLat, 3);
                rightELevation = tileRight.getElevationAtIndices(iLat, 1);
                assertEquals(rightELevation, zipperElevation, epsilonLatLon);
            }
        }
    }
    
    private void checkZipperTileDifferentStep(SimpleTile zipperTile, Tile.Location zipperLocation, 
                                              SimpleTile originTile, double tileSizeDeg, 
                                              TilesCache<SimpleTile> cache, double epsilonLatLon) {

        SurroundingTiles surroundingTiles = new SurroundingTiles(originTile, tileSizeDeg, cache);

        if (zipperLocation == Tile.Location.SOUTH) {
            
            SimpleTile tileBelow = surroundingTiles.getTileBelow();
            
            for (int jLon = 0; jLon < zipperTile.getLongitudeColumns(); jLon++) {
                
                // We assure to be inside a cell by adding a delta step (to avoid inappropriate index computation)
                double deltaLon = 0.1*zipperTile.getLongitudeStep();
                double zipperLongitude = zipperTile.getLongitudeAtIndex(jLon) + deltaLon;
                
                double originTileDoubleLongitudeIndex = (zipperLongitude - originTile.getMinimumLongitude()) / originTile.getLongitudeStep();
                int originTileLongitudeIndex = FastMath.max(0, FastMath.min(originTile.getLongitudeColumns() - 1, (int) FastMath.floor(originTileDoubleLongitudeIndex)));

                double belowTileDoubleLongitudeIndex = (zipperLongitude - tileBelow.getMinimumLongitude()) / tileBelow.getLongitudeStep();
                int belowTileLongitudeIndex = FastMath.max(0, FastMath.min(tileBelow.getLongitudeColumns() - 1, (int) FastMath.floor(belowTileDoubleLongitudeIndex)));

 
                // row 0 of zipper
                double zipperLat0 = zipperTile.getMinimumLatitude() + 0.1*zipperTile.getLatitudeStep();
                double belowTileDoubleLatitudeIndex =  (zipperLat0  - tileBelow.getMinimumLatitude())  / tileBelow.getLatitudeStep();
                int belowTileLatitudeIndex0 = FastMath.max(0, FastMath.min(tileBelow.getLatitudeRows() - 1, (int) FastMath.floor(belowTileDoubleLatitudeIndex)));

                double zipperElevation = zipperTile.getElevationAtIndices(0, jLon);
                double belowELevation = tileBelow.getElevationAtIndices(belowTileLatitudeIndex0, belowTileLongitudeIndex);
                assertEquals(belowELevation, zipperElevation, epsilonLatLon);

                // row 1 of zipper
                double zipperLat1 = zipperLat0 + zipperTile.getLatitudeStep();
                belowTileDoubleLatitudeIndex =  (zipperLat1  - tileBelow.getMinimumLatitude())  / tileBelow.getLatitudeStep();
                int originTileLatitudeIndex1 = FastMath.max(0, FastMath.min(tileBelow.getLatitudeRows() - 1, (int) FastMath.floor(belowTileDoubleLatitudeIndex)));

                zipperElevation = zipperTile.getElevationAtIndices(1, jLon);
                belowELevation = tileBelow.getElevationAtIndices(originTileLatitudeIndex1, belowTileLongitudeIndex);
                assertEquals(belowELevation, zipperElevation, epsilonLatLon);

                // row 2 of zipper 
                double zipperLat2 = zipperLat0 + 2*zipperTile.getLatitudeStep();
                double originTileDoubleLatitudeIndex =  (zipperLat2  - originTile.getMinimumLatitude())  / originTile.getLatitudeStep();
                int originTileLatitudeIndex2 = FastMath.max(0, FastMath.min(originTile.getLatitudeRows() - 1, (int) FastMath.floor(originTileDoubleLatitudeIndex)));
               
                zipperElevation = zipperTile.getElevationAtIndices(2, jLon);
                double originELevation = originTile.getElevationAtIndices(originTileLatitudeIndex2, originTileLongitudeIndex);
                assertEquals(originELevation, zipperElevation, epsilonLatLon);

                // row 3 of zipper 
                double zipperLat3 = zipperLat0 + 3*zipperTile.getLatitudeStep();
                originTileDoubleLatitudeIndex =  (zipperLat3  - originTile.getMinimumLatitude())  / originTile.getLatitudeStep();
                int originTileLatitudeIndex3 = FastMath.max(0, FastMath.min(originTile.getLatitudeRows() - 1, (int) FastMath.floor(originTileDoubleLatitudeIndex)));

                zipperElevation = zipperTile.getElevationAtIndices(3, jLon);
                originELevation = originTile.getElevationAtIndices(originTileLatitudeIndex3, originTileLongitudeIndex);
                assertEquals(originELevation, zipperElevation, epsilonLatLon);
            }
        } else if (zipperLocation == Tile.Location.NORTH) {
            
            SimpleTile tileAbove = surroundingTiles.getTileAbove();
                        
            for (int jLon = 0; jLon < zipperTile.getLongitudeColumns(); jLon++) {
                
                // We assure to be inside a cell by adding a delta step (to avoid inappropriate index computation)
                double deltaLon = 0.1*zipperTile.getLongitudeStep();
                double zipperLongitude = zipperTile.getLongitudeAtIndex(jLon) + deltaLon;
                
                double originTileDoubleLongitudeIndex = (zipperLongitude - originTile.getMinimumLongitude()) / originTile.getLongitudeStep();
                int originTileLongitudeIndex = FastMath.max(0, FastMath.min(originTile.getLongitudeColumns() - 1, (int) FastMath.floor(originTileDoubleLongitudeIndex)));

                double aboveTileDoubleLongitudeIndex = (zipperLongitude - tileAbove.getMinimumLongitude()) / tileAbove.getLongitudeStep();
                int aboveTileLongitudeIndex = FastMath.max(0, FastMath.min(tileAbove.getLongitudeColumns() - 1, (int) FastMath.floor(aboveTileDoubleLongitudeIndex)));

                // row 0 of zipper
                double zipperLat0 = zipperTile.getMinimumLatitude() + 0.1*zipperTile.getLatitudeStep();
                double originTileDoubleLatitudeIndex =  (zipperLat0  - originTile.getMinimumLatitude())  / originTile.getLatitudeStep();
                int originTileLatitudeIndex0 = FastMath.max(0, FastMath.min(originTile.getLatitudeRows() - 1, (int) FastMath.floor(originTileDoubleLatitudeIndex)));

                double zipperElevation = zipperTile.getElevationAtIndices(0, jLon);
                double originELevation = originTile.getElevationAtIndices(originTileLatitudeIndex0, originTileLongitudeIndex);
                assertEquals(originELevation, zipperElevation, epsilonLatLon);

                // row 1 of zipper 
                double zipperLat1 = zipperLat0 + zipperTile.getLatitudeStep();
                originTileDoubleLatitudeIndex =  (zipperLat1  - originTile.getMinimumLatitude())  / originTile.getLatitudeStep();
                int originTileLatitudeIndex1 = FastMath.max(0, FastMath.min(originTile.getLatitudeRows() - 1, (int) FastMath.floor(originTileDoubleLatitudeIndex)));

                zipperElevation = zipperTile.getElevationAtIndices(1, jLon);
                originELevation = originTile.getElevationAtIndices(originTileLatitudeIndex1, originTileLongitudeIndex);
                assertEquals(originELevation, zipperElevation, epsilonLatLon);

                // row 2 of zipper 
                double zipperLat2 = zipperLat0 + 2*zipperTile.getLatitudeStep();
                double aboveTileDoubleLatitudeIndex =  (zipperLat2  - tileAbove.getMinimumLatitude())  / tileAbove.getLatitudeStep();
                int aboveTileLatitudeIndex2 = FastMath.max(0, FastMath.min(tileAbove.getLatitudeRows() - 1, (int) FastMath.floor(aboveTileDoubleLatitudeIndex)));

                zipperElevation = zipperTile.getElevationAtIndices(2, jLon);
                double aboveELevation = tileAbove.getElevationAtIndices(aboveTileLatitudeIndex2, aboveTileLongitudeIndex);
                assertEquals(aboveELevation, zipperElevation, epsilonLatLon);

                // row 3 of zipper 
                double zipperLat3 = zipperLat0 + 3*zipperTile.getLatitudeStep();
                aboveTileDoubleLatitudeIndex =  (zipperLat3  - tileAbove.getMinimumLatitude())  / tileAbove.getLatitudeStep();
                int aboveTileLatitudeIndex3 = FastMath.max(0, FastMath.min(tileAbove.getLatitudeRows() - 1, (int) FastMath.floor(aboveTileDoubleLatitudeIndex)));

                zipperElevation = zipperTile.getElevationAtIndices(3, jLon);
                aboveELevation = tileAbove.getElevationAtIndices(aboveTileLatitudeIndex3, aboveTileLongitudeIndex);
                assertEquals(aboveELevation, zipperElevation, epsilonLatLon);
            }
        } else if (zipperLocation == Tile.Location.WEST) {
            SimpleTile tileLeft = surroundingTiles.getTileLeft();
            for (int iLat = 0; iLat < zipperTile.getLatitudeRows(); iLat++) {
                // col 0 of zipper 
                double zipperElevation = zipperTile.getElevationAtIndices(iLat, 0);
                double leftELevation = tileLeft.getElevationAtIndices(iLat, tileLeft.getLongitudeColumns() - 2);
                assertEquals(leftELevation, zipperElevation, epsilonLatLon);

                // col 1 of zipper 
                zipperElevation = zipperTile.getElevationAtIndices(iLat, 1);
                leftELevation = tileLeft.getElevationAtIndices(iLat, tileLeft.getLongitudeColumns() - 1);
                assertEquals(leftELevation, zipperElevation, epsilonLatLon);

                // col 2 of zipper 
                zipperElevation = zipperTile.getElevationAtIndices(iLat, 2);
                double originELevation = originTile.getElevationAtIndices(iLat, 0);
                assertEquals(originELevation, zipperElevation, epsilonLatLon);

                // col 3 of zipper 
                zipperElevation = zipperTile.getElevationAtIndices(iLat, 3);
                originELevation = originTile.getElevationAtIndices(iLat, 1);
                assertEquals(originELevation, zipperElevation, epsilonLatLon);
            }

        } else if (zipperLocation == Tile.Location.EAST) {
            SimpleTile tileRight = surroundingTiles.getTileRight();
            for (int iLat = 0; iLat < zipperTile.getLatitudeRows(); iLat++) {
                // col 0 of zipper 
                double zipperElevation = zipperTile.getElevationAtIndices(iLat, 0);
                double originELevation = originTile.getElevationAtIndices(iLat, originTile.getLongitudeColumns() - 2);
                assertEquals(originELevation, zipperElevation, epsilonLatLon);

                // col 1 of zipper 
                zipperElevation = zipperTile.getElevationAtIndices(iLat, 1);
                originELevation = originTile.getElevationAtIndices(iLat, originTile.getLongitudeColumns() - 1);
                assertEquals(originELevation, zipperElevation, epsilonLatLon);

                // col 2 of zipper 
                zipperElevation = zipperTile.getElevationAtIndices(iLat, 2);
                double rightELevation = tileRight.getElevationAtIndices(iLat, 0);
                assertEquals(rightELevation, zipperElevation, epsilonLatLon);

                // col 3 of zipper 
                zipperElevation = zipperTile.getElevationAtIndices(iLat, 3);
                rightELevation = tileRight.getElevationAtIndices(iLat, 1);
                assertEquals(rightELevation, zipperElevation, epsilonLatLon);
            }
        }
    }


    private final static double getNorthernEdgeOfTile(SimpleTile tile) {
        double northernLatitudeForTile = getNorthernLatitudeForTile(tile);
        // Inside the northern row of latitude
        return northernLatitudeForTile - 0.1*FastMath.toDegrees(tile.getLatitudeStep());
    }
    
    private final static double getSouthernEdgeOfTile(SimpleTile tile) {
      double southernLatitudeForTile = getSouthernLatitudeForTile(tile);
      // Inside the southern row of latitude
      return southernLatitudeForTile + 0.1*FastMath.toDegrees(tile.getLatitudeStep());
    }

    private final static double getWesternEdgeOfTile(SimpleTile tile) {
        double westernLongitudeForTile = getWesternLongitudeForTile(tile);
        // Inside the western column of longitude
        return westernLongitudeForTile + 0.1*FastMath.toDegrees(tile.getLongitudeStep());
    }

    private final static double getEasternEdgeOfTile(SimpleTile tile) {
        double easternLongitudeForTile = getEasternLongitudeForTile(tile);
        // Inside the eastern column of longitude
        return easternLongitudeForTile - 0.1*FastMath.toDegrees(tile.getLongitudeStep());
    }
    
    
    private final static double getNorthernLatitudeForTile(SimpleTile tile) {
        return FastMath.toDegrees(tile.getMaximumLatitude()) + 0.5*FastMath.toDegrees(tile.getLatitudeStep());
    }
    
    private final static double getSouthernLatitudeForTile(SimpleTile tile) {
        return FastMath.toDegrees(tile.getMinimumLatitude()) - 0.5*FastMath.toDegrees(tile.getLatitudeStep());
    }

    private final static double getWesternLongitudeForTile(SimpleTile tile) {
        return FastMath.toDegrees(tile.getMinimumLongitude()) - 0.5*FastMath.toDegrees(tile.getLongitudeStep());
    }

    private final static double getEasternLongitudeForTile(SimpleTile tile) {
        return FastMath.toDegrees(tile.getMaximumLongitude()) + 0.5*FastMath.toDegrees(tile.getLongitudeStep());
    }


    
    private void checkLatLonIndex(final double latDeg, final int expectedLatIndex, 
                                  final double lonDeg, final int expectedLonIndex, 
                                  final double expectedElevation,
                                  final SimpleTile tile) {
        
        double latRad = FastMath.toRadians(latDeg);
        double lonRad = FastMath.toRadians(lonDeg);

        assertTrue(tile.getFloorLatitudeIndex(latRad) == expectedLatIndex);
        assertTrue((FastMath.toDegrees(tile.getLatitudeAtIndex(expectedLatIndex)) <= latDeg &&
                (FastMath.toDegrees(tile.getLatitudeAtIndex(expectedLatIndex+1)) >= latDeg)));
        
        assertTrue(tile.getFloorLongitudeIndex(lonRad) == expectedLonIndex);
        assertTrue((FastMath.toDegrees(tile.getLongitudeAtIndex(expectedLonIndex)) <= lonDeg &&
                (FastMath.toDegrees(tile.getLongitudeAtIndex(expectedLonIndex+1)) >= lonDeg)));
        
        assertEquals(expectedElevation, tile.getElevationAtIndices(expectedLatIndex, expectedLonIndex), epsilonElevation);
    }
    
    final static double epsilonElevation = 1.e-6;

    final static java.text.DecimalFormatSymbols symbols = new java.text.DecimalFormatSymbols(java.util.Locale.US);
    final static java.text.DecimalFormat dfs = new java.text.DecimalFormat("#.#####",symbols);

    private void printTileInfo(SimpleTile tile, double stepLatDeg, double stepLonDeg){
        try {
            System.out.format(
                    "Bilan found Tile: " + tile.getLatitudeRows() + " lat rows x " + 
                                           tile.getLongitudeColumns() + " long columns \n" +  
                    "                   Rugged Convention { " +
                    " min lat = " + dfs.format(FastMath.toDegrees(tile.getMinimumLatitude())) + 
                    " max lat = " + dfs.format(FastMath.toDegrees(tile.getMaximumLatitude())) + "}" + 
                    " {" + 
                    " min lon = " + dfs.format(FastMath.toDegrees(tile.getMinimumLongitude())) + 
                    " max lon = " + dfs.format(FastMath.toDegrees(tile.getMaximumLongitude())) + "}\n");
            System.out.format(
                    "                   SRTM Convention { "+
                    " min lat = " + dfs.format(FastMath.toDegrees(tile.getMinimumLatitude())-0.5*stepLatDeg) + 
                    " max lat = " + dfs.format(FastMath.toDegrees(tile.getMaximumLatitude())+0.5*stepLatDeg) + "}" + 
                    " {" + 
                    " min lon = " + dfs.format(FastMath.toDegrees(tile.getMinimumLongitude())-0.5*stepLonDeg) + 
                    " max lon = " + dfs.format(FastMath.toDegrees(tile.getMaximumLongitude())+0.5*stepLonDeg) + "}" +
                    " \n\n");
        } catch (Exception e) {
            System.out.println("problem");
        }
    }
    
    private void printTileData(String tileName, SimpleTile tile, int stepPrint) throws FileNotFoundException, UnsupportedEncodingException {
        
        printExtractTileData(tileName, tile, stepPrint, 0, tile.getLatitudeRows(), 0, tile.getLongitudeColumns());
    }

    private void printExtractTileData(String tileName, SimpleTile tile, int stepPrint,
            int iLatMin, int iLatMax, int jLonMin, int jLonMax) throws FileNotFoundException, UnsupportedEncodingException {
        
        PrintWriter elevationPrinter = new PrintWriter(tileName, "UTF-8");

//        elevationPrinter.println("# latitude(°) longitude(°) h (m)" + "\n" + delimiter);
        for (int jLon = jLonMin; jLon < jLonMax; jLon += stepPrint) {
//            if (jLon % 100 == 0) System.out.println("longitude = " + jLon);
            for (int iLat = iLatMin; iLat < iLatMax; iLat += stepPrint) {

                double elevation = tile.getElevationAtIndices(iLat, jLon);
                double latitude = tile.getLatitudeAtIndex(iLat);
                double longitude = tile.getLongitudeAtIndex(jLon);
                elevationPrinter.format(Locale.US, "%3.8f %3.8f %.4f%n",
                                        FastMath.toDegrees(latitude), FastMath.toDegrees(longitude), elevation);
            }
        }
        if (elevationPrinter != null) elevationPrinter.close();

    }
 
    private void printEdgeTileData(String tileName, SimpleTile tile, int stepPrint,
            int iLatMax, int jLonMax, int nbRowCols) throws FileNotFoundException, UnsupportedEncodingException {
        
        PrintWriter elevationPrinter = new PrintWriter(tileName, "UTF-8");

        for (int jLon = 0; jLon < jLonMax; jLon += stepPrint) {
//            if (jLon % 100 == 0) System.out.println("longitude = " + jLon);
            for (int iLat = 0; iLat < iLatMax; iLat += stepPrint) {

                if ((((0 <= iLat) && (iLat < nbRowCols)) || ((iLatMax - nbRowCols <= iLat) && (iLat < iLatMax))) ||
                    (((0 <= jLon) && (jLon < nbRowCols)) || ((jLonMax - nbRowCols <= jLon) && (jLon < jLonMax)))
                    ) {
                    double elevation = tile.getElevationAtIndices(iLat, jLon);
                    double latitude = tile.getLatitudeAtIndex(iLat);
                    double longitude = tile.getLongitudeAtIndex(jLon);
                    elevationPrinter.format(Locale.US, "%3.8f %3.8f %.4f%n",
                            FastMath.toDegrees(latitude), FastMath.toDegrees(longitude), elevation);
                }
            }
        }
        if (elevationPrinter != null) elevationPrinter.close();
        
//        +        printExtractTileData(tileName + "NorthEdge", tile, stepPrint, iLatMax - nbRowCols, iLatMax, 0, jLonMax);
//        +        printExtractTileData(tileName + "SouthEdge", tile, stepPrint, 0, nbRowCols, 0, jLonMax);
//        +        printExtractTileData(tileName + "WestEdge", tile, stepPrint, 0, iLatMax, 0, nbRowCols);
//        +        printExtractTileData(tileName + "EastEdge", tile, stepPrint, 0, iLatMax, jLonMax - nbRowCols, jLonMax);

    }      

    
    /** Clean up of factory map
     * @param factoryClass
     */
    private static void clearFactoryMaps(Class<?> factoryClass) {
        try {
            
            for (Field field : factoryClass.getDeclaredFields()) {
                if (Modifier.isStatic(field.getModifiers()) &&
                    Map.class.isAssignableFrom(field.getType())) {
                    field.setAccessible(true);
                    ((Map<?, ?>) field.get(null)).clear();
                }
            }
        } catch (IllegalAccessException iae) {
            Assert.fail(iae.getMessage());
        }
    }

//    // Test for development purpose only: use a real DEM (SRTM)
//    // Needs:
//    // * install GDAL
//    // * add in pom.xml
//    // For <properties>:
//    //    <!-- GDAL version -->
//    //    <!--urugged.gdal.version>2.4.0</urugged.gdal.version-->
//    //    <rugged.gdal.version>3.0.0</rugged.gdal.version>
//    //    <!-- GDAL native library path -->
//    //    <rugged.gdal.native.library.path>${env.GDAL_PATH}</rugged.gdal.native.library.path>
//    // and for <dependencies>:
//    //    <dependency>
//    //      <groupId>org.gdal</groupId>
//    //      <artifactId>gdal</artifactId>
//    //      <version>${rugged.gdal.version}</version>
//    //      <type>jar</type>
//    //      <optional>false</optional>
//    //      <scope>test</scope>
//    //    </dependency>
//    // * Get the following SRTM tiles (for instance from : http://dwtkns.com/srtm/
//    //    01/{15,16}; 22/{02,03,04}; 23/{02,03,04}; 24/{02,03,04,17,18,19}; 25/{17,18,19}; 26/{17,18,19}; 
//    //    38/{02,03,04}; 39/{02,03,04}; 40/{02,03,04,15,16,17}; 41/{15,16,17}; 42/{15,16,17}; 72/{15,16}
//
//    @Test
//    public void testRealSRTM() throws URISyntaxException, FileNotFoundException, UnsupportedEncodingException {
//
//        // Initialize GDAL
//        org.gdal.gdal.gdal.AllRegister();
//
//        String demRootDir = "/home/guylaine/RuggedAndCo/rugged/dem-data/SRTM";
//        SRTMElevationUpdater srtmUpdater = new SRTMElevationUpdater(demRootDir);
//
//        CountingFactory factory = new CountingFactory();
//        TilesCache<SimpleTile> cache = new TilesCache<SimpleTile>(factory, srtmUpdater , 5, false);
//
//        boolean isDifferentStep = false;
//
//        double epsilonLatLon = 1.e-5;
//        double latTileDeg;
//        double lonTileDeg;
//        double latDeg;
//        double lonDeg;
//
//        // North-East hemisphere 
//        // =====================
//        System.out.println("#################################");
//        System.out.println("NORTH EAST hemisphere");
//        // tile SRTM: 45N - 50N / 10E-15E
//        latTileDeg = 47.;
//        lonTileDeg = 12.3;
//
//        System.out.println(">>>> Search lat deg = " + latTileDeg + " lon deg= " + lonTileDeg + "\n");
//        SimpleTile tileNEhemisphere = cache.getTile(FastMath.toRadians(latTileDeg), FastMath.toRadians(lonTileDeg));
//        // same step and size in longitude and latitude
//        double tileSizeDeg = FastMath.toDegrees(tileNEhemisphere.getLatitudeRows()*tileNEhemisphere.getLatitudeStep());
//        double rasterStepDeg = FastMath.toDegrees(tileNEhemisphere.getLatitudeStep());
//
//
//        // Latitude North of the tile
//        System.out.println("##### Bord de tuile au niveau latitude North #####");
//        latDeg = getNorthernEdgeOfTile(tileNEhemisphere);
//        lonDeg = lonTileDeg;
//
//        searchAndVerifyZipperTile(latDeg, lonDeg, Tile.Location.NORTH, tileNEhemisphere, tileSizeDeg, cache, epsilonLatLon, isDifferentStep);
//
//
//        // Latitude South of the tile
//        System.out.println("#### Bord de tuile au niveau latitude South #####");
//        latDeg = getSouthernEdgeOfTile(tileNEhemisphere);
//        lonDeg = lonTileDeg;
//
//        searchAndVerifyZipperTile(latDeg, lonDeg, Tile.Location.SOUTH, tileNEhemisphere, tileSizeDeg, cache, epsilonLatLon, isDifferentStep);
//
//        // Longitude West of the tile
//        System.out.println("#### Bord de tuile au niveau longitude West #####");
//        latDeg = latTileDeg;
//        lonDeg = getWesternEdgeOfTile(tileNEhemisphere);
//
//        searchAndVerifyZipperTile(latDeg, lonDeg, Tile.Location.WEST, tileNEhemisphere, tileSizeDeg, cache, epsilonLatLon, isDifferentStep);
//
//
//        // Longitude East of the tile
//        System.out.println("#### Bord de tuile au niveau longitude East #####");
//        latDeg = latTileDeg;
//        lonDeg = getEasternEdgeOfTile(tileNEhemisphere);
//
//        searchAndVerifyZipperTile(latDeg, lonDeg, Tile.Location.EAST,tileNEhemisphere, tileSizeDeg, cache, epsilonLatLon, isDifferentStep);
//
//
//        // Check the 4 corner zipper tiles
//        check4cornersZipperTiles(tileNEhemisphere, tileSizeDeg, cache, epsilonLatLon, isDifferentStep);
//
//        // South-East hemisphere 
//        // =====================
//        // Cleanup
//        clearFactoryMaps(CountingFactory.class);
//        factory = new CountingFactory();
//        cache = new TilesCache<SimpleTile>(factory, srtmUpdater , 5, false);
//
//        System.out.println("#################################");
//        System.out.println("SOUTH EAST hemisphere");
//
//        latTileDeg = -16.2;
//        lonTileDeg = 22.3;
//        System.out.println(">>>> Search lat deg = " + latTileDeg + " lon deg= " + lonTileDeg + "\n");
//        SimpleTile tileSEhemisphere = cache.getTile(FastMath.toRadians(latTileDeg), FastMath.toRadians(lonTileDeg));
//        tileSizeDeg = FastMath.toDegrees(tileSEhemisphere.getLatitudeRows()*tileSEhemisphere.getLatitudeStep());
//        rasterStepDeg = FastMath.toDegrees(tileSEhemisphere.getLatitudeStep());
//
//        printTileInfo(tileSEhemisphere, rasterStepDeg, rasterStepDeg);
//
//
//        // Latitude North of the tile
//        System.out.println("##### Bord de tuile au niveau latitude North #####");
//        latDeg = getNorthernEdgeOfTile(tileSEhemisphere);
//        lonDeg = lonTileDeg;
//
//        searchAndVerifyZipperTile(latDeg, lonDeg, Tile.Location.NORTH, tileSEhemisphere, tileSizeDeg, cache, epsilonLatLon, isDifferentStep);
//
//
//        // Latitude South of the tile
//        System.out.println("#### Bord de tuile au niveau latitude South #####");
//        latDeg = getSouthernEdgeOfTile(tileSEhemisphere);
//        lonDeg = lonTileDeg;
//
//        searchAndVerifyZipperTile(latDeg, lonDeg, Tile.Location.SOUTH, tileSEhemisphere, tileSizeDeg, cache, epsilonLatLon, isDifferentStep);
//
//
//        // Longitude West of the tile
//        System.out.println("#### Bord de tuile au niveau longitude West #####");
//        latDeg = latTileDeg;
//        lonDeg = getWesternEdgeOfTile(tileSEhemisphere);
//
//        searchAndVerifyZipperTile(latDeg, lonDeg, Tile.Location.WEST, tileSEhemisphere, tileSizeDeg, cache, epsilonLatLon, isDifferentStep);
//
//
//        // Longitude East of the tile
//        System.out.println("#### Bord de tuile au niveau longitude East #####");
//        latDeg = latTileDeg;
//        lonDeg = getEasternEdgeOfTile(tileSEhemisphere);
//        searchAndVerifyZipperTile(latDeg, lonDeg, Tile.Location.EAST, tileSEhemisphere, tileSizeDeg, cache, epsilonLatLon, isDifferentStep);
//
//        // Check the 4 corner zipper tiles
//        check4cornersZipperTiles(tileSEhemisphere, tileSizeDeg, cache, epsilonLatLon, isDifferentStep);
//
//        // North-West hemisphere 
//        // =====================
//        // Cleanup
//        clearFactoryMaps(CountingFactory.class);
//        factory = new CountingFactory();
//        cache = new TilesCache<SimpleTile>(factory, srtmUpdater , 5, false);
//
//        System.out.println("#################################");
//        System.out.println("NORTH WEST hemisphere");
//        latTileDeg = 46.8;
//        lonTileDeg = -66.5; 
//
//        System.out.println(">>>> Search lat deg = " + latTileDeg + " lon deg= " + lonTileDeg + "\n");
//        SimpleTile tileNWhemisphere = cache.getTile(FastMath.toRadians(latTileDeg), FastMath.toRadians(lonTileDeg));
//        tileSizeDeg = FastMath.toDegrees(tileNWhemisphere.getLatitudeRows()*tileNWhemisphere.getLatitudeStep());
//        rasterStepDeg = FastMath.toDegrees(tileNWhemisphere.getLatitudeStep());
//        printTileInfo(tileNWhemisphere, rasterStepDeg, rasterStepDeg);
//
//
//        // Latitude North of the tile
//        System.out.println("##### Bord de tuile au niveau latitude North #####");
//        latDeg = getNorthernEdgeOfTile(tileNWhemisphere);
//        lonDeg = lonTileDeg;
//
//        searchAndVerifyZipperTile(latDeg, lonDeg, Tile.Location.NORTH, tileNWhemisphere, tileSizeDeg, cache, epsilonLatLon, isDifferentStep);
//
//
//        // Latitude South of the tile
//        System.out.println("#### Bord de tuile au niveau latitude South #####");
//        latDeg = getSouthernEdgeOfTile(tileNWhemisphere);
//        lonDeg = lonTileDeg;
//
//        searchAndVerifyZipperTile(latDeg, lonDeg, Tile.Location.SOUTH, tileNWhemisphere, tileSizeDeg, cache, epsilonLatLon, isDifferentStep);
//
//
//        // Longitude West of the tile
//        System.out.println("#### Bord de tuile au niveau longitude West #####");
//        latDeg = latTileDeg;
//        lonDeg = getWesternEdgeOfTile(tileNWhemisphere);
//
//        searchAndVerifyZipperTile(latDeg, lonDeg, Tile.Location.WEST, tileNWhemisphere, tileSizeDeg, cache, epsilonLatLon, isDifferentStep);
//
//
//        // Longitude East of the tile
//        System.out.println("#### Bord de tuile au niveau longitude East #####");
//        latDeg = latTileDeg;
//        lonDeg = getEasternEdgeOfTile(tileNWhemisphere);
//
//        searchAndVerifyZipperTile(latDeg, lonDeg, Tile.Location.EAST, tileNWhemisphere, tileSizeDeg, cache, epsilonLatLon, isDifferentStep);
//
//        // Check the 4 corner zipper tiles
//        check4cornersZipperTiles(tileNWhemisphere, tileSizeDeg, cache, epsilonLatLon, isDifferentStep);
//
//        // South-West hemisphere 
//        // =====================
//        // Cleanup
//        clearFactoryMaps(CountingFactory.class);
//        factory = new CountingFactory();
//        cache = new TilesCache<SimpleTile>(factory, srtmUpdater , 5, false);
//
//        System.out.println("#################################");
//        System.out.println("SOUTH WEST hemisphere");
//
//        latTileDeg = -28.8  ;
//        lonTileDeg = -58.4; 
//        System.out.println(">>>> Search lat deg = " + latTileDeg + " lon deg= " + lonTileDeg + "\n");
//        SimpleTile tileSWhemisphere = cache.getTile(FastMath.toRadians(latTileDeg), FastMath.toRadians(lonTileDeg));
//        tileSizeDeg = FastMath.toDegrees(tileSWhemisphere.getLatitudeRows()*tileSWhemisphere.getLatitudeStep());
//        rasterStepDeg = FastMath.toDegrees(tileSWhemisphere.getLatitudeStep());
//        printTileInfo(tileSWhemisphere, rasterStepDeg, rasterStepDeg);
//
//        // Latitude North of the tile
//        System.out.println("##### Bord de tuile au niveau latitude North #####");
//        latDeg = getNorthernEdgeOfTile(tileSWhemisphere);
//        lonDeg = lonTileDeg;
//
//        searchAndVerifyZipperTile(latDeg, lonDeg, Tile.Location.NORTH, tileSWhemisphere, tileSizeDeg, cache, epsilonLatLon, isDifferentStep);
//
//
//        // Latitude South of the tile
//        System.out.println("#### Bord de tuile au niveau latitude South #####");
//        latDeg = getSouthernEdgeOfTile(tileSWhemisphere);
//        lonDeg = lonTileDeg;
//
//        searchAndVerifyZipperTile(latDeg, lonDeg, Tile.Location.SOUTH, tileSWhemisphere, tileSizeDeg, cache, epsilonLatLon, isDifferentStep);
//
//
//        // Longitude West of the tile
//        System.out.println("#### Bord de tuile au niveau longitude West #####");
//        latDeg = latTileDeg;
//        lonDeg = getWesternEdgeOfTile(tileSWhemisphere);
//
//        searchAndVerifyZipperTile(latDeg, lonDeg, Tile.Location.WEST, tileSWhemisphere, tileSizeDeg, cache, epsilonLatLon, isDifferentStep);
//
//        // Longitude East of the tile
//        System.out.println("#### Bord de tuile au niveau longitude East #####");
//        latDeg = latTileDeg;
//        lonDeg = getEasternEdgeOfTile(tileSWhemisphere);
//
//        searchAndVerifyZipperTile(latDeg, lonDeg, Tile.Location.EAST, tileSWhemisphere, tileSizeDeg, cache, epsilonLatLon, isDifferentStep);
//
//        // Check the 4 corner zipper tiles
//        check4cornersZipperTiles(tileSWhemisphere, tileSizeDeg, cache, epsilonLatLon, isDifferentStep);
//        
//        // Cleanup
//        clearFactoryMaps(CountingFactory.class);
//        factory = new CountingFactory();
//        cache = new TilesCache<SimpleTile>(factory, srtmUpdater , 5, false);
//
//        // Anti meridians  (180 degrees W/E)
//        // =====================
//        // Cleanup
//        clearFactoryMaps(CountingFactory.class);
//        factory = new CountingFactory();
//        cache = new TilesCache<SimpleTile>(factory, srtmUpdater , 5, false);
//
//
//        System.out.println("#################################");
//        System.out.println("Anti meridien test (180 degre East)");
//        // tile SRTM 72/16: 175 - 180 East / 15 - 20 South  
//        latTileDeg = -18.;
//        lonTileDeg = 178.;
//        System.out.println(">>>> Search lat deg = " + latTileDeg + " lon deg= " + lonTileDeg + "\n");
//        SimpleTile tileAntiMeridianEast = cache.getTile(FastMath.toRadians(latTileDeg), FastMath.toRadians(lonTileDeg));
//        // same step and size in longitude and latitude
//        tileSizeDeg = FastMath.toDegrees(tileAntiMeridianEast.getLatitudeRows()*tileAntiMeridianEast.getLatitudeStep());
//        rasterStepDeg = FastMath.toDegrees(tileAntiMeridianEast.getLatitudeStep());
//
//        // Longitude East of the tile
//        System.out.println("#### Bord de tuile au niveau longitude East #####");
//        latDeg = latTileDeg;
//        lonDeg = getEasternEdgeOfTile(tileAntiMeridianEast);
//
//        searchAndVerifyZipperTile(latDeg, lonDeg, Tile.Location.EAST,tileAntiMeridianEast, tileSizeDeg, cache, epsilonLatLon, isDifferentStep);
//
//
//        System.out.println("#################################");
//        System.out.println("Anti meridien test (180 degre West)");
//        // tile SRTM 01/16: 175 - 180 West / 15 - 20 South  
//        latTileDeg = -18.;
//        lonTileDeg = -178.;
//        System.out.println(">>>> Search lat deg = " + latTileDeg + " lon deg= " + lonTileDeg + "\n");
//        SimpleTile tileAntiMeridianWest = cache.getTile(FastMath.toRadians(latTileDeg), FastMath.toRadians(lonTileDeg));
//        // same step and size in longitude and latitude
//        tileSizeDeg = FastMath.toDegrees(tileAntiMeridianWest.getLatitudeRows()*tileAntiMeridianWest.getLatitudeStep());
//        rasterStepDeg = FastMath.toDegrees(tileAntiMeridianWest.getLatitudeStep());
//
//        // Longitude West of the tile
//        System.out.println("#### Bord de tuile au niveau longitude West #####");
//        latDeg = latTileDeg;
//        lonDeg = getWesternEdgeOfTile(tileAntiMeridianWest);
//
//        searchAndVerifyZipperTile(latDeg, lonDeg, Tile.Location.WEST,tileAntiMeridianWest, tileSizeDeg, cache, epsilonLatLon, isDifferentStep);
//
//    }
    

}

//// Test for development purpose only: use a real DEM (SRTM)
//
///**
// * To read SRTM tiles (get from http://dwtkns.com/srtm/)
// * The tiles are seamless = no overlapping 
// */
//class SRTMElevationUpdater implements TileUpdater {
//    
//    /** Last lat value used to ask to update a tile. */
//    private double lastLat = Double.NaN;
//
//    /** Last lon value used to ask to update a tile. */
//    private double lastLon = Double.NaN;
//
//    /** Nb times the same lat/lon value is used to ask to update a tile (used to avoid infinite loop). */
//    private int nbCall = 0;
//
//    /** Nb time rugged ask exactly same DEM tile: means that infinite loop. */
//    private static final int NB_TIME_ASK_SAME_TILE = 1000;
//
//    /** Raster root directory, should contains raster files. */
//    private String rootDirectory;
//
//
//    /**
//     * Constructor.
//     */
//    public SRTMElevationUpdater(final String rootDirectory) {
//        
//        this.rootDirectory = rootDirectory;
//        checkRasterDirectory(rootDirectory);
//    }
//    
//    /** Update given tile using DEM elevation.
//     * @param latitude latitude that must be covered by the tile (rad)
//     * @param longitude longitude that must be covered by the tile (rad)
//     * @param tile to update
//     */
//    @Override
//    public void updateTile(final double latitude, final double longitude, final UpdatableTile tile) {
//
//        // Check if latitude and longitude already known
//        if (latitude == this.lastLat && longitude == this.lastLon) {
//            this.nbCall++;
//        } else {
//            this.lastLat = latitude;
//            this.lastLon = longitude;
//            this.nbCall = 0;
//        }
//        if (this.nbCall > NB_TIME_ASK_SAME_TILE) {
//            String str = String.format("infinite loop for %3.8f long %3.8f lat ", longitude, latitude);
//            org.hipparchus.exception.DummyLocalizable message = new org.hipparchus.exception.DummyLocalizable(str);
//            throw new org.orekit.rugged.errors.RuggedException(message);
//        }
//
//        String rasterFileName = getRasterFilePath(latitude, longitude);
//        
////        java.io.FileFile rasterFile = new java.io.File(rasterFileName);
////        if (!rasterFile.exists()) {
////            // No DEM => we are on water => read geoid
////            GeoidHandler geoidHandler = GeoidHandler.getInstance();
////            geoidHandler.updateTile(latitude, longitude, tile);
////            System.out.format("WARNING: No DEM tile found for latitude (deg) = %3.8f  and longitude (deg) = %3.8f." +
////                              " DEM file %s doesn't exist. Use of Geoid data instead.%n",
////                              FastMath.toDegrees(latitude), FastMath.toDegrees(longitude), rasterFileName);
////            return;
////        }
//
//        org.gdal.gdal.Dataset ds = org.gdal.gdal.gdal.Open(rasterFileName, org.gdal.gdalconst.gdalconst.GA_ReadOnly);
//        int rasterLonSize = ds.GetRasterXSize();
//        int rasterLatSize = ds.GetRasterYSize();
//        double[] geoTransformInfo = ds.GetGeoTransform();
//
//        double minLat = Double.NaN;
//        double minLon = Double.NaN;
//        double lonStep = Double.NaN;
//        double latStep = Double.NaN;
//
//        if (geoTransformInfo.length < 6) { // Check that the geoTransformInfo is correct
//            String str = "GDALGeoTransform has < 6 elements";
//            org.hipparchus.exception.DummyLocalizable message = new org.hipparchus.exception.DummyLocalizable(str);
//            throw new org.orekit.rugged.errors.RuggedException(message);
//        } else {
//            lonStep = FastMath.abs(geoTransformInfo[1]);
//            latStep = FastMath.abs(geoTransformInfo[5]);
//
//            minLon = geoTransformInfo[0] + 0.5 * lonStep;
//            minLat = geoTransformInfo[3] - (rasterLatSize - 0.5) * latStep;
//        }
//
//        org.gdal.gdal.Band band = ds.GetRasterBand(1);
//
//        // Define Tile Geometry
//        tile.setGeometry(FastMath.toRadians(minLat), FastMath.toRadians(minLon), FastMath.toRadians(latStep), FastMath.toRadians(lonStep), rasterLatSize, rasterLonSize);
//
//        // Loop over raster values
//        double[] data = new double[rasterLatSize * rasterLonSize];
//
////        // SRTM is given above the geoid
////         GeoidHandler geoidHandler = GeoidHandler.getInstance();
////         GdalTransformTools gtTools = new GdalTransformTools(geoTransformInfo, rasterLonSize, rasterLatSize);
//
//        // We read all raster at once to limit the numbers of Band.ReadRaster calls
//        band.ReadRaster(0, 0, rasterLonSize, rasterLatSize, data);
//
//        // Get the no data value from the raster
//        Double[] noDataValue = new Double[1];
//        band.GetNoDataValue(noDataValue);
//
//        // test if the no data value exists
//        Boolean noDataValueExist = false;
//        if (noDataValue[0] != null) noDataValueExist = true;
//
//        // from bottom left to upper right corner
//        for (int iLat = rasterLatSize - 1; iLat >= 0; iLat--) {
//            for (int jLon = 0; jLon < rasterLonSize; jLon++) {
//
//                double elevationOverEllipsoid = 0.0;
//                elevationOverEllipsoid = data[iLat * rasterLonSize + jLon];
//
//                if (noDataValueExist && (elevationOverEllipsoid == noDataValue[0])) {
//                    elevationOverEllipsoid = 0.0;
//                }
//                
////                // The elevation value we send to rugged must be computed against ellipsoid
////                // => when DEM is SRTM , we must add geoid value
////                double lon = gtTools.getXFromPixelLine(jLon, iLat);
////                double lat = gtTools.getYFromPixelLine(jLon, iLat);
////                elevationOverEllipsoid = elevationOverEllipsoid + geoidHandler.getElevationDegree(lat, lon);
//
//                // Set elevation over the ellipsoid
//                tile.setElevation(rasterLatSize - 1 - iLat, jLon, elevationOverEllipsoid);
//            }
//        }
//        band.delete();
//        band = null;
//        ds.delete();
//        ds = null;
//    }
//
//    private String getRasterFilePath(final double latitude, final double longitude) {
//
//        double latDeg = FastMath.toDegrees(latitude);
//        // Assure that the longitude belongs to [-180, + 180]
//        double lonDeg = FastMath.toDegrees(MathUtils.normalizeAngle(longitude, 0.0));
//
//        // Compute parent dir with longitude value
//        int parentDirValue = (int) (1 + (lonDeg + 180.) / 5);
//        String parentDir = String.format("%02d", parentDirValue);
//
//        // Compute sub dir with latitude value
//        int subDirValue = (int) (1 + (60. - latDeg) / 5);
//        String subDir = String.format("%02d", subDirValue);
//
//        String filePath = this.rootDirectory + java.io.File.separator + parentDir + java.io.File.separator + subDir + java.io.File.separator +
//                          "srtm_" + parentDir + "_" + subDir + ".tif";
//        return filePath;
//    }
//    
//    private boolean checkRasterDirectory(final String directory) {
//        
//        if (directory == null) {
//
//            String str = "Directory not defined";
//            org.hipparchus.exception.DummyLocalizable message = new org.hipparchus.exception.DummyLocalizable(str);
//            throw new org.orekit.rugged.errors.RuggedException(message);
//
//        } else {
//            try {
//                java.nio.file.Path dir = java.nio.file.FileSystems.getDefault().getPath(directory);
//                java.nio.file.DirectoryStream<java.nio.file.Path> stream = java.nio.file.Files.newDirectoryStream(dir);
//                boolean found = false;
//                for (java.nio.file.Path path : stream) {
//                    if (!found) {
//                        java.io.File currentFile = path.toFile();
//                        if (currentFile.isDirectory()) {
//                            found = checkRasterDirectory(currentFile.getAbsolutePath());
//                        } else {
//                            String filePath = currentFile.getAbsolutePath();
//                            if (filePath.matches(".*.tif")) {
//                                found = true;
//                            }
//                        }
//                        if (found) {
//                            stream.close();
//                            return true;
//                        }
//                    }
//                }
//                stream.close();
//
//                String str = "raster not found  in" + directory;
//                org.hipparchus.exception.DummyLocalizable message = new org.hipparchus.exception.DummyLocalizable(str);
//                throw new org.orekit.rugged.errors.RuggedException(message);
//
//            } catch (java.io.IOException e) {
//                String str = "dir not found " + directory;
//                org.hipparchus.exception.DummyLocalizable message = new org.hipparchus.exception.DummyLocalizable(str);
//                throw new org.orekit.rugged.errors.RuggedException(message);
//            }
//        }
//    }
//}

class SurroundingTiles {

    private double latDeg;
    private double lonDeg;
    private TilesCache<SimpleTile> cache;
    private double tileSizeDeg;
	
	SurroundingTiles(SimpleTile originTile, double tileSizeDeg, TilesCache<SimpleTile> cache) {

	    this.cache = cache;
	    this.tileSizeDeg = tileSizeDeg;
	    
		int latRows = originTile.getLatitudeRows();
		int loncols = originTile.getLongitudeColumns();
		
		// Get a middle point of the tile
		this.latDeg = FastMath.toDegrees(originTile.getLatitudeAtIndex(latRows/2));
		this.lonDeg = FastMath.toDegrees(originTile.getLongitudeAtIndex(loncols/2));
	}

	public SimpleTile getTileAbove() {
	    
	    // get tile above ...
        double latAbove =  latDeg + tileSizeDeg;
        return cache.getTile(FastMath.toRadians(latAbove), FastMath.toRadians(lonDeg));
	}

	public SimpleTile getTileBelow() {
	    
        // get tile below
        double latBelow = latDeg - tileSizeDeg;
        return cache.getTile(FastMath.toRadians(latBelow), FastMath.toRadians(lonDeg));
	}

	public SimpleTile getTileLeft() {
	    
	    // get tile left
	    double lonLeftRad = FastMath.toRadians(lonDeg - tileSizeDeg);
        // Assure that the longitude belongs to [-180, + 180]
        double lonLeftNorm = MathUtils.normalizeAngle(lonLeftRad, 0.0);
        return cache.getTile(FastMath.toRadians(latDeg), lonLeftNorm);
	}

	public SimpleTile getTileRight() {
	    
        // get tile right
	    double lonRightRad = FastMath.toRadians(lonDeg + tileSizeDeg);
        // Assure that the longitude belongs to [-180, + 180]
        double lonRightNorm = MathUtils.normalizeAngle(lonRightRad, 0.0);
        return cache.getTile(FastMath.toRadians(latDeg), lonRightNorm);
	}
}
