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

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.io.UnsupportedEncodingException;
import java.net.URISyntaxException;
import java.util.Locale;

import org.hipparchus.random.RandomGenerator;
import org.hipparchus.random.Well19937a;
import org.hipparchus.util.FastMath;
import org.junit.Assert;
import org.junit.Test;

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
        final int rowCols = 1000;
        final double tileSizeDeg = 5.;
        DummySRTMsimpleElevationUpdater srtmUpdater = new DummySRTMsimpleElevationUpdater(tileSizeDeg, rowCols, 10.0, 20.0);
        final double rasterStepDeg = srtmUpdater.getTileStepDeg(); // 1./1200.;
        
//        // Complex SRTM with random values
//        final int rowCols = 4097; // power of 2 + 1
//        final double tileSizeDeg = 5.;
//        DummySRTMelevationUpdater srtmUpdater = new DummySRTMelevationUpdater(tileSizeDeg, rowCols, 800.0, 9000.0, 0.1, 0xf0a401650191f9f6l);
//        final double rasterStepDeg = srtmUpdater.getTileStepDeg(); // 1./1200.;
       
        CountingFactory factory = new CountingFactory();
        TilesCache<SimpleTile> cache = new TilesCache<SimpleTile>(factory, srtmUpdater , 5, false);
        double epsilonLatLon = 1.e-5;
        double latDeg;
        double lonDeg;
        SimpleTile zipperTile;
        
        // North-East hemisphere 
        // =====================
        System.out.println("#################################");
        System.out.println("NORTH EAST hemisphere");
        latDeg = 37.1;
        lonDeg = 17.5;

        System.out.println(">>>> Search lat deg = " + latDeg + " lon deg= " + lonDeg + "\n");
        SimpleTile tileNE = cache.getTile(FastMath.toRadians(latDeg), FastMath.toRadians(lonDeg));
        printTileInfo(tileNE, rasterStepDeg, rasterStepDeg);
        if (rowCols == 6000) checkLatLonIndex(latDeg, 2519, lonDeg, 2999, 77.729902, tileNE);

        // Latitude North of the tile
        System.out.println("##### Bord de tuile au niveau latitude North #####");
        latDeg = getNorthernEdgeOfTile(tileNE);
        lonDeg = 17.5;
        zipperTile = searchAndVerifyZipperTile(latDeg, lonDeg, Tile.Location.NORTH, tileNE, tileSizeDeg, rasterStepDeg, cache, epsilonLatLon);

        if (rowCols == 6000) checkOneValueZipperTile(latDeg, lonDeg, zipperTile, 4, rowCols, epsilonLatLon,
                39.99875, 40.00125, 15.00042, 19.99958,
                1, 2999,  82.248141);

//      String zipperName = "zipperNorthLatitude.txt";
//      printTileData(zipperName, zipperTile, stepPrintZipper);

        // Latitude South of the tile
        System.out.println("#### Changement de tuile au niveau latitude South #####");
        latDeg = getSouthernEdgeOfTile(tileNE);
        lonDeg = 17.5;
        zipperTile = searchAndVerifyZipperTile(latDeg, lonDeg, Tile.Location.SOUTH, tileNE, tileSizeDeg, rasterStepDeg, cache, epsilonLatLon);

        if (rowCols == 6000) checkOneValueZipperTile(latDeg, lonDeg, zipperTile, 4, rowCols, epsilonLatLon,
                34.99875, 35.00125, 15.00042, 19.99958,
                1, 2999,  73.224625);

        //      zipperName = "zipperSouthLatitude.txt";
//      printTileData(zipperName, zipperTile, stepPrintZipper);

        // Longitude West of the tile
        System.out.println("#### Changement de tuile au niveau longitude West #####");
        latDeg = 37.1;
        lonDeg = getWesternEdgeOfTile(tileNE);
        zipperTile = searchAndVerifyZipperTile(latDeg, lonDeg, Tile.Location.WEST, tileNE, tileSizeDeg, rasterStepDeg, cache, epsilonLatLon);

        if (rowCols == 6000) checkOneValueZipperTile(latDeg, lonDeg, zipperTile, rowCols, 4, epsilonLatLon,
                35.00042, 39.99958, 14.99875, 15.00125,
                2519, 1,  90.134193);

//        zipperName = "zipperWestLongitude.txt";
//        printTileData(zipperName, zipperTile, stepPrintZipper);

        // Longitude East of the tile
        System.out.println("#### Changement de tuile au niveau longitude East #####");
        latDeg = 37.1;
        lonDeg = getEasternEdgeOfTile(tileNE);
        zipperTile = searchAndVerifyZipperTile(latDeg, lonDeg, Tile.Location.EAST,tileNE, tileSizeDeg, rasterStepDeg, cache, epsilonLatLon);

        if (rowCols == 6000) checkOneValueZipperTile(latDeg, lonDeg, zipperTile, rowCols, 4, epsilonLatLon,
                35.00042, 39.99958, 19.99875, 20.00125,
                2519, 1,  63.887513);

//        zipperName = "zipperEastLongitude.txt";
//        printTileData(zipperName, zipperTile, stepPrintZipper);
        
        // Check the 4 corner zipper tiles
        check4cornersZipperTiles(tileNE, tileSizeDeg, rasterStepDeg, cache, epsilonLatLon);

        // South-East hemisphere 
        // =====================
        System.out.println("#################################");
        System.out.println("SOUTH EAST hemisphere");
        
        latDeg = -37.1;
        lonDeg = 17.5;
        System.out.println(">>>> Search lat deg = " + latDeg + " lon deg= " + lonDeg + "\n");
        SimpleTile tileSE = cache.getTile(FastMath.toRadians(latDeg), FastMath.toRadians(lonDeg));
        printTileInfo(tileSE, rasterStepDeg, rasterStepDeg);
        if (rowCols == 6000) checkLatLonIndex(latDeg, 3479, lonDeg, 2999,  -307.157295, tileSE);

//        String tileName = "lat" + Double.toString(latDeg) + "lon" + Double.toString(lonDeg) + ".txt";
//        int stepPrint = 1;
//        int stepPrintZipper = 1;
//        int nbRowColForPrint = 10;
////        printTileData(tileName, tileNE, stepPrint);
////        printExtractTileData(tileName, tileNE, stepPrint, tileNE.getLatitudeRows() - nbRowColForPrint, tileNE.getLatitudeRows(), 0, tileNE.getLongitudeColumns());
//        printEdgeTileData(tileName, tileSE, stepPrint, tileSE.getLatitudeRows(), tileSE.getLongitudeColumns(), nbRowColForPrint);
//        printSurroundingTiles(latDeg, lonDeg, tileSizeDeg, rasterStepDeg, cache, stepPrint, nbRowColForPrint);

        // Latitude North of the tile
        System.out.println("##### Bord de tuile au niveau latitude North #####");
        latDeg = getNorthernEdgeOfTile(tileSE);
        lonDeg = 17.5;
        zipperTile = searchAndVerifyZipperTile(latDeg, lonDeg, Tile.Location.NORTH, tileSE, tileSizeDeg, rasterStepDeg, cache, epsilonLatLon);

        if (rowCols == 6000) checkOneValueZipperTile(latDeg, lonDeg, zipperTile, 4, rowCols, epsilonLatLon,
                -35.00125, -34.99875, 15.00042, 19.99958,
                1, 2999, -302.652423);

//        String zipperName = "zipperNorthLatitude.txt";
//        printTileData(zipperName, zipperTile, stepPrintZipper);

        // Latitude South of the tile
        System.out.println("#### Changement de tuile au niveau latitude South #####");
        latDeg = getSouthernEdgeOfTile(tileSE);
        lonDeg = 17.5;
        zipperTile = searchAndVerifyZipperTile(latDeg, lonDeg, Tile.Location.SOUTH, tileSE, tileSizeDeg, rasterStepDeg, cache, epsilonLatLon);

        if (rowCols == 6000) checkOneValueZipperTile(latDeg, lonDeg, zipperTile, 4, rowCols, epsilonLatLon,
                -40.00125, -39.99875, 15.00042, 19.99958,
                1, 2999, -327.258708);
//        zipperName = "zipperSouthLatitude.txt";
//        printTileData(zipperName, zipperTile, stepPrintZipper);

        // Longitude West of the tile
        System.out.println("#### Changement de tuile au niveau longitude West #####");
        latDeg = -37.1;
        lonDeg = getWesternEdgeOfTile(tileSE);
        zipperTile = searchAndVerifyZipperTile(latDeg, lonDeg, Tile.Location.WEST, tileSE, tileSizeDeg, rasterStepDeg, cache, epsilonLatLon);

        if (rowCols == 6000) checkOneValueZipperTile(latDeg, lonDeg, zipperTile, rowCols, 4, epsilonLatLon,
                -39.99958, -35.00042, 14.99875, 15.00125,
                3479, 1, -292.441872);

        //        zipperName = "zipperWestLongitude.txt";
//        printTileData(zipperName, zipperTile, stepPrintZipper);

        // Longitude East of the tile
        System.out.println("#### Changement de tuile au niveau longitude East #####");
        latDeg = -37.1;
        lonDeg = getEasternEdgeOfTile(tileSE);
        zipperTile = searchAndVerifyZipperTile(latDeg, lonDeg, Tile.Location.EAST, tileSE, tileSizeDeg, rasterStepDeg, cache, epsilonLatLon);

        if (rowCols == 6000) checkOneValueZipperTile(latDeg, lonDeg, zipperTile, rowCols, 4, epsilonLatLon,
                -39.99958, -35.00042, 19.99875, 20.00125,
                3479, 1, -320.999684);
        
        //        zipperName = "zipperEastLongitude.txt";
        //        printTileData(zipperName, zipperTile, stepPrintZipper);

        // Check the 4 corner zipper tiles
        check4cornersZipperTiles(tileSE, tileSizeDeg, rasterStepDeg, cache, epsilonLatLon);

        // North-West hemisphere 
        // =====================
        System.out.println("#################################");
        System.out.println("NORTH WEST hemisphere");
        latDeg = 37.1;
        lonDeg = -17.5;
        
        System.out.println(">>>> Search lat deg = " + latDeg + " lon deg= " + lonDeg + "\n");
        SimpleTile tileNW = cache.getTile(FastMath.toRadians(latDeg), FastMath.toRadians(lonDeg));
        printTileInfo(tileNW, rasterStepDeg, rasterStepDeg);
        if (rowCols == 6000) checkLatLonIndex(latDeg, 2519, lonDeg, 2999,  76.790119, tileNW);

        // Latitude North of the tile
        System.out.println("##### Bord de tuile au niveau latitude North #####");
        latDeg = getNorthernEdgeOfTile(tileNW);
        lonDeg = -17.5;
        zipperTile = searchAndVerifyZipperTile(latDeg, lonDeg, Tile.Location.NORTH, tileNW, tileSizeDeg, rasterStepDeg, cache, epsilonLatLon);

        if (rowCols == 6000) checkOneValueZipperTile(latDeg, lonDeg, zipperTile, 4, rowCols, epsilonLatLon,
                39.99875, 40.00125, -19.99958, -15.00042,
                1, 2999, 77.919679);

        // Latitude South of the tile
        System.out.println("#### Changement de tuile au niveau latitude South #####");
        latDeg = getSouthernEdgeOfTile(tileNW);
        lonDeg = -17.5;
        zipperTile = searchAndVerifyZipperTile(latDeg, lonDeg, Tile.Location.SOUTH, tileNW, tileSizeDeg, rasterStepDeg, cache, epsilonLatLon);

        if (rowCols == 6000) checkOneValueZipperTile(latDeg, lonDeg, zipperTile, 4, rowCols, epsilonLatLon,
                34.99875, 35.00125, -19.99958, -15.00042,
                1, 2999, 90.796560);

        // Longitude West of the tile
        System.out.println("#### Changement de tuile au niveau longitude West #####");
        latDeg = 37.1;
        lonDeg = getWesternEdgeOfTile(tileNW);
        zipperTile = searchAndVerifyZipperTile(latDeg, lonDeg, Tile.Location.WEST, tileNW, tileSizeDeg, rasterStepDeg, cache, epsilonLatLon);

        if (rowCols == 6000) checkOneValueZipperTile(latDeg, lonDeg, zipperTile, rowCols, 4, epsilonLatLon,
                35.00042, 39.99958, -20.00125, -19.99875,
                2519, 1, 80.250639);

        // Longitude East of the tile
        System.out.println("#### Changement de tuile au niveau longitude East #####");
        latDeg = 37.1;
        lonDeg = getEasternEdgeOfTile(tileNW);
        zipperTile = searchAndVerifyZipperTile(latDeg, lonDeg, Tile.Location.EAST, tileNW, tileSizeDeg, rasterStepDeg, cache, epsilonLatLon);
        
        if (rowCols == 6000) checkOneValueZipperTile(latDeg, lonDeg, zipperTile, rowCols, 4, epsilonLatLon,
                35.00042, 39.99958, -15.00125, -14.99875,
                2519, 1, 73.111331);

        // Check the 4 corner zipper tiles
        check4cornersZipperTiles(tileNW, tileSizeDeg, rasterStepDeg, cache, epsilonLatLon);

      // South-West hemisphere 
      // =====================
      System.out.println("#################################");
      System.out.println("SOUTH WEST hemisphere");
      
      latDeg = -37.1;
      lonDeg = -17.5;
      System.out.println(">>>> Search lat deg = " + latDeg + " lon deg= " + lonDeg + "\n");
      SimpleTile tileSW = cache.getTile(FastMath.toRadians(latDeg), FastMath.toRadians(lonDeg));
      printTileInfo(tileSW, rasterStepDeg, rasterStepDeg);
      if (rowCols == 6000) checkLatLonIndex(latDeg, 3479, lonDeg, 2999,  -64.1245445, tileSW);

      // Latitude North of the tile
      System.out.println("##### Bord de tuile au niveau latitude North #####");
      latDeg = getNorthernEdgeOfTile(tileSW);
      lonDeg = -17.5;
      zipperTile = searchAndVerifyZipperTile(latDeg, lonDeg, Tile.Location.NORTH, tileSW, tileSizeDeg, rasterStepDeg, cache, epsilonLatLon);

      if (rowCols == 6000) checkOneValueZipperTile(latDeg, lonDeg, zipperTile, 4, rowCols, epsilonLatLon,
              -35.00125, -34.99875, -19.99958, -15.000416,
              1, 2999, -60.408025);

      // Latitude South of the tile
      System.out.println("#### Changement de tuile au niveau latitude South #####");
      latDeg = getSouthernEdgeOfTile(tileSW);
      lonDeg = -17.5;
      zipperTile = searchAndVerifyZipperTile(latDeg, lonDeg, Tile.Location.SOUTH, tileSW, tileSizeDeg, rasterStepDeg, cache, epsilonLatLon);

      if (rowCols == 6000) checkOneValueZipperTile(latDeg, lonDeg, zipperTile, 4, rowCols, epsilonLatLon,
              -40.00125, -39.99875, -19.99958, -15.000417,
              1, 2999, -67.851618);

      // Longitude West of the tile
      System.out.println("#### Changement de tuile au niveau longitude West #####");
      latDeg = -37.1;
      lonDeg = getWesternEdgeOfTile(tileSW);
      zipperTile = searchAndVerifyZipperTile(latDeg, lonDeg, Tile.Location.WEST, tileSW, tileSizeDeg, rasterStepDeg, cache, epsilonLatLon);

      if (rowCols == 6000) checkOneValueZipperTile(latDeg, lonDeg, zipperTile, rowCols, 4, epsilonLatLon,
              -39.99958, -35.00042, -20.00125, -19.99875,
              3479, 1, -51.107712);

      // Longitude East of the tile
      System.out.println("#### Changement de tuile au niveau longitude East #####");
      latDeg = -37.1;
      lonDeg = getEasternEdgeOfTile(tileSW);
      zipperTile = searchAndVerifyZipperTile(latDeg, lonDeg, Tile.Location.EAST, tileSW, tileSizeDeg, rasterStepDeg, cache, epsilonLatLon);
 
      if (rowCols == 6000) checkOneValueZipperTile(latDeg, lonDeg, zipperTile, rowCols, 4, epsilonLatLon,
              -39.99958, -35.00042, -15.00125, -14.998750,
              3479, 1, -76.264544);

      // Check the 4 corner zipper tiles
      check4cornersZipperTiles(tileSW, tileSizeDeg, rasterStepDeg, cache, epsilonLatLon);
    }


    private SimpleTile searchAndVerifyZipperTile(double latDeg, double lonDeg, Tile.Location zipperLocation, SimpleTile tile, final double tileSizeDeg,
            final double rasterStepDeg, TilesCache<SimpleTile> cache, double epsilonLatLon) {
 
        System.out.println(">>>> Search lat deg = " + latDeg + " lon deg= " + lonDeg + "\n");

        SimpleTile zipperTile = cache.getTile(FastMath.toRadians(latDeg), FastMath.toRadians(lonDeg));
        
        printTileInfo(zipperTile, rasterStepDeg, rasterStepDeg);

        checkGeodeticPointBelongsToZipper(FastMath.toRadians(latDeg), FastMath.toRadians(lonDeg), zipperTile);
        checkZipperTile(zipperTile, zipperLocation, tile, tileSizeDeg, cache, epsilonLatLon);
        return zipperTile;
    }

    private void check4cornersZipperTiles(SimpleTile tile, final double tileSizeDeg, final double rasterStepDeg,
                                          TilesCache<SimpleTile> cache, double epsilonLatLon) {
        double latDeg;
        double lonDeg;
        
        SimpleTile cornerZipperTile;
        // Latitude/Longitude corner NW
        System.out.println("##### Coin de tuile au niveau latitude North et longitude West #####");
        latDeg = getNorthernEdgeOfTile(tile);
        lonDeg = getWesternEdgeOfTile(tile);
        System.out.println(">>>> Search lat deg = " + latDeg + " lon deg= " + lonDeg + "\n");
        cornerZipperTile = cache.getTile(FastMath.toRadians(latDeg), FastMath.toRadians(lonDeg));
        printTileInfo(cornerZipperTile, rasterStepDeg, rasterStepDeg);

        checkGeodeticPointBelongsToZipper(FastMath.toRadians(latDeg), FastMath.toRadians(lonDeg), cornerZipperTile);
        checkCornerZipperTile(cornerZipperTile, Tile.Location.NORTH_WEST, tile, tileSizeDeg, cache, epsilonLatLon);
        
        // Latitude/Longitude corner NE
        System.out.println("##### Coin de tuile au niveau latitude North et longitude Est #####");
        latDeg = getNorthernEdgeOfTile(tile);
        lonDeg = getEasternEdgeOfTile(tile);
        System.out.println(">>>> Search lat deg = " + latDeg + " lon deg= " + lonDeg + "\n");
        cornerZipperTile = cache.getTile(FastMath.toRadians(latDeg), FastMath.toRadians(lonDeg));
        printTileInfo(cornerZipperTile, rasterStepDeg, rasterStepDeg);

        checkGeodeticPointBelongsToZipper(FastMath.toRadians(latDeg), FastMath.toRadians(lonDeg), cornerZipperTile);
        checkCornerZipperTile(cornerZipperTile, Tile.Location.NORTH_EAST, tile, tileSizeDeg, cache, epsilonLatLon);

        // Latitude/Longitude corner SW
        System.out.println("##### Coin de tuile au niveau latitude South et longitude West #####");
        latDeg = getSouthernEdgeOfTile(tile);
        lonDeg = getWesternEdgeOfTile(tile);
        System.out.println(">>>> Search lat deg = " + latDeg + " lon deg= " + lonDeg + "\n");
        cornerZipperTile = cache.getTile(FastMath.toRadians(latDeg), FastMath.toRadians(lonDeg));
        printTileInfo(cornerZipperTile, rasterStepDeg, rasterStepDeg);

        checkGeodeticPointBelongsToZipper(FastMath.toRadians(latDeg), FastMath.toRadians(lonDeg), cornerZipperTile);
        checkCornerZipperTile(cornerZipperTile, Tile.Location.SOUTH_WEST, tile, tileSizeDeg, cache, epsilonLatLon);

        // Latitude/Longitude corner SE
        System.out.println("##### Coin de tuile au niveau latitude South et longitude Est #####");
        latDeg = getSouthernEdgeOfTile(tile);
        lonDeg = getEasternEdgeOfTile(tile);
        System.out.println(">>>> Search lat deg = " + latDeg + " lon deg= " + lonDeg + "\n");
        cornerZipperTile = cache.getTile(FastMath.toRadians(latDeg), FastMath.toRadians(lonDeg));
        printTileInfo(cornerZipperTile, rasterStepDeg, rasterStepDeg);

        checkGeodeticPointBelongsToZipper(FastMath.toRadians(latDeg), FastMath.toRadians(lonDeg), cornerZipperTile);
        checkCornerZipperTile(cornerZipperTile, Tile.Location.SOUTH_EAST, tile, tileSizeDeg, cache, epsilonLatLon);
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
            double tileSizeDeg, TilesCache<SimpleTile> cache, double epsilonLatLon) {
    
        SurroundingTiles surroundingTiles = new SurroundingTiles(originTile, tileSizeDeg, cache);

        if (location == Tile.Location.SOUTH_WEST) {

            SimpleTile tileBelow = surroundingTiles.getTileBelow();
            SimpleTile tileLeft = surroundingTiles.getTileLeft();
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

        } else if (location == Tile.Location.NORTH_WEST) {

            SimpleTile tileAbove = surroundingTiles.getTileAbove();
            SimpleTile tileLeft = surroundingTiles.getTileLeft();
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

        } else if (location == Tile.Location.SOUTH_EAST) {
            
            SimpleTile tileBelow = surroundingTiles.getTileBelow();
            SimpleTile tileRight = surroundingTiles.getTileRight();
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

        } else if (location == Tile.Location.NORTH_EAST) {

            SimpleTile tileAbove = surroundingTiles.getTileAbove();
            SimpleTile tileRight = surroundingTiles.getTileRight();
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
//        double leftAboveOfCurrentELevation = leftAboveOfCurrent.getElevationAtIndices(0, leftAboveOfCurrent.getLongitudeColumns() - 2);
//        assertEquals(leftAboveOfCurrentELevation, cornerZipperElevation, epsilonLatLon);

        cornerZipperElevation = cornerZipperTile.getElevationAtIndices(2, 1);
        aboveLeftELevation = aboveLeft.getElevationAtIndices(0, aboveLeft.getLongitudeColumns() - 1);
        assertEquals(aboveLeftELevation, cornerZipperElevation, epsilonLatLon);
//        leftAboveOfCurrentELevation = leftAboveOfCurrent.getElevationAtIndices(0, leftAboveOfCurrent.getLongitudeColumns() - 1);
//        assertEquals(leftAboveOfCurrentELevation, cornerZipperElevation, epsilonLatLon);

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
//        leftAboveOfCurrentELevation = leftAboveOfCurrent.getElevationAtIndices(1, leftAboveOfCurrent.getLongitudeColumns() - 2);
//        assertEquals(leftAboveOfCurrentELevation, cornerZipperElevation, epsilonLatLon);

        cornerZipperElevation = cornerZipperTile.getElevationAtIndices(3, 1);
        aboveLeftELevation = aboveLeft.getElevationAtIndices(1, aboveLeft.getLongitudeColumns() - 1);
        assertEquals(aboveLeftELevation, cornerZipperElevation, epsilonLatLon);
//        leftAboveOfCurrentELevation = leftAboveOfCurrent.getElevationAtIndices(1, leftAboveOfCurrent.getLongitudeColumns() - 1);
//        assertEquals(leftAboveOfCurrentELevation, cornerZipperElevation, epsilonLatLon);

        cornerZipperElevation = cornerZipperTile.getElevationAtIndices(3, 2);
        aboveRightElevation = aboveRight.getElevationAtIndices(1, 0);
        assertEquals(aboveRightElevation, cornerZipperElevation, epsilonLatLon);
         
        cornerZipperElevation = cornerZipperTile.getElevationAtIndices(3, 3);
        aboveRightElevation = aboveRight.getElevationAtIndices(1, 1);
        assertEquals(aboveRightElevation, cornerZipperElevation, epsilonLatLon);
    }

    	
    private void checkZipperTile(SimpleTile zipperTile, Tile.Location location, SimpleTile originTile,
            double tileSizeDeg, TilesCache<SimpleTile> cache, double epsilonLatLon) {

    	SurroundingTiles surroundingTiles = new SurroundingTiles(originTile, tileSizeDeg, cache);
    	
        if (location == Tile.Location.SOUTH) {
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
        } else if (location == Tile.Location.NORTH) {
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
        } else if (location == Tile.Location.WEST) {
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

        } else if (location == Tile.Location.EAST) {
        	SimpleTile tileRight = surroundingTiles.getTileRight();
            for (int iLat = 0; iLat < zipperTile.getLatitudeRows(); iLat++) {
                // row 0 of zipper 
                double zipperElevation = zipperTile.getElevationAtIndices(iLat, 0);
                double originELevation = originTile.getElevationAtIndices(iLat, originTile.getLongitudeColumns() - 2);
                assertEquals(originELevation, zipperElevation, epsilonLatLon);
   
                // row 1 of zipper 
                zipperElevation = zipperTile.getElevationAtIndices(iLat, 1);
                originELevation = originTile.getElevationAtIndices(iLat, originTile.getLongitudeColumns() - 1);
                assertEquals(originELevation, zipperElevation, epsilonLatLon);
                
                // row 2 of zipper 
                zipperElevation = zipperTile.getElevationAtIndices(iLat, 2);
                double rightELevation = tileRight.getElevationAtIndices(iLat, 0);
                assertEquals(rightELevation, zipperElevation, epsilonLatLon);

                // row 3 of zipper 
                zipperElevation = zipperTile.getElevationAtIndices(iLat, 3);
                rightELevation = tileRight.getElevationAtIndices(iLat, 1);
                assertEquals(rightELevation, zipperElevation, epsilonLatLon);
            }
        }

    }

    final static double getNorthernEdgeOfTile(SimpleTile tile) {
        double northernLatitudeForTile = getNorthernLatitudeForTile(tile);
        // Inside the northern row of latitude
        return northernLatitudeForTile - 0.1*FastMath.toDegrees(tile.getLatitudeStep());
    }
    
    final static double getSouthernEdgeOfTile(SimpleTile tile) {
      double southernLatitudeForTile = getSouthernLatitudeForTile(tile);
      // Inside the southern row of latitude
      return southernLatitudeForTile + 0.1*FastMath.toDegrees(tile.getLatitudeStep());
    }

    final static double getWesternEdgeOfTile(SimpleTile tile) {
        double westernLongitudeForTile = getWesternLongitudeForTile(tile);
        // Inside the western column of longitude
        return westernLongitudeForTile + 0.1*FastMath.toDegrees(tile.getLongitudeStep());
    }

    final static double getEasternEdgeOfTile(SimpleTile tile) {
        double easternLongitudeForTile = getEasternLongitudeForTile(tile);
        // Inside the eastern column of longitude
        return easternLongitudeForTile - 0.1*FastMath.toDegrees(tile.getLongitudeStep());
    }
    
    
    final static double getNorthernLatitudeForTile(SimpleTile tile) {
        return FastMath.toDegrees(tile.getMaximumLatitude()) + 0.5*FastMath.toDegrees(tile.getLatitudeStep());
    }
    
    final static double getSouthernLatitudeForTile(SimpleTile tile) {
        return FastMath.toDegrees(tile.getMinimumLatitude()) - 0.5*FastMath.toDegrees(tile.getLatitudeStep());
    }

    final static double getWesternLongitudeForTile(SimpleTile tile) {
        return FastMath.toDegrees(tile.getMinimumLongitude()) - 0.5*FastMath.toDegrees(tile.getLongitudeStep());
    }

    final static double getEasternLongitudeForTile(SimpleTile tile) {
        return FastMath.toDegrees(tile.getMaximumLongitude()) + 0.5*FastMath.toDegrees(tile.getLongitudeStep());
    }



    final static double epsilonElevation = 1.e-6;
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

    }

//    @Test
//    public void testDummyTileCreationOld() {
//    //        double[] latDeg = {48.53631399 , 50};
//    ////        double[] latDeg = {60., 59.99999, 59., 58., 57, 56,   
//    ////                           55, 54, 53,52, 51,  
//    ////                           50, 46, 
//    ////                           45, 42, 
//    ////                           40, 38, 36, 
//    ////                           35, 
//    ////                           -35, -36, -38, 
//    ////                           -49, 
//    ////                           -50, -51, -54, 
//    ////                           -55};
//    //        int[] expected =  {1,1,1,1,1,1, 
//    //                           2, 2,2,2,2,
//    //                           3,3,
//    //                           4,4,
//    //                           5,5,5,
//    //                           6,
//    //                           20, 20, 20,
//    //                           22,
//    //                           23, 23, 23,
//    //                           24};
//    //        for (int i= 0; i < latDeg.length; i++) {
//    //            int tileName = (int) (1 + (60. - latDeg[i])/5); 
//    ////          int subDirValue = (int) FastMath.ceil((125. - 2.*latDeg[i])/10.);
//    //
//    //            System.out.println(" lat =  " + latDeg[i] +  " nb= " + tileName);
//    ////            assertTrue(tileName == expected[i]);
//    //        }
//    //        
//    //        double[] lonDeg =    {2.20809251, 5};
//    ////        double[] lonDeg =    {-180, -179, -176, -175, -174, -11, -10, -6, -5, -1,  0 , 4, 5,  20, 22, 26, 175, 177, 180};
//    ////        double[] expected =  {   1,    1,  1 ,   2,    2,    34,   35, 35, 36, 36, 37, 37, 38, 41, 41, 42, 72, 72, 73};
//    //        for (int i= 0; i < lonDeg.length; i++) {
//    //            int tileName = (int) (1 + (lonDeg[i] + 180.)/5); 
//    ////            int parentDirValue = (int) (FastMath.ceil(2.*lonDeg[i] + 365.)/10.);
//    //
//    //            System.out.println(" lon =  " + lonDeg[i] +  " nb= " + tileName);
//    ////            assertTrue(tileName == parentDirValue);
//    ////            assertTrue(tileName == expected[i]);
//    //        }
//    
//            CountingFactory factory = new CountingFactory();
//            TilesCache<SimpleTile> cache = new TilesCache<SimpleTile>(factory,
//                                               new DummySRTMelevationUpdater(10.0, 20.0),
//                                               3, false);
//    
//            // Centre of tile 36/06
//            //        Origin = (-5.000000000000000,35.000000000000000)
//            //        Pixel Size = (0.000833333333333,-0.000833333333333)
//            //        Upper Left  (  -5.0000000,  35.0000000) (  5d 0' 0.00"W, 35d 0' 0.00"N)
//            //        Lower Left  (  -5.0000000,  30.0000000) (  5d 0' 0.00"W, 30d 0' 0.00"N)
//            //        Upper Right (   0.0000000,  35.0000000) (  0d 0' 0.01"E, 35d 0' 0.00"N)
//            //        Lower Right (   0.0000000,  30.0000000) (  0d 0' 0.01"E, 30d 0' 0.00"N)
//    
//            final double rasterStepDeg = 1./1200.;
//            
//            double latDeg = 37.1;
//            double lonDeg = 7.2;
//            System.out.println("Search lat deg = " + latDeg + " lon deg= " + lonDeg);
//            SimpleTile tile = cache.getTile(FastMath.toRadians(latDeg), FastMath.toRadians(lonDeg));
//            printTileInfo(tile, rasterStepDeg, rasterStepDeg);
//            
//    //        latDeg = 39.5;
//    //        lonDeg = 7.5;
//    //        System.out.println("Search lat deg = " + latDeg + " lon deg= " + lonDeg);
//    //        tile = cache.getTile(FastMath.toRadians(latDeg), FastMath.toRadians(lonDeg));
//    //        printTileInfo(tile);
//            
//    //        System.out.println("!!!! Changement de tuile au niveau longitude Est");
//    //        latDeg = 37.5;
//    //        lonDeg = 10. + 0.1*rasterStepDeg;
//    //        System.out.println("Search lat deg = " + latDeg + " lon deg= " + lonDeg);
//    //        tile = cache.getTile(FastMath.toRadians(latDeg), FastMath.toRadians(lonDeg));
//    //        printTileInfo(tile);
//    
//    
//            
//    //        System.out.println("!!!! Bord de tuile au niveau latitude Sud");
//    //        latDeg = 35.  + 0.9*rasterStepDeg;
//    //        lonDeg = 7.5;
//    //        System.out.println("Search lat deg = " + latDeg + " lon deg= " + lonDeg);
//    //        tile = cache.getTile(FastMath.toRadians(latDeg), FastMath.toRadians(lonDeg));
//    //        printTileInfo(tile);
//    
//    //        System.out.println("!!!! Changement de tuile au niveau latitude Sud");
//    //        latDeg = 35. - 0.1*rasterStepDeg;
//    ////        System.out.println(" calcul brut = " + (((latDeg - 30.) / rasterStepDeg)) +
//    ////                " floorInt " + ((int) FastMath.floor((latDeg - 30.) / rasterStepDeg)));
//    ////        System.out.println(" calcul = " + ((int) FastMath.floor((latDeg - 30.) / rasterStepDeg)));
//    //        lonDeg = 7.5;
//    //        System.out.println("Search lat deg = " + latDeg + " lon deg= " + lonDeg);
//    //        tile = cache.getTile(FastMath.toRadians(latDeg), FastMath.toRadians(lonDeg));
//    //        printTileInfo(tile);
//    
//            System.out.println("!!!! Bord de tuile au niveau latitude Nord");
//            latDeg = 40. - 0.1*rasterStepDeg;
//            lonDeg = 7.5;
//            System.out.println("Search lat deg = " + latDeg + " lon deg= " + lonDeg);
//            tile = cache.getTile(FastMath.toRadians(latDeg), FastMath.toRadians(lonDeg));
//            printTileInfo(tile);
//    
//    //        System.out.println("!!!! Changement de tuile au niveau latitude Nord");
//    //        latDeg = 40. + 0.1*rasterStepDeg;
//    //        lonDeg = 7.5;
//    //        System.out.println("Search lat deg = " + latDeg + " lon deg= " + lonDeg);
//    //        tile = cache.getTile(FastMath.toRadians(latDeg), FastMath.toRadians(lonDeg));
//    //        printTileInfo(tile);
//    
//    
//        }
        
}

class SurroundingTiles {

	private SimpleTile tileAbove;
	private SimpleTile tileBelow;
	private SimpleTile tileLeft;
	private SimpleTile tileRight;
	
	SurroundingTiles(SimpleTile originTile, double tileSizeDeg, TilesCache<SimpleTile> cache) {

		int latRows = originTile.getLatitudeRows();
		int loncols = originTile.getLongitudeColumns();
		double latDeg = FastMath.toDegrees(originTile.getLatitudeAtIndex(latRows/2));
		double lonDeg = FastMath.toDegrees(originTile.getLongitudeAtIndex(loncols/2));

		// get tile above ...
		double latAbove =  latDeg + tileSizeDeg;
		this.tileAbove = cache.getTile(FastMath.toRadians(latAbove), FastMath.toRadians(lonDeg));

		// get tile below
		double latBelow = latDeg - tileSizeDeg;
		this.tileBelow = cache.getTile(FastMath.toRadians(latBelow), FastMath.toRadians(lonDeg));

		// get tile left
		double lonLeft = lonDeg - tileSizeDeg;
		this.tileLeft = cache.getTile(FastMath.toRadians(latDeg), FastMath.toRadians(lonLeft));

		// get tile right
		double lonRight = lonDeg + tileSizeDeg;
		this.tileRight = cache.getTile(FastMath.toRadians(latDeg), FastMath.toRadians(lonRight));

	}

	public SimpleTile getTileAbove() {
		return tileAbove;
	}

	public SimpleTile getTileBelow() {
		return tileBelow;
	}

	public SimpleTile getTileLeft() {
		return tileLeft;
	}

	public SimpleTile getTileRight() {
		return tileRight;
	}
	
	

}
