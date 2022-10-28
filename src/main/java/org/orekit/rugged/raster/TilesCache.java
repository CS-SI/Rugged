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

import org.hipparchus.util.FastMath;
import java.lang.reflect.Array;
import java.text.DecimalFormat;
import java.text.DecimalFormatSymbols;
import java.util.Locale;

import org.orekit.bodies.GeodeticPoint;
import org.orekit.rugged.errors.DumpManager;
import org.orekit.rugged.errors.RuggedException;
import org.orekit.rugged.errors.RuggedMessages;

/** Cache for Digital Elevation Model {@link Tile tiles}.
 * <p>
 * Beware, this cache is <em>not</em> thread-safe!
 * </p>
 * @param <T> Type of tiles.
 * @author Luc Maisonobe
 * @author Guylaine Prat
 */
public class TilesCache<T extends Tile> {

    /** Factory for empty tiles. */
    private final TileFactory<T> factory;

    /** Updater for retrieving tiles data. */
    private final TileUpdater updater;
    
    /**Flag to tell if the Digital Elevation Model tiles are overlapping. 
     * @since X.x */
    private final boolean isOvelapping;

    /** Cache. */
    private final T[] tiles;

    /** Simple constructor.
     * @param factory factory for creating empty tiles
     * @param updater updater for retrieving tiles data
     * @param maxTiles maximum number of tiles stored simultaneously in the cache
     * @param isOvelappingTiles flag to tell if the DEM tiles are overlapping: 
     *                          true if overlapping; false otherwise. 
     */
    public TilesCache(final TileFactory<T> factory, final TileUpdater updater, 
                      final int maxTiles, final boolean isOverlappingTiles) {
        this.factory       = factory;
        this.updater    = updater;
        this.isOvelapping = isOverlappingTiles;
        @SuppressWarnings("unchecked")
        final T[] array = (T[]) Array.newInstance(Tile.class, maxTiles);
        this.tiles = array;
    }

    /** Get the tile covering a ground point.
     * @param latitude ground point latitude (rad)
     * @param longitude ground point longitude (rad)
     * @return tile covering the ground point
     */
    public T getTile(final double latitude, final double longitude) {

        // Search the current (latitude, longitude) in the tiles from the cache
        for (int i = 0; i < tiles.length; ++i) {
            final T tile = tiles[i];
            if (tile != null && tile.getLocation(latitude, longitude) == Tile.Location.HAS_INTERPOLATION_NEIGHBORS) {
                // we have found the tile in the cache

                // put it on the front as it becomes the most recently used
                while (i > 0) {
                    tiles[i] = tiles[i - 1];
                    --i;
                }
                tiles[0] = tile;
                return tile;
            }
        }

        // None of the tiles in the cache covers the specified point

        // Make some room in the cache, possibly evicting the least recently used one
        // in order to add the new tile
        for (int i = tiles.length - 1; i > 0; --i) {
            tiles[i] = tiles[i - 1];
        }

        // Fully create a tile given a latitude and longitude
        final T tile = createTile(latitude, longitude);

        // At this stage the found tile must be checked (HAS_INTERPOLATION_NEIGHBORS ?)
        // taking into account if the DEM tiles are overlapping or not

//        // TODO AFAC #############################
//        tilePrint(tile, "Current tile:");
//        final DecimalFormatSymbols symbols = new DecimalFormatSymbols(Locale.US);
//        final DecimalFormat dfs = new DecimalFormat("#.#####",symbols);
//        System.out.format("Geodetic point: latitude= " + dfs.format(FastMath.toDegrees(latitude)) + " longitude= " + dfs.format(FastMath.toDegrees(longitude)) + "\n" +
//                          "                location= " + tile.getLocation(latitude, longitude) + "\n\n");
//
//        // #############################

        if ( !isOvelapping ) { // DEM with seamless tiles (no overlapping)
            
            // Check if the tile HAS INTERPOLATION NEIGHBORS (the point (latitude, longitude) is inside the tile),
            // otherwise the point (latitude, longitude) is on the edge of the tile: 
            // one must create a zipper tile because tiles are not overlapping ...
            final Tile.Location pointLocation = tile.getLocation(latitude, longitude);

            // TODO GP check if useful: 
            final Tile.Location neighborhood = tile.checkNeighborhood(latitude, longitude);

            // We are on the edge of the tile 
            if (pointLocation != Tile.Location.HAS_INTERPOLATION_NEIGHBORS) {

                // Create a "zipper tile"
                return createZipperTile(tile, latitude, longitude, pointLocation);

            } else { // we are not on the edge of the tile
                
                tiles[0] = tile;
                return tile;
                
            }   // end if (location != Tile.Location.HAS_INTERPOLATION_NEIGHBORS) 

        } else { // isOvelapping: DEM with overlapping tiles (according to the flag ...)
            
            // Check if the tile HAS INTERPOLATION NEIGHBORS (the (latitude, longitude) is inside the tile)
            if (tile.getLocation(latitude, longitude) != Tile.Location.HAS_INTERPOLATION_NEIGHBORS) {
                // this should happen only if user set up an inconsistent TileUpdater
                throw new RuggedException(RuggedMessages.TILE_WITHOUT_REQUIRED_NEIGHBORS_SELECTED,
                                          FastMath.toDegrees(latitude), FastMath.toDegrees(longitude));
            }
            tiles[0] = tile;
            return tile;
        }
    }

    /** Create a tile defines by its latitude and longitude
     * @param latitude latitude of the desired tile (rad)
     * @param longitude longitude of the desired tile (rad)
     * @return the tile
     * @since X.x
     */
    private T createTile(final double latitude, final double longitude) {
        
        // Create the tile according to the current (latitude, longitude) and retrieve its data
        final T tile = factory.createTile();

        // In case dump is asked for, suspend the dump manager as we don't need to dump anything here
        // For instance for SRTM DEM, the user needs to read Geoid data that are not useful in the dump
        final Boolean wasSuspended = DumpManager.suspend();

        // Retrieve the tile data
        updater.updateTile(latitude, longitude, tile);

        // Resume the dump manager if necessary
        DumpManager.resume(wasSuspended);

        // Last step to fully create the tile (in order to create the MinMax kd tree)
        tile.tileUpdateCompleted();
        return tile;
    }


    /** Create a zipper tile for DEM with seamless tiles (no overlapping)
     * @param currentTile current tile
     * @param latitude ground point latitude (rad)
     * @param longitude ground point longitude (rad)
     * @param pointLocation ground point location with respect to the tile
     * @return zipper tile covering the ground point
     * @since X.x
     */
    private T createZipperTile(final T currentTile, 
                               final double latitude, final double longitude, 
                               final Tile.Location pointLocation) {

        T zipperTile = null;
        
        // One must create a zipper tile between this tile and the neighbor tile 
        // according to the ground point location
        switch (pointLocation) {

        case NORTH: // latitudeIndex > latitudeRows - 2

            zipperTile = createZipperNorthOrSouth(EarthHemisphere.NORTH, longitude, currentTile);
            break;
            
        case SOUTH: // latitudeIndex < 0

            zipperTile = createZipperNorthOrSouth(EarthHemisphere.SOUTH, longitude, currentTile);
            break;

        case WEST: // longitudeIndex < 0

            zipperTile = createZipperWestOrEast(EarthHemisphere.WEST, latitude, currentTile);
            break;
            
        case EAST: // longitudeIndex > longitudeColumns - 2

            zipperTile = createZipperWestOrEast(EarthHemisphere.EAST, latitude, currentTile);
            break;
            
        case NORTH_WEST: 
            
            zipperTile = createCornerZipperNorthWest(latitude, longitude, currentTile);
            break;
            
        case NORTH_EAST: 
            
            zipperTile = createCornerZipperNorthEast(latitude, longitude, currentTile);
            break;
            
        case SOUTH_WEST: 
            
            zipperTile = createCornerZipperSouthWest(latitude, longitude, currentTile);
            break;
            
        case SOUTH_EAST: 
            
            zipperTile = createCornerZipperSouthEast(latitude, longitude, currentTile);
            break;
            
      default:
            // impossible to reach
            throw new RuggedException(RuggedMessages.INTERNAL_ERROR);

        } // end switch
        
        // again make some room in the cache, possibly evicting the 2nd least recently used one
        for (int i = tiles.length - 1; i > 0; --i) {
            tiles[i] = tiles[i - 1];
        }
        tiles[1] = currentTile;
        tiles[0] = zipperTile;
        return (T) zipperTile;
    }

    /** Initialize the zipper tile for a given geometry and the full set of elevations 
     * @param zipperLatitudeMin
     * @param zipperLongitudeMin
     * @param zipperLatitudeStep
     * @param zipperLongitudeStep
     * @param zipperLatitudeRows
     * @param zipperLongitudeColumns
     * @param zipperElevations
     * @return the zipper tile
     * @since X.x
     */
    private T initializeZipperTile(final double zipperLatitudeMin, final double zipperLongitudeMin, 
                                   final double zipperLatitudeStep, final double zipperLongitudeStep, 
                                   final int zipperLatitudeRows, final int zipperLongitudeColumns,
                                   final double[][] zipperElevations) {
        
        // Create an empty tile
        final T zipperTile = factory.createTile();

        // Set the tile geometry
        zipperTile.setGeometry(zipperLatitudeMin, zipperLongitudeMin, zipperLatitudeStep, zipperLongitudeStep, 
                               zipperLatitudeRows, zipperLongitudeColumns);
        
        // Fill in the tile with the relevant elevations
        for (int iLat = 0; iLat < zipperLatitudeRows; iLat++) {
            for (int jLon = 0; jLon < zipperLongitudeColumns; jLon++) {
                zipperTile.setElevation(iLat, jLon, zipperElevations[iLat][jLon]);
            }
        }
        
        // Last step in order to create the MinMax kd tree
        zipperTile.tileUpdateCompleted();

        return zipperTile;
    }
    
    /** Create the zipper tile between the current tile and the Northern or Southern tile of the current tile
     * for a given longitude.
     * The zipper have 4 rows in latitude.
     * @param latitudeHemisphere hemisphere for latitude: NORTH / SOUTH
     * @param longitude given longitude (rad)
     * @param tile current tile
     * @return northern or southern zipper tile of the current tile according to latitudeHemisphere
     * @since X.x
     */
    private T createZipperNorthOrSouth(final EarthHemisphere latitudeHemisphere, final double longitude, final T tile) {
                
        final int latitudeRows = tile.getLatitudeRows();
        final int longitudeCols = tile.getLongitudeColumns();
        final double latStep = tile.getLatitudeStep();
        final double lonStep = tile.getLongitudeStep();
        final double minLat = tile.getMinimumLatitude();
         
        // Get the North or South tile
        final T tileNorthOrSouth = createNorthOrSouthTile(latitudeHemisphere, longitude, minLat, latitudeRows, latStep);
        
        // TODO AFAC ################
        switch (latitudeHemisphere) {
        case NORTH:
            tilePrint(tileNorthOrSouth, "North tile:");
            break;
        case SOUTH:
            tilePrint(tileNorthOrSouth, "South tile:");
            break;          
        default:
            // impossible to reach
            throw new RuggedException(RuggedMessages.INTERNAL_ERROR);
        }
        
        // If NORTH hemisphere:
        // Create zipper tile between the current tile and the North tile;
        // 2 rows belong to the North part of the origin/current tile
        // 2 rows belong to the South part of the North tile

        // If SOUTH hemisphere:
        // Create zipper tile between the current tile and the South tile;
        // 2 rows belong to the South part of the origin/current tile
        // 2 rows belong to the North part of the South tile

        // TODO #########################
        // TODO we suppose here the 2 tiles have same origin, step and size along longitude
        // TODO #########################

        double zipperLatStep = latStep;
        int zipperLatRows = 4;

        int zipperLonCols = longitudeCols;

        double zipperLatMin;
        double[][] elevations;
        
        switch (latitudeHemisphere) {
        case NORTH:
            zipperLatMin = minLat + (latitudeRows - 2)*latStep;
            
            elevations = getZipperNorthSouthElevations(zipperLatRows, zipperLonCols, 
                                                       tileNorthOrSouth, tile, latitudeRows);
            break;
            
        case SOUTH:
            zipperLatMin = minLat - 2*latStep;
            
            elevations= getZipperNorthSouthElevations(zipperLatRows, zipperLonCols, 
                                                      tile, tileNorthOrSouth, latitudeRows);
            break;
            
        default:
            // impossible to reach
            throw new RuggedException(RuggedMessages.INTERNAL_ERROR);
        }

        final T zipperNorthOrSouth = initializeZipperTile(zipperLatMin, tile.getMinimumLongitude(), 
                                                          zipperLatStep, lonStep, 
                                                          zipperLatRows, zipperLonCols, elevations);
        
        // TODO AFAC ################
        switch (latitudeHemisphere) {
        case NORTH:
            tilePrint(zipperNorthOrSouth, "NORTH Zipper tile:");
            break;
        case SOUTH:
            tilePrint(zipperNorthOrSouth, "SOUTH Zipper tile:");
            break;          
        default:
            // impossible to reach
            throw new RuggedException(RuggedMessages.INTERNAL_ERROR);
        }
        
        return zipperNorthOrSouth;
    }

    /** Create the zipper tile between the current tile and the Western or Eastern tile of the current tile
     * for a given latitude.
     * The zipper have 4 columns in longitude.
     * @param longitudeHemisphere hemisphere for longitude: WEST / EAST
     * @param latitude given latitude (rad)
     * @param tile current tile
     * @return western or eastern zipper tile of the current tile according to longitudeHemisphere
     * @since X.x
     */
    private T createZipperWestOrEast(final EarthHemisphere longitudeHemisphere, double latitude, final T tile) {
        
        final int latitudeRows = tile.getLatitudeRows();
        final int longitudeCols = tile.getLongitudeColumns();
        final double latStep = tile.getLatitudeStep();
        final double lonStep = tile.getLongitudeStep();
        final double minLon = tile.getMinimumLongitude();
        
        // Get the West or East Tile
        final T tileWestOrEast = createEastOrWestTile(longitudeHemisphere, latitude, minLon, longitudeCols, lonStep);

        // TODO AFAC ################
        switch (longitudeHemisphere) {
        case WEST:
            tilePrint(tileWestOrEast, "West tile:");
            break;
        case EAST:
            tilePrint(tileWestOrEast, "East tile:");
            break;          
        default:
            // impossible to reach
            throw new RuggedException(RuggedMessages.INTERNAL_ERROR);
        }

        // If WEST hemisphere
        // Create zipper tile between the current tile and the West tile;
        // 2 cols belong to the West part of the origin/current tile
        // 2 cols belong to the East part of the West tile
        
        // If EAST hemisphere
        // Create zipper tile between the current tile and the East tile;
        // 2 cols belong to the East part of the origin/current tile
        // 2 cols belong to the West part of the East tile

        // TODO we suppose here the 2 tiles have same origin, step and size along longitude

        double zipperLonStep = lonStep;
        int zipperLatRows = latitudeRows;
        int zipperLonCols = 4;

        double zipperLonMin;
        double[][] elevations;
        
        switch (longitudeHemisphere) {
        case WEST:
            zipperLonMin = minLon - 2*lonStep;
            elevations = getZipperEastWestElevations(zipperLatRows, zipperLonCols, 
                                                     tile, tileWestOrEast, longitudeCols);
            break;
            
        case EAST:
            zipperLonMin = minLon + (longitudeCols - 2)*lonStep;

            elevations = getZipperEastWestElevations(zipperLatRows, zipperLonCols, 
                                                     tileWestOrEast, tile, longitudeCols);
            break;
            
        default:
            // impossible to reach
            throw new RuggedException(RuggedMessages.INTERNAL_ERROR);
        }
        
        
        final T zipperWestOrEast = initializeZipperTile(tile.getMinimumLatitude(), zipperLonMin, 
                                                     latStep, zipperLonStep, 
                                                     zipperLatRows, zipperLonCols, elevations);
        
        // TODO AFAC ################
        switch (longitudeHemisphere) {
        case WEST:
            tilePrint(zipperWestOrEast, "WEST Zipper tile:");
            break;
        case EAST:
            tilePrint(zipperWestOrEast, "EAST Zipper tile:");
            break;          
        default:
            // impossible to reach
            throw new RuggedException(RuggedMessages.INTERNAL_ERROR);
        }

        return zipperWestOrEast;
    }

    /** Get the elevations for the zipper tile between a northern and a southern tiles
      * @param zipperLatRows number of rows in latitude
      * @param zipperLonCols number of column in longitude
      * @param northernTile the tile which is the northern
      * @param southernTile the tile which is the southern
      * @param southernLatRows latitude rows of the southern tile
      * @return the elevations to fill in the zipper tile between a northern and a southern tiles
      * @since X.x
      */
     private double[][] getZipperNorthSouthElevations(int zipperLatRows, int zipperLonCols, 
                                                      final T northernTile, final T southernTile,
                                                      final int southernLatRows) {

         double[][] elevations = new double[zipperLatRows][zipperLonCols];

         for (int jLon = 0; jLon < zipperLonCols; jLon++) {
             // Part from the northern tile
             int lat3 = 1;
             elevations[3][jLon] = northernTile.getElevationAtIndices(lat3, jLon);
             int lat2 = 0;
             elevations[2][jLon] = northernTile.getElevationAtIndices(lat2, jLon);

             // Part from the southern tile
             int lat1 = southernLatRows - 1;
             elevations[1][jLon] = southernTile.getElevationAtIndices(lat1, jLon);
             int lat0 = southernLatRows - 2;
             elevations[0][jLon] = southernTile.getElevationAtIndices(lat0, jLon);
         }
         return elevations;
     }

    /** Get the elevations for the zipper tile between a eastern and a western tiles
     * @param zipperLatRows number of rows in latitude
     * @param zipperLonCols number of column in longitude
     * @param easternTile the tile which is the eastern
     * @param westernTile the tile which is the western
     * @param westernLonCols longitude columns of the western tile
     * @return the elevations to fill in the zipper tile between a eastern and a western tiles
     * @since X.x
     */
     private double[][] getZipperEastWestElevations(int zipperLatRows, int zipperLonCols, 
                                                    final T easternTile, final T westernTile,
                                                    final int westernLonCols) {
         
         double[][] elevations = new double[zipperLatRows][zipperLonCols];

         for (int iLat = 0; iLat < zipperLatRows; iLat++) {
             // Part from the eastern tile
             int lon3 = 1;
             elevations[iLat][3] = easternTile.getElevationAtIndices(iLat, lon3);
             int lon2 = 0;
             elevations[iLat][2] = easternTile.getElevationAtIndices(iLat, lon2);

             // Part from the western tile
             int lon1 = westernLonCols - 1;
             elevations[iLat][1] = westernTile.getElevationAtIndices(iLat, lon1);
             int lon0 = westernLonCols - 2;
             elevations[iLat][0] = westernTile.getElevationAtIndices(iLat, lon0);
         }
         return elevations;
     }

    /** Create a corner zipper tile (with 4 rows in latitude and 4 columns in longitude)
     * @param zipperCorner zipper tile point at minimum latitude and minimum longitude
     * @param zipperLatStep latitude step (rad)
     * @param zipperLonStep longitude step (rad)
     * @param currentTile current tile
     * @param belowLeftTile below left tile
     * @param aboveLeftTile above left tile
     * @param belowRightTile below right tile
     * @param aboveRightTile above right tile
     * @return corner zipper tile
     * @since X.x
     */
    private T createCornerZipper(final GeodeticPoint zipperCorner,
                                 final double zipperLatStep, final double zipperLonStep,
                                 final T currentTile,
                                 final T belowLeftTile, final T aboveLeftTile, final T belowRightTile, final T aboveRightTile) {
        
        // Defines with 4 cells from each of the 4 tiles at the corner
        final int zipperLatRows = 4;
        final int zipperLonCols = 4;
        double[][] elevations = new double[zipperLatRows][zipperLonCols];
        
        // rows 0 and 1 of zipper: 
        // 2 first cells belong to the below left tile and 2 last cells to the below right tile
        
        // row 0
        elevations[0][0] = belowLeftTile.getElevationAtIndices(belowLeftTile.getLatitudeRows() - 2, belowLeftTile.getLongitudeColumns() - 2);
        elevations[0][1] = belowLeftTile.getElevationAtIndices(belowLeftTile.getLatitudeRows() - 2, belowLeftTile.getLongitudeColumns() - 1);
        
        elevations[0][2] = belowRightTile.getElevationAtIndices(belowRightTile.getLatitudeRows() - 2, 0);
        elevations[0][3] = belowRightTile.getElevationAtIndices(belowRightTile.getLatitudeRows() - 2, 1);
        
        // row 1
        elevations[1][0] = belowLeftTile.getElevationAtIndices(belowLeftTile.getLatitudeRows() - 1, belowLeftTile.getLongitudeColumns() - 2);
        elevations[1][1] = belowLeftTile.getElevationAtIndices(belowLeftTile.getLatitudeRows() - 1, belowLeftTile.getLongitudeColumns() - 1);
        
        elevations[1][2] = belowRightTile.getElevationAtIndices(belowRightTile.getLatitudeRows() - 1, 0);
        elevations[1][3] = belowRightTile.getElevationAtIndices(belowRightTile.getLatitudeRows() - 1, 1);
        
        // rows 2 and 3 of zipper: 
        // 2 first cells belong to the above left tile and 2 last cells to the above right tile
        
        // row 2
        elevations[2][0] = aboveLeftTile.getElevationAtIndices(0, aboveLeftTile.getLongitudeColumns() - 2);
        elevations[2][1] = aboveLeftTile.getElevationAtIndices(0, aboveLeftTile.getLongitudeColumns() - 1);

        elevations[2][2] = aboveRightTile.getElevationAtIndices(0, 0);
        elevations[2][3] = aboveRightTile.getElevationAtIndices(0, 1);

        // row 3
        elevations[3][0] = aboveLeftTile.getElevationAtIndices(1, aboveLeftTile.getLongitudeColumns() - 2);
        elevations[3][1] = aboveLeftTile.getElevationAtIndices(1, aboveLeftTile.getLongitudeColumns() - 1);

        elevations[3][2] = aboveRightTile.getElevationAtIndices(1, 0);
        elevations[3][3] = aboveRightTile.getElevationAtIndices(1, 1);

        final T cornerZipperTile = initializeZipperTile(zipperCorner.getLatitude(), zipperCorner.getLongitude(), 
                                                        zipperLatStep, zipperLonStep, 
                                                        zipperLatRows, zipperLonCols, elevations);
        // TODO AFAC ################
        tilePrint(cornerZipperTile, "Corner Zipper tile:");
//            printTileFile(cornerZipperTile, "Plots/Zipper_lat_");
        // ################
        
        return cornerZipperTile;
    }
    
    /** Create the North West corner zipper tile 
     * @param latitude ground point latitude (rad)
     * @param longitude ground point longitude (rad)
     * @param currentTile current tile
     * @return the North West corner zipper tile 
     * @since X.x
    */
    private T createCornerZipperNorthWest(final double latitude, final double longitude, final T currentTile) {

        final int latitudeRows = currentTile.getLatitudeRows();
        final int longitudeCols = currentTile.getLongitudeColumns();
        final double latStep = currentTile.getLatitudeStep();
        final double lonStep = currentTile.getLongitudeStep();
        final double minLon = currentTile.getMinimumLongitude();
        final double minLat = currentTile.getMinimumLatitude();

        // Get the West Tile
        final T tileWest = createEastOrWestTile(EarthHemisphere.WEST, latitude, minLon, longitudeCols, lonStep);
        // Get the North Tile
        final T tileNorth = createNorthOrSouthTile(EarthHemisphere.NORTH, longitude, minLat, latitudeRows, latStep);
        // Get the North-West Tile
        final T tileNorthWest = createIntercardinalTile(EarthHemisphere.NORTH, 
                                                        minLat, latitudeRows, latStep,
                                                        EarthHemisphere.WEST,
                                                        minLon, longitudeCols, lonStep);
        T belowLeftTile = tileWest;
        T belowRightTile = currentTile;
        T aboveLeftTile = tileNorthWest;
        T aboveRightTile = tileNorth;
        
        GeodeticPoint zipperCorner = computeZipperOrigin(EarthHemisphere.NORTH, 
                                                         minLat, latitudeRows, latStep,
                                                         EarthHemisphere.WEST,
                                                         minLon, longitudeCols, lonStep);
        
        // TODO we suppose here the 2 tiles have same origin, step and size along latitude and longitude
        final double zipperLatStep = latStep;
        final double zipperLonStep = lonStep;

        // Create zipper tile at the corner North-West of the tile
        return createCornerZipper(zipperCorner, zipperLatStep, zipperLonStep,
                                  currentTile, belowLeftTile, aboveLeftTile, belowRightTile, aboveRightTile);

    }
   
    /** Create the North East corner zipper tile 
     * @param latitude ground point latitude (rad)
     * @param longitude ground point longitude (rad)
     * @param currentTile current tile
     * @return the North East corner zipper tile 
     * @since X.x
     */
    private T createCornerZipperNorthEast(final double latitude, final double longitude, final T currentTile) {
       
       final int latitudeRows = currentTile.getLatitudeRows();
       final int longitudeCols = currentTile.getLongitudeColumns();
       final double latStep = currentTile.getLatitudeStep();
       final double lonStep = currentTile.getLongitudeStep();
       final double minLon = currentTile.getMinimumLongitude();
       final double minLat = currentTile.getMinimumLatitude();

       // Get the East Tile
       final T tileEast = createEastOrWestTile(EarthHemisphere.EAST, latitude, minLon, longitudeCols, lonStep);
       // Get the North Tile
       final T tileNorth = createNorthOrSouthTile(EarthHemisphere.NORTH, longitude, minLat, latitudeRows, latStep);
       // Get the North-East Tile
       final T tileNorthEast = createIntercardinalTile(EarthHemisphere.NORTH, 
                                                       minLat, latitudeRows, latStep,
                                                       EarthHemisphere.EAST,
                                                       minLon, longitudeCols, lonStep);
       T belowLeftTile = currentTile;
       T belowRightTile = tileEast;
       T aboveLeftTile = tileNorth;
       T aboveRightTile = tileNorthEast;
       
       GeodeticPoint zipperCorner = computeZipperOrigin(EarthHemisphere.NORTH, 
                                                        minLat, latitudeRows, latStep,
                                                        EarthHemisphere.EAST,
                                                        minLon, longitudeCols, lonStep);
               
       // TODO we suppose here the 2 tiles have same origin, step and size along latitude and longitude
       final double zipperLatStep = latStep;
       final double zipperLonStep = lonStep;

       // Create zipper tile at the corner North-East of the tile
       return createCornerZipper(zipperCorner, zipperLatStep, zipperLonStep,
                                 currentTile, belowLeftTile, aboveLeftTile, belowRightTile, aboveRightTile);
   }
   
    /** Create the South West corner zipper tile 
     * @param latitude ground point latitude (rad)
     * @param longitude ground point longitude (rad)
     * @param currentTile current tile
     * @return the South West corner zipper tile 
     * @since X.x
     */
    private T createCornerZipperSouthWest(final double latitude, final double longitude, final T currentTile) {
       
       final int latitudeRows = currentTile.getLatitudeRows();
       final int longitudeCols = currentTile.getLongitudeColumns();
       final double latStep = currentTile.getLatitudeStep();
       final double lonStep = currentTile.getLongitudeStep();
       final double minLon = currentTile.getMinimumLongitude();
       final double minLat = currentTile.getMinimumLatitude();

       // Get the West Tile
       final T tileWest = createEastOrWestTile(EarthHemisphere.WEST, latitude, minLon, longitudeCols, lonStep);
       // Get the South Tile
       final T tileSouth = createNorthOrSouthTile(EarthHemisphere.SOUTH, longitude, minLat, latitudeRows, latStep);
       // Get the South-West Tile
       final T tileSouthWest = createIntercardinalTile(EarthHemisphere.SOUTH,
                                                       minLat, latitudeRows, latStep,
                                                       EarthHemisphere.WEST,
                                                       minLon, longitudeCols, lonStep);
       T belowLeftTile  = tileSouthWest;
       T belowRightTile = tileSouth;
       T aboveLeftTile  = tileWest;
       T aboveRightTile = currentTile;
       
       GeodeticPoint zipperCorner = computeZipperOrigin(EarthHemisphere.SOUTH,
                                                        minLat, latitudeRows, latStep,
                                                        EarthHemisphere.WEST,
                                                        minLon, longitudeCols, lonStep);
       
       // TODO we suppose here the 2 tiles have same origin, step and size along latitude and longitude
       final double zipperLatStep = latStep;
       final double zipperLonStep = lonStep;

       // Create zipper tile at the corner North-West of the tile
       return createCornerZipper(zipperCorner, zipperLatStep, zipperLonStep,
                                 currentTile, belowLeftTile, aboveLeftTile, belowRightTile, aboveRightTile);
   }

    /** Create the South East corner zipper tile 
     * @param latitude ground point latitude (rad)
     * @param longitude ground point longitude (rad)
     * @param currentTile current tile
     * @return the South East corner zipper tile 
     * @since X.x
     */
    private T createCornerZipperSouthEast(final double latitude, final double longitude, final T currentTile) {
       
       final int latitudeRows = currentTile.getLatitudeRows();
       final int longitudeCols = currentTile.getLongitudeColumns();
       final double latStep = currentTile.getLatitudeStep();
       final double lonStep = currentTile.getLongitudeStep();
       final double minLon = currentTile.getMinimumLongitude();
       final double minLat = currentTile.getMinimumLatitude();

       // Get the East Tile
       final T tileEast = createEastOrWestTile(EarthHemisphere.EAST, latitude, minLon, longitudeCols, lonStep);
       // Get the South Tile
       final T tileSouth = createNorthOrSouthTile(EarthHemisphere.SOUTH, longitude, minLat, latitudeRows, latStep);
       // Get the South-East Tile
       final T tileSouthEast = createIntercardinalTile(EarthHemisphere.SOUTH,
                                                       minLat, latitudeRows, latStep,
                                                       EarthHemisphere.EAST,
                                                       minLon, longitudeCols, lonStep);
       T belowLeftTile  = tileSouth;
       T belowRightTile = tileSouthEast;
       T aboveLeftTile  = currentTile;
       T aboveRightTile = tileEast;
       
       GeodeticPoint zipperCorner = computeZipperOrigin(EarthHemisphere.SOUTH,
                                                        minLat, latitudeRows, latStep,
                                                        EarthHemisphere.EAST,
                                                        minLon, longitudeCols, lonStep);
       
       // TODO we suppose here the 2 tiles have same origin, step and size along latitude and longitude
       final double zipperLatStep = latStep;
       final double zipperLonStep = lonStep;

       // Create zipper tile at the corner North-West of the tile
       return createCornerZipper(zipperCorner, zipperLatStep, zipperLonStep,
                                 currentTile, belowLeftTile, aboveLeftTile, belowRightTile, aboveRightTile);
   }
  
    /**
     * Create the tile in intercardinal direction of the current Tile i.e. NW, NE, SW, SE
     * @param latitudeHemisphere hemisphere for latitude: NORTH / SOUTH
     * @param minLat latitude minimum for the tile (rad)
     * @param latitudeRows latitude rows
     * @param latStep latitude step (rad)
     * @param longitudeHemisphere hemisphere for longitude : WEST / EAST
     * @param minLon longitude minimum for the tile (rad)
     * @param longitudeCols longitude columns
     * @param lonStep longitude step (rad)
     * @return the tile in intercardinal direction
     * @since X.x
     */
    private T createIntercardinalTile(final EarthHemisphere latitudeHemisphere, 
                                      final double minLat, final int latitudeRows, final double latStep,
                                      final EarthHemisphere longitudeHemisphere,
                                      final double minLon, final int longitudeCols, final double lonStep) {

      // lonHemisphere = +1 : East or = -1 : West
      int lonHemisphere;
      switch (longitudeHemisphere) {
      case EAST:
          lonHemisphere = +1;
          break;
      case WEST:
          lonHemisphere = -1;
          break;          
      default:
          // impossible to reach
          throw new RuggedException(RuggedMessages.INTERNAL_ERROR);
      }

      // latHemisphere = +1 : North or = -1 : South
      int latHemisphere;
      switch (latitudeHemisphere) {
      case NORTH:
          latHemisphere = +1;
          break;
      case SOUTH:
          latHemisphere = -1;
          break;          
      default:
          // impossible to reach
          throw new RuggedException(RuggedMessages.INTERNAL_ERROR);
      }

      final double latToGetIntercardinalTile = minLat + latHemisphere*latitudeRows*latStep;
      final double lonToGetIntercardinalTile = minLon + lonHemisphere*longitudeCols*lonStep;
      final T intercardinalTile = createTile(latToGetIntercardinalTile, lonToGetIntercardinalTile);
      return intercardinalTile;
  }

  
    /** Compute the zipper tile origin
     * @param latitudeHemisphere latitude hemisphere
     * @param minLat zipper tile latitude origin (rad)
     * @param latitudeRows latitude rows
     * @param latStep latitude step (rad)
     * @param longitudeHemisphere longitude hemisphere
     * @param minLon zipper tile longitude origin (rad)
     * @param longitudeCols longitude columns
     * @param lonStep longitude step (rad)
     * @return zipper tile origin point
     * @since X.x
     */
    private GeodeticPoint computeZipperOrigin(final EarthHemisphere latitudeHemisphere, 
                                              final double minLat, final int latitudeRows, final double latStep,
                                              final EarthHemisphere longitudeHemisphere,
                                              final double minLon, final int longitudeCols, final double lonStep) {
      
      double addLon;
      switch (longitudeHemisphere) {
      case EAST:
          addLon = longitudeCols*lonStep;
          break;
      case WEST:
          addLon = 0;
          break;          
      default:
          // impossible to reach
          throw new RuggedException(RuggedMessages.INTERNAL_ERROR);
      }

      double addLat;
      switch (latitudeHemisphere) {
      case NORTH:
          addLat = latitudeRows*latStep;
          break;
      case SOUTH:
          addLat = 0;
          break;          
      default:
          // impossible to reach
          throw new RuggedException(RuggedMessages.INTERNAL_ERROR);
      }
      
      final double zipperLatMin = minLat - 2*latStep + addLat;
      final double zipperLonMin = minLon - 2*lonStep + addLon;

      return new GeodeticPoint(zipperLatMin, zipperLonMin, 0);
  }
  
    /** Create the Northern or Southern tile of a given tile defined by minLat, longitude, latitudeRows and latStep
     * @param latitudeHemisphere latitude hemisphere
     * @param longitude longitude to define the tile (rad)
     * @param minLat minimum latitude to define the tile (rad)
     * @param latitudeRows latitude rows
     * @param latStep latitude step (rad)
     * @return North or South tile according to the Earth hemisphere
     * @since X.x
     */
    private T createNorthOrSouthTile(final EarthHemisphere latitudeHemisphere, final double longitude, 
                                     final double minLat, final int latitudeRows, final double latStep) {
        // hemisphere = +1 : North or = -1 : South
        int hemisphere;
        switch (latitudeHemisphere) {
        case NORTH:
            hemisphere = +1;
            break;
        case SOUTH:
            hemisphere = -1;
            break;          
        default:
            // impossible to reach
            throw new RuggedException(RuggedMessages.INTERNAL_ERROR);
        }
        
        final double latToGetNewTile = minLat + hemisphere*latitudeRows*latStep;
        return createTile(latToGetNewTile, longitude);
    }
    
    
    /** Create the Eastern or Western tile of a given tile defined by latitude, minLon, longitudeCols and lonStep
     * @param longitudeHemisphere longitude hemisphere
     * @param latitude latitude to define the tile (rad)
     * @param minLon minimum longitude to define the tile (rad)
     * @param longitudeCols longitude columns
     * @param lonStep longitude step (rad)
     * @return East or West tile  tile according to the Earth hemisphere
     * @since X.x
     */
    private T createEastOrWestTile(final EarthHemisphere longitudeHemisphere, final double latitude, 
                                   final double minLon, final int longitudeCols, final double lonStep) {
        // hemisphere = +1 : East or = -1 : West
        int hemisphere;
        switch (longitudeHemisphere) {
        case EAST:
            hemisphere = +1;
            break;
        case WEST:
            hemisphere = -1;
            break;          
        default:
            // impossible to reach
            throw new RuggedException(RuggedMessages.INTERNAL_ERROR);
        }
        
        final double lonToGetNewTile = minLon + hemisphere*longitudeCols*lonStep;
        return createTile(latitude, lonToGetNewTile);
    }
 
    // TODO GP AFAC
    protected void tilePrint(final Tile tile, String comment) {
        
        final DecimalFormatSymbols symbols = new DecimalFormatSymbols(Locale.US);
        final DecimalFormat dfs = new DecimalFormat("#.####",symbols);
//        final DecimalFormat dfs2 = new DecimalFormat("#.#####",symbols);

        System.out.format(comment + "\n" +
                "                   rows " + tile.getLatitudeRows() + " col " + tile.getLongitudeColumns() + 
//                " lat step " + dfs2.format(FastMath.toDegrees(tile.getLatitudeStep())) + 
//                " long step " + dfs2.format(FastMath.toDegrees(tile.getLongitudeStep())) + 
                "\n" +  
//                "              lat min " + dfs.format(FastMath.toDegrees(tile.getMinimumLatitude())) + 
//                      " lat max " + dfs.format(FastMath.toDegrees(tile.getMaximumLatitude())) +
                " (SRTM convention)" +
                " lat min " + dfs.format(FastMath.toDegrees(tile.getMinimumLatitude()-0.5*tile.getLatitudeStep())) + 
                " lat max " + dfs.format(FastMath.toDegrees(tile.getMaximumLatitude()+0.5*tile.getLatitudeStep())) +
                "\n" +  
//                "              lon min " + dfs.format(FastMath.toDegrees(tile.getMinimumLongitude())) + " lon max " + dfs.format(FastMath.toDegrees(tile.getMaximumLongitude())) +
//                " SRTM convention :" +
                "                   lon min " + dfs.format(FastMath.toDegrees(tile.getMinimumLongitude()-0.5*tile.getLongitudeStep())) +
                " lon max " + dfs.format(FastMath.toDegrees(tile.getMaximumLongitude()+0.5*tile.getLongitudeStep())) +
//                "\n" +  
//                "       min elevation " + tile.getMinElevation() + " lat index " + tile.getMinElevationLatitudeIndex() + " long index " + tile.getMinElevationLongitudeIndex() +
//                "\n" +
//                "       max elevation " + tile.getMaxElevation() +  " lat index " + tile.getMaxElevationLatitudeIndex() + " long index " + tile.getMaxElevationLongitudeIndex() +
                "\n");
    }

    // TODO GP AFAC
    private void printTileFile(final Tile tile, String prefix) {   
        String tileFileName = prefix + (int) FastMath.toDegrees(tile.getMinimumLatitude()) + 
                "_lon_" + (int) FastMath.toDegrees(tile.getMinimumLongitude()) + ".txt";
        java.io.PrintWriter tilePrinter;
        try {
            tilePrinter = new java.io.PrintWriter(tileFileName, "UTF-8");

            for (int iLat = 0; iLat < tile.getLatitudeRows(); ++iLat) {
                for (int jLon = 0; jLon < tile.getLongitudeColumns(); ++jLon) {
                    tilePrinter.format(Locale.US, "%d %d %3.5f %3.5f %3.5f%n",
                            jLon, iLat, FastMath.toDegrees(tile.getLongitudeAtIndex(jLon)), FastMath.toDegrees(tile.getLatitudeAtIndex(iLat)), 
                            tile.getElevationAtIndices(iLat, jLon));
                    //                inverseLocPrinter.format(Locale.US, "%3.8f %3.8f %.4f %.2f %.2f %3.5f %3.5f%n", 
                    //                        FastMath.toDegrees(lokilokGP.getLatitude()),
                    //                        FastMath.toDegrees(lokilokGP.getLongitude()),
                    //                        lokilokGP.getAltitude(),
                    //                        ruggedInverseLoc.getPixelNumber(), ruggedInverseLoc.getLineNumber(), deltaPixel, deltaLine);
                }
            }

            tilePrinter.close();
        } catch (java.io.FileNotFoundException | java.io.UnsupportedEncodingException e) {
            e.printStackTrace();
        }
        System.out.println("########### File  created : " + tileFileName + " ##############");
    }
}
