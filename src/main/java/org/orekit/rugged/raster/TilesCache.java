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

import java.lang.reflect.Array;

import org.hipparchus.util.FastMath;
import org.hipparchus.util.MathUtils;
import org.hipparchus.util.Precision;
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

        if ( !isOvelapping ) { // DEM with seamless tiles (no overlapping)
            
            // Check if the tile HAS INTERPOLATION NEIGHBORS (the point (latitude, longitude) is inside the tile),
            // otherwise the point (latitude, longitude) is on the edge of the tile: 
            // one must create a zipper tile because tiles are not overlapping ...
            final Tile.Location pointLocation = tile.getLocation(latitude, longitude);

            // We are on the edge of the tile 
            if (pointLocation != Tile.Location.HAS_INTERPOLATION_NEIGHBORS) {

                // Create a "zipper tile"
                return createZipperTile(tile, latitude, longitude, pointLocation);

            } else { // we are NOT on the edge of the tile
                
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
            
        } // end if (!isOvelapping)
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

        // In case dump is asked for, suspend the dump manager as we don't need to dump anything here.
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

        case NORTH: // pointLocation such as latitudeIndex > latitudeRows - 2

            zipperTile = createZipperNorthOrSouth(EarthHemisphere.NORTH, longitude, currentTile);
            break;
            
        case SOUTH: // pointLocation such as latitudeIndex < 0

            zipperTile = createZipperNorthOrSouth(EarthHemisphere.SOUTH, longitude, currentTile);
            break;

        case WEST: // pointLocation such as longitudeIndex < 0

            zipperTile = createZipperWestOrEast(EarthHemisphere.WEST, latitude, currentTile);
            break;
            
        case EAST: // pointLocation such as longitudeIndex > longitudeColumns - 2

            zipperTile = createZipperWestOrEast(EarthHemisphere.EAST, latitude, currentTile);
            break;
            
        // One must create a corner zipper tile between this tile and the 3 surrounding tiles 
        // according to the ground point location
        case NORTH_WEST: 
            
            zipperTile = createCornerZipper(EarthHemisphere.NORTH, EarthHemisphere.WEST, latitude, longitude, currentTile);
            break;
            
        case NORTH_EAST: 
            
            zipperTile = createCornerZipper(EarthHemisphere.NORTH, EarthHemisphere.EAST, latitude, longitude, currentTile);
            break;
            
        case SOUTH_WEST: 
            
            zipperTile = createCornerZipper(EarthHemisphere.SOUTH, EarthHemisphere.WEST, latitude, longitude, currentTile);
            break;
            
        case SOUTH_EAST: 
            
            zipperTile = createCornerZipper(EarthHemisphere.SOUTH, EarthHemisphere.EAST, latitude, longitude, currentTile);
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
     * for a given longitude. The Northern or Southern tiles of the current tile may have a change in 
     * resolution either in longitude or in latitude wrt the current tile.
     * The zipper has 4 rows in latitude. 
     * @param latitudeHemisphere hemisphere for latitude: NORTH / SOUTH
     * @param longitude given longitude (rad)
     * @param currentTile current tile
     * @return Northern or Southern zipper tile of the current tile (according to latitudeHemisphere)
     * @since X.x
     */
    private T createZipperNorthOrSouth(final EarthHemisphere latitudeHemisphere, final double longitude, final T currentTile) {
                
        final int currentTileLatRows = currentTile.getLatitudeRows();
        final int currentTileLonCols = currentTile.getLongitudeColumns();
        final double currentTileLatStep = currentTile.getLatitudeStep();
        final double currentTileLonStep = currentTile.getLongitudeStep();
        final double currentTileMinLat = currentTile.getMinimumLatitude();
        final double currentTileMinLon = currentTile.getMinimumLongitude();
         
        // Get the Northern or Southern tile
        final T tileNorthOrSouth = createNorthOrSouthTile(latitudeHemisphere, longitude, currentTileMinLat, currentTileLatRows, currentTileLatStep);

        // If NORTH hemisphere:
        // Create zipper tile between the current tile and the Northern tile;
        // 2 rows belong to the North part of the current tile
        // 2 rows belong to the South part of the Northern tile

        // If SOUTH hemisphere:
        // Create zipper tile between the current tile and the Southern tile;
        // 2 rows belong to the South part of the current tile
        // 2 rows belong to the North part of the Southern tile

        // Initialize the zipper latitude step and number of rows (4)
        int zipperLatRows = 4;
        double zipperLatStep = currentTileLatStep;
        
        // Check if latitude steps are the same between the current tile and the Northern or Southern tile
        boolean isSameStepLat = true;
        double northSouthLatitudeStep = tileNorthOrSouth.getLatitudeStep();
        
        if (! (Math.abs(currentTileLatStep - northSouthLatitudeStep) < 5.*Precision.EPSILON)) {
            // Steps are not the same
            isSameStepLat = false;
            // Recompute zipper latitude step if the North or South tile latitude step is smaller
            if (northSouthLatitudeStep < currentTileLatStep) {
                zipperLatStep = northSouthLatitudeStep;
            }
        }
        
        // Initialize the zipper longitude step and number of columns with current tile step and columns
        double zipperLonStep = currentTileLonStep;
        int zipperLonCols = currentTileLonCols;
        
        // Check if longitude steps are the same
        boolean isSameStepLon = true;
        double northSouthLongitudeStep = tileNorthOrSouth.getLongitudeStep();
        
        if (! (Math.abs(currentTileLonStep - northSouthLongitudeStep) < 5.*Precision.EPSILON)) {
            // Steps are not the same
            isSameStepLon = false;
            // Recompute zipper longitude step and columns if the North or South tile longitude step is smaller
             if (northSouthLongitudeStep < currentTileLonStep) {
                 zipperLonStep = northSouthLongitudeStep;
                 zipperLonCols = tileNorthOrSouth.getLongitudeColumns();
             }
        }

        double zipperLatMin;
        double zipperLonMin;
        double[][] elevations;
        
        switch (latitudeHemisphere) {
        case NORTH:
            // Defines the zipper min latitude wrt the current tile max latitude minus 2 zipper step in latitude 
            // Explanation: we want the 2 first rows belongs to the current tile and 2 last rows to the Northern tile
            //    If zipperLatStep = latStep, the 2 first rows = exactly lines of the current tile
            //    otherwise (latStep > North tile step), the 2 first rows will be smaller (in latitude) than the 2 lines of the current tile
            zipperLatMin = currentTileMinLat + currentTileLatRows * currentTileLatStep - 2*zipperLatStep;
            
            // Define the zipper min longitude = current tile min longitude
            zipperLonMin = currentTileMinLon;
            
            // Fill in the zipper elevations 
            elevations = getZipperNorthSouthElevations(zipperLonCols, tileNorthOrSouth, currentTile, 
                                                       isSameStepLat, isSameStepLon, 
                                                       zipperLatMin, zipperLatStep, zipperLonMin, zipperLonStep);
            break;
            
        case SOUTH:
            // Defines the zipper min latitude wrt the current tile min latitude
            // Explanation: we want the 2 last rows belongs to the current tile and 2 first rows to the Southern tile
            //    If zipperLatStep = latStep, the 2 last rows = exactly lines of the current tile
            //    otherwise (latStep > South tile step), the 2 last rows will be smaller (in latitude) than the 2 lines of the current tile
            zipperLatMin = currentTileMinLat - 2*zipperLatStep;
            
            // Define the zipper min longitude = current tile min longitude
            zipperLonMin = currentTileMinLon;

            // Fill in the zipper elevations 
            elevations= getZipperNorthSouthElevations(zipperLonCols, currentTile, tileNorthOrSouth, 
                                                      isSameStepLat, isSameStepLon, 
                                                      zipperLatMin, zipperLatStep, zipperLonMin, zipperLonStep);
            break;
            
        default:
            // impossible to reach
            throw new RuggedException(RuggedMessages.INTERNAL_ERROR);
        }

        final T zipperNorthOrSouth = initializeZipperTile(zipperLatMin, zipperLonMin, 
                                                          zipperLatStep, zipperLonStep, 
                                                          zipperLatRows, zipperLonCols, elevations);
        return zipperNorthOrSouth;
    }

    /** Create the zipper tile between the current tile and the Western or Eastern tile of the current tile
     * for a given latitude. The Western or Eastern tiles of the current tile have the same 
     * resolution in longitude and in latitude as the current tile (for known DEMs).
     * The zipper has 4 columns in longitude.
     * @param longitudeHemisphere hemisphere for longitude: WEST / EAST
     * @param latitude given latitude (rad)
     * @param currentTile current tile
     * @return Western or Eastern zipper tile of the current tile (according to longitudeHemisphere)
     * @since X.x
     */
    private T createZipperWestOrEast(final EarthHemisphere longitudeHemisphere, final double latitude, final T currentTile) {
        
        final int currentTileLatRows = currentTile.getLatitudeRows();
        final int currentTileLonCols = currentTile.getLongitudeColumns();
        final double currentTileLatStep = currentTile.getLatitudeStep();
        final double currentTileLonStep = currentTile.getLongitudeStep();
        final double currentTileMinLon = currentTile.getMinimumLongitude();
        
        // Get the West or East Tile
        final T tileWestOrEast = createEastOrWestTile(longitudeHemisphere, latitude, currentTileMinLon, currentTileLonCols, currentTileLonStep);

        if (! (Math.abs(currentTileLatStep - tileWestOrEast.getLatitudeStep()) < 5.*Precision.EPSILON) ||
            ! (Math.abs(currentTileLonStep - tileWestOrEast.getLongitudeStep()) < 5.*Precision.EPSILON)) {        
            // Steps are not the same.
            // Along Western or Eastern edges of the tiles: no change of resolution may occurs in known DEMs.
            throw new RuggedException(RuggedMessages.INTERNAL_ERROR);
        }

        // If WEST hemisphere
        // Create zipper tile between the current tile and the Western tile;
        // 2 cols belong to the West part of the current tile
        // 2 cols belong to the East part of the West tile
        
        // If EAST hemisphere
        // Create zipper tile between the current tile and the Eastern tile;
        // 2 cols belong to the East part of the current tile
        // 2 cols belong to the West part of the East tile

        double zipperLonStep = currentTileLonStep;
        int zipperLatRows = currentTileLatRows;
        int zipperLonCols = 4;

        double zipperLonMin;
        double[][] elevations;
        
        switch (longitudeHemisphere) {
        case WEST:
            zipperLonMin = currentTileMinLon - 2*currentTileLonStep;
            elevations = getZipperEastWestElevations(zipperLatRows, currentTile, tileWestOrEast);
            break;
            
        case EAST:
            zipperLonMin = currentTileMinLon + (currentTileLonCols - 2)*currentTileLonStep;

            elevations = getZipperEastWestElevations(zipperLatRows, tileWestOrEast, currentTile);
            break;
            
        default:
            // impossible to reach
            throw new RuggedException(RuggedMessages.INTERNAL_ERROR);
        }

        final T zipperWestOrEast = initializeZipperTile(currentTile.getMinimumLatitude(), zipperLonMin, 
                                                        currentTileLatStep, zipperLonStep, 
                                                        zipperLatRows, zipperLonCols, elevations);
        return zipperWestOrEast;
    }
    
    /** Get the latitude index of a point.
     * @param latitude geodetic latitude (rad)
     * @param minLatitude min latitude of the tile (rad)
     * @param latitudeStep latitude step of the tile (rad)
     * @param latitudeRows number of rows in latitude
     * @return latitude index (it may lie outside of the tile!)
     * @since X.x
     */
    private int computeLatitudeIndex(final double latitude, final double minLatitude, final double latitudeStep, final int latitudeRows) {
        double doubleLatitudeIndex =  (latitude  - minLatitude)  / latitudeStep;
        return FastMath.max(0, FastMath.min(latitudeRows - 1, (int) FastMath.floor(doubleLatitudeIndex)));
    }
    
    /** Get the longitude index of a point.
     * @param longitude geodetic longitude (rad)
     * @param minLongitude min longitude of the tile (rad)
     * @param longitudeStep longitude step of the tile (rad)
     * @param longitudeColumns number of columns in longitude
     * @return longitude index (it may lie outside of the tile!)
     * @since X.x
     */
    private int computeLongitudeIndex(final double longitude, final double minLongitude, final double longitudeStep, final int longitudeColumns) {
        double doubleLongitudeIndex = (longitude - minLongitude) / longitudeStep;
        return FastMath.max(0, FastMath.min(longitudeColumns - 1, (int) FastMath.floor(doubleLongitudeIndex)));
    }
    
    /** Get the elevations for the zipper tile between a northern and a southern tiles with 4 rows in latitude.
     * @param zipperLonCols number of column in longitude
     * @param northernTile the tile which is the northern
     * @param southernTile the tile which is the southern
     * @param isSameStepLat flag to tell is latitude steps are the same (= true)
     * @param isSameStepLon flag to tell is longitude steps are the same (= true)
     * @param zipperLatMin zipper tile min latitude (rad)
     * @param zipperLatStep zipper tile latitude step (rad)
     * @param zipperLonMin zipper tile min longitude (rad)
     * @param zipperLonStep zipper tile longitude step (rad)
     * @return the elevations to fill in the zipper tile between a northern and a southern tiles
     * @since X.x
     */
    private double[][] getZipperNorthSouthElevations(final int zipperLonCols, 
                                                     final T northernTile, final T southernTile,
                                                     final boolean isSameStepLat, final boolean isSameStepLon,
                                                     final double zipperLatMin, final double zipperLatStep, 
                                                     final double zipperLonMin, final double zipperLonStep) {
        
         double[][] elevations = new double[4][zipperLonCols];
         
         if (isSameStepLat && isSameStepLon) { // tiles with same steps in latitude and longitude
             
             for (int jLon = 0; jLon < zipperLonCols; jLon++) {
                 // Part from the northern tile
                 int lat3 = 1;
                 elevations[3][jLon] = northernTile.getElevationAtIndices(lat3, jLon);
                 int lat2 = 0;
                 elevations[2][jLon] = northernTile.getElevationAtIndices(lat2, jLon);

                 // Part from the southern tile
                 int lat1 = southernTile.getLatitudeRows() - 1;
                 elevations[1][jLon] = southernTile.getElevationAtIndices(lat1, jLon);
                 int lat0 = southernTile.getLatitudeRows() - 2;
                 elevations[0][jLon] = southernTile.getElevationAtIndices(lat0, jLon);
             }
             
         } else { // tiles with different steps
             
             // To cover every cases and as zipper with such characteristics are rare, 
             // we will use conversion from (latitude, longitude) to tile indices
             
             // We assure to be inside a cell by adding a delta step (to avoid inappropriate index computation)
             // TBN: zipperLatStep/zipperLonStep are the smallest steps vs Northern and Southern tiles
             double deltaLon = 0.1*zipperLonStep;
             double zipperLonCurrent = zipperLonMin + deltaLon;
             
             // Assure that the longitude belongs to [-180, + 180]
             double zipperLonMax = MathUtils.normalizeAngle(zipperLonMin + (zipperLonCols - 1) * zipperLonStep, 0.0);
             
             // Northern Tile 
             double northernMinLat = northernTile.getMinimumLatitude();
             double northernLatStep = northernTile.getLatitudeStep();
             int northernLatRows = northernTile.getLatitudeRows();
             double northernMinLon = northernTile.getMinimumLongitude();
             double northernLonStep = northernTile.getLongitudeStep();
             int northernLonCols = northernTile.getLongitudeColumns();

             // Southern Tile 
             double southernMinLat = southernTile.getMinimumLatitude();
             double southernLatStep = southernTile.getLatitudeStep();
             int southernLatRows = southernTile.getLatitudeRows();
             double southernMinLon = southernTile.getMinimumLongitude();
             double southernLonStep = southernTile.getLongitudeStep();
             int southernLonCols = southernTile.getLongitudeColumns();

             while (zipperLonCurrent <= zipperLonMax + 2*deltaLon) {
                     
                     // Compute zipper tile longitude index
                     int zipperLonIndex = computeLongitudeIndex(zipperLonCurrent, zipperLonMin, zipperLonStep, zipperLonCols);

                     // Part from the northern tile
                     // ---------------------------
                     // Compute northern longitude
                     int northenLongitudeIndex = computeLongitudeIndex(zipperLonCurrent, northernMinLon, northernLonStep, northernLonCols);

                     double zipperLat3 = zipperLatMin + (3 + 0.1)*zipperLatStep;
                     // lat3 would be 1 if Northern latitude step smallest; could be 0 if biggest
                     int lat3Index = computeLatitudeIndex(zipperLat3, northernMinLat, northernLatStep, northernLatRows);
                     elevations[3][zipperLonIndex] = northernTile.getElevationAtIndices(lat3Index, northenLongitudeIndex);
                     
                     // lat2 is 0 whatever Northern latitude step
                     int lat2Index = 0;
                     elevations[2][zipperLonIndex] = northernTile.getElevationAtIndices(lat2Index, northenLongitudeIndex);

                     // Part from the southern tile
                     // ---------------------------
                     // Compute southern longitude
                     int southernLongitudeIndex = computeLongitudeIndex(zipperLonCurrent, southernMinLon, southernLonStep, southernLonCols);

                     // lat1 is Southern latitude rows - 1  whatever Southern latitude step
                     int lat1Index = southernTile.getLatitudeRows() - 1;
                     elevations[1][zipperLonIndex] = southernTile.getElevationAtIndices(lat1Index, southernLongitudeIndex);
                     
                     double zipperLat0 = zipperLatMin + 0.1*zipperLatStep;
                     // lat1 would be Southern latitude rows - 2 if Southern latitude step smallest; could be Southern latitude rows - 1 if biggest
                     int lat0Index = computeLatitudeIndex(zipperLat0, southernMinLat, southernLatStep, southernLatRows);
                     elevations[0][zipperLonIndex] = southernTile.getElevationAtIndices(lat0Index, southernLongitudeIndex);

                     // Next longitude
                     // --------------
                     zipperLonCurrent += zipperLonStep;

             } // end loop on zipperLonCurrent
         }
         return elevations;
     }

    /** Get the elevations for the zipper tile between a eastern and a western tiles with 4 columns in longitude.
     * @param zipperLatRows number of rows in latitude
     * @param easternTile the tile which is the eastern
     * @param westernTile the tile which is the western
     * @return the elevations to fill in the zipper tile between a eastern and a western tiles
     * @since X.x
     */
     private double[][] getZipperEastWestElevations(final int zipperLatRows,
                                                    final T easternTile, final T westernTile) {
         
         double[][] elevations = new double[zipperLatRows][4];

         for (int iLat = 0; iLat < zipperLatRows; iLat++) {
             // Part from the eastern tile
             int lon3 = 1;
             elevations[iLat][3] = easternTile.getElevationAtIndices(iLat, lon3);
             int lon2 = 0;
             elevations[iLat][2] = easternTile.getElevationAtIndices(iLat, lon2);

             // Part from the western tile
             int lon1 = westernTile.getLongitudeColumns() - 1;
             elevations[iLat][1] = westernTile.getElevationAtIndices(iLat, lon1);
             int lon0 = westernTile.getLongitudeColumns() - 2;
             elevations[iLat][0] = westernTile.getElevationAtIndices(iLat, lon0);
         }
         return elevations;
     }
     
     /** Create the corner zipper tile.
      * Hypothesis: along Western or Eastern edges of the tiles: no change of resolution may occurs in known DEMs.
      * @param latitudeHemisphere latitude hemisphere (North or South)
      * @param longitudeHemisphere longitude hemisphere (West or East)
      * @param latitude ground point latitude (rad)
      * @param longitude ground point longitude (rad)
      * @param currentTile current tile
      * @return the corner zipper tile 
      * @since X.x
     */
     private T createCornerZipper(final EarthHemisphere latitudeHemisphere, final EarthHemisphere longitudeHemisphere, 
                                  final double latitude, final double longitude, final T currentTile) {

         final int currentTileLatRows = currentTile.getLatitudeRows();
         final int currentTileLonCols = currentTile.getLongitudeColumns();
         final double currentTileLatStep = currentTile.getLatitudeStep();
         final double currentTileLonStep = currentTile.getLongitudeStep();
         final double currentTileMinLon = currentTile.getMinimumLongitude();
         final double currentTileMinLat = currentTile.getMinimumLatitude();

         T belowLeftTile;
         T belowRightTile;
         T aboveLeftTile;
         T aboveRightTile;
         
         switch (latitudeHemisphere) {
         case NORTH:

             switch (longitudeHemisphere) {
             case WEST:

                 // Get the West Tile
                 T tileWest = createEastOrWestTile(EarthHemisphere.WEST, latitude, currentTileMinLon, currentTileLonCols, currentTileLonStep);
                 // Get the North Tile
                 T tileNorth = createNorthOrSouthTile(EarthHemisphere.NORTH, longitude, currentTileMinLat, currentTileLatRows, currentTileLatStep);
                 // Get the North-West Tile
                 T tileNorthWest = createIntercardinalTile(EarthHemisphere.NORTH, currentTileMinLat, currentTileLatRows, currentTileLatStep,
                                                           EarthHemisphere.WEST, currentTileMinLon, currentTileLonCols, currentTileLonStep);
                 belowLeftTile = tileWest;
                 belowRightTile = currentTile;
                 aboveLeftTile = tileNorthWest;
                 aboveRightTile = tileNorth;

                 break;

             case EAST:
                 
                 // Get the East Tile
                 T tileEast = createEastOrWestTile(EarthHemisphere.EAST, latitude, currentTileMinLon, currentTileLonCols, currentTileLonStep);
                 // Get the North Tile
                 tileNorth = createNorthOrSouthTile(EarthHemisphere.NORTH, longitude, currentTileMinLat, currentTileLatRows, currentTileLatStep);
                 // Get the North-East Tile
                 T tileNorthEast = createIntercardinalTile(EarthHemisphere.NORTH, currentTileMinLat, currentTileLatRows, currentTileLatStep,
                                                           EarthHemisphere.EAST, currentTileMinLon, currentTileLonCols, currentTileLonStep);
                 belowLeftTile = currentTile;
                 belowRightTile = tileEast;
                 aboveLeftTile = tileNorth;
                 aboveRightTile = tileNorthEast;

                 break;
                 
             default:
                 // impossible to reach
                 throw new RuggedException(RuggedMessages.INTERNAL_ERROR);
             } // end switch longitudeHemisphere

             break;

         case SOUTH:

             switch (longitudeHemisphere) {
             case WEST:    
                 
                 // Get the West Tile
                 T tileWest = createEastOrWestTile(EarthHemisphere.WEST, latitude, currentTileMinLon, currentTileLonCols, currentTileLonStep);
                 // Get the South Tile
                 T tileSouth = createNorthOrSouthTile(EarthHemisphere.SOUTH, longitude, currentTileMinLat, currentTileLatRows, currentTileLatStep);
                 // Get the South-West Tile
                 T tileSouthhWest = createIntercardinalTile(EarthHemisphere.SOUTH, currentTileMinLat, currentTileLatRows, currentTileLatStep,
                                                            EarthHemisphere.WEST, currentTileMinLon, currentTileLonCols, currentTileLonStep);
                 belowLeftTile = tileSouthhWest;
                 belowRightTile = tileSouth;
                 aboveLeftTile = tileWest;
                 aboveRightTile = currentTile;

                 break;

             case EAST:
                 
                 // Get the East Tile
                 T tileEast = createEastOrWestTile(EarthHemisphere.EAST, latitude, currentTileMinLon, currentTileLonCols, currentTileLonStep);
                 // Get the South Tile
                 tileSouth = createNorthOrSouthTile(EarthHemisphere.SOUTH, longitude, currentTileMinLat, currentTileLatRows, currentTileLatStep);
                 // Get the South-East Tile
                 T tileSouthhEast = createIntercardinalTile(EarthHemisphere.SOUTH, currentTileMinLat, currentTileLatRows, currentTileLatStep,
                                                            EarthHemisphere.EAST, currentTileMinLon, currentTileLonCols, currentTileLonStep);
                 belowLeftTile = tileSouth;
                 belowRightTile = tileSouthhEast;
                 aboveLeftTile = currentTile;
                 aboveRightTile = tileEast;

                 break;
                 
             default:
                 // case impossible to reach
                 throw new RuggedException(RuggedMessages.INTERNAL_ERROR);
             } // end switch longitudeHemisphere
             
             break;

         default:
             // case impossible to reach
             throw new RuggedException(RuggedMessages.INTERNAL_ERROR);
         } // end switch latitudeHemisphere

         
         // Check that tiles at same latitude have same steps in longitude and latitude
         // Along Western or Eastern edges of the tiles: no change of resolution may occurs in known DEMs.
         if (! (Math.abs(belowLeftTile.getLatitudeStep() - belowRightTile.getLatitudeStep()) < 5.*Precision.EPSILON) ||
             ! (Math.abs(belowLeftTile.getLongitudeStep() - belowRightTile.getLongitudeStep()) < 5.*Precision.EPSILON) ||
             ! (Math.abs(aboveLeftTile.getLatitudeStep() - aboveRightTile.getLatitudeStep()) < 5.*Precision.EPSILON) ||
             ! (Math.abs(aboveLeftTile.getLongitudeStep() - aboveRightTile.getLongitudeStep()) < 5.*Precision.EPSILON)) {        
                 // Steps are not the same.
                 throw new RuggedException(RuggedMessages.INTERNAL_ERROR);
             }
         
         
         // Compute the zipper steps in latitude and longitude.
         // If steps are different, the zipper will have the smallest steps.
         
         // at this stage the latitude steps of the right and left tiles are the same
         double belowLatitudeStep = belowRightTile.getLatitudeStep();
         double aboveLatitudeStep = aboveRightTile.getLatitudeStep();

         // initialize the zipper latitude step
         double zipperLatStep = belowLatitudeStep;
         
         // check if latitude steps are the same between the above and below tiles
         boolean isSameStepLat = true;
         
         if (! (Math.abs(belowLatitudeStep - aboveLatitudeStep) < 5.*Precision.EPSILON)) {

             // Latitude steps are not the same
             isSameStepLat = false;
             
             // Recompute zipper latitude step if the above tiles latitude step is smaller
             if (aboveLatitudeStep < belowLatitudeStep) {
                 zipperLatStep = aboveLatitudeStep;
             }
         }
         
         // at this stage the longitude steps of the right and left tiles are the same
         double belowLongitudeStep = belowRightTile.getLongitudeStep();
         double aboveLongitudeStep = aboveRightTile.getLongitudeStep();

         // initialize the zipper longitude step
         double zipperLonStep = belowLongitudeStep;
         
         // check if longitude steps are the same between the above and below tiles
         boolean isSameStepLon = true;
         
         if (! (Math.abs(belowLongitudeStep - aboveLongitudeStep) < 5.*Precision.EPSILON)) {
             
             // Longitude steps are not the same
             isSameStepLon = false;
             
             // Recompute zipper longitude step if the above tiles longitude step is smaller
              if (aboveLongitudeStep < belowLongitudeStep) {
                  zipperLonStep = aboveLongitudeStep;
              }
         }

         // Define the zipper min latitude and min longitude.
         // Explanation: 
         // We want the zipper 2 first rows belongs to the below tiles and the zipper 2 last rows to the above tiles.
         // We want the zipper 2 first columns belongs to the left tiles and the zipper 2 last columns to the right tiles.
         // We use the current tile origin AND the zipper steps to compute the zipper origin
         GeodeticPoint zipperCorner = computeCornerZipperOrigin(zipperLatStep, zipperLonStep,
                                                                latitudeHemisphere, currentTileMinLat, 
                                                                currentTileLatRows, currentTileLatStep,
                                                                longitudeHemisphere, currentTileMinLon, 
                                                                currentTileLonCols,  currentTileLonStep);

         // Initialize corner tile
         return initializeCornerZipperTile(zipperCorner.getLatitude(), zipperLatStep, zipperCorner.getLongitude(), zipperLonStep,
                                           belowLeftTile, aboveLeftTile, belowRightTile, aboveRightTile,
                                           isSameStepLat, isSameStepLon);

     }

    /** Initialize a corner zipper tile (with 4 rows in latitude and 4 columns in longitude)
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
    private T initializeCornerZipperTile(final double zipperLatMin, final double zipperLatStep, 
                                         final double zipperLonMin, final double zipperLonStep,
                                         final T belowLeftTile, final T aboveLeftTile, final T belowRightTile, final T aboveRightTile,
                                         final boolean isSameStepLat, final boolean isSameStepLon) {
        
        // Defines with 4 cells from each of the 4 tiles at the corner
        final int zipperLatRows = 4;
        final int zipperLonCols = 4;
        double[][] elevations = new double[zipperLatRows][zipperLonCols];
        
        if (isSameStepLat && isSameStepLon) { // tiles with same steps in latitude and longitude

            // Rows 0 and 1 of zipper: 
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

            // Rows 2 and 3 of zipper: 
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


        } else { // tiles with different steps

            // To cover every cases and as zipper with such characteristics are rare, 
            // we will use conversion from (latitude, longitude) to tile indices

            // We assure to be inside a cell by adding a delta step (to avoid inappropriate index computation)
            // TBN: zipperLatStep/zipperLonStep are the smallest steps vs below and above tiles

            // Compute latitude and longitude of each zipper cells
            double zipperLat0 = zipperLatMin + 0.1*zipperLatStep;
            double zipperLat1 = zipperLat0 + zipperLatStep;
            double zipperLat2 = zipperLat1 + zipperLatStep;
            double zipperLat3 = zipperLat2 + zipperLatStep;

            double zipperLon0 = zipperLonMin + 0.1*zipperLonStep;
            double zipperLon1 = zipperLon0 + zipperLonStep;
            double zipperLon2 = zipperLon1 + zipperLonStep;
            double zipperLon3 = zipperLon2 + zipperLonStep;

            // Compute the tiles index in latitude
            int belowLeftLatitudeIndex0 = computeLatitudeIndex(zipperLat0, belowLeftTile.getMinimumLatitude(), belowLeftTile.getLatitudeStep(), belowLeftTile.getLatitudeRows());
            int belowLeftLatitudeIndex1 = computeLatitudeIndex(zipperLat1, belowLeftTile.getMinimumLatitude(), belowLeftTile.getLatitudeStep(), belowLeftTile.getLatitudeRows());

            int belowRightLatitudeIndex0 = computeLatitudeIndex(zipperLat0, belowRightTile.getMinimumLatitude(), belowRightTile.getLatitudeStep(), belowRightTile.getLatitudeRows());
            int belowRightLatitudeIndex1 = computeLatitudeIndex(zipperLat1, belowRightTile.getMinimumLatitude(), belowRightTile.getLatitudeStep(), belowRightTile.getLatitudeRows());

            int aboveLeftLatitudeIndex2 = computeLatitudeIndex(zipperLat2, aboveLeftTile.getMinimumLatitude(), aboveLeftTile.getLatitudeStep(), aboveLeftTile.getLatitudeRows());
            int aboveLeftLatitudeIndex3 = computeLatitudeIndex(zipperLat3, aboveLeftTile.getMinimumLatitude(), aboveLeftTile.getLatitudeStep(), aboveLeftTile.getLatitudeRows());

            int aboveRightLatitudeIndex2 = computeLatitudeIndex(zipperLat2, aboveRightTile.getMinimumLatitude(), aboveRightTile.getLatitudeStep(), aboveRightTile.getLatitudeRows());
            int aboveRightLatitudeIndex3 = computeLatitudeIndex(zipperLat3, aboveRightTile.getMinimumLatitude(), aboveRightTile.getLatitudeStep(), aboveRightTile.getLatitudeRows());

            // Compute the tiles index in longitude
            int belowLeftLongitudeIndex0 = computeLongitudeIndex(zipperLon0, belowLeftTile.getMinimumLongitude(), belowLeftTile.getLongitudeStep(), belowLeftTile.getLongitudeColumns());
            int belowLeftLongitudeIndex1 = computeLongitudeIndex(zipperLon1, belowLeftTile.getMinimumLongitude(), belowLeftTile.getLongitudeStep(), belowLeftTile.getLongitudeColumns());

            int belowRightLongitudeIndex2 = computeLongitudeIndex(zipperLon2, belowRightTile.getMinimumLongitude(), belowRightTile.getLongitudeStep(), belowRightTile.getLongitudeColumns());
            int belowRightLongitudeIndex3 = computeLongitudeIndex(zipperLon3, belowRightTile.getMinimumLongitude(), belowRightTile.getLongitudeStep(), belowRightTile.getLongitudeColumns());

            int aboveLeftLongitudeIndex0 = computeLongitudeIndex(zipperLon0, aboveLeftTile.getMinimumLongitude(), aboveLeftTile.getLongitudeStep(), aboveLeftTile.getLongitudeColumns());
            int aboveLeftLongitudeIndex1 = computeLongitudeIndex(zipperLon1, aboveLeftTile.getMinimumLongitude(), aboveLeftTile.getLongitudeStep(), aboveLeftTile.getLongitudeColumns());

            int aboveRightLongitudeIndex2 = computeLongitudeIndex(zipperLon2, aboveRightTile.getMinimumLongitude(), aboveRightTile.getLongitudeStep(), aboveRightTile.getLongitudeColumns());
            int aboveRightLongitudeIndex3 = computeLongitudeIndex(zipperLon3, aboveRightTile.getMinimumLongitude(), aboveRightTile.getLongitudeStep(), aboveRightTile.getLongitudeColumns());

            // Rows 0 and 1 of zipper: 
            // 2 first cells belong to the below left tile and 2 last cells to the below right tile

            // row 0 
            elevations[0][0] = belowLeftTile.getElevationAtIndices(belowLeftLatitudeIndex0, belowLeftLongitudeIndex0);
            elevations[0][1] = belowLeftTile.getElevationAtIndices(belowLeftLatitudeIndex0, belowLeftLongitudeIndex1);

            elevations[0][2] = belowRightTile.getElevationAtIndices(belowRightLatitudeIndex0, belowRightLongitudeIndex2);
            elevations[0][3] = belowRightTile.getElevationAtIndices(belowRightLatitudeIndex0, belowRightLongitudeIndex3);

            // row 1
            elevations[1][0] = belowLeftTile.getElevationAtIndices(belowLeftLatitudeIndex1, belowLeftLongitudeIndex0);
            elevations[1][1] = belowLeftTile.getElevationAtIndices(belowLeftLatitudeIndex1, belowLeftLongitudeIndex1);

            elevations[1][2] = belowRightTile.getElevationAtIndices(belowRightLatitudeIndex1, belowRightLongitudeIndex2);
            elevations[1][3] = belowRightTile.getElevationAtIndices(belowRightLatitudeIndex1, belowRightLongitudeIndex3);

            // Rows 2 and 3 of zipper: 
            // 2 first cells belong to the above left tile and 2 last cells to the above right tile

            // row 2
            elevations[2][0] = aboveLeftTile.getElevationAtIndices(aboveLeftLatitudeIndex2, aboveLeftLongitudeIndex0);
            elevations[2][1] = aboveLeftTile.getElevationAtIndices(aboveLeftLatitudeIndex2, aboveLeftLongitudeIndex1);

            elevations[2][2] = aboveRightTile.getElevationAtIndices(aboveRightLatitudeIndex2, aboveRightLongitudeIndex2);
            elevations[2][3] = aboveRightTile.getElevationAtIndices(aboveRightLatitudeIndex2, aboveRightLongitudeIndex3);

            // row 3
            elevations[3][0] = aboveLeftTile.getElevationAtIndices(aboveLeftLatitudeIndex3, aboveLeftLongitudeIndex0);
            elevations[3][1] = aboveLeftTile.getElevationAtIndices(aboveLeftLatitudeIndex3, aboveLeftLongitudeIndex1);

            elevations[3][2] = aboveRightTile.getElevationAtIndices(aboveRightLatitudeIndex3, aboveRightLongitudeIndex2);
            elevations[3][3] = aboveRightTile.getElevationAtIndices(aboveRightLatitudeIndex3, aboveRightLongitudeIndex3);

        } // end test isSameStepLat && isSameStepLon
        
        // Initialize the corner zipper tile
        final T cornerZipperTile = initializeZipperTile(zipperLatMin, zipperLonMin, 
                                                        zipperLatStep, zipperLonStep, 
                                                        zipperLatRows, zipperLonCols, elevations);

        return cornerZipperTile;
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

    /** Compute the corner zipper tile origin (min latitude and min longitude).
     * @param zipperLatStep zipper latitude step (rad)
     * @param zipperLonStep zipper longitude step (rad)
     * @param latitudeHemisphere latitude hemisphere of the zipper vs the current tile
     * @param currentTileMinLat current tile latitude origin (rad)
     * @param currentTileLatRows current tile latitude rows
     * @param currentTileLatStep current tile latitude step (rad)
     * @param longitudeHemisphere longitude hemisphere of the zipper vs the current tile
     * @param currentTileMinLon current tile tile longitude origin (rad)
     * @param currentTileLonCols current tile longitude columns
     * @param currentTileLonStep current tile longitude step (rad)
     * @return corner zipper tile origin point
     * @since X.x
     */
    private GeodeticPoint computeCornerZipperOrigin(final double zipperLatStep, final double zipperLonStep,
                                                    final EarthHemisphere latitudeHemisphere, final double currentTileMinLat, 
                                                    final int currentTileLatRows, final double currentTileLatStep,
                                                    final EarthHemisphere longitudeHemisphere, final double currentTileMinLon, 
                                                    final int currentTileLonCols, final double currentTileLonStep) {
        double zipperLatMin;
        double zipperLonMin;
        
        // Explanation: 
        // We want the zipper 2 first rows belongs to the below tiles and the zipper 2 last rows to the above tiles.
        // We want the zipper 2 first columns belongs to the left tiles and the zipper 2 last columns to the right tiles.
        // We use the current tile origin AND the zipper steps to compute the zipper origin

        switch (latitudeHemisphere) {
        case NORTH:

            switch (longitudeHemisphere) {
            case WEST:
                zipperLatMin = currentTileMinLat + currentTileLatRows * currentTileLatStep - 2*zipperLatStep;
                zipperLonMin = currentTileMinLon - 2*zipperLonStep;
                break;

            case EAST:
                zipperLatMin = currentTileMinLat + currentTileLatRows * currentTileLatStep - 2*zipperLatStep;
                zipperLonMin = currentTileMinLon + currentTileLonCols * currentTileLonStep - 2*zipperLonStep;
                break;

            default:
                // case impossible to reach
                throw new RuggedException(RuggedMessages.INTERNAL_ERROR);
            } // end switch longitudeHemisphere

            break;

        case SOUTH:
            
            switch (longitudeHemisphere) {
            case WEST:
                zipperLatMin = currentTileMinLat - 2*zipperLatStep;
                zipperLonMin = currentTileMinLon - 2*zipperLonStep;
                break;

            case EAST:
                zipperLatMin = currentTileMinLat - 2*zipperLatStep;
                zipperLonMin = currentTileMinLon + currentTileLonCols * currentTileLonStep - 2*zipperLonStep;
                break;

            default:
                // case impossible to reach
                throw new RuggedException(RuggedMessages.INTERNAL_ERROR);
            } // end switch longitudeHemisphere

            break;

        default:
            // impossible to reach
            throw new RuggedException(RuggedMessages.INTERNAL_ERROR);
        } // end switch latitudeHemisphere

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
}
