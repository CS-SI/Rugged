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

    /** Epsilon to test step equality in latitude and longitude. */
    private static double STEP_EQUALITY = 5 * Precision.EPSILON;

    /** Factory for empty tiles. */
    private final TileFactory<T> factory;

    /** Updater for retrieving tiles data. */
    private final TileUpdater updater;

    /**Flag to tell if the Digital Elevation Model tiles are overlapping.
     * @since 4.0 */
    private final boolean isOverlapping;

    /** Cache. */
    private final T[] tiles;

    /** Simple constructor.
     * @param factory factory for creating empty tiles
     * @param updater updater for retrieving tiles data
     * @param maxTiles maximum number of tiles stored simultaneously in the cache
     * @param isOverlappingTiles flag to tell if the DEM tiles are overlapping:
     *                          true if overlapping; false otherwise.
     */
    public TilesCache(final TileFactory<T> factory, final TileUpdater updater,
                      final int maxTiles, final boolean isOverlappingTiles) {
        this.factory       = factory;
        this.updater    = updater;
        this.isOverlapping = isOverlappingTiles;
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

        if ( !isOverlapping ) { // DEM with seamless tiles (no overlapping)

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

        } else { // isOverlapping: DEM with overlapping tiles (according to the flag ...)

            // Check if the tile HAS INTERPOLATION NEIGHBORS (the (latitude, longitude) is inside the tile)
            if (tile.getLocation(latitude, longitude) != Tile.Location.HAS_INTERPOLATION_NEIGHBORS) {
                // this should happen only if user set up an inconsistent TileUpdater
                throw new RuggedException(RuggedMessages.TILE_WITHOUT_REQUIRED_NEIGHBORS_SELECTED,
                                          FastMath.toDegrees(latitude), FastMath.toDegrees(longitude));
            }

            tiles[0] = tile;
            return tile;

        } // end if (!isOverlapping)
    }

    /** Create a tile defines by its latitude and longitude.
     * @param latitude latitude of the desired tile (rad)
     * @param longitude longitude of the desired tile (rad)
     * @return the tile
     * @since 4.0
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

    /** Create a zipper tile for DEM with seamless tiles (no overlapping).
     * @param currentTile current tile
     * @param latitude ground point latitude (rad)
     * @param longitude ground point longitude (rad)
     * @param pointLocation ground point location with respect to the tile
     * @return zipper tile covering the ground point
     * @since 4.0
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

    /** Initialize the zipper tile for a given geometry and the full set of elevations.
     * @param zipperLatMin zipper min latitude (ra)
     * @param zipperLonMin zipper min longitude (rad)
     * @param zipperLatStep zipper latitude step (rad)
     * @param zipperLonStep zipper longitude step (rad)
     * @param zipperLatRows zipper latitude rows
     * @param zipperLonCols zipper longitude columns
     * @param zipperElevations zipper elevations
     * @return the zipper tile
     * @since 4.0
     */
    private T initializeZipperTile(final double zipperLatMin, final double zipperLonMin,
                                   final double zipperLatStep, final double zipperLonStep,
                                   final int zipperLatRows, final int zipperLonCols,
                                   final double[][] zipperElevations) {

        // Create an empty tile
        final T zipperTile = factory.createTile();

        // Set the tile geometry
        zipperTile.setGeometry(zipperLatMin, zipperLonMin, zipperLatStep, zipperLonStep,
                               zipperLatRows, zipperLonCols);

        // Fill in the tile with the relevant elevations
        for (int iLat = 0; iLat < zipperLatRows; iLat++) {
            for (int jLon = 0; jLon < zipperLonCols; jLon++) {
                zipperTile.setElevation(iLat, jLon, zipperElevations[iLat][jLon]);
            }
        }

        // Last step in order to create the MinMax kd tree
        zipperTile.tileUpdateCompleted();

        return zipperTile;
    }

    /** Create the zipper tile between the current tile and the Northern or Southern tile of the current tile.
     * for a given longitude. The Northern or Southern tiles of the current tile may have a change in
     * resolution either in longitude or in latitude wrt the current tile.
     * The zipper has 4 rows in latitude.
     * @param latitudeHemisphere hemisphere for latitude: NORTH / SOUTH
     * @param longitude given longitude (rad)
     * @param currentTile current tile
     * @return Northern or Southern zipper tile of the current tile (according to latitudeHemisphere)
     * @since 4.0
     */
    private T createZipperNorthOrSouth(final EarthHemisphere latitudeHemisphere, final double longitude, final T currentTile) {

        final int currentTileLatRows = currentTile.getLatitudeRows();
        final int currentTileLonCols = currentTile.getLongitudeColumns();
        final double currentTileLatStep = currentTile.getLatitudeStep();
        final double currentTileLonStep = currentTile.getLongitudeStep();
        final double currentTileMinLat = currentTile.getMinimumLatitude();
        final double currentTileMinLon = currentTile.getMinimumLongitude();

        // Get the Northern or Southern tile
        final T tileNorthOrSouth = createNorthOrSouthTile(latitudeHemisphere, longitude, currentTileMinLat, currentTileLatStep, currentTileLatRows);

        // If NORTH hemisphere:
        // Create zipper tile between the current tile and the Northern tile;
        // 2 rows belong to the North part of the current tile
        // 2 rows belong to the South part of the Northern tile

        // If SOUTH hemisphere:
        // Create zipper tile between the current tile and the Southern tile;
        // 2 rows belong to the South part of the current tile
        // 2 rows belong to the North part of the Southern tile

        // Initialize the zipper latitude step and number of rows (4)
        final int zipperLatRows = 4;
        double zipperLatStep = currentTileLatStep;

        // Check if latitude steps are the same between the current tile and the Northern or Southern tile
        boolean isSameStepLat = true;
        final double northSouthLatitudeStep = tileNorthOrSouth.getLatitudeStep();

        if (!(Math.abs(currentTileLatStep - northSouthLatitudeStep) < STEP_EQUALITY)) {
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
        final double northSouthLongitudeStep = tileNorthOrSouth.getLongitudeStep();

        if (!(Math.abs(currentTileLonStep - northSouthLongitudeStep) < STEP_EQUALITY)) {
            // Steps are not the same
            isSameStepLon = false;
            // Recompute zipper longitude step and columns if the North or South tile longitude step is smaller
            if (northSouthLongitudeStep < currentTileLonStep) {
                zipperLonStep = northSouthLongitudeStep;
                zipperLonCols = tileNorthOrSouth.getLongitudeColumns();
            }
        }

        final double zipperLatMin;
        final double zipperLonMin;
        final double[][] elevations;

        switch (latitudeHemisphere) {
            case NORTH:
                // Defines the zipper min latitude (center of the cell) wrt the current tile Northern latitude
                // Explanation: we want the 2 first rows belongs to the current tile and 2 last rows to the Northern tile
                //    If zipperLatStep = currentTileLatStep, the 2 first rows = exactly lines of the current tile
                //    otherwise (currentTileLatStep > North tile step), the 2 first rows will be smaller (in latitude) than the 2 lines of the current tile
                final double currentTileNorthernLatitude = currentTileMinLat - 0.5 * currentTileLatStep + currentTileLatRows * currentTileLatStep;
                // Zipper min latitude = center of the Southern zipper cell in latitude
                // In case of resolution change zipperLatStep may be different from currentTileLatStep
                zipperLatMin = currentTileNorthernLatitude - 2 * zipperLatStep + 0.5 * zipperLatStep;

                // Define the zipper min longitude = current tile min longitude
                zipperLonMin = currentTileMinLon;

                // Fill in the zipper elevations
                elevations = getZipperNorthSouthElevations(zipperLonCols, tileNorthOrSouth, currentTile,
                                                           isSameStepLat, isSameStepLon,
                                                           zipperLatMin, zipperLonMin, zipperLatStep, zipperLonStep);
                break;

            case SOUTH:
                // Defines the zipper min latitude wrt the current tile Southern latitude
                // Explanation: we want the 2 last rows belongs to the current tile and 2 first rows to the Southern tile
                //    If zipperLatStep = currentTileLatStep, the 2 last rows = exactly lines of the current tile
                //    otherwise (currentTileLatStep > South tile step), the 2 last rows will be smaller (in latitude) than the 2 lines of the current tile
                final double currentTileSouthernLatitude = currentTileMinLat - 0.5 * currentTileLatStep;
                // Zipper min latitude = center of the Southern zipper cell in latitude
                // In case of resolution change zipperLatStep may be different from currentTileLatStep
                zipperLatMin = currentTileSouthernLatitude - 2 * zipperLatStep + 0.5 * zipperLatStep;

                // Define the zipper min longitude = current tile min longitude
                zipperLonMin = currentTileMinLon;

                // Fill in the zipper elevations
                elevations = getZipperNorthSouthElevations(zipperLonCols, currentTile, tileNorthOrSouth,
                                                          isSameStepLat, isSameStepLon,
                                                          zipperLatMin, zipperLonMin, zipperLatStep, zipperLonStep);
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

    /** Create the zipper tile between the current tile and the Western or Eastern tile of the current tile.
     * for a given latitude. The Western or Eastern tiles of the current tile have the same
     * resolution in longitude and in latitude as the current tile (for known DEMs).
     * The zipper has 4 columns in longitude.
     * @param longitudeHemisphere hemisphere for longitude: WEST / EAST
     * @param latitude given latitude (rad)
     * @param currentTile current tile
     * @return Western or Eastern zipper tile of the current tile (according to longitudeHemisphere)
     * @since 4.0
     */
    private T createZipperWestOrEast(final EarthHemisphere longitudeHemisphere, final double latitude, final T currentTile) {

        final int currentTileLatRows = currentTile.getLatitudeRows();
        final int currentTileLonCols = currentTile.getLongitudeColumns();
        final double currentTileLatStep = currentTile.getLatitudeStep();
        final double currentTileLonStep = currentTile.getLongitudeStep();
        final double currentTileMinLon = currentTile.getMinimumLongitude();

        // Get the West or East Tile
        final T tileWestOrEast = createEastOrWestTile(longitudeHemisphere, latitude, currentTileMinLon, currentTileLonStep, currentTileLonCols);

        if (!(Math.abs(currentTileLatStep - tileWestOrEast.getLatitudeStep()) < STEP_EQUALITY) ||
            !(Math.abs(currentTileLonStep - tileWestOrEast.getLongitudeStep()) < STEP_EQUALITY)) {
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

        final double zipperLonStep = currentTileLonStep;
        final int zipperLatRows = currentTileLatRows;
        final int zipperLonCols = 4;

        final double zipperLonMin;
        final double[][] elevations;

        switch (longitudeHemisphere) {
            case WEST:
                zipperLonMin = currentTileMinLon - 2 * currentTileLonStep;
                elevations = getZipperEastWestElevations(zipperLatRows, currentTile, tileWestOrEast);
                break;

            case EAST:
                zipperLonMin = currentTileMinLon + (currentTileLonCols - 2) * currentTileLonStep;
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
     * @param latitudeMin min latitude of the tile (rad)
     * @param latitudeStep latitude step of the tile (rad)
     * @param latitudeRows number of rows in latitude
     * @return latitude index (it may lie outside of the tile!)
     * @since 4.0
     */
    private int computeLatitudeIndex(final double latitude, final double latitudeMin, final double latitudeStep, final int latitudeRows) {
        // Compute the difference in latitude wrt the Southern edge latitude of the tile
        // TBN: latitude min is at the center of the Southern cell tiles.
        final double doubleLatitudeIndex =  (latitude  - (latitudeMin - 0.5 * latitudeStep))  / latitudeStep;
        return FastMath.max(0, FastMath.min(latitudeRows - 1, (int) FastMath.floor(doubleLatitudeIndex)));
    }

    /** Get the longitude index of a point.
     * @param longitude geodetic longitude (rad)
     * @param longitudeMin min longitude of the tile (rad)
     * @param longitudeStep longitude step of the tile (rad)
     * @param longitudeColumns number of columns in longitude
     * @return longitude index (it may lie outside of the tile!)
     * @since 4.0
     */
    private int computeLongitudeIndex(final double longitude, final double longitudeMin, final double longitudeStep, final int longitudeColumns) {
        // Compute the difference in longitude wrt the Western edge longitude of the tile
        // TBN: longitude min is at the center of the Western cell tiles.
        final double doubleLongitudeIndex = (longitude - (longitudeMin - 0.5 * longitudeStep)) / longitudeStep;
        return FastMath.max(0, FastMath.min(longitudeColumns - 1, (int) FastMath.floor(doubleLongitudeIndex)));
    }

    /** Get the elevations for the zipper tile between a northern and a southern tiles with 4 rows in latitude.
     * @param zipperLonCols number of column in longitude
     * @param northernTile the tile which is the northern
     * @param southernTile the tile which is the southern
     * @param isSameStepLat flag to tell is latitude steps are the same (= true)
     * @param isSameStepLon flag to tell is longitude steps are the same (= true)
     * @param zipperLatMin zipper tile min latitude (rad)
     * @param zipperLonMin zipper tile min longitude (rad)
     * @param zipperLatStep zipper tile latitude step (rad)
     * @param zipperLonStep zipper tile longitude step (rad)
     * @return the elevations to fill in the zipper tile between a northern and a southern tiles
     * @since 4.0
     */
    private double[][] getZipperNorthSouthElevations(final int zipperLonCols,
                                                     final T northernTile, final T southernTile,
                                                     final boolean isSameStepLat, final boolean isSameStepLon,
                                                     final double zipperLatMin, final double zipperLonMin,
                                                     final double zipperLatStep, final double zipperLonStep) {

        final double[][] elevations = new double[4][zipperLonCols];

        if (isSameStepLat && isSameStepLon) { // tiles with same steps in latitude and longitude

            for (int jLon = 0; jLon < zipperLonCols; jLon++) {
                // Part from the northern tile
                final int lat3 = 1;
                elevations[3][jLon] = northernTile.getElevationAtIndices(lat3, jLon);
                final int lat2 = 0;
                elevations[2][jLon] = northernTile.getElevationAtIndices(lat2, jLon);

                // Part from the southern tile
                final int lat1 = southernTile.getLatitudeRows() - 1;
                elevations[1][jLon] = southernTile.getElevationAtIndices(lat1, jLon);
                final int lat0 = southernTile.getLatitudeRows() - 2;
                elevations[0][jLon] = southernTile.getElevationAtIndices(lat0, jLon);
            }

        } else { // tiles with different steps

            // To cover every cases and as zipper with such characteristics are rare,
            // we will use conversion from (latitude, longitude) to tile indices

            // We assure to be inside a cell by adding a delta step (to avoid inappropriate index computation)
            // TBN: zipperLatStep/zipperLonStep are the smallest steps vs Northern and Southern tiles
            final double deltaLon = 0.1 * zipperLonStep;
            double zipperLonCurrent = zipperLonMin + deltaLon;

            // Assure that the longitude belongs to [-180, + 180]
            final double zipperLonMax = MathUtils.normalizeAngle(zipperLonMin + (zipperLonCols - 1) * zipperLonStep, 0.0);

            // Northern Tile
            final double northernMinLat = northernTile.getMinimumLatitude();
            final double northernLatStep = northernTile.getLatitudeStep();
            final int northernLatRows = northernTile.getLatitudeRows();
            final double northernMinLon = northernTile.getMinimumLongitude();
            final double northernLonStep = northernTile.getLongitudeStep();
            final int northernLonCols = northernTile.getLongitudeColumns();

            // Southern Tile
            final double southernMinLat = southernTile.getMinimumLatitude();
            final double southernLatStep = southernTile.getLatitudeStep();
            final int southernLatRows = southernTile.getLatitudeRows();
            final double southernMinLon = southernTile.getMinimumLongitude();
            final double southernLonStep = southernTile.getLongitudeStep();
            final int southernLonCols = southernTile.getLongitudeColumns();

            while (zipperLonCurrent <= zipperLonMax + 2 * deltaLon) {

                // Compute zipper tile longitude index
                final int zipperLonIndex = computeLongitudeIndex(zipperLonCurrent, zipperLonMin, zipperLonStep, zipperLonCols);

                // Part from the northern tile
                // ---------------------------
                // Compute northern longitude
                final int northenLongitudeIndex = computeLongitudeIndex(zipperLonCurrent, northernMinLon, northernLonStep, northernLonCols);

                final double zipperLat3 = zipperLatMin + (3 + 0.1) * zipperLatStep;
                // lat3 would be 1 if Northern latitude step smallest; could be 0 if biggest
                final int lat3Index = computeLatitudeIndex(zipperLat3, northernMinLat, northernLatStep, northernLatRows);
                elevations[3][zipperLonIndex] = northernTile.getElevationAtIndices(lat3Index, northenLongitudeIndex);

                // lat2 is 0 whatever Northern latitude step
                final int lat2Index = 0;
                elevations[2][zipperLonIndex] = northernTile.getElevationAtIndices(lat2Index, northenLongitudeIndex);

                // Part from the southern tile
                // ---------------------------
                // Compute southern longitude
                final int southernLongitudeIndex = computeLongitudeIndex(zipperLonCurrent, southernMinLon, southernLonStep, southernLonCols);

                // lat1 is Southern latitude rows - 1  whatever Southern latitude step
                final int lat1Index = southernTile.getLatitudeRows() - 1;
                elevations[1][zipperLonIndex] = southernTile.getElevationAtIndices(lat1Index, southernLongitudeIndex);

                final double zipperLat0 = zipperLatMin + 0.1 * zipperLatStep;
                // lat1 would be Southern latitude rows - 2 if Southern latitude step smallest; could be Southern latitude rows - 1 if biggest
                final int lat0Index = computeLatitudeIndex(zipperLat0, southernMinLat, southernLatStep, southernLatRows);
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
     * @since 4.0
     */
    private double[][] getZipperEastWestElevations(final int zipperLatRows,
                                                   final T easternTile, final T westernTile) {

        final double[][] elevations = new double[zipperLatRows][4];

        for (int iLat = 0; iLat < zipperLatRows; iLat++) {
            // Part from the eastern tile
            final int lon3 = 1;
            elevations[iLat][3] = easternTile.getElevationAtIndices(iLat, lon3);
            final int lon2 = 0;
            elevations[iLat][2] = easternTile.getElevationAtIndices(iLat, lon2);

            // Part from the western tile
            final int lon1 = westernTile.getLongitudeColumns() - 1;
            elevations[iLat][1] = westernTile.getElevationAtIndices(iLat, lon1);
            final int lon0 = westernTile.getLongitudeColumns() - 2;
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
     * @since 4.0
     */
    private T createCornerZipper(final EarthHemisphere latitudeHemisphere, final EarthHemisphere longitudeHemisphere,
                                 final double latitude, final double longitude, final T currentTile) {

        final int currentTileLatRows = currentTile.getLatitudeRows();
        final int currentTileLonCols = currentTile.getLongitudeColumns();
        final double currentTileLatStep = currentTile.getLatitudeStep();
        final double currentTileLonStep = currentTile.getLongitudeStep();
        final double currentTileLonMin = currentTile.getMinimumLongitude();
        final double currentTileLatMin = currentTile.getMinimumLatitude();

        final T belowLeftTile;
        final T belowRightTile;
        final T aboveLeftTile;
        final T aboveRightTile;

        switch (latitudeHemisphere) {
            case NORTH:

                switch (longitudeHemisphere) {
                    case WEST:

                        // Get the West Tile
                        final T tileWest = createEastOrWestTile(EarthHemisphere.WEST, latitude,
                                                                currentTileLonMin, currentTileLonStep, currentTileLonCols);
                        // Get the North Tile
                        T tileNorth = createNorthOrSouthTile(EarthHemisphere.NORTH, longitude,
                                                             currentTileLatMin, currentTileLatStep, currentTileLatRows);
                        // Get the North-West Tile
                        final T tileNorthWest = createIntercardinalTile(EarthHemisphere.NORTH,
                                                                        currentTileLatMin, currentTileLatStep, currentTileLatRows,
                                                                        EarthHemisphere.WEST,
                                                                        currentTileLonMin, currentTileLonStep, currentTileLonCols);
                        belowLeftTile = tileWest;
                        belowRightTile = currentTile;
                        aboveLeftTile = tileNorthWest;
                        aboveRightTile = tileNorth;

                        break;

                    case EAST:

                        // Get the East Tile
                        final T tileEast = createEastOrWestTile(EarthHemisphere.EAST, latitude,
                                                                currentTileLonMin, currentTileLonStep, currentTileLonCols);
                        // Get the North Tile
                        tileNorth = createNorthOrSouthTile(EarthHemisphere.NORTH, longitude,
                                                           currentTileLatMin, currentTileLatStep, currentTileLatRows);
                        // Get the North-East Tile
                        final T tileNorthEast = createIntercardinalTile(EarthHemisphere.NORTH,
                                                                        currentTileLatMin, currentTileLatStep, currentTileLatRows,
                                                                        EarthHemisphere.EAST,
                                                                        currentTileLonMin, currentTileLonStep, currentTileLonCols);
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
                        final T tileWest = createEastOrWestTile(EarthHemisphere.WEST, latitude,
                                                                currentTileLonMin, currentTileLonStep, currentTileLonCols);
                        // Get the South Tile
                        T tileSouth = createNorthOrSouthTile(EarthHemisphere.SOUTH, longitude,
                                                             currentTileLatMin, currentTileLatStep, currentTileLatRows);
                        // Get the South-West Tile
                        final T tileSouthhWest = createIntercardinalTile(EarthHemisphere.SOUTH,
                                                                         currentTileLatMin, currentTileLatStep, currentTileLatRows,
                                                                         EarthHemisphere.WEST,
                                                                         currentTileLonMin, currentTileLonStep, currentTileLonCols);
                        belowLeftTile = tileSouthhWest;
                        belowRightTile = tileSouth;
                        aboveLeftTile = tileWest;
                        aboveRightTile = currentTile;

                        break;

                    case EAST:

                        // Get the East Tile
                        final T tileEast = createEastOrWestTile(EarthHemisphere.EAST, latitude,
                                                                currentTileLonMin, currentTileLonStep, currentTileLonCols);
                        // Get the South Tile
                        tileSouth = createNorthOrSouthTile(EarthHemisphere.SOUTH, longitude,
                                                           currentTileLatMin, currentTileLatStep, currentTileLatRows);
                        // Get the South-East Tile
                        final T tileSouthhEast = createIntercardinalTile(EarthHemisphere.SOUTH,
                                                                         currentTileLatMin, currentTileLatStep, currentTileLatRows,
                                                                         EarthHemisphere.EAST,
                                                                         currentTileLonMin, currentTileLonStep, currentTileLonCols);
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
        if (!(Math.abs(belowLeftTile.getLatitudeStep() - belowRightTile.getLatitudeStep()) < STEP_EQUALITY) ||
            !(Math.abs(belowLeftTile.getLongitudeStep() - belowRightTile.getLongitudeStep()) < STEP_EQUALITY) ||
            !(Math.abs(aboveLeftTile.getLatitudeStep() - aboveRightTile.getLatitudeStep()) < STEP_EQUALITY) ||
            !(Math.abs(aboveLeftTile.getLongitudeStep() - aboveRightTile.getLongitudeStep()) < STEP_EQUALITY)) {
            // Steps are not the same.
            throw new RuggedException(RuggedMessages.INTERNAL_ERROR);
        }


        // Compute the zipper steps in latitude and longitude.
        // If steps are different, the zipper will have the smallest steps.

        // at this stage the latitude steps of the right and left tiles are the same
        final double belowLatitudeStep = belowRightTile.getLatitudeStep();
        final double aboveLatitudeStep = aboveRightTile.getLatitudeStep();

        // initialize the zipper latitude step
        double zipperLatStep = belowLatitudeStep;

        // check if latitude steps are the same between the above and below tiles
        boolean isSameStepLat = true;

        if (!(Math.abs(belowLatitudeStep - aboveLatitudeStep) < STEP_EQUALITY)) {

            // Latitude steps are not the same
            isSameStepLat = false;

            // Recompute zipper latitude step if the above tiles latitude step is smaller
            if (aboveLatitudeStep < belowLatitudeStep) {
                zipperLatStep = aboveLatitudeStep;
            }
        }

        // at this stage the longitude steps of the right and left tiles are the same
        final double belowLongitudeStep = belowRightTile.getLongitudeStep();
        final double aboveLongitudeStep = aboveRightTile.getLongitudeStep();

        // initialize the zipper longitude step
        double zipperLonStep = belowLongitudeStep;

        // check if longitude steps are the same between the above and below tiles
        boolean isSameStepLon = true;

        if (!(Math.abs(belowLongitudeStep - aboveLongitudeStep) < STEP_EQUALITY)) {

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
        final GeodeticPoint zipperCorner = computeCornerZipperOrigin(zipperLatStep, zipperLonStep,
                                                                     latitudeHemisphere, currentTileLatMin,
                                                                     currentTileLatStep, currentTileLatRows,
                                                                     longitudeHemisphere, currentTileLonMin,
                                                                     currentTileLonStep, currentTileLonCols);

        // Initialize corner tile
        return initializeCornerZipperTile(zipperCorner.getLatitude(), zipperCorner.getLongitude(), zipperLatStep, zipperLonStep,
                                          belowLeftTile, aboveLeftTile, belowRightTile, aboveRightTile,
                                          isSameStepLat, isSameStepLon);

    }

    /** Initialize a corner zipper tile (with 4 rows in latitude and 4 columns in longitude).
     * @param zipperLatMin zipper min latitude (ra)
     * @param zipperLonMin zipper min longitude (rad)
     * @param zipperLatStep zipper latitude step (rad)
     * @param zipperLonStep zipper longitude step (rad)
     * @param belowLeftTile below left tile
     * @param aboveLeftTile above left tile
     * @param belowRightTile below right tile
     * @param aboveRightTile above right tile
     * @param isSameStepLat flag to tell if latitude steps are the same (true)
     * @param isSameStepLon flag to tell if longitude steps are the same (true)
     * @return corner zipper tile
     * @since 4.0
     */
    private T initializeCornerZipperTile(final double zipperLatMin, final double zipperLonMin,
                                         final double zipperLatStep, final double zipperLonStep,
                                         final T belowLeftTile, final T aboveLeftTile, final T belowRightTile, final T aboveRightTile,
                                         final boolean isSameStepLat, final boolean isSameStepLon) {

        // Defines with 4 cells from each of the 4 tiles at the corner
        final int zipperLatRows = 4;
        final int zipperLonCols = 4;
        final double[][] elevations = new double[zipperLatRows][zipperLonCols];

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
            final double zipperLat0 = zipperLatMin + 0.1 * zipperLatStep;
            final double zipperLat1 = zipperLat0 + zipperLatStep;
            final double zipperLat2 = zipperLat1 + zipperLatStep;
            final double zipperLat3 = zipperLat2 + zipperLatStep;

            final double zipperLon0 = zipperLonMin + 0.1 * zipperLonStep;
            final double zipperLon1 = zipperLon0 + zipperLonStep;
            final double zipperLon2 = zipperLon1 + zipperLonStep;
            final double zipperLon3 = zipperLon2 + zipperLonStep;

            // Compute the tiles index in latitude
            final int belowLeftLatitudeIndex0 = computeLatitudeIndex(zipperLat0, belowLeftTile.getMinimumLatitude(), belowLeftTile.getLatitudeStep(), belowLeftTile.getLatitudeRows());
            final int belowLeftLatitudeIndex1 = computeLatitudeIndex(zipperLat1, belowLeftTile.getMinimumLatitude(), belowLeftTile.getLatitudeStep(), belowLeftTile.getLatitudeRows());

            final int belowRightLatitudeIndex0 = computeLatitudeIndex(zipperLat0, belowRightTile.getMinimumLatitude(), belowRightTile.getLatitudeStep(), belowRightTile.getLatitudeRows());
            final int belowRightLatitudeIndex1 = computeLatitudeIndex(zipperLat1, belowRightTile.getMinimumLatitude(), belowRightTile.getLatitudeStep(), belowRightTile.getLatitudeRows());

            final int aboveLeftLatitudeIndex2 = computeLatitudeIndex(zipperLat2, aboveLeftTile.getMinimumLatitude(), aboveLeftTile.getLatitudeStep(), aboveLeftTile.getLatitudeRows());
            final int aboveLeftLatitudeIndex3 = computeLatitudeIndex(zipperLat3, aboveLeftTile.getMinimumLatitude(), aboveLeftTile.getLatitudeStep(), aboveLeftTile.getLatitudeRows());

            final int aboveRightLatitudeIndex2 = computeLatitudeIndex(zipperLat2, aboveRightTile.getMinimumLatitude(), aboveRightTile.getLatitudeStep(), aboveRightTile.getLatitudeRows());
            final int aboveRightLatitudeIndex3 = computeLatitudeIndex(zipperLat3, aboveRightTile.getMinimumLatitude(), aboveRightTile.getLatitudeStep(), aboveRightTile.getLatitudeRows());

            // Compute the tiles index in longitude
            final int belowLeftLongitudeIndex0 = computeLongitudeIndex(zipperLon0, belowLeftTile.getMinimumLongitude(), belowLeftTile.getLongitudeStep(), belowLeftTile.getLongitudeColumns());
            final int belowLeftLongitudeIndex1 = computeLongitudeIndex(zipperLon1, belowLeftTile.getMinimumLongitude(), belowLeftTile.getLongitudeStep(), belowLeftTile.getLongitudeColumns());

            final int belowRightLongitudeIndex2 = computeLongitudeIndex(zipperLon2, belowRightTile.getMinimumLongitude(), belowRightTile.getLongitudeStep(), belowRightTile.getLongitudeColumns());
            final int belowRightLongitudeIndex3 = computeLongitudeIndex(zipperLon3, belowRightTile.getMinimumLongitude(), belowRightTile.getLongitudeStep(), belowRightTile.getLongitudeColumns());

            final int aboveLeftLongitudeIndex0 = computeLongitudeIndex(zipperLon0, aboveLeftTile.getMinimumLongitude(), aboveLeftTile.getLongitudeStep(), aboveLeftTile.getLongitudeColumns());
            final int aboveLeftLongitudeIndex1 = computeLongitudeIndex(zipperLon1, aboveLeftTile.getMinimumLongitude(), aboveLeftTile.getLongitudeStep(), aboveLeftTile.getLongitudeColumns());

            final int aboveRightLongitudeIndex2 = computeLongitudeIndex(zipperLon2, aboveRightTile.getMinimumLongitude(), aboveRightTile.getLongitudeStep(), aboveRightTile.getLongitudeColumns());
            final int aboveRightLongitudeIndex3 = computeLongitudeIndex(zipperLon3, aboveRightTile.getMinimumLongitude(), aboveRightTile.getLongitudeStep(), aboveRightTile.getLongitudeColumns());

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
     * Create the tile in intercardinal direction of the current Tile i.e. NW, NE, SW, SE.
     * @param latitudeHemisphere hemisphere for latitude: NORTH / SOUTH
     * @param latitudeMin latitude minimum for the tile (rad)
     * @param latitudeStep latitude step (rad)
     * @param latitudeRows latitude rows
     * @param longitudeHemisphere hemisphere for longitude : WEST / EAST
     * @param longitudeMin longitude minimum for the tile (rad)
     * @param longitudeStep longitude step (rad)
     * @param longitudeCols longitude columns
     * @return the tile in intercardinal direction
     * @since 4.0
     */
    private T createIntercardinalTile(final EarthHemisphere latitudeHemisphere,
            final double latitudeMin, final double latitudeStep, final int latitudeRows,
            final EarthHemisphere longitudeHemisphere,
            final double longitudeMin, final double longitudeStep, final int longitudeCols) {

        // longitudeHemisphere = +1 : East or = -1 : West
        final int lonHemisphere;
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

        // latitudeHemisphere = +1 : North or = -1 : South
        final int latHemisphere;
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

        final double latToGetIntercardinalTile = latitudeMin + latHemisphere * latitudeRows * latitudeStep;
        final double lonToGetIntercardinalTile = longitudeMin + lonHemisphere * longitudeCols * longitudeStep;
        final T intercardinalTile = createTile(latToGetIntercardinalTile, lonToGetIntercardinalTile);
        return intercardinalTile;
    }

    /** Compute the corner zipper tile origin (min latitude and min longitude).
     * @param zipperLatStep zipper latitude step (rad)
     * @param zipperLonStep zipper longitude step (rad)
     * @param latitudeHemisphere latitude hemisphere of the zipper vs the current tile
     * @param currentTileLatMin current tile latitude origin (rad)
     * @param currentTileLatStep current tile latitude step (rad)
     * @param currentTileLatRows current tile latitude rows
     * @param longitudeHemisphere longitude hemisphere of the zipper vs the current tile
     * @param currentTileLonMin current tile tile longitude origin (rad)
     * @param currentTileLonStep current tile longitude step (rad)
     * @param currentTileLonCols current tile longitude columns
     * @return corner zipper tile origin point
     * @since 4.0
     */
    private GeodeticPoint computeCornerZipperOrigin(final double zipperLatStep, final double zipperLonStep,
                                                    final EarthHemisphere latitudeHemisphere,
                                                    final double currentTileLatMin, final double currentTileLatStep, final int currentTileLatRows,
                                                    final EarthHemisphere longitudeHemisphere,
                                                    final double currentTileLonMin, final double currentTileLonStep, final int currentTileLonCols) {
        final double zipperLatMin;
        final double zipperLonMin;

        // Explanation:
        // We want the zipper 2 first rows belongs to the below tiles and the zipper 2 last rows to the above tiles.
        // We want the zipper 2 first columns belongs to the left tiles and the zipper 2 last columns to the right tiles.
        // We use the current tile origin AND the zipper steps to compute the zipper origin

        switch (latitudeHemisphere) {
            case NORTH:

                switch (longitudeHemisphere) {
                    case WEST:
                        zipperLatMin = currentTileLatMin + currentTileLatRows * currentTileLatStep - 2 * zipperLatStep;
                        zipperLonMin = currentTileLonMin - 2 * zipperLonStep;
                        break;

                    case EAST:
                        zipperLatMin = currentTileLatMin + currentTileLatRows * currentTileLatStep - 2 * zipperLatStep;
                        zipperLonMin = currentTileLonMin + currentTileLonCols * currentTileLonStep - 2 * zipperLonStep;
                        break;

                    default:
                        // case impossible to reach
                        throw new RuggedException(RuggedMessages.INTERNAL_ERROR);
                } // end switch longitudeHemisphere

                break;

            case SOUTH:

                switch (longitudeHemisphere) {
                    case WEST:
                        zipperLatMin = currentTileLatMin - 2 * zipperLatStep;
                        zipperLonMin = currentTileLonMin - 2 * zipperLonStep;
                        break;

                    case EAST:
                        zipperLatMin = currentTileLatMin - 2 * zipperLatStep;
                        zipperLonMin = currentTileLonMin + currentTileLonCols * currentTileLonStep - 2 * zipperLonStep;
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


    /** Create the Northern or Southern tile of a given tile defined by minLat, longitude, latitudeRows and latStep.
     * @param latitudeHemisphere latitude hemisphere
     * @param longitude longitude to define the tile (rad)
     * @param latitudeMin minimum latitude to define the tile (rad)
     * @param latitudeStep latitude step (rad)
     * @param latitudeRows latitude rows
     * @return North or South tile according to the Earth hemisphere
     * @since 4.0
     */
    private T createNorthOrSouthTile(final EarthHemisphere latitudeHemisphere, final double longitude,
                                     final double latitudeMin, final double latitudeStep, final int latitudeRows) {
        // hemisphere = +1 : North or = -1 : South
        final int hemisphere;
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

        final double latToGetNewTile = latitudeMin + hemisphere * latitudeRows * latitudeStep;
        return createTile(latToGetNewTile, longitude);
    }


    /** Create the Eastern or Western tile of a given tile defined by latitude, minLon, longitudeCols and lonStep.
     * @param longitudeHemisphere longitude hemisphere
     * @param latitude latitude to define the tile (rad)
     * @param longitudeMin minimum longitude to define the tile (rad)
     * @param longitudeStep longitude step (rad)
     * @param longitudeCols longitude columns
     * @return East or West tile  tile according to the Earth hemisphere
     * @since 4.0
     */
    private T createEastOrWestTile(final EarthHemisphere longitudeHemisphere, final double latitude,
                                   final double longitudeMin, final double longitudeStep, final int longitudeCols) {
        // hemisphere = +1 : East or = -1 : West
        final int hemisphere;
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

        final double lonToGetNewTile = longitudeMin + hemisphere * longitudeCols * longitudeStep;
        return createTile(latitude, lonToGetNewTile);
    }
}
