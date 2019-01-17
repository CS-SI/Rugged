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

import org.hipparchus.util.FastMath;
import java.lang.reflect.Array;
import java.text.DecimalFormat;
import java.text.DecimalFormatSymbols;
import java.util.Locale;

import org.orekit.rugged.errors.RuggedException;
import org.orekit.rugged.errors.RuggedMessages;

/** Cache for Digital Elevation Model {@link Tile tiles}.
 * <p>
 * Beware, this cache is <em>not</em> thread-safe!
 * </p>
 * @param <T> Type of tiles.
 * @author Luc Maisonobe
 */
public class TilesCache<T extends Tile> {

    /** Factory for empty tiles. */
    private final TileFactory<T> factory;

    /** Updater for retrieving tiles data. */
    private final TileUpdater updater;

    /** Cache. */
    private final T[] tiles;

    /** Simple constructor.
     * @param factory factory for creating empty tiles
     * @param updater updater for retrieving tiles data
     * @param maxTiles maximum number of tiles stored simultaneously in the cache
     */
    public TilesCache(final TileFactory<T> factory, final TileUpdater updater, final int maxTiles) {
        this.factory       = factory;
        this.updater    = updater;
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

        // none of the tiles in the cache covers the specified points

        // make some room in the cache, possibly evicting the least recently used one
        for (int i = tiles.length - 1; i > 0; --i) {
            tiles[i] = tiles[i - 1];
        }

        // create the tile and retrieve its data
        final T tile = factory.createTile();
        updater.updateTile(latitude, longitude, tile);
        tile.tileUpdateCompleted();

        
        // Check if the tile HAS INTERPOLATION NEIGHBORS : the (latitude, longitude) is inside the tile
        // otherwise the (latitude, longitude) is on the edge of the tile.
        // One must create a zipper tile in case tiles are not overlapping ...
        
        tilePrint(tile, "Current tile:");

final DecimalFormatSymbols symbols = new DecimalFormatSymbols(Locale.US);
final DecimalFormat dfs = new DecimalFormat("#.###",symbols);
System.out.format("longitude= " + dfs.format(FastMath.toDegrees(longitude)) + " location= " + tile.getLocation(latitude, longitude) + "\n");

        final Tile.Location location = tile.getLocation(latitude, longitude);

// System.out.format("Origin : lat min " + dfs.format(FastMath.toDegrees(tile.getMinimumLatitude())) +
//        " lat max " + dfs.format(FastMath.toDegrees(tile.getMaximumLatitude())) +
//        " lon min " + dfs.format(FastMath.toDegrees(tile.getMinimumLongitude())) +
//        " lon max " + dfs.format(FastMath.toDegrees(tile.getMaximumLongitude())) +
//        "\n");


        // We are on the edge of the tile 
        if (location != Tile.Location.HAS_INTERPOLATION_NEIGHBORS) {

            // One must create a zipper tile between this tile and the neighbor tile
            switch (location) {
            
            case NORTH: // latitudeIndex > latitudeRows - 2
                final int latitudeRows = tile.getLatitudeRows();
                final double latStep = tile.getLatitudeStep();

                // Get the North Tile
                final double latToGetNorthTile = latitudeRows*latStep + tile.getMinimumLatitude();
                final T tileNorth = factory.createTile();
                updater.updateTile(latToGetNorthTile, longitude, tileNorth);
                tileNorth.tileUpdateCompleted();
                
                tilePrint(tileNorth, "North tile:");


// System.out.println("North : lat min " + dfs.format(FastMath.toDegrees(tileNorth.getMinimumLatitude())) +
//                           " lat max " + dfs.format(FastMath.toDegrees(tileNorth.getMaximumLatitude())) +
//                           " lon min " + dfs.format(FastMath.toDegrees(tileNorth.getMinimumLongitude())) +
//                           " lon max " + dfs.format(FastMath.toDegrees(tileNorth.getMaximumLongitude())) +
//                           "\n");

                // Create zipper tile between the current tile and the North tile;
                // 2 rows belong to the North part of the origin tile
                // 1 row belongs to the South part of the North tile
                final Tile tileZipperNorth = new ZipperTile();

                // TODO we suppose here the 2 tiles have same origin and size along longitude

                double zipperLatitudeMin = (latitudeRows - 2)*latStep + tile.getMinimumLatitude();
                double zipperLatitudeStep = tile.getLatitudeStep();
                int zipperLatitudeRows = 4 + 5;
                
                int zipperLongitudeColumns = tile.getLongitudeColumns();
                
                double[][] elevations = new double[zipperLatitudeRows][zipperLongitudeColumns];
                
                for (int jLon = 0; jLon < zipperLongitudeColumns; jLon++) {
                    // Part from the current tile
                    int latitudeIndex = latitudeRows - 2;
                    elevations[0][jLon] = tile.getElevationAtIndices(latitudeIndex, jLon);
                    latitudeIndex = latitudeRows - 1;
                    elevations[1][jLon] = tile.getElevationAtIndices(latitudeIndex, jLon);
                    
                    // Part from the North tile
                    elevations[2][jLon] = tileNorth.getElevationAtIndices(0, jLon);
                    elevations[3][jLon] = tileNorth.getElevationAtIndices(1, jLon);
                    
                    // + 5 latitude
                    elevations[4][jLon] = tileNorth.getElevationAtIndices(2, jLon);
                    elevations[5][jLon] = tileNorth.getElevationAtIndices(3, jLon);
                    elevations[6][jLon] = tileNorth.getElevationAtIndices(4, jLon);
                    elevations[7][jLon] = tileNorth.getElevationAtIndices(5, jLon);
                    elevations[8][jLon] = tileNorth.getElevationAtIndices(6, jLon);

                }
                tileZipperNorth.setGeometry(zipperLatitudeMin, tile.getMinimumLongitude(), zipperLatitudeStep, tile.getLongitudeStep(), 
                                            zipperLatitudeRows, zipperLongitudeColumns);
                
                // Fill in the tile with the relevant elevations
                for (int iLat = 0; iLat < zipperLatitudeRows; iLat++) {
                    for (int jLon = 0; jLon < zipperLongitudeColumns; jLon++) {
                        tileZipperNorth.setElevation(iLat, jLon, elevations[iLat][jLon]);
                    }
                }
                tileZipperNorth.tileUpdateCompleted();

                tilePrint(tileZipperNorth, "Zipper tile:");
                
                // again make some room in the cache, possibly evicting the least recently used one
                for (int i = tiles.length - 1; i > 0; --i) {
                    tiles[i] = tiles[i - 1];
                }
                tiles[1] = tile;
                tiles[0] = (T) tileZipperNorth;
                return (T) tileZipperNorth;
//break;               
//                tiles[0] = tile;
//                return tile;

            default:
                System.out.println("Location= " + location + " => Case not yet implemented");
                break;
            }

        }
   
        if (tile.getLocation(latitude, longitude) != Tile.Location.HAS_INTERPOLATION_NEIGHBORS) {
            // this should happen only if user set up an inconsistent TileUpdater
            throw new RuggedException(RuggedMessages.TILE_WITHOUT_REQUIRED_NEIGHBORS_SELECTED,
                                      FastMath.toDegrees(latitude),
                                      FastMath.toDegrees(longitude));
        }

        tiles[0] = tile;
        return tile;

    }
    
    protected void tilePrint(final Tile tile, String comment) {
        
        final DecimalFormatSymbols symbols = new DecimalFormatSymbols(Locale.US);
        final DecimalFormat dfs = new DecimalFormat("#.###",symbols);
        final DecimalFormat dfs2 = new DecimalFormat("#.#####",symbols);

        System.out.format(comment + " rows " + tile.getLatitudeRows() + " col " + tile.getLongitudeColumns() + 
                " lat step " + dfs2.format(FastMath.toDegrees(tile.getLatitudeStep())) + " long step " + dfs2.format(FastMath.toDegrees(tile.getLongitudeStep())) + 
                "\n" +  
                "       lat min " + dfs.format(FastMath.toDegrees(tile.getMinimumLatitude())) + " lat max " + dfs.format(FastMath.toDegrees(tile.getMaximumLatitude())) + 
                "\n" +  
                "       lon min " + dfs.format(FastMath.toDegrees(tile.getMinimumLongitude())) + " lon max " + dfs.format(FastMath.toDegrees(tile.getMaximumLongitude())) +
                "\n" +  
                "       min elevation " + tile.getMinElevation() + " lat index " + tile.getMinElevationLatitudeIndex() + " long index " + tile.getMinElevationLongitudeIndex() +
                "\n " +
                "       max elevation " + tile.getMaxElevation() +  " lat index " + tile.getMaxElevationLatitudeIndex() + " long index " + tile.getMaxElevationLongitudeIndex() +
                "\n");
    }

}
