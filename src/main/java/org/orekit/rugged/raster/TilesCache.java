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

import org.orekit.rugged.errors.DumpManager;
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
     * @param latitude ground point latitude
     * @param longitude ground point longitude
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

        // In case dump is asked for, suspend the dump manager as we don't need to dump anything here
        // For instance for SRTM DEM, the user needs to read Geoid data that are not useful in the dump
        final Boolean wasSuspended = DumpManager.suspend();

        updater.updateTile(latitude, longitude, tile);

        // Resume the dump manager if necessary
        DumpManager.resume(wasSuspended);

        tile.tileUpdateCompleted();

        if (tile.getLocation(latitude, longitude) != Tile.Location.HAS_INTERPOLATION_NEIGHBORS) {
            // this should happen only if user set up an inconsistent TileUpdater
            throw new RuggedException(RuggedMessages.TILE_WITHOUT_REQUIRED_NEIGHBORS_SELECTED,
                                      FastMath.toDegrees(latitude),
                                      FastMath.toDegrees(longitude));
        }

        tiles[0] = tile;
        return tile;

    }

}
