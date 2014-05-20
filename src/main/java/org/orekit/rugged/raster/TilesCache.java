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

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.Iterator;
import java.util.List;

import org.orekit.rugged.api.RuggedException;
import org.orekit.rugged.api.TileUpdater;
import org.orekit.rugged.raster.Tile.Location;

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

    /** Search optimized cache. */
    private List<TilesStrip> searchCache;

    /** Eviction queue. */
    private T[] evictionQueue;

    /** Index for next tile. */
    private int next;

    /** Simple constructor.
     * @param factory factory for creating empty tiles
     * @param updater updater for retrieving tiles data
     * @param maxTiles maximum number of tiles stored simultaneously in the cache
     */
    public TilesCache(final TileFactory<T> factory, final TileUpdater updater, final int maxTiles) {
        this.factory       = factory;
        this.updater       = updater;
        this.searchCache   = new ArrayList<TilesStrip>();
        @SuppressWarnings("unchecked")
        final T[] array    = (T[]) Array.newInstance(Tile.class, maxTiles + 1);
        this.evictionQueue = array;
        this.next          = 0;
    }

    /** Get the tile covering a ground point.
     * @param latitude ground point latitude
     * @param longitude ground point longitude
     * @return tile covering the ground point
     * @exception RuggedException if newly created tile cannot be updated
     */
    public T getTile(final double latitude, final double longitude)
        throws RuggedException {
        return getStrip(latitude, longitude).getTile(latitude, longitude);
    }

    /** Create a tile covering a ground point.
     * @param latitude latitude of the point
     * @param longitude longitude of the point
     * @return new tile covering the point
     * @exception RuggedException if tile cannot be updated
     */
    private T createTile(final double latitude, final double longitude)
        throws RuggedException {

        // create the tile and retrieve its data
        final T tile = factory.createTile();
        updater.updateTile(latitude, longitude, tile);
        tile.tileUpdateCompleted();

        return tile;

    }

    /** Append newly created tile at the end of the eviction queue.
     * @param tile tile to append to queue
     */
    private void appendToEvictionQueue(final T tile) {

        evictionQueue[next] = tile;
        next                = (next + 1) % evictionQueue.length;

        if (evictionQueue[next] != null) {
            // the cache is full, we need to evict one tile
            // from both the eviction cache and the search cache
            for (final Iterator<TilesStrip> iterator = searchCache.iterator(); iterator.hasNext();) {
                if (iterator.next().removeTile(evictionQueue[next])) {
                    evictionQueue[next] = null;
                    return;
                }
            }
        }

    }

    /** Get a strip covering a ground point.
     * @param latitude ground point latitude
     * @param longitude ground point longitude
     * @return strip covering the ground point
     * @exception RuggedException if tile cannot be updated
     */
    private TilesStrip getStrip(final double latitude, final double longitude)
        throws RuggedException {

        // look for a strip at the specified latitude
        final int index = Collections.binarySearch(searchCache, new BasicLatitudeProvider(latitude),
                                                   new LatitudeComparator());
        if (index >= 0) {
            // rare case, the latitude is an exact maximum latitude for a strip
            return searchCache.get(index);
        } else {

            final int insertionPoint  = -(index + 1);

            if (insertionPoint < searchCache.size()) {
                final TilesStrip strip = searchCache.get(insertionPoint);
                if (strip.covers(latitude)) {
                    // we have found an existing strip
                    return strip;
                }
            }

            // no existing strip covers the specified latitude, we need to create a new one
            final T          tile  = createTile(latitude, longitude);
            final TilesStrip strip = new TilesStrip(tile);
            searchCache.add(insertionPoint, strip);
            appendToEvictionQueue(tile);
            return strip;

        }

    }

    /** Interface for retrieving latitude. */
    private interface LatitudeProvider {

        /** Get latitude.
         * @return latitude
         */
        double getLatitude();

    }

    /** Basic implementation of {@link LatitudeProvider}. */
    private static class BasicLatitudeProvider implements LatitudeProvider {

        /** Latitude. */
        private final double latitude;

        /** Simple constructor.
         * @param latitude latitude
         */
        public BasicLatitudeProvider(final double latitude) {
            this.latitude = latitude;
        }

        /** {@inheritDoc} */
        @Override
        public double getLatitude() {
            return latitude;
        }

    }

    /** Interface for retrieving longitude. */
    private interface LongitudeProvider {

        /** Get longitude.
         * @return longitude
         */
        double getLongitude();

    }

    /** Basic implementation of {@link LongitudeProvider}. */
    private static class BasicLongitudeProvider implements LongitudeProvider {

        /** Longitude. */
        private final double longitude;

        /** Simple constructor.
         * @param longitude longitude
         */
        public BasicLongitudeProvider(final double longitude) {
            this.longitude = longitude;
        }

        /** {@inheritDoc} */
        @Override
        public double getLongitude() {
            return longitude;
        }

    }

    /** Strip of tiles for a given latitude. */
    private class TilesStrip implements LatitudeProvider {

        /** Minimum latitude. */
        private final double minLatitude;

        /** Maximum latitude. */
        private final double maxLatitude;

        /** Tiles list. */
        private final List<TileDecorator> tiles;

        /** Simple constructor.
         * @param tile first tile to insert in the strip
         */
        public TilesStrip(final T tile) {
            minLatitude = tile.getMinimumLatitude();
            maxLatitude = tile.getMaximumLatitude();
            tiles       = new ArrayList<TileDecorator>();
            tiles.add(new TileDecorator(tile));
        }

        /** Check if the strip covers a specified latitude.
         * @param latitude latitude to check
         * @return true if the strip covers the latitude
         */
        public boolean covers(final double latitude) {
            return (minLatitude <= latitude) && (maxLatitude >= latitude);
        }

        /** {@inheritDoc} */
        @Override
        public double getLatitude() {
            return maxLatitude;
        }

        /** Get a tile covering a ground point.
         * @param latitude ground point latitude
         * @param longitude ground point longitude
         * @return strip covering the ground point
         * @exception RuggedException if tile cannot be updated
         */
        public T getTile(final double latitude, final double longitude)
            throws RuggedException {

            // look for a tile at the specified longitude
            final int index = Collections.binarySearch(tiles, new BasicLongitudeProvider(longitude),
                                                       new LongitudeComparator());
            if (index >= 0) {
                // rare case, the longitude is an exact maximum longitude for a tile
                return tiles.get(index).getTile();
            } else {

                final int insertionPoint  = -(index + 1);

                if (insertionPoint < tiles.size()) {
                    final T tile = tiles.get(insertionPoint).getTile();
                    if (tile.getLocation(latitude, longitude) == Location.IN_TILE) {
                        // we have found an existing tile
                        return tile;
                    }
                }

                // no existing tile covers the specified ground point, we need to create a new one
                final T tile = createTile(latitude, longitude);
                tiles.add(insertionPoint, new TileDecorator(tile));
                appendToEvictionQueue(tile);
                return tile;
            }

        }

        /** Remove a tile from the strip.
         * @param tile tile to remove
         * @return true if the tile has been removed
         */
        public boolean removeTile(final Tile tile) {
            for (final Iterator<TileDecorator> iterator = tiles.iterator(); iterator.hasNext();) {
                if (iterator.next().getTile() == tile) {
                    iterator.remove();
                    return true;
                }
            }
            return false;
        }

    }

    /** Decorator for tiles, implementing {@link LongitudeProvider}. */
    private class TileDecorator implements LongitudeProvider {

        /** Underlying tile. */
        private final T tile;

        /** Simple constructor.
         * @param tile tile to decorate
         */
        public TileDecorator(final T tile) {
            this.tile = tile;
        }

        /** Get the underlying tile.
         * @return underlying tile
         */
        public T getTile() {
            return tile;
        }

        /** {@inheritDoc} */
        @Override
        public double getLongitude() {
            return tile.getMaximumLongitude();
        }

    }

    /** Comparator for sorting with respect to latitude. */
    private static class LatitudeComparator implements Comparator<LatitudeProvider> {

        /** {@inheritDoc} */
        @Override
        public int compare(final LatitudeProvider o1, final LatitudeProvider o2) {
            return Double.compare(o1.getLatitude(), o2.getLatitude());
        }

    }

    /** Comparator for sorting with respect to longitude. */
    private static class LongitudeComparator implements Comparator<LongitudeProvider> {

        /** {@inheritDoc} */
        @Override
        public int compare(final LongitudeProvider o1, final LongitudeProvider o2) {
            return Double.compare(o1.getLongitude(), o2.getLongitude());
        }

    }

}
