/* Copyright 2013-2015 CS Systèmes d'Information
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
package org.orekit.rugged.errors;

import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.List;
import java.util.Locale;
import java.util.TimeZone;

import org.apache.commons.math3.util.OpenIntToDoubleHashMap;
import org.orekit.rugged.raster.Tile;
import org.orekit.rugged.utils.ExtendedEllipsoid;

/**
 * Dump data class.
 * @author Luc Maisonobe
 */
class Dump {

    /** Dump file. */
    private final PrintWriter writer;

    /** Tiles map. */
    private final List<DumpedTileData> tiles;

    /** Flag for dumped ellipsoid. */
    private boolean ellipsoidDumped;

    /** Simple constructor.
     * @param writer writer to the dump file
     */
    public Dump(final PrintWriter writer) {
        this.writer          = writer;
        this.tiles           = new ArrayList<DumpedTileData>();
        this.ellipsoidDumped = false;
        dumpHeader();
    }

    /** Dump header.
     */
    private void dumpHeader() {
        writer.format(Locale.US,
                      "# Rugged library dump file, created on %1$tFT%1$tTZ%n",
                      Calendar.getInstance(TimeZone.getTimeZone("Etc/UTC"), Locale.US));
        writer.format(Locale.US,
                      "# all units are SI units (m, m/s, rad ...)%n");
    }

    /** Dump some context data.
     * @param tile tile to which the cell belongs
     * @param latitudeIndex latitude index of the cell
     * @param longitudeIndex longitude index of the cell
     * @param elevation elevation of the cell
     */
    public void dumpTileCell(final Tile tile,
                             final int latitudeIndex, final int longitudeIndex,
                             final double elevation) {
        getTileData(tile).setElevation(latitudeIndex, longitudeIndex, elevation);
    }

    /** Dump ellipsoid data.
     * @param ellipsoid ellipsoid to dump
     */
    public void dumpEllipsoid(final ExtendedEllipsoid ellipsoid) {
        if (!ellipsoidDumped) {
            writer.format(Locale.US,
                          "ellipsoid: ae = %22.15e f = %22.15e frame = %s%n",
                          ellipsoid.getA(), ellipsoid.getFlattening(),
                          ellipsoid.getBodyFrame().getName());            
            ellipsoidDumped = true;
        }
    }

    /** Get tile data.
     * @param tile tile to which the cell belongs
     * @return index of the tile
     */
    private DumpedTileData getTileData(final Tile tile) {

        for (final DumpedTileData dumpedTileData : tiles) {
            if (tile == dumpedTileData.getTile()) {
                // the tile is already known
                return dumpedTileData;
            }
        }

        // it is the first time we encounter this tile, we need to dump its data
        final DumpedTileData dumpedTileData = new DumpedTileData("t" + tiles.size(), tile);
        tiles.add(dumpedTileData);
        return dumpedTileData;

    }

    /** Deactivate dump.
     */
    public void deactivate() {
        writer.close();
    }

    /** Local class for handling already dumped tile data. */
    private class DumpedTileData {

        /** Name of the tile. */
        private final String name;

        /** Tile associated with this dump. */
        private final Tile tile;

        /** Dumped elevations. */
        private final OpenIntToDoubleHashMap elevations;

        /** Simple constructor.
         * @param name of the tile
         * @param tile tile associated with this dump
         */
        public DumpedTileData(final String name, final Tile tile) {
            this.name       = name;
            this.tile       = tile;
            this.elevations = new OpenIntToDoubleHashMap();
            writer.format(Locale.US,
                          "DEM tile: %s latMin = %22.15e latStep = %22.15e lonMin = %22.15e lonStep = %22.15e%n",
                          name,
                          tile.getMinimumLatitude(), tile.getLatitudeStep(),
                          tile.getMinimumLongitude(), tile.getLongitudeStep());
        }

        /** Get tile associated with this dump.
         * @return tile associated with this dump
         */
        public Tile getTile() {
            return tile;
        }

        /** Set an elevation.
         * @param latitudeIndex latitude index of the cell
         * @param longitudeIndex longitude index of the cell
         * @param elevation elevation of the cell
         */
        public void setElevation(final int latitudeIndex, final int longitudeIndex, final double elevation) {
            final int key = latitudeIndex * tile.getLongitudeColumns() + longitudeIndex;
            if (!elevations.containsKey(key)) {
                // new cell
                elevations.put(key, elevation);
                writer.format(Locale.US,
                              "DEM cell: %s latIndex = %d lonIndex = %d elevation = %22.15e%n",
                              name, latitudeIndex, longitudeIndex, elevation);
            }
        }

    }

}
