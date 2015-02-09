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

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.util.FastMath;
import org.apache.commons.math3.util.OpenIntToDoubleHashMap;
import org.orekit.errors.OrekitException;
import org.orekit.frames.Transform;
import org.orekit.rugged.api.AlgorithmId;
import org.orekit.rugged.raster.Tile;
import org.orekit.rugged.utils.ExtendedEllipsoid;
import org.orekit.rugged.utils.SpacecraftToObservedBody;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.DateTimeComponents;
import org.orekit.time.TimeScalesFactory;

/**
 * Dump data class.
 * @author Luc Maisonobe
 */
class Dump {

    /** Dump file. */
    private final PrintWriter writer;

    /** Tiles map. */
    private final List<DumpedTileData> tiles;

    /** Flag for dumped algorithm. */
    private boolean algorithmDumped;

    /** Flag for dumped ellipsoid. */
    private boolean ellipsoidDumped;

    /** Flags for dumped observation transforms. */
    private boolean[] tranformsDumped;

    /** Simple constructor.
     * @param writer writer to the dump file
     */
    public Dump(final PrintWriter writer) {
        this.writer          = writer;
        this.tiles           = new ArrayList<DumpedTileData>();
        this.algorithmDumped = false;
        this.ellipsoidDumped = false;
        this.tranformsDumped = null;
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

    /** Dump algorithm data.
     * @param algorithmId algorithm ID
     */
    public void dumpAlgorithm(final AlgorithmId algorithmId) {
        if (!algorithmDumped) {
            writer.format(Locale.US,
                          "algorithm: %s%n",
                          algorithmId.name());            
            algorithmDumped = true;
        }
    }

    /** Dump algorithm data.
     * @param algorithmId algorithm ID
     * @param specific algorithm specific extra data
     */
    public void dumpAlgorithm(final AlgorithmId algorithmId, final double specific) {
        if (!algorithmDumped) {
            writer.format(Locale.US,
                          "algorithm: %s %22.15e%n",
                          algorithmId.name(), specific);            
            algorithmDumped = true;
        }
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

    /** Dump a direct location computation.
     * @param date computation date
     * @param date date of the location
     * @param position pixel position in spacecraft frame
     * @param los normalized line-of-sight in spacecraft frame
     * @param lightTimeCorrection flag for light time correction
     * @param aberrationOfLightCorrection flag for aberration of light correction
     * @exception RuggedException if date cannot be converted to UTC
     */
    public void dumpDirectLocation(final AbsoluteDate date, final Vector3D position, final Vector3D los,
                                   final boolean lightTimeCorrection, final boolean aberrationOfLightCorrection)
        throws RuggedException {
        writer.format(Locale.US,
                      "direct location: date = %s position = %22.15e %22.15e %22.15e los = %22.15e %22.15e %22.15e ligthTime = %b aberration = %b%n",
                      highAccuracyDate(date),
                      position.getX(), position.getY(), position.getZ(),
                      los.getX(),      los.getY(),      los.getZ(),
                      lightTimeCorrection, aberrationOfLightCorrection);
    }

    /** Dump an observation transform transform.
     * @param scToBody provider for observation
     * @param index index of the transform
     * @param transform transform
     * @exception RuggedException if reference date cannot be converted to UTC
     */
    public void dumpTransform(final SpacecraftToObservedBody scToBody,
                              final int index, final Transform transform)
        throws RuggedException {
        if (tranformsDumped == null) {
            final AbsoluteDate minDate = scToBody.getMinDate();
            final AbsoluteDate maxDate = scToBody.getMaxDate();
            final double       tStep   = scToBody.getTStep();
            final int          n       = 1 + (int) FastMath.rint(maxDate.durationFrom(minDate) / tStep);
            writer.format(Locale.US,
                          "spacecraft to observed body: min date = %s max date = %s tStep = %22.15e inertial frame = %s body frame = %s%n",
                          highAccuracyDate(minDate), highAccuracyDate(maxDate), tStep,
                          scToBody.getInertialFrameName(), scToBody.getBodyFrameName());
            tranformsDumped = new boolean[n];
        }
        if (!tranformsDumped[index]) {
            writer.format(Locale.US,
                          "transform: index = %d r = %22.15e %22.15e %22.15e %22.15e Ω = %22.15e %22.15e %22.15e ΩDot = %22.15e %22.15e %22.15e%n",
                          index,
                          transform.getRotation().getQ0(), transform.getRotation().getQ1(),
                          transform.getRotation().getQ2(), transform.getRotation().getQ3(), 
                          transform.getRotationRate().getX(), transform.getRotationRate().getY(), transform.getRotationRate().getZ(), 
                          transform.getRotationAcceleration().getX(), transform.getRotationAcceleration().getY(), transform.getRotationAcceleration().getZ());
            tranformsDumped[index] = true;
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

    /** Convert a date to string with high accuracy.
     * @param date computation date
     * @return converted date
     * @exception RuggedException if date cannot be converted to UTC
     */
    private String highAccuracyDate(final AbsoluteDate date)
        throws RuggedException {
        try {
            final DateTimeComponents dt = date.getComponents(TimeScalesFactory.getUTC());
            return String.format(Locale.US, "%04d-%02d-%02dT%02d:%02d:%017.14fZ",
                                 dt.getDate().getYear(), dt.getDate().getMonth(), dt.getDate().getDay(),
                                 dt.getTime().getHour(), dt.getTime().getMinute(), dt.getTime().getSecond());
        } catch (OrekitException oe) {
            throw new RuggedException(oe, oe.getSpecifier(), oe.getParts());
        }
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
