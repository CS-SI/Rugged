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

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.util.FastMath;
import org.apache.commons.math3.util.OpenIntToDoubleHashMap;
import org.orekit.errors.OrekitException;
import org.orekit.frames.FactoryManagedFrame;
import org.orekit.frames.Frame;
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
                          "algorithm: %s elevation %22.15e%n",
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
                          "ellipsoid: ae %22.15e f %22.15e frame %s%n",
                          ellipsoid.getA(), ellipsoid.getFlattening(),
                          getKeyOrName(ellipsoid.getBodyFrame()));
            ellipsoidDumped = true;
        }
    }

    /** Dump a direct location computation.
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
                      "direct location: date %s position %22.15e %22.15e %22.15e los %22.15e %22.15e %22.15e lightTime %b aberration %b%n",
                      convertDate(date),
                      position.getX(), position.getY(), position.getZ(),
                      los.getX(),      los.getY(),      los.getZ(),
                      lightTimeCorrection, aberrationOfLightCorrection);
    }

    /** Dump an observation transform transform.
     * @param scToBody provider for observation
     * @param index index of the transform
     * @param bodyToInertial transform from body frame to inertial frame
     * @param scToInertial transfrom from spacecraft frame to inertial frame
     * @exception RuggedException if reference date cannot be converted to UTC
     */
    public void dumpTransform(final SpacecraftToObservedBody scToBody, final int index,
                              final Transform bodyToInertial, final Transform scToInertial)
        throws RuggedException {
        if (tranformsDumped == null) {
            final AbsoluteDate minDate = scToBody.getMinDate();
            final AbsoluteDate maxDate = scToBody.getMaxDate();
            final double       tStep   = scToBody.getTStep();
            final int          n       = (int) FastMath.ceil(maxDate.durationFrom(minDate) / tStep);
            writer.format(Locale.US,
                          "span: minDate %s maxDate %s tStep %22.15e inertialFrame %s%n",
                          convertDate(minDate), convertDate(maxDate), tStep,
                          getKeyOrName(scToBody.getInertialFrame()));
            tranformsDumped = new boolean[n];
        }
        if (!tranformsDumped[index]) {
            writer.format(Locale.US,
                          "transform: index %d body %s spacecraft %s %s%n",
                          index,
                          convertRotation(bodyToInertial.getRotation(), bodyToInertial.getRotationRate(), bodyToInertial.getRotationAcceleration()),
                          convertTranslation(scToInertial.getTranslation(), scToInertial.getVelocity(), scToInertial.getAcceleration()),
                          convertRotation(scToInertial.getRotation(), scToInertial.getRotationRate(), scToInertial.getRotationAcceleration()));
            tranformsDumped[index] = true;
        }
    }

    /** Get a frame key or name.
     * @param frame frame to convert
     * @return frame key or name
     */
    private String getKeyOrName(final Frame frame) {
        if (frame instanceof FactoryManagedFrame) {
            // if possible, use the predefined frames key, as it is easier to parse
            return ((FactoryManagedFrame) frame).getFactoryKey().toString();
        } else {
            // as a fallback, use the full name of the frame
            return frame.getName();
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
    private String convertDate(final AbsoluteDate date)
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

    /** Convert a translation to string.
     * @param translation translation
     * @param velocity linear velocity
     * @param acceleration linear acceleration
     * @return converted rotation
     */
    private String convertTranslation(final Vector3D translation, final Vector3D velocity, final Vector3D acceleration) {
        return String.format(Locale.US,
                             "p %22.15e %22.15e %22.15e v %22.15e %22.15e %22.15e a %22.15e %22.15e %22.15e",
                             translation.getX(),  translation.getY(),  translation.getZ(),
                             velocity.getX(),     velocity.getY(),     velocity.getZ(),
                             acceleration.getX(), acceleration.getY(), acceleration.getZ());
    }

    /** Convert a rotation to string.
     * @param rotation rotation
     * @param rate rate of the rotation
     * @param acceleration angular acceleration
     * @return converted rotation
     */
    private String convertRotation(final Rotation rotation, final Vector3D rate, final Vector3D acceleration) {
        return String.format(Locale.US,
                             "r %22.15e %22.15e %22.15e %22.15e Ω %22.15e %22.15e %22.15e ΩDot %22.15e %22.15e %22.15e",
                             rotation.getQ0(), rotation.getQ1(), rotation.getQ2(), rotation.getQ3(),
                             rate.getX(), rate.getY(), rate.getZ(),
                             acceleration.getX(), acceleration.getY(), acceleration.getZ());
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
                          "DEM tile: %s latMin %22.15e latStep %22.15e latRows %d lonMin %22.15e lonStep %22.15e lonCols %d%n",
                          name,
                          tile.getMinimumLatitude(), tile.getLatitudeStep(), tile.getLatitudeRows(),
                          tile.getMinimumLongitude(), tile.getLongitudeStep(), tile.getLongitudeColumns());
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
                              "DEM cell: %s latIndex %d lonIndex %d elevation %22.15e%n",
                              name, latitudeIndex, longitudeIndex, elevation);
            }
        }

    }

}
