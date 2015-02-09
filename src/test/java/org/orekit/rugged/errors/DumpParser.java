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


import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.apache.commons.math3.exception.util.DummyLocalizable;
import org.apache.commons.math3.exception.util.LocalizedFormats;
import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.util.FastMath;
import org.orekit.bodies.OneAxisEllipsoid;
import org.orekit.errors.OrekitException;
import org.orekit.frames.Frame;
import org.orekit.frames.FramesFactory;
import org.orekit.frames.Predefined;
import org.orekit.frames.Transform;
import org.orekit.rugged.api.AlgorithmId;
import org.orekit.rugged.api.Rugged;
import org.orekit.rugged.api.RuggedBuilder;
import org.orekit.rugged.raster.TileUpdater;
import org.orekit.rugged.raster.UpdatableTile;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeScalesFactory;

/** Parser for Rugged debug dumps.
 * @author Luc Maisonobe
 * @see DumpManager
 * @see Dump
 */
public class DumpParser {

    /** Constant elevation for constant elevation algorithm. */
    private double constantElevation;

    /** Algorithm identifier. */
    private AlgorithmId algorithmId;

    /** Ellipsoid. */
    private OneAxisEllipsoid ellipsoid;

    /** Tiles map. */
    private final List<ParsedTile> tiles;

    /** Interpolator min date. */
    private AbsoluteDate minDate;

    /** Interpolator max date. */
    private AbsoluteDate maxDate;

    /** Interpolator step. */
    private double tStep;

    /** Inertial frame. */
    private Frame inertialFrame;

    /** Transforms sample from observed body frame to inertial frame. */
    private Transform[] bodyToInertial;

    /** Transforms sample from spacecraft frame to inertial frame. */
    private Transform[] scToInertial;

    /** Flag for light time correction. */
    private boolean lightTimeCorrection;

    /** Flag for aberration of light correction. */
    private boolean aberrationOfLightCorrection;

    /** Dumped calls. */
    private final List<DumpedCall> calls;

    /** 
    /** Simple constructor.
     */
    public DumpParser() {
        tiles = new ArrayList<ParsedTile>();
        calls = new ArrayList<DumpedCall>();
    }

    /** Parse a dump file.
     * @param file dump file to parse
     * @exception RuggedException if file cannot be parsed
     */
    public void parse(final File file) throws RuggedException {
        try {
            BufferedReader reader = new BufferedReader(new FileReader(file));
            int l = 0;
            for (String line = reader.readLine(); line != null; line = reader.readLine()) {
                ++l;
                LineParser.parse(l, file, line, this);
            }
            reader.close();
        } catch (IOException ioe) {
            throw new RuggedException(ioe, LocalizedFormats.SIMPLE_MESSAGE, ioe.getLocalizedMessage());
        }
    }

    /** Create a Rugged instance from parsed data.
     * @return rugged instance
     * @exception RuggedException if some data are inconsistent or incomplete
     */
    public Rugged createRugged() throws RuggedException {

        final RuggedBuilder builder = new RuggedBuilder();

        builder.setAlgorithm(algorithmId);
        if (algorithmId == AlgorithmId.CONSTANT_ELEVATION_OVER_ELLIPSOID) {
            builder.setConstantElevation(constantElevation);
        } else if (algorithmId != AlgorithmId.IGNORE_DEM_USE_ELLIPSOID) {
            builder.setDigitalElevationModel(new ParsedTilesUpdater(), 8);
        }

        builder.setLightTimeCorrection(lightTimeCorrection);
        builder.setAberrationOfLightCorrection(aberrationOfLightCorrection);

        builder.setEllipsoid(ellipsoid);

        // TODO: set up a SpacecraftToObservedBody instance, serialize it and load it back into the Rugged instance

        return builder.build();

    }

    /** Execute all dumped calls.
     * @param rugged Rugged instance on which calls will be performed
     * @exception RuggedException if a call fails
     */
    public void execute(final Rugged rugged) throws RuggedException {
        for (final DumpedCall call : calls) {
            call.execute(rugged);
        }
    }

    /** Line parsers. */
    private enum LineParser {

        ALGORITHM() {

            /** {@inheritDoc} */
            @Override
            public void parse(final int l, final File file, final String line, final String[] fields, final DumpParser global)
                throws RuggedException {
                try {
                    if (fields.length < 1) {
                        throw error(l, file, line);
                    }
                    global.algorithmId = AlgorithmId.valueOf(fields[0]);
                    if (global.algorithmId == AlgorithmId.CONSTANT_ELEVATION_OVER_ELLIPSOID) {
                        if (fields.length < 3 || !fields[1].equals("elevation")) {
                            throw error(l, file, line);
                        }
                        global.constantElevation = Double.parseDouble(fields[2]);
                    }
                } catch (IllegalArgumentException iae) {
                    throw error(l, file, line);
                }
            }

        },

        ELLIPSOID() {

            /** {@inheritDoc} */
            @Override
            public void parse(final int l, final File file, final String line, final String[] fields, final DumpParser global)
                throws RuggedException {
                if (fields.length < 6 || !fields[0].equals("ae") || !fields[2].equals("f") || !fields[4].equals("frame")) {
                    throw error(l, file, line);
                }
                final double ae   = Double.parseDouble(fields[1]);
                final double f    = Double.parseDouble(fields[2]);
                final Frame  bodyFrame;
                try {
                    bodyFrame = FramesFactory.getFrame(Predefined.valueOf(fields[5]));
                } catch (OrekitException oe) {
                    throw new RuggedException(oe, oe.getSpecifier(), oe.getParts());
                } catch (IllegalArgumentException iae) {
                    throw error(l, file, line);
                }
                global.ellipsoid = new OneAxisEllipsoid(ae, f, bodyFrame);
            }

        },

        DIRECT_LOCATION() {

            /** {@inheritDoc} */
            @Override
            public void parse(final int l, final File file, final String line, final String[] fields, final DumpParser global)
                throws RuggedException {
                try {
                    if (fields.length < 14 ||
                        !fields[0].equals("date") ||
                        !fields[2].equals("position") || !fields[6].equals("los") ||
                        !fields[10].equals("lightTime") || !fields[12].equals("aberration")) {
                        throw error(l, file, line);
                    }
                    final AbsoluteDate date = new AbsoluteDate(fields[1], TimeScalesFactory.getUTC());
                    final Vector3D position = new Vector3D(Double.parseDouble(fields[3]),
                                                           Double.parseDouble(fields[4]),
                                                           Double.parseDouble(fields[5]));
                    final Vector3D los      = new Vector3D(Double.parseDouble(fields[7]),
                                                           Double.parseDouble(fields[8]),
                                                           Double.parseDouble(fields[9]));
                    if (global.calls.isEmpty()) {
                        global.lightTimeCorrection         = Boolean.parseBoolean(fields[11]);
                        global.aberrationOfLightCorrection = Boolean.parseBoolean(fields[13]);
                    } else {
                        if (global.lightTimeCorrection != Boolean.parseBoolean(fields[11])) {
                            throw new RuggedException(new DummyLocalizable("light time correction redefined, line {0}, file {1}: {2}"),
                                                      l, file.getAbsolutePath(), line);
                        }
                        if (global.aberrationOfLightCorrection != Boolean.parseBoolean(fields[13])) {
                            throw new RuggedException(new DummyLocalizable("aberration of light scorrection redefined, line {0}, file {1}: {2}"),
                                                      l, file.getAbsolutePath(), line);
                        }
                    }
                    global.calls.add(new DumpedCall() {

                        /** {@inheritDoc} */
                        @Override
                        public void execute(Rugged rugged) throws RuggedException {
                            rugged.directLocation(date, position, los);
                        }

                    });
                } catch (OrekitException oe) {
                    throw new RuggedException(oe, oe.getSpecifier(), oe.getParts());
                }
            }

        },

        SPAN() {

            /** {@inheritDoc} */
            @Override
            public void parse(final int l, final File file, final String line, final String[] fields, final DumpParser global)
                throws RuggedException {
                try {
                    if (fields.length < 8 ||
                        !fields[0].equals("minDate") || !fields[2].equals("madDate") ||
                        !fields[4].equals("tStep")   || !fields[6].equals("inertialFrame")) {
                        throw error(l, file, line);
                    }
                    global.minDate        = new AbsoluteDate(fields[1], TimeScalesFactory.getUTC());
                    global.maxDate        = new AbsoluteDate(fields[3], TimeScalesFactory.getUTC());
                    global.tStep          = Double.parseDouble(fields[5]);
                    final int n           = (int) FastMath.ceil(global.maxDate.durationFrom(global.minDate) / global.tStep);
                    global.bodyToInertial = new Transform[n];
                    global.scToInertial   = new Transform[n];
                    try {
                        global.inertialFrame = FramesFactory.getFrame(Predefined.valueOf(fields[7]));
                    } catch (IllegalArgumentException iae) {
                        throw error(l, file, line);
                    }
                } catch (OrekitException oe) {
                    throw new RuggedException(oe, oe.getSpecifier(), oe.getParts());
                }
            }

        },

        TRANSFORM() {

            /** {@inheritDoc} */
            @Override
            public void parse(final int l, final File file, final String line, final String[] fields, final DumpParser global)
                throws RuggedException {
                if (fields.length < 42 ||
                    !fields[0].equals("index") ||
                    !fields[2].equals("body")  ||
                    !fields[3].equals("r")     || !fields[8].equals("Ω")    || !fields[12].equals("ΩDot") ||
                    !fields[16].equals("spacecraft") ||
                    !fields[17].equals("p")    || !fields[21].equals("v")   || !fields[25].equals("a") ||
                    !fields[29].equals("r")    || !fields[34].equals("Ω")   || !fields[38].equals("ΩDot")) {
                    throw error(l, file, line);
                }
                final int i   = Integer.parseInt(fields[1]);
                final AbsoluteDate date = global.minDate.shiftedBy(i * global.tStep);
                global.bodyToInertial[i] =
                        new Transform(date,
                                      new Rotation(Double.parseDouble(fields[4]),
                                                   Double.parseDouble(fields[5]),
                                                   Double.parseDouble(fields[6]),
                                                   Double.parseDouble(fields[7]),
                                                   false),
                                      new Vector3D(Double.parseDouble(fields[9]),
                                                   Double.parseDouble(fields[10]),
                                                   Double.parseDouble(fields[11])),
                                      new Vector3D(Double.parseDouble(fields[13]),
                                                   Double.parseDouble(fields[14]),
                                                   Double.parseDouble(fields[15])));
                global.scToInertial[i] =
                        new Transform(date,
                                      new Transform(date,
                                                    new Vector3D(Double.parseDouble(fields[18]),
                                                                 Double.parseDouble(fields[19]),
                                                                 Double.parseDouble(fields[20])),
                                                    new Vector3D(Double.parseDouble(fields[22]),
                                                                 Double.parseDouble(fields[23]),
                                                                 Double.parseDouble(fields[24])),
                                                    new Vector3D(Double.parseDouble(fields[26]),
                                                                 Double.parseDouble(fields[27]),
                                                                 Double.parseDouble(fields[28]))),
                                      new Transform(date,
                                                    new Rotation(Double.parseDouble(fields[30]),
                                                                 Double.parseDouble(fields[31]),
                                                                 Double.parseDouble(fields[32]),
                                                                 Double.parseDouble(fields[33]),
                                                                 false),
                                                    new Vector3D(Double.parseDouble(fields[35]),
                                                                 Double.parseDouble(fields[36]),
                                                                 Double.parseDouble(fields[37])),
                                                    new Vector3D(Double.parseDouble(fields[39]),
                                                                 Double.parseDouble(fields[40]),
                                                                 Double.parseDouble(fields[41]))));
            }

        },

        DEM_TILE() {

            /** {@inheritDoc} */
            @Override
            public void parse(final int l, final File file, final String line, final String[] fields, final DumpParser global)
                throws RuggedException {
                if (fields.length < 13 ||
                        !fields[1].equals("latMin") || !fields[3].equals("latStep") || !fields[5].equals("latRows") ||
                        !fields[7].equals("lonMin") || !fields[9].equals("lonStep") || !fields[11].equals("lonRows")) {
                    throw error(l, file, line);
                }
                final String name             = fields[0];
                final double minLatitude      = Double.parseDouble(fields[2]);
                final double latitudeStep     = Double.parseDouble(fields[4]);
                final int    latitudeRows     = Integer.parseInt(fields[6]);
                final double minLongitude     = Double.parseDouble(fields[8]);
                final double longitudeStep    = Double.parseDouble(fields[10]);
                final int    longitudeColumns = Integer.parseInt(fields[12]);
                for (final ParsedTile tile : global.tiles) {
                    if (tile.name.equals(name)) {
                        throw new RuggedException(new DummyLocalizable("tile {0} already defined, line {1}, file {2}: {3}"),
                                                  name, l, file.getAbsolutePath(), line);
                    }
                }
                global.tiles.add(new ParsedTile(name,
                                                minLatitude, latitudeStep, latitudeRows,
                                                minLongitude, longitudeStep, longitudeColumns));
            }

        },

        DEM_CELL() {

            /** {@inheritDoc} */
            @Override
            public void parse(final int l, final File file, final String line, final String[] fields, final DumpParser global)
                throws RuggedException {
                if (fields.length < 7 ||
                        !fields[1].equals("latIndex") || !fields[3].equals("lonIndex") || !fields[5].equals("elevation")) {
                    throw error(l, file, line);
                }
                final String name      = fields[0];
                final int    latIndex  = Integer.parseInt(fields[2]);
                final int    lonIndex  = Integer.parseInt(fields[4]);
                final double elevation = Double.parseDouble(fields[6]);
                for (final ParsedTile tile : global.tiles) {
                    if (tile.name.equals(name)) {
                        tile.elevations[latIndex * tile.longitudeColumns + lonIndex] = elevation;
                    }
                }
                throw new RuggedException(new DummyLocalizable("unknown tile {0}, line {1}, file {2}: {3}"),
                                          name, l, file.getAbsolutePath(), line);
            }

        };

        /** Parse a line.
         * @param l line number
         * @param file dump file
         * @param line line to parse
         * @param global global parser to store parsed data
         * @exception RuggedException if line is not supported
         */
        public static void parse(final int l, final File file, final String line, final DumpParser global)
            throws RuggedException {

            final String trimmed = line.trim();
            if (trimmed.length() == 0 || trimmed.startsWith("#")) {
                return;
            }

            int colon = line.indexOf(':');
            if (colon > 0) {
                final String parsedKey = line.substring(0, colon).trim().replaceAll(" ", "_").toUpperCase();
                try {
                    final LineParser parser = LineParser.valueOf(parsedKey);
                    parser.parse(colon, file, line, line.substring(colon + 1).trim().split("\\s+"), global);
                } catch (IllegalArgumentException iae) {
                    throw error(l, file, line);
                }

            }

        }

        /** Parse a line
         * @param l line number
         * @param file dump file
         * @param line complete line
         * @param fields data fields from the line
         * @param global global parser to store parsed data
         * @execption RuggedException if line cannot be parsed
         */
        public abstract void parse(final int l, final File file, final String line, final String[] fields,
                                   final DumpParser global)
            throws RuggedException;

    }

    /** Trigger a parse error.
     * @param l line number
     * @param file dump file
     * @param line line that cannot be parsed
     * @return built exception
     */
    private static RuggedException error(final int l, final File file, final String line) {
        return new RuggedException(new DummyLocalizable("cannot parse line {0}, file {1}: {2}"),
                                   l, file.getAbsolutePath(), line);
    }

    /** Local class for handling already dumped tile data. */
    private static class ParsedTile {

        /** Name of the tile. */
        private final String name;

        /** Minimum latitude. */
        private final double minLatitude;

        /** Step in latitude (size of one raster element). */
        private final double latitudeStep;

        /** Minimum longitude. */
        private final double minLongitude;

        /** Step in longitude (size of one raster element). */
        private final double longitudeStep;

        /** Number of latitude rows. */
        private int latitudeRows;

        /** Number of longitude columns. */
        private int longitudeColumns;

        /** Raster elevation data. */
        private final double[] elevations;

        /** Simple constructor.
         * @param name of the tile
         */
        public ParsedTile(final String name,
                          final double minLatitude, final double latitudeStep, final int latitudeRows,
                          final double minLongitude, final double longitudeStep, final int longitudeColumns) {
            this.name             = name;
            this.minLatitude      = minLatitude;
            this.latitudeStep     = latitudeStep;
            this.minLongitude     = minLongitude;
            this.longitudeStep    = longitudeStep;
            this.latitudeRows     = latitudeRows;
            this.longitudeColumns = longitudeColumns;
            this.elevations       = new double[latitudeRows * longitudeColumns];
        }

    }

    /** Local interface for dumped calls. */
    private static interface DumpedCall {
        /** Execute a call.
         * @param rugged Rugged instance on which called should be performed
         * @exception RuggedException if the call fails
         */
        void execute(Rugged rugged) throws RuggedException;
    }

    /** Local tile updater for parsed tiles. */
    private class ParsedTilesUpdater implements TileUpdater {

        /** {@inheritDoc} */
        @Override
        public void updateTile(final double latitude, final double longitude, final UpdatableTile tile)
            throws RuggedException {
            // TODO: implement method, based on the parsed tiles
        }
        
    }

}
