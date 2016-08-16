/* Copyright 2013-2016 CS Systèmes d'Information
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

import org.hipparchus.analysis.differentiation.DerivativeStructure;
import org.hipparchus.exception.LocalizedCoreFormats;
import org.hipparchus.geometry.euclidean.threed.FieldVector3D;
import org.hipparchus.geometry.euclidean.threed.Rotation;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.util.FastMath;
import org.hipparchus.util.OpenIntToDoubleHashMap;
import org.hipparchus.util.Pair;
import java.io.BufferedReader;
import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.ObjectOutputStream;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.NavigableMap;
import java.util.Optional;
import java.util.TreeMap;
import java.util.stream.Stream;

import org.orekit.bodies.GeodeticPoint;
import org.orekit.bodies.OneAxisEllipsoid;
import org.orekit.errors.OrekitException;
import org.orekit.frames.Frame;
import org.orekit.frames.FramesFactory;
import org.orekit.frames.Predefined;
import org.orekit.frames.Transform;
import org.orekit.rugged.api.AlgorithmId;
import org.orekit.rugged.api.Rugged;
import org.orekit.rugged.api.RuggedBuilder;
import org.orekit.rugged.linesensor.LineDatation;
import org.orekit.rugged.linesensor.LineSensor;
import org.orekit.rugged.linesensor.SensorMeanPlaneCrossing;
import org.orekit.rugged.linesensor.SensorMeanPlaneCrossing.CrossingResult;
import org.orekit.rugged.linesensor.SensorPixel;
import org.orekit.rugged.los.TimeDependentLOS;
import org.orekit.rugged.raster.TileUpdater;
import org.orekit.rugged.raster.UpdatableTile;
import org.orekit.rugged.utils.ExtendedParameterDriver;
import org.orekit.rugged.utils.SpacecraftToObservedBody;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeScalesFactory;

/** Replayer for Rugged debug dumps.
 * @author Luc Maisonobe
 * @see DumpManager
 * @see Dump
 */
public class DumpReplayer {

    /** Comment start marker. */
    private static final String COMMENT_START = "#";

    /** Keyword for latitude fields. */
    private static final String LATITUDE = "latitude";

    /** Keyword for longitude fields. */
    private static final String LONGITUDE = "longitude";

    /** Keyword for elevation fields. */
    private static final String ELEVATION = "elevation";

    /** Keyword for ellipsoid equatorial radius fields. */
    private static final String AE = "ae";

    /** Keyword for ellipsoid flattening fields. */
    private static final String F = "f";

    /** Keyword for frame fields. */
    private static final String FRAME = "frame";

    /** Keyword for date fields. */
    private static final String DATE = "date";

    /** Keyword for sensor position fields. */
    private static final String POSITION = "position";

    /** Keyword for sensor line-of-sight fields. */
    private static final String LOS = "los";

    /** Keyword for light-time correction fields. */
    private static final String LIGHT_TIME = "lightTime";

    /** Keyword for aberration of light correction fields. */
    private static final String ABERRATION = "aberration";

    /** Keyword for min date fields. */
    private static final String MIN_DATE = "minDate";

    /** Keyword for max date fields. */
    private static final String MAX_DATE = "maxDate";

    /** Keyword for time step fields. */
    private static final String T_STEP = "tStep";

    /** Keyword for overshoot tolerance fields. */
    private static final String TOLERANCE = "tolerance";

    /** Keyword for inertial frames fields. */
    private static final String INERTIAL_FRAME = "inertialFrame";

    /** Keyword for observation transform index fields. */
    private static final String INDEX = "index";

    /** Keyword for body meta-fields. */
    private static final String BODY = "body";

    /** Keyword for rotation fields. */
    private static final String R = "r";

    /** Keyword for rotation rate fields. */
    private static final String OMEGA = "Ω";

    /** Keyword for rotation acceleration fields. */
    private static final String OMEGA_DOT = "ΩDot";

    /** Keyword for spacecraft meta-fields. */
    private static final String SPACECRAFT = "spacecraft";

    /** Keyword for position fields. */
    private static final String P = "p";

    /** Keyword for velocity fields. */
    private static final String V = "v";

    /** Keyword for acceleration fields. */
    private static final String A = "a";

    /** Keyword for minimum latitude fields. */
    private static final String LAT_MIN = "latMin";

    /** Keyword for latitude step fields. */
    private static final String LAT_STEP = "latStep";

    /** Keyword for latitude rows fields. */
    private static final String LAT_ROWS = "latRows";

    /** Keyword for minimum longitude fields. */
    private static final String LON_MIN = "lonMin";

    /** Keyword for longitude step fields. */
    private static final String LON_STEP = "lonStep";

    /** Keyword for longitude columns fields. */
    private static final String LON_COLS = "lonCols";

    /** Keyword for latitude index fields. */
    private static final String LAT_INDEX = "latIndex";

    /** Keyword for longitude index fields. */
    private static final String LON_INDEX = "lonIndex";

    /** Keyword for sensor name. */
    private static final String SENSOR_NAME = "sensorName";

    /** Keyword for min line. */
    private static final String MIN_LINE = "minLine";

    /** Keyword for max line. */
    private static final String MAX_LINE = "maxLine";

    /** Keyword for line number. */
    private static final String LINE_NUMBER = "lineNumber";

    /** Keyword for number of pixels. */
    private static final String NB_PIXELS = "nbPixels";

    /** Keyword for pixel number. */
    private static final String PIXEL_NUMBER = "pixelNumber";

    /** Keyword for max number of evaluations. */
    private static final String MAX_EVAL = "maxEval";

    /** Keyword for accuracy. */
    private static final String ACCURACY = "accuracy";

    /** Keyword for normal. */
    private static final String NORMAL = "normal";

    /** Keyword for rate. */
    private static final String RATE = "rate";

    /** Keyword for cached results. */
    private static final String CACHED_RESULTS = "cachedResults";

    /** Keyword for target. */
    private static final String TARGET = "target";

    /** Keyword for target direction. */
    private static final String TARGET_DIRECTION = "targetDirection";

    /** Constant elevation for constant elevation algorithm. */
    private double constantElevation;

    /** Algorithm identifier. */
    private AlgorithmId algorithmId;

    /** Ellipsoid. */
    private OneAxisEllipsoid ellipsoid;

    /** Tiles list. */
    private final List<ParsedTile> tiles;

    /** Sensors list. */
    private final List<ParsedSensor> sensors;

    /** Interpolator min date. */
    private AbsoluteDate minDate;

    /** Interpolator max date. */
    private AbsoluteDate maxDate;

    /** Interpolator step. */
    private double tStep;

    /** Interpolator overshoot tolerance. */
    private double tolerance;

    /** Inertial frame. */
    private Frame inertialFrame;

    /** Transforms sample from observed body frame to inertial frame. */
    private NavigableMap<Integer, Transform> bodyToInertial;

    /** Transforms sample from spacecraft frame to inertial frame. */
    private NavigableMap<Integer, Transform> scToInertial;

    /** Flag for light time correction. */
    private boolean lightTimeCorrection;

    /** Flag for aberration of light correction. */
    private boolean aberrationOfLightCorrection;

    /** Dumped calls. */
    private final List<DumpedCall> calls;

    /** Simple constructor.
     */
    public DumpReplayer() {
        tiles   = new ArrayList<ParsedTile>();
        sensors = new ArrayList<ParsedSensor>();
        calls   = new ArrayList<DumpedCall>();
    }

    /** Parse a dump file.
     * @param file dump file to parse
     * @exception RuggedException if file cannot be parsed
     */
    public void parse(final File file) throws RuggedException {
        try {
            final BufferedReader reader =
                    new BufferedReader(new InputStreamReader(new FileInputStream(file), "UTF-8"));
            int l = 0;
            for (String line = reader.readLine(); line != null; line = reader.readLine()) {
                LineParser.parse(++l, file, line, this);
            }
            reader.close();
        } catch (IOException ioe) {
            throw new RuggedException(ioe, LocalizedCoreFormats.SIMPLE_MESSAGE, ioe.getLocalizedMessage());
        }
    }

    /** Create a Rugged instance from parsed data.
     * @return rugged instance
     * @exception RuggedException if some data are inconsistent or incomplete
     */
    public Rugged createRugged() throws RuggedException {
        try {
            final RuggedBuilder builder = new RuggedBuilder();

            if (algorithmId == null) {
                algorithmId = AlgorithmId.IGNORE_DEM_USE_ELLIPSOID;
            }
            builder.setAlgorithm(algorithmId);
            if (algorithmId == AlgorithmId.CONSTANT_ELEVATION_OVER_ELLIPSOID) {
                builder.setConstantElevation(constantElevation);
            } else if (algorithmId != AlgorithmId.IGNORE_DEM_USE_ELLIPSOID) {
                builder.setDigitalElevationModel(new TileUpdater() {

                    /** {@inheritDoc} */
                    @Override
                    public void updateTile(final double latitude, final double longitude, final UpdatableTile tile)
                        throws RuggedException {
                        for (final ParsedTile parsedTile : tiles) {
                            if (parsedTile.isInterpolable(latitude, longitude)) {
                                parsedTile.updateTile(tile);
                                return;
                            }
                        }
                        throw new RuggedException(RuggedMessages.NO_DEM_DATA,
                                                  FastMath.toDegrees(latitude), FastMath.toDegrees(longitude));
                    }

                }, 8);
            }

            builder.setLightTimeCorrection(lightTimeCorrection);
            builder.setAberrationOfLightCorrection(aberrationOfLightCorrection);

            builder.setEllipsoid(ellipsoid);

            // build missing transforms by extrapolating the parsed ones
            final int n = (int) FastMath.ceil(maxDate.durationFrom(minDate) / tStep);
            final List<Transform> b2iList = new ArrayList<Transform>(n);
            final List<Transform> s2iList = new ArrayList<Transform>(n);
            for (int i = 0; i < n; ++i) {
                if (bodyToInertial.containsKey(i)) {
                    // the i-th transform was dumped
                    b2iList.add(bodyToInertial.get(i));
                    s2iList.add(scToInertial.get(i));
                } else {
                    // the i-th transformed was not dumped, we have to extrapolate it
                    final Map.Entry<Integer, Transform> lower  = bodyToInertial.lowerEntry(i);
                    final Map.Entry<Integer, Transform> higher = bodyToInertial.higherEntry(i);
                    final int closest;
                    if (lower == null) {
                        closest = higher.getKey();
                    } else if (higher == null) {
                        closest = lower.getKey();
                    } else {
                        closest = (i - lower.getKey() <= higher.getKey() - i) ? lower.getKey() : higher.getKey();
                    }
                    b2iList.add(bodyToInertial.get(closest).shiftedBy((i - closest) * tStep));
                    s2iList.add(scToInertial.get(closest).shiftedBy((i - closest) * tStep));
                }
            }

            // we use Rugged transforms reloading mechanism to ensure the spacecraft
            // to body transforms will be the same as the ones dumped
            final SpacecraftToObservedBody scToBody =
                    new SpacecraftToObservedBody(inertialFrame, ellipsoid.getBodyFrame(),
                                                 minDate, maxDate, tStep, tolerance,
                                                 b2iList, s2iList);
            final ByteArrayOutputStream bos = new ByteArrayOutputStream();
            new ObjectOutputStream(bos).writeObject(scToBody);
            final ByteArrayInputStream  bis = new ByteArrayInputStream(bos.toByteArray());
            builder.setTrajectoryAndTimeSpan(bis);

            final List<SensorMeanPlaneCrossing> planeCrossings = new ArrayList<SensorMeanPlaneCrossing>();
            for (final ParsedSensor parsedSensor : sensors) {
                final LineSensor sensor = new LineSensor(parsedSensor.name,
                                                         parsedSensor,
                                                         parsedSensor.position,
                                                         parsedSensor);
                if (parsedSensor.meanPlane != null) {
                    planeCrossings.add(new SensorMeanPlaneCrossing(sensor, scToBody,
                                                                   parsedSensor.meanPlane.minLine,
                                                                   parsedSensor.meanPlane.maxLine,
                                                                   lightTimeCorrection, aberrationOfLightCorrection,
                                                                   parsedSensor.meanPlane.maxEval,
                                                                   parsedSensor.meanPlane.accuracy,
                                                                   parsedSensor.meanPlane.normal,
                                                                   Arrays.stream(parsedSensor.meanPlane.cachedResults)));
                }
                builder.addLineSensor(sensor);
            }

            final Rugged rugged = builder.build();

            final Method setPlaneCrossing = Rugged.class.getDeclaredMethod("setPlaneCrossing",
                                                                           SensorMeanPlaneCrossing.class);
            setPlaneCrossing.setAccessible(true);
            for (final SensorMeanPlaneCrossing planeCrossing : planeCrossings) {
                setPlaneCrossing.invoke(rugged, planeCrossing);
            }

            return rugged;

        } catch (IOException ioe) {
            throw new RuggedException(ioe, LocalizedCoreFormats.SIMPLE_MESSAGE, ioe.getLocalizedMessage());
        } catch (SecurityException e) {
            // this should never happen
            throw RuggedException.createInternalError(e);
        } catch (NoSuchMethodException e) {
            // this should never happen
            throw RuggedException.createInternalError(e);
        } catch (IllegalArgumentException e) {
            // this should never happen
            throw RuggedException.createInternalError(e);
        } catch (IllegalAccessException e) {
            // this should never happen
            throw RuggedException.createInternalError(e);
        } catch (InvocationTargetException e) {
            // this should never happen
            throw RuggedException.createInternalError(e);
        }

    }

    /** Get a sensor by name.
     * @param name sensor name
     * @return parsed sensor
     */
    private ParsedSensor getSensor(final String name) {
        for (final ParsedSensor sensor : sensors) {
            if (sensor.name.equals(name)) {
                return sensor;
            }
        }
        final ParsedSensor sensor = new ParsedSensor(name);
        sensors.add(sensor);
        return sensor;
    }

    /** Execute all dumped calls.
     * <p>
     * The dumped calls correspond to computation methods like direct or inverse
     * location.
     * </p>
     * @param rugged Rugged instance on which calls will be performed
     * @return results of all dumped calls
     * @exception RuggedException if a call fails
     */
    public Result[] execute(final Rugged rugged) throws RuggedException {
        final Result[] results = new Result[calls.size()];
        for (int i = 0; i < calls.size(); ++i) {
            results[i] = new Result(calls.get(i).expected,
                                    calls.get(i).execute(rugged));
        }
        return results;
    }

    /** Container for replay results. */
    public static class Result {

        /** Expected result. */
        private final Object expected;

        /** Replayed result. */
        private final Object replayed;

        /** Simple constructor.
         * @param expected expected result
         * @param replayed replayed result
         */
        private Result(final Object expected, final Object replayed) {
            this.expected = expected;
            this.replayed = replayed;
        }

        /** Get the expected result.
         * @return expected result
         */
        public Object getExpected() {
            return expected;
        }

        /** Get the replayed result.
         * @return replayed result
         */
        public Object getReplayed() {
            return replayed;
        }

    }

    /** Line parsers. */
    private enum LineParser {

        /** Parser for algorithm dump lines. */
        ALGORITHM() {

            /** {@inheritDoc} */
            @Override
            public void parse(final int l, final File file, final String line, final String[] fields, final DumpReplayer global)
                throws RuggedException {
                try {
                    if (fields.length < 1) {
                        throw new RuggedException(RuggedMessages.CANNOT_PARSE_LINE, l, file, line);
                    }
                    global.algorithmId = AlgorithmId.valueOf(fields[0]);
                    if (global.algorithmId == AlgorithmId.CONSTANT_ELEVATION_OVER_ELLIPSOID) {
                        if (fields.length < 3 || !fields[1].equals(ELEVATION)) {
                            throw new RuggedException(RuggedMessages.CANNOT_PARSE_LINE, l, file, line);
                        }
                        global.constantElevation = Double.parseDouble(fields[2]);
                    }
                } catch (IllegalArgumentException iae) {
                    throw new RuggedException(RuggedMessages.CANNOT_PARSE_LINE, l, file, line);
                }
            }

        },

        /** Parser for ellipsoid dump lines. */
        ELLIPSOID() {

            /** {@inheritDoc} */
            @Override
            public void parse(final int l, final File file, final String line, final String[] fields, final DumpReplayer global)
                throws RuggedException {
                if (fields.length < 6 || !fields[0].equals(AE) || !fields[2].equals(F) || !fields[4].equals(FRAME)) {
                    throw new RuggedException(RuggedMessages.CANNOT_PARSE_LINE, l, file, line);
                }
                final double ae   = Double.parseDouble(fields[1]);
                final double f    = Double.parseDouble(fields[3]);
                final Frame  bodyFrame;
                try {
                    bodyFrame = FramesFactory.getFrame(Predefined.valueOf(fields[5]));
                } catch (OrekitException oe) {
                    throw new RuggedException(oe, oe.getSpecifier(), oe.getParts());
                } catch (IllegalArgumentException iae) {
                    throw new RuggedException(RuggedMessages.CANNOT_PARSE_LINE, l, file, line);
                }
                global.ellipsoid = new OneAxisEllipsoid(ae, f, bodyFrame);
            }

        },

        /** Parser for direct location calls dump lines. */
        DIRECT_LOCATION() {

            /** {@inheritDoc} */
            @Override
            public void parse(final int l, final File file, final String line, final String[] fields, final DumpReplayer global)
                throws RuggedException {
                try {
                    if (fields.length < 14 ||
                        !fields[0].equals(DATE) ||
                        !fields[2].equals(POSITION) || !fields[6].equals(LOS) ||
                        !fields[10].equals(LIGHT_TIME) || !fields[12].equals(ABERRATION)) {
                        throw new RuggedException(RuggedMessages.CANNOT_PARSE_LINE, l, file, line);
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
                            throw new RuggedException(RuggedMessages.LIGHT_TIME_CORRECTION_REDEFINED,
                                                      l, file.getAbsolutePath(), line);
                        }
                        if (global.aberrationOfLightCorrection != Boolean.parseBoolean(fields[13])) {
                            throw new RuggedException(RuggedMessages.ABERRATION_OF_LIGHT_CORRECTION_REDEFINED,
                                                      l, file.getAbsolutePath(), line);
                        }
                    }
                    global.calls.add(new DumpedCall() {

                        /** {@inheritDoc} */
                        @Override
                        public Object execute(final Rugged rugged) throws RuggedException {
                            return rugged.directLocation(date, position, los);
                        }

                    });
                } catch (OrekitException oe) {
                    throw new RuggedException(oe, oe.getSpecifier(), oe.getParts());
                }
            }

        },

        /** Parser for direct location result dump lines. */
        DIRECT_LOCATION_RESULT() {

            /** {@inheritDoc} */
            @Override
            public void parse(final int l, final File file, final String line, final String[] fields, final DumpReplayer global)
                throws RuggedException {
                if (fields.length < 6 || !fields[0].equals(LATITUDE) ||
                    !fields[2].equals(LONGITUDE) || !fields[4].equals(ELEVATION)) {
                    throw new RuggedException(RuggedMessages.CANNOT_PARSE_LINE, l, file, line);
                }
                final GeodeticPoint gp = new GeodeticPoint(Double.parseDouble(fields[1]),
                                                           Double.parseDouble(fields[3]),
                                                           Double.parseDouble(fields[5]));
                final DumpedCall last = global.calls.get(global.calls.size() - 1);
                last.expected = gp;
            }

        },

        /** Parser for search span dump lines. */
        SPAN() {

            /** {@inheritDoc} */
            @Override
            public void parse(final int l, final File file, final String line, final String[] fields, final DumpReplayer global)
                throws RuggedException {
                try {
                    if (fields.length < 10 ||
                        !fields[0].equals(MIN_DATE)  || !fields[2].equals(MAX_DATE) || !fields[4].equals(T_STEP)   ||
                        !fields[6].equals(TOLERANCE) || !fields[8].equals(INERTIAL_FRAME)) {
                        throw new RuggedException(RuggedMessages.CANNOT_PARSE_LINE, l, file, line);
                    }
                    global.minDate        = new AbsoluteDate(fields[1], TimeScalesFactory.getUTC());
                    global.maxDate        = new AbsoluteDate(fields[3], TimeScalesFactory.getUTC());
                    global.tStep          = Double.parseDouble(fields[5]);
                    global.tolerance      = Double.parseDouble(fields[7]);
                    global.bodyToInertial = new TreeMap<Integer, Transform>();
                    global.scToInertial   = new TreeMap<Integer, Transform>();
                    try {
                        global.inertialFrame = FramesFactory.getFrame(Predefined.valueOf(fields[9]));
                    } catch (IllegalArgumentException iae) {
                        throw new RuggedException(RuggedMessages.CANNOT_PARSE_LINE, l, file, line);
                    }
                } catch (OrekitException oe) {
                    throw new RuggedException(oe, oe.getSpecifier(), oe.getParts());
                }
            }

        },

        /** Parser for observation transforms dump lines. */
        TRANSFORM() {

            /** {@inheritDoc} */
            @Override
            public void parse(final int l, final File file, final String line, final String[] fields, final DumpReplayer global)
                throws RuggedException {
                if (fields.length < 42 ||
                    !fields[0].equals(INDEX) ||
                    !fields[2].equals(BODY)  ||
                    !fields[3].equals(R)     || !fields[8].equals(OMEGA)    || !fields[12].equals(OMEGA_DOT) ||
                    !fields[16].equals(SPACECRAFT) ||
                    !fields[17].equals(P)    || !fields[21].equals(V)   || !fields[25].equals(A) ||
                    !fields[29].equals(R)    || !fields[34].equals(OMEGA)   || !fields[38].equals(OMEGA_DOT)) {
                    throw new RuggedException(RuggedMessages.CANNOT_PARSE_LINE, l, file, line);
                }
                final int i   = Integer.parseInt(fields[1]);
                final AbsoluteDate date = global.minDate.shiftedBy(i * global.tStep);
                global.bodyToInertial.put(i,
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
                                                                     Double.parseDouble(fields[15]))));
                global.scToInertial.put(i,
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
                                                                                 Double.parseDouble(fields[41])))));
            }

        },

        /** Parser for DEM tile global geometry dump lines. */
        DEM_TILE() {

            /** {@inheritDoc} */
            @Override
            public void parse(final int l, final File file, final String line, final String[] fields, final DumpReplayer global)
                throws RuggedException {
                if (fields.length < 13 ||
                        !fields[1].equals(LAT_MIN) || !fields[3].equals(LAT_STEP) || !fields[5].equals(LAT_ROWS) ||
                        !fields[7].equals(LON_MIN) || !fields[9].equals(LON_STEP) || !fields[11].equals(LON_COLS)) {
                    throw new RuggedException(RuggedMessages.CANNOT_PARSE_LINE, l, file, line);
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
                        throw new RuggedException(RuggedMessages.TILE_ALREADY_DEFINED,
                                                  name, l, file.getAbsolutePath(), line);
                    }
                }
                global.tiles.add(new ParsedTile(name,
                                                minLatitude, latitudeStep, latitudeRows,
                                                minLongitude, longitudeStep, longitudeColumns));
            }

        },

        /** Parser for DEM cells dump lines. */
        DEM_CELL() {

            /** {@inheritDoc} */
            @Override
            public void parse(final int l, final File file, final String line, final String[] fields, final DumpReplayer global)
                throws RuggedException {
                if (fields.length < 7 ||
                    !fields[1].equals(LAT_INDEX) || !fields[3].equals(LON_INDEX) || !fields[5].equals(ELEVATION)) {
                    throw new RuggedException(RuggedMessages.CANNOT_PARSE_LINE, l, file, line);
                }
                final String name      = fields[0];
                final int    latIndex  = Integer.parseInt(fields[2]);
                final int    lonIndex  = Integer.parseInt(fields[4]);
                final double elevation = Double.parseDouble(fields[6]);
                for (final ParsedTile tile : global.tiles) {
                    if (tile.name.equals(name)) {
                        final int index = latIndex * tile.longitudeColumns + lonIndex;
                        tile.elevations.put(index, elevation);
                        return;
                    }
                }
                throw new RuggedException(RuggedMessages.UNKNOWN_TILE,
                                          name, l, file.getAbsolutePath(), line);
            }

        },

        /** Parser for inverse location calls dump lines. */
        INVERSE_LOCATION() {

            /** {@inheritDoc} */
            @Override
            public void parse(final int l, final File file, final String line, final String[] fields, final DumpReplayer global)
                throws RuggedException {
                if (fields.length < 16 ||
                        !fields[0].equals(SENSOR_NAME) ||
                        !fields[2].equals(LATITUDE) || !fields[4].equals(LONGITUDE) || !fields[6].equals(ELEVATION) ||
                        !fields[8].equals(MIN_LINE) || !fields[10].equals(MAX_LINE) ||
                        !fields[12].equals(LIGHT_TIME) || !fields[14].equals(ABERRATION)) {
                    throw new RuggedException(RuggedMessages.CANNOT_PARSE_LINE, l, file, line);
                }
                final String sensorName = fields[1];
                final GeodeticPoint point = new GeodeticPoint(Double.parseDouble(fields[3]),
                                                              Double.parseDouble(fields[5]),
                                                              Double.parseDouble(fields[7]));
                final int minLine = Integer.parseInt(fields[9]);
                final int maxLine = Integer.parseInt(fields[11]);
                if (global.calls.isEmpty()) {
                    global.lightTimeCorrection         = Boolean.parseBoolean(fields[13]);
                    global.aberrationOfLightCorrection = Boolean.parseBoolean(fields[15]);
                } else {
                    if (global.lightTimeCorrection != Boolean.parseBoolean(fields[13])) {
                        throw new RuggedException(RuggedMessages.LIGHT_TIME_CORRECTION_REDEFINED,
                                                  l, file.getAbsolutePath(), line);
                    }
                    if (global.aberrationOfLightCorrection != Boolean.parseBoolean(fields[15])) {
                        throw new RuggedException(RuggedMessages.ABERRATION_OF_LIGHT_CORRECTION_REDEFINED,
                                                  l, file.getAbsolutePath(), line);
                    }
                }
                global.calls.add(new DumpedCall() {

                    /** {@inheritDoc} */
                    @Override
                    public Object execute(final Rugged rugged) throws RuggedException {
                        return rugged.inverseLocation(sensorName, point, minLine, maxLine);
                    }

                });
            }

        },

        /** Parser for inverse location result dump lines. */
        INVERSE_LOCATION_RESULT() {

            /** {@inheritDoc} */
            @Override
            public void parse(final int l, final File file, final String line, final String[] fields, final DumpReplayer global)
                throws RuggedException {
                if (fields.length < 4 || !fields[0].equals(LINE_NUMBER) || !fields[2].equals(PIXEL_NUMBER)) {
                    throw new RuggedException(RuggedMessages.CANNOT_PARSE_LINE, l, file, line);
                }
                final SensorPixel sp = new SensorPixel(Double.parseDouble(fields[1]),
                                                       Double.parseDouble(fields[3]));
                final DumpedCall last = global.calls.get(global.calls.size() - 1);
                last.expected = sp;
            }

        },

        /** Parser for sensor dump lines. */
        SENSOR() {

            /** {@inheritDoc} */
            @Override
            public void parse(final int l, final File file, final String line, final String[] fields, final DumpReplayer global)
                throws RuggedException {
                if (fields.length < 8 || !fields[0].equals(SENSOR_NAME) ||
                    !fields[2].equals(NB_PIXELS) || !fields[4].equals(POSITION)) {
                    throw new RuggedException(RuggedMessages.CANNOT_PARSE_LINE, l, file, line);
                }
                final ParsedSensor sensor = global.getSensor(fields[1]);
                sensor.setNbPixels(Integer.parseInt(fields[3]));
                sensor.setPosition(new Vector3D(Double.parseDouble(fields[5]),
                                                Double.parseDouble(fields[6]),
                                                Double.parseDouble(fields[7])));
            }

        },

        /** Parser for sensor mean plane dump lines. */
        SENSOR_MEAN_PLANE() {

            /** {@inheritDoc} */
            @Override
            public void parse(final int l, final File file, final String line, final String[] fields, final DumpReplayer global)
                throws RuggedException {
                try {
                    if (fields.length < 16 || !fields[0].equals(SENSOR_NAME) ||
                        !fields[2].equals(MIN_LINE) || !fields[4].equals(MAX_LINE) ||
                        !fields[6].equals(MAX_EVAL) || !fields[8].equals(ACCURACY) ||
                        !fields[10].equals(NORMAL)  || !fields[14].equals(CACHED_RESULTS)) {
                        throw new RuggedException(RuggedMessages.CANNOT_PARSE_LINE, l, file, line);
                    }
                    final String   sensorName = fields[1];
                    final int      minLine    = Integer.parseInt(fields[3]);
                    final int      maxLine    = Integer.parseInt(fields[5]);
                    final int      maxEval    = Integer.parseInt(fields[7]);
                    final double   accuracy   = Double.parseDouble(fields[9]);
                    final Vector3D normal     = new Vector3D(Double.parseDouble(fields[11]),
                                                             Double.parseDouble(fields[12]),
                                                             Double.parseDouble(fields[13]));
                    final int      n          = Integer.parseInt(fields[15]);
                    final CrossingResult[] cachedResults = new CrossingResult[n];
                    int base = 16;
                    for (int i = 0; i < n; ++i) {
                        if (fields.length < base + 15 || !fields[base].equals(LINE_NUMBER) ||
                            !fields[base + 2].equals(DATE) || !fields[base + 4].equals(TARGET) ||
                            !fields[base + 8].equals(TARGET_DIRECTION)) {
                            throw new RuggedException(RuggedMessages.CANNOT_PARSE_LINE, l, file, line);
                        }
                        final double       ln     = Double.parseDouble(fields[base + 1]);
                        final AbsoluteDate date   = new AbsoluteDate(fields[base + 3], TimeScalesFactory.getUTC());
                        final Vector3D     target = new Vector3D(Double.parseDouble(fields[base + 5]),
                                                                 Double.parseDouble(fields[base + 6]),
                                                                 Double.parseDouble(fields[base + 7]));
                        final DerivativeStructure tdX = new DerivativeStructure(1, 1,
                                                                                Double.parseDouble(fields[base +  9]),
                                                                                Double.parseDouble(fields[base + 12]));
                        final DerivativeStructure tdY = new DerivativeStructure(1, 1,
                                                                                Double.parseDouble(fields[base + 10]),
                                                                                Double.parseDouble(fields[base + 13]));
                        final DerivativeStructure tdZ = new DerivativeStructure(1, 1,
                                                                                Double.parseDouble(fields[base + 11]),
                                                                                Double.parseDouble(fields[base + 14]));
                        final FieldVector3D<DerivativeStructure> targetDirection =
                                new FieldVector3D<DerivativeStructure>(tdX, tdY, tdZ);
                        cachedResults[i] = new CrossingResult(date, ln, target, targetDirection);
                        base += 15;
                    }
                    global.getSensor(sensorName).setMeanPlane(new ParsedMeanPlane(minLine, maxLine, maxEval, accuracy, normal, cachedResults));

                } catch (OrekitException oe) {
                    throw new RuggedException(oe, oe.getSpecifier(), oe.getParts());
                }
            }

        },

        /** Parser for sensor LOS dump lines. */
        SENSOR_LOS() {

            /** {@inheritDoc} */
            @Override
            public void parse(final int l, final File file, final String line, final String[] fields, final DumpReplayer global)
                throws RuggedException {
                try {
                    if (fields.length < 10 || !fields[0].equals(SENSOR_NAME) ||
                            !fields[2].equals(DATE) || !fields[4].equals(PIXEL_NUMBER) ||
                            !fields[6].equals(LOS)) {
                        throw new RuggedException(RuggedMessages.CANNOT_PARSE_LINE, l, file, line);
                    }
                    final String       sensorName  = fields[1];
                    final AbsoluteDate date        = new AbsoluteDate(fields[3], TimeScalesFactory.getUTC());
                    final int          pixelNumber = Integer.parseInt(fields[5]);
                    final Vector3D     los         = new Vector3D(Double.parseDouble(fields[7]),
                                                                  Double.parseDouble(fields[8]),
                                                                  Double.parseDouble(fields[9]));
                    global.getSensor(sensorName).setLOS(date, pixelNumber, los);

                } catch (OrekitException oe) {
                    throw new RuggedException(oe, oe.getSpecifier(), oe.getParts());
                }
            }

        },

        /** Parser for sensor datation dump lines. */
        SENSOR_DATATION() {

            /** {@inheritDoc} */
            @Override
            public void parse(final int l, final File file, final String line, final String[] fields, final DumpReplayer global)
                throws RuggedException {
                try {
                    if (fields.length < 5 || !fields[0].equals(SENSOR_NAME) ||
                        !fields[2].equals(LINE_NUMBER) || !fields[4].equals(DATE)) {
                        throw new RuggedException(RuggedMessages.CANNOT_PARSE_LINE, l, file, line);
                    }
                    final String       sensorName  = fields[1];
                    final double       lineNumber  = Double.parseDouble(fields[3]);
                    final AbsoluteDate date        = new AbsoluteDate(fields[5], TimeScalesFactory.getUTC());
                    global.getSensor(sensorName).setDatation(lineNumber, date);

                } catch (OrekitException oe) {
                    throw new RuggedException(oe, oe.getSpecifier(), oe.getParts());
                }

            }

        },

        /** Parser for sensor rate dump lines. */
        SENSOR_RATE() {

            /** {@inheritDoc} */
            @Override
            public void parse(final int l, final File file, final String line, final String[] fields, final DumpReplayer global)
                throws RuggedException {
                if (fields.length < 5 || !fields[0].equals(SENSOR_NAME) ||
                    !fields[2].equals(LINE_NUMBER) || !fields[4].equals(RATE)) {
                    throw new RuggedException(RuggedMessages.CANNOT_PARSE_LINE, l, file, line);
                }
                final String       sensorName  = fields[1];
                final double       lineNumber  = Double.parseDouble(fields[3]);
                final double       rate  = Double.parseDouble(fields[5]);
                global.getSensor(sensorName).setRate(lineNumber, rate);

            }

        };

        /** Parse a line.
         * @param l line number
         * @param file dump file
         * @param line line to parse
         * @param global global parser to store parsed data
         * @exception RuggedException if line is not supported
         */
        public static void parse(final int l, final File file, final String line, final DumpReplayer global)
            throws RuggedException {

            final String trimmed = line.trim();
            if (trimmed.length() == 0 || trimmed.startsWith(COMMENT_START)) {
                return;
            }

            final int colon = line.indexOf(':');
            if (colon > 0) {
                final String parsedKey = line.substring(0, colon).trim().replaceAll(" ", "_").toUpperCase();
                try {
                    final LineParser parser = LineParser.valueOf(parsedKey);
                    parser.parse(l, file, line, line.substring(colon + 1).trim().split("\\s+"), global);
                } catch (IllegalArgumentException iae) {
                    throw new RuggedException(RuggedMessages.CANNOT_PARSE_LINE, l, file, line);
                }

            }

        }

        /** Parse a line.
         * @param l line number
         * @param file dump file
         * @param line complete line
         * @param fields data fields from the line
         * @param global global parser to store parsed data
         * @exception RuggedException if line cannot be parsed
         */
        public abstract void parse(final int l, final File file, final String line, final String[] fields,
                                   final DumpReplayer global)
            throws RuggedException;

    }

    /** Local class for handling already parsed tile data. */
    private static class ParsedTile {

        /** Name of the tile. */
        private final String name;

        /** Minimum latitude. */
        private final double minLatitude;

        /** Step in latitude (size of one raster element). */
        private final double latitudeStep;

        /** Number of latitude rows. */
        private int latitudeRows;

        /** Minimum longitude. */
        private final double minLongitude;

        /** Step in longitude (size of one raster element). */
        private final double longitudeStep;

        /** Number of longitude columns. */
        private int longitudeColumns;

        /** Raster elevation data. */
        private final OpenIntToDoubleHashMap elevations;

        /** Simple constructor.
         * @param name of the tile
         * @param minLatitude minimum latitude
         * @param latitudeStep step in latitude (size of one raster element)
         * @param latitudeRows number of latitude rows
         * @param minLongitude minimum longitude
         * @param longitudeStep step in longitude (size of one raster element)
         * @param longitudeColumns number of longitude columns
         */
        ParsedTile(final String name,
                   final double minLatitude, final double latitudeStep, final int latitudeRows,
                   final double minLongitude, final double longitudeStep, final int longitudeColumns) {
            this.name             = name;
            this.minLatitude      = minLatitude;
            this.latitudeStep     = latitudeStep;
            this.minLongitude     = minLongitude;
            this.longitudeStep    = longitudeStep;
            this.latitudeRows     = latitudeRows;
            this.longitudeColumns = longitudeColumns;
            this.elevations       = new OpenIntToDoubleHashMap();
        }

        /** Check if a point is in the interpolable region of the tile.
         * @param latitude point latitude
         * @param longitude point longitude
         * @return true if the point is in the interpolable region of the tile
         */
        public boolean isInterpolable(final double latitude, final double longitude) {
            final int latitudeIndex  = (int) FastMath.floor((latitude  - minLatitude)  / latitudeStep);
            final int longitudeIndex = (int) FastMath.floor((longitude - minLongitude) / longitudeStep);
            return (latitudeIndex  >= 0) && (latitudeIndex  <= latitudeRows     - 2) &&
                   (longitudeIndex >= 0) && (longitudeIndex <= longitudeColumns - 2);
        }

        /** Update the tile according to the Digital Elevation Model.
         * @param tile to update
         * @exception RuggedException if tile cannot be updated
         */
        public void updateTile(final UpdatableTile tile)
            throws RuggedException {

            tile.setGeometry(minLatitude, minLongitude,
                             latitudeStep, longitudeStep,
                             latitudeRows, longitudeColumns);

            final OpenIntToDoubleHashMap.Iterator iterator = elevations.iterator();
            while (iterator.hasNext()) {
                iterator.advance();
                final int    index          = iterator.key();
                final int    latitudeIndex  = index / longitudeColumns;
                final int    longitudeIndex = index % longitudeColumns;
                final double elevation      = iterator.value();
                tile.setElevation(latitudeIndex, longitudeIndex, elevation);
            }

        }

    }

    /** Local class for handling already parsed sensor data. */
    private static class ParsedSensor implements LineDatation, TimeDependentLOS {

        /** Name of the sensor. */
        private final String name;

        /** Number of pixels. */
        private int nbPixels;

        /** Position. */
        private Vector3D position;

        /** Mean plane crossing finder. */
        private ParsedMeanPlane meanPlane;

        /** LOS map. */
        private final Map<Integer, List<Pair<AbsoluteDate, Vector3D>>> losMap;

        /** Datation. */
        private final List<Pair<Double, AbsoluteDate>> datation;

        /** Rate. */
        private final List<Pair<Double, Double>> rates;

        /** simple constructor.
         * @param name name of the sensor
         */
        ParsedSensor(final String name) {
            this.name     = name;
            this.losMap   = new HashMap<Integer, List<Pair<AbsoluteDate, Vector3D>>>();
            this.datation = new ArrayList<Pair<Double, AbsoluteDate>>();
            this.rates    = new ArrayList<Pair<Double, Double>>();
        }

        /** Set the mean place finder.
         * @param meanPlane mean plane finder
         */
        public void setMeanPlane(final ParsedMeanPlane meanPlane) {
            this.meanPlane = meanPlane;
        }

        /** Set the position.
         * @param position position
         */
        public void setPosition(final Vector3D position) {
            this.position = position;
        }

        /** Set the number of pixels.
         * @param nbPixels number of pixels
         */
        public void setNbPixels(final int nbPixels) {
            this.nbPixels = nbPixels;
        }

        /** {@inheritDoc} */
        @Override
        public int getNbPixels() {
            return nbPixels;
        }

        /** Set a los direction.
         * @param date date
         * @param pixelNumber number of the pixel
         * @param los los direction
         */
        public void setLOS(final AbsoluteDate date, final int pixelNumber, final Vector3D los) {
            List<Pair<AbsoluteDate, Vector3D>> list = losMap.get(pixelNumber);
            if (list == null) {
                list = new ArrayList<Pair<AbsoluteDate, Vector3D>>();
                losMap.put(pixelNumber, list);
            }
            // find insertion index to have LOS sorted chronologically
            int index = 0;
            while (index < list.size()) {
                if (list.get(index).getFirst().compareTo(date) > 0) {
                    break;
                }
                ++index;
            }
            list.add(index, new Pair<AbsoluteDate, Vector3D>(date, los));
        }

        /** {@inheritDoc} */
        @Override
        public Vector3D getLOS(final int index, final AbsoluteDate date) {
            final List<Pair<AbsoluteDate, Vector3D>> list = losMap.get(index);
            if (list == null) {
                throw RuggedException.createInternalError(null);
            }

            if (list.size() < 2) {
                return list.get(0).getSecond();
            }

            // find entries bracketing the the date
            int sup = 0;
            while (sup < list.size() - 1) {
                if (list.get(sup).getFirst().compareTo(date) >= 0) {
                    break;
                }
                ++sup;
            }
            final int inf = (sup == 0) ? sup++ : (sup - 1);

            final AbsoluteDate dInf  = list.get(inf).getFirst();
            final Vector3D     lInf  = list.get(inf).getSecond();
            final AbsoluteDate dSup  = list.get(sup).getFirst();
            final Vector3D     lSup  = list.get(sup).getSecond();
            final double       alpha = date.durationFrom(dInf) / dSup.durationFrom(dInf);
            return new Vector3D(alpha, lSup, 1 - alpha, lInf);

        }

        /** {@inheritDoc} */
        @Override
        public FieldVector3D<DerivativeStructure> getLOSDerivatives(final int index, final AbsoluteDate date) {
            final Optional<ExtendedParameterDriver> first = getExtendedParametersDrivers().findFirst();
            final int nbEstimated = first.isPresent() ? first.get().getNbEstimated() : 0;
            final Vector3D los = getLOS(index, date);
            return new FieldVector3D<DerivativeStructure>(new DerivativeStructure(nbEstimated, 1, los.getX()),
                                                          new DerivativeStructure(nbEstimated, 1, los.getY()),
                                                          new DerivativeStructure(nbEstimated, 1, los.getZ()));
        }

        /** Set a datation pair.
         * @param lineNumber line number
         * @param date date
         */
        public void setDatation(final double lineNumber, final AbsoluteDate date) {
            // find insertion index to have datations sorted chronologically
            int index = 0;
            while (index < datation.size()) {
                if (datation.get(index).getSecond().compareTo(date) > 0) {
                    break;
                }
                ++index;
            }
            datation.add(index, new Pair<Double, AbsoluteDate>(lineNumber, date));
        }

        /** {@inheritDoc} */
        @Override
        public AbsoluteDate getDate(final double lineNumber) {

            if (datation.size() < 2) {
                return datation.get(0).getSecond();
            }

            // find entries bracketing the line number
            int sup = 0;
            while (sup < datation.size() - 1) {
                if (datation.get(sup).getFirst() >= lineNumber) {
                    break;
                }
                ++sup;
            }
            final int inf = (sup == 0) ? sup++ : (sup - 1);

            final double       lInf  = datation.get(inf).getFirst();
            final AbsoluteDate dInf  = datation.get(inf).getSecond();
            final double       lSup  = datation.get(sup).getFirst();
            final AbsoluteDate dSup  = datation.get(sup).getSecond();
            final double       alpha = (lineNumber - lInf) / (lSup - lInf);
            return dInf.shiftedBy(alpha * dSup.durationFrom(dInf));

        }

        /** {@inheritDoc} */
        @Override
        public double getLine(final AbsoluteDate date) {

            if (datation.size() < 2) {
                return datation.get(0).getFirst();
            }

            // find entries bracketing the date
            int sup = 0;
            while (sup < datation.size() - 1) {
                if (datation.get(sup).getSecond().compareTo(date) >= 0) {
                    break;
                }
                ++sup;
            }
            final int inf = (sup == 0) ? sup++ : (sup - 1);

            final double       lInf  = datation.get(inf).getFirst();
            final AbsoluteDate dInf  = datation.get(inf).getSecond();
            final double       lSup  = datation.get(sup).getFirst();
            final AbsoluteDate dSup  = datation.get(sup).getSecond();
            final double       alpha = date.durationFrom(dInf) / dSup.durationFrom(dInf);
            return alpha * lSup + (1 - alpha) * lInf;

        }

        /** Set a rate.
         * @param lineNumber line number
         * @param rate lines rate
         */
        public void setRate(final double lineNumber, final double rate) {
            // find insertion index to have rates sorted by line numbers
            int index = 0;
            while (index < rates.size()) {
                if (rates.get(index).getFirst() > lineNumber) {
                    break;
                }
                ++index;
            }
            rates.add(index, new Pair<Double, Double>(lineNumber, rate));
        }

        /** {@inheritDoc} */
        @Override
        public double getRate(final double lineNumber) {

            if (rates.size() < 2) {
                return rates.get(0).getSecond();
            }

            // find entries bracketing the line number
            int sup = 0;
            while (sup < rates.size() - 1) {
                if (rates.get(sup).getFirst() >= lineNumber) {
                    break;
                }
                ++sup;
            }
            final int inf = (sup == 0) ? sup++ : (sup - 1);

            final double lInf  = rates.get(inf).getFirst();
            final double rInf  = rates.get(inf).getSecond();
            final double lSup  = rates.get(sup).getFirst();
            final double rSup  = rates.get(sup).getSecond();
            final double alpha = (lineNumber - lInf) / (lSup - lInf);
            return alpha * rSup + (1 - alpha) * rInf;

        }

        /** {@inheritDoc} */
        @Override
        public Stream<ExtendedParameterDriver> getExtendedParametersDrivers() {
            return Stream.<ExtendedParameterDriver>empty();
        }

    }

    /** Local class for handling already parsed mean plane data. */
    private static class ParsedMeanPlane {

        /** Min line. */
        private final int minLine;

        /** Max line. */
        private final int maxLine;

        /** Maximum number of evaluations. */
        private final int maxEval;

        /** Accuracy to use for finding crossing line number. */
        private final double accuracy;

        /** Mean plane normal. */
        private final Vector3D normal;

        /** Cached results. */
        private final CrossingResult[] cachedResults;

        /** simple constructor.
         * @param minLine min line
         * @param maxLine max line
         * @param maxEval maximum number of evaluations
         * @param accuracy accuracy to use for finding crossing line number
         * @param normal mean plane normal
         * @param cachedResults cached results
         */
        ParsedMeanPlane(final int minLine, final int maxLine,
                        final int maxEval, final double accuracy, final Vector3D normal,
                        final CrossingResult[] cachedResults) {
            this.minLine       = minLine;
            this.maxLine       = maxLine;
            this.maxEval       = maxEval;
            this.accuracy      = accuracy;
            this.normal        = normal;
            this.cachedResults = cachedResults.clone();
        }

    }

    /** Local interface for dumped calls. */
    private abstract static class DumpedCall {

        /** Expected result. */
        private Object expected;

        /** Execute a call.
         * @param rugged Rugged instance on which called should be performed
         * @return result of the call
         * @exception RuggedException if the call fails
         */
        public abstract Object execute(Rugged rugged) throws RuggedException;

    }

}
