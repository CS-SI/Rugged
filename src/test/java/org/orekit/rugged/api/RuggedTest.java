/* Copyright 2013-2017 CS Systèmes d'Information
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
package org.orekit.rugged.api;


import java.io.File;
import java.io.IOException;
import java.io.RandomAccessFile;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.net.URISyntaxException;
import java.nio.MappedByteBuffer;
import java.nio.channels.FileChannel;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

import org.hipparchus.analysis.differentiation.DSFactory;
import org.hipparchus.analysis.differentiation.DerivativeStructure;
import org.hipparchus.analysis.differentiation.FiniteDifferencesDifferentiator;
import org.hipparchus.analysis.differentiation.UnivariateDifferentiableFunction;
import org.hipparchus.geometry.euclidean.threed.Rotation;
import org.hipparchus.geometry.euclidean.threed.RotationConvention;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.stat.descriptive.StreamingStatistics;
import org.hipparchus.stat.descriptive.rank.Percentile;
import org.hipparchus.util.FastMath;
import org.junit.Assert;
import org.junit.Ignore;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;
import org.orekit.bodies.BodyShape;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.data.DataProvidersManager;
import org.orekit.data.DirectoryCrawler;
import org.orekit.errors.OrekitException;
import org.orekit.errors.OrekitExceptionWrapper;
import org.orekit.forces.gravity.potential.NormalizedSphericalHarmonicsProvider;
import org.orekit.frames.Frame;
import org.orekit.frames.FramesFactory;
import org.orekit.orbits.Orbit;
import org.orekit.propagation.Propagator;
import org.orekit.rugged.TestUtils;
import org.orekit.rugged.adjustment.GroundOptimizationProblemBuilder;
import org.orekit.rugged.errors.RuggedException;
import org.orekit.rugged.errors.RuggedExceptionWrapper;
import org.orekit.rugged.errors.RuggedMessages;
import org.orekit.rugged.linesensor.LineDatation;
import org.orekit.rugged.linesensor.LineSensor;
import org.orekit.rugged.linesensor.LinearLineDatation;
import org.orekit.rugged.linesensor.SensorPixel;
import org.orekit.rugged.los.FixedRotation;
import org.orekit.rugged.los.LOSBuilder;
import org.orekit.rugged.los.TimeDependentLOS;
import org.orekit.rugged.raster.RandomLandscapeUpdater;
import org.orekit.rugged.raster.TileUpdater;
import org.orekit.rugged.raster.VolcanicConeElevationUpdater;
import org.orekit.rugged.refining.measures.Observables;
import org.orekit.rugged.utils.DSGenerator;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeScale;
import org.orekit.time.TimeScalesFactory;
import org.orekit.utils.AngularDerivativesFilter;
import org.orekit.utils.CartesianDerivativesFilter;
import org.orekit.utils.Constants;
import org.orekit.utils.IERSConventions;
import org.orekit.utils.ParameterDriver;
import org.orekit.utils.TimeStampedAngularCoordinates;
import org.orekit.utils.TimeStampedPVCoordinates;

public class RuggedTest {

    @Rule
    public TemporaryFolder tempFolder = new TemporaryFolder();

    // the following test is disabled by default
    // it is only used to check timings, and also creates a large (66M) temporary file
    @Ignore
    @Test
    public void testMayonVolcanoTiming()
        throws RuggedException, OrekitException, URISyntaxException {

        long t0 = System.currentTimeMillis();
        int dimension = 2000;

        String path = getClass().getClassLoader().getResource("orekit-data").toURI().getPath();
        DataProvidersManager.getInstance().addProvider(new DirectoryCrawler(new File(path)));
        BodyShape  earth                                  = TestUtils.createEarth();
        NormalizedSphericalHarmonicsProvider gravityField = TestUtils.createGravityField();
        Orbit      orbit                                  = TestUtils.createOrbit(gravityField.getMu());

        // Mayon Volcano location according to Wikipedia: 13°15′24″N 123°41′6″E
        GeodeticPoint summit =
                new GeodeticPoint(FastMath.toRadians(13.25667), FastMath.toRadians(123.685), 2463.0);
        VolcanicConeElevationUpdater updater =
                new VolcanicConeElevationUpdater(summit,
                                                 FastMath.toRadians(30.0), 16.0,
                                                 FastMath.toRadians(1.0), 1201);
        final AbsoluteDate crossing = new AbsoluteDate("2012-01-06T02:27:16.139", TimeScalesFactory.getUTC());

        // one line sensor
        // position: 1.5m in front (+X) and 20 cm above (-Z) of the S/C center of mass
        // los: swath in the (YZ) plane, centered at +Z, ±10° aperture, 960 pixels
        Vector3D position = new Vector3D(1.5, 0, -0.2);
        TimeDependentLOS los = TestUtils.createLOSPerfectLine(Vector3D.PLUS_K, Vector3D.PLUS_I,
                                                              FastMath.toRadians(10.0), dimension).build();

        // linear datation model: at reference time we get line 1000, and the rate is one line every 1.5ms
        LineDatation lineDatation = new LinearLineDatation(crossing, dimension / 2, 1.0 / 1.5e-3);
        int firstLine = 0;
        int lastLine  = dimension;
        LineSensor lineSensor = new LineSensor("line", lineDatation, position, los);

        Propagator propagator = TestUtils.createPropagator(earth, gravityField, orbit);
        propagator.propagate(lineDatation.getDate(firstLine).shiftedBy(-1.0));
        propagator.setEphemerisMode();
        propagator.propagate(lineDatation.getDate(lastLine).shiftedBy(+1.0));
        Propagator ephemeris = propagator.getGeneratedEphemeris();

        Rugged rugged = new RuggedBuilder().
                setDigitalElevationModel(updater, 8).
                setAlgorithm(AlgorithmId.DUVENHAGE).
                setEllipsoid(EllipsoidId.WGS84, BodyRotatingFrameId.ITRF).
                setTimeSpan(lineDatation.getDate(firstLine), lineDatation.getDate(lastLine), 0.001, 5.0).
                setTrajectory(1.0 / lineDatation.getRate(0.0),
                              8, CartesianDerivativesFilter.USE_PV,AngularDerivativesFilter.USE_R,
                              ephemeris).
                addLineSensor(lineSensor).
                build();

        try {

            int              size   = (lastLine - firstLine) * los.getNbPixels() * 3 * Integer.SIZE / 8;
            RandomAccessFile out    = new RandomAccessFile(tempFolder.newFile(), "rw");
            MappedByteBuffer buffer = out.getChannel().map(FileChannel.MapMode.READ_WRITE, 0, size);

            long t1 = System.currentTimeMillis();
            int pixels = 0;
            for (double line = firstLine; line < lastLine; line++) {
                GeodeticPoint[] gp = rugged.directLocation("line", line);
                for (int i = 0; i < gp.length; ++i) {
                    final int latCode = (int) FastMath.rint(FastMath.scalb(gp[i].getLatitude(),  29));
                    final int lonCode = (int) FastMath.rint(FastMath.scalb(gp[i].getLongitude(), 29));
                    final int altCode = (int) FastMath.rint(FastMath.scalb(gp[i].getAltitude(),  17));
                    buffer.putInt(latCode);
                    buffer.putInt(lonCode);
                    buffer.putInt(altCode);
                }
                pixels += los.getNbPixels();
                if  (line % 100 == 0) {
                    System.out.format(Locale.US, "%5.0f%n", line);
                }
            }
            long t2 = System.currentTimeMillis();
            out.close();
            int sizeM = size / (1024 * 1024);
            System.out.format(Locale.US,
                              "%n%n%5dx%5d:%n" +
                              "  Orekit initialization and DEM creation   : %5.1fs%n" +
                              "  direct location and %3dM grid writing: %5.1fs (%.1f px/s)%n",
                              lastLine - firstLine, los.getNbPixels(),
                              1.0e-3 * (t1 - t0), sizeM, 1.0e-3 * (t2 - t1), pixels / (1.0e-3 * (t2 - t1)));
        } catch (IOException ioe) {
            Assert.fail(ioe.getLocalizedMessage());
        }

    }

    @Test
    public void testLightTimeCorrection()
        throws RuggedException, OrekitException, URISyntaxException {

        int dimension = 400;

        String path = getClass().getClassLoader().getResource("orekit-data").toURI().getPath();
        DataProvidersManager.getInstance().addProvider(new DirectoryCrawler(new File(path)));
        final BodyShape  earth = TestUtils.createEarth();
        final Orbit      orbit = TestUtils.createOrbit(Constants.EIGEN5C_EARTH_MU);

        AbsoluteDate crossing = new AbsoluteDate("2012-01-07T11:21:15.000", TimeScalesFactory.getUTC());

        // one line sensor
        // position: 1.5m in front (+X) and 20 cm above (-Z) of the S/C center of mass
        // los: swath in the (YZ) plane, centered at +Z, ±10° aperture, 960 pixels
        Vector3D position = new Vector3D(1.5, 0, -0.2);
        TimeDependentLOS los = TestUtils.createLOSPerfectLine(Vector3D.PLUS_K, Vector3D.PLUS_I,
                                                              FastMath.toRadians(10.0), dimension).build();

        // linear datation model: at reference time we get line 200, and the rate is one line every 1.5ms
        LineDatation lineDatation = new LinearLineDatation(crossing, dimension / 2, 1.0 / 1.5e-3);
        int firstLine = 0;
        int lastLine  = dimension;
        LineSensor lineSensor = new LineSensor("line", lineDatation, position, los);
        AbsoluteDate minDate = lineSensor.getDate(firstLine);
        AbsoluteDate maxDate = lineSensor.getDate(lastLine);

        RuggedBuilder builder = new RuggedBuilder().
                setAlgorithm(AlgorithmId.IGNORE_DEM_USE_ELLIPSOID).
                setEllipsoid(EllipsoidId.IERS2003, BodyRotatingFrameId.ITRF).
                setTimeSpan(minDate, maxDate, 0.001, 5.0).
                setTrajectory(InertialFrameId.EME2000,
                              TestUtils.orbitToPV(orbit, earth, minDate.shiftedBy(-1.0), maxDate.shiftedBy(+1.0), 0.25),
                              8, CartesianDerivativesFilter.USE_PV,
                              TestUtils.orbitToQ(orbit, earth, minDate.shiftedBy(-1.0), maxDate.shiftedBy(+1.0), 0.25),
                              2, AngularDerivativesFilter.USE_R).
                addLineSensor(lineSensor);

        Rugged rugged = builder.build();
        Assert.assertEquals(1, rugged.getLineSensors().size());
        Assert.assertTrue(lineSensor == rugged.getLineSensor("line"));
        try {
            rugged.getLineSensor("dummy");
            Assert.fail("an exception should have been thrown");
        } catch (RuggedException re) {
            Assert.assertEquals(RuggedMessages.UNKNOWN_SENSOR, re.getSpecifier());
            Assert.assertEquals("dummy", re.getParts()[0]);
        }
        Assert.assertEquals(7176419.526,
                            rugged.getScToInertial(lineSensor.getDate(dimension / 2)).getTranslation().getNorm(),
                            1.0e-3);
        Assert.assertEquals(0.0,
                            rugged.getBodyToInertial(lineSensor.getDate(dimension / 2)).getTranslation().getNorm(),
                            1.0e-3);
        Assert.assertEquals(0.0,
                            rugged.getInertialToBody(lineSensor.getDate(dimension / 2)).getTranslation().getNorm(),
                            1.0e-3);

        builder.setLightTimeCorrection(true);
        builder.setAberrationOfLightCorrection(false);
        rugged = builder.build();
        GeodeticPoint[] gpWithLightTimeCorrection = rugged.directLocation("line", 200);

        builder.setLightTimeCorrection(false);
        builder.setAberrationOfLightCorrection(false);
        rugged = builder.build();
        GeodeticPoint[] gpWithoutLightTimeCorrection = rugged.directLocation("line", 200);

        for (int i = 0; i < gpWithLightTimeCorrection.length; ++i) {
            Vector3D pWith    = earth.transform(gpWithLightTimeCorrection[i]);
            Vector3D pWithout = earth.transform(gpWithoutLightTimeCorrection[i]);
            Assert.assertTrue(Vector3D.distance(pWith, pWithout) > 1.23);
            Assert.assertTrue(Vector3D.distance(pWith, pWithout) < 1.27);
        }

    }

    @Test
    public void testAberrationOfLightCorrection()
        throws RuggedException, OrekitException, URISyntaxException {

        int dimension = 400;

        String path = getClass().getClassLoader().getResource("orekit-data").toURI().getPath();
        DataProvidersManager.getInstance().addProvider(new DirectoryCrawler(new File(path)));
        final BodyShape  earth = TestUtils.createEarth();
        final Orbit      orbit = TestUtils.createOrbit(Constants.EIGEN5C_EARTH_MU);

        AbsoluteDate crossing = new AbsoluteDate("2012-01-07T11:46:35.000", TimeScalesFactory.getUTC());

        // one line sensor
        // position: 1.5m in front (+X) and 20 cm above (-Z) of the S/C center of mass
        // los: swath in the (YZ) plane, centered at +Z, ±10° aperture, 960 pixels
        Vector3D position = new Vector3D(1.5, 0, -0.2);
        TimeDependentLOS los = TestUtils.createLOSPerfectLine(Vector3D.PLUS_K, Vector3D.PLUS_I,
                                                              FastMath.toRadians(10.0), dimension).build();

        // linear datation model: at reference time we get line 200, and the rate is one line every 1.5ms
        LineDatation lineDatation = new LinearLineDatation(crossing, dimension / 2, 1.0 / 1.5e-3);
        int firstLine = 0;
        int lastLine  = dimension;
        LineSensor lineSensor = new LineSensor("line", lineDatation, position, los);
        AbsoluteDate minDate = lineSensor.getDate(firstLine);
        AbsoluteDate maxDate = lineSensor.getDate(lastLine);

        RuggedBuilder builder = new RuggedBuilder().
                
                setAlgorithm(AlgorithmId.IGNORE_DEM_USE_ELLIPSOID).
                setEllipsoid(EllipsoidId.WGS84, BodyRotatingFrameId.ITRF).
                setTimeSpan(minDate, maxDate, 0.001, 5.0).
                setTrajectory(InertialFrameId.EME2000,
                              TestUtils.orbitToPV(orbit, earth, minDate.shiftedBy(-1.0), maxDate.shiftedBy(+1.0), 0.25),
                              8, CartesianDerivativesFilter.USE_PV,
                              TestUtils.orbitToQ(orbit, earth, minDate.shiftedBy(-1.0), maxDate.shiftedBy(+1.0), 0.25),
                              2, AngularDerivativesFilter.USE_R).
                addLineSensor(lineSensor).
                setLightTimeCorrection(false).
                setAberrationOfLightCorrection(true);
        Rugged rugged = builder.build();
        GeodeticPoint[] gpWithAberrationOfLightCorrection = rugged.directLocation("line", 200);

        builder.setLightTimeCorrection(false);
        builder.setAberrationOfLightCorrection(false);
        rugged = builder.build();
        GeodeticPoint[] gpWithoutAberrationOfLightCorrection = rugged.directLocation("line", 200);

        for (int i = 0; i < gpWithAberrationOfLightCorrection.length; ++i) {
            Vector3D pWith    = earth.transform(gpWithAberrationOfLightCorrection[i]);
            Vector3D pWithout = earth.transform(gpWithoutAberrationOfLightCorrection[i]);
            Assert.assertTrue(Vector3D.distance(pWith, pWithout) > 20.0);
            Assert.assertTrue(Vector3D.distance(pWith, pWithout) < 20.5);
        }

    }

    @Test
    public void testFlatBodyCorrection()
        throws RuggedException, OrekitException, URISyntaxException {

        int dimension = 200;

        String path = getClass().getClassLoader().getResource("orekit-data").toURI().getPath();
        DataProvidersManager.getInstance().addProvider(new DirectoryCrawler(new File(path)));
        final BodyShape  earth = TestUtils.createEarth();
        final Orbit      orbit = TestUtils.createOrbit(Constants.EIGEN5C_EARTH_MU);

        AbsoluteDate crossing = new AbsoluteDate("2012-01-01T12:30:00.000", TimeScalesFactory.getUTC());

        // one line sensor
        // position: 1.5m in front (+X) and 20 cm above (-Z) of the S/C center of mass
        // los: swath in the (YZ) plane, looking at 50° roll, ±1° aperture
        Vector3D position = new Vector3D(1.5, 0, -0.2);
        TimeDependentLOS los = TestUtils.createLOSPerfectLine(new Rotation(Vector3D.PLUS_I,
                                                                           FastMath.toRadians(50.0),
                                                                           RotationConvention.VECTOR_OPERATOR).applyTo(Vector3D.PLUS_K),
                                                              Vector3D.PLUS_I, FastMath.toRadians(1.0), dimension).build();

        // linear datation model: at reference time we get line 100, and the rate is one line every 1.5ms
        LineDatation lineDatation = new LinearLineDatation(crossing, dimension / 2, 1.0 / 1.5e-3);
        int firstLine = 0;
        int lastLine  = dimension;
        LineSensor lineSensor = new LineSensor("line", lineDatation, position, los);
        AbsoluteDate minDate = lineSensor.getDate(firstLine);
        AbsoluteDate maxDate = lineSensor.getDate(lastLine);

        TileUpdater updater =
                new RandomLandscapeUpdater(0.0, 9000.0, 0.5, 0xf0a401650191f9f6l,
                                           FastMath.toRadians(1.0), 257);

        RuggedBuilder builder = new RuggedBuilder().
                setDigitalElevationModel(updater, 8).
                setEllipsoid(EllipsoidId.WGS84, BodyRotatingFrameId.ITRF).
                setTimeSpan(minDate, maxDate, 0.001, 5.0).
                setTrajectory(InertialFrameId.EME2000,
                              TestUtils.orbitToPV(orbit, earth, minDate.shiftedBy(-1.0), maxDate.shiftedBy(+1.0), 0.25),
                              8, CartesianDerivativesFilter.USE_PV,
                              TestUtils.orbitToQ(orbit, earth, minDate.shiftedBy(-1.0), maxDate.shiftedBy(+1.0), 0.25),
                              2, AngularDerivativesFilter.USE_R).
                addLineSensor(lineSensor);
        GeodeticPoint[] gpWithFlatBodyCorrection =
                builder.setAlgorithm(AlgorithmId.DUVENHAGE).build().directLocation("line", 100);

        GeodeticPoint[] gpWithoutFlatBodyCorrection =
                builder.setAlgorithm(AlgorithmId.DUVENHAGE_FLAT_BODY).build().directLocation("line", 100);

        StreamingStatistics stats = new StreamingStatistics();
        for (int i = 0; i < gpWithFlatBodyCorrection.length; ++i) {
            Vector3D pWith    = earth.transform(gpWithFlatBodyCorrection[i]);
            Vector3D pWithout = earth.transform(gpWithoutFlatBodyCorrection[i]);
            stats.addValue(Vector3D.distance(pWith, pWithout));
        }
        Assert.assertEquals( 0.004, stats.getMin(),  1.0e-3);
        Assert.assertEquals(28.344, stats.getMax(),  1.0e-3);
        Assert.assertEquals( 4.801, stats.getMean(), 1.0e-3);

    }

    @Test
    public void testLocationsinglePoint()
        throws RuggedException, OrekitException, URISyntaxException {

        int dimension = 200;

        String path = getClass().getClassLoader().getResource("orekit-data").toURI().getPath();
        DataProvidersManager.getInstance().addProvider(new DirectoryCrawler(new File(path)));
        final BodyShape  earth = TestUtils.createEarth();
        final Orbit      orbit = TestUtils.createOrbit(Constants.EIGEN5C_EARTH_MU);

        AbsoluteDate crossing = new AbsoluteDate("2012-01-01T12:30:00.000", TimeScalesFactory.getUTC());

        // one line sensor
        // position: 1.5m in front (+X) and 20 cm above (-Z) of the S/C center of mass
        // los: swath in the (YZ) plane, looking at 50° roll, ±1° aperture
        Vector3D position = new Vector3D(1.5, 0, -0.2);
        TimeDependentLOS los = TestUtils.createLOSPerfectLine(new Rotation(Vector3D.PLUS_I,
                                                                           FastMath.toRadians(50.0),
                                                                           RotationConvention.VECTOR_OPERATOR).applyTo(Vector3D.PLUS_K),
                                                              Vector3D.PLUS_I, FastMath.toRadians(1.0), dimension).build();

        // linear datation model: at reference time we get line 100, and the rate is one line every 1.5ms
        LineDatation lineDatation = new LinearLineDatation(crossing, dimension / 2, 1.0 / 1.5e-3);
        int firstLine = 0;
        int lastLine  = dimension;
        LineSensor lineSensor = new LineSensor("line", lineDatation, position, los);
        AbsoluteDate minDate = lineSensor.getDate(firstLine);
        AbsoluteDate maxDate = lineSensor.getDate(lastLine);

        TileUpdater updater =
                new RandomLandscapeUpdater(0.0, 9000.0, 0.5, 0xf0a401650191f9f6l,
                                           FastMath.toRadians(1.0), 257);

        Rugged rugged = new RuggedBuilder().
                setDigitalElevationModel(updater, 8).
                setAlgorithm(AlgorithmId.DUVENHAGE).
                setEllipsoid(EllipsoidId.WGS84, BodyRotatingFrameId.ITRF).
                setTimeSpan(minDate, maxDate, 0.001, 5.0).
                setTrajectory(InertialFrameId.EME2000,
                              TestUtils.orbitToPV(orbit, earth, minDate.shiftedBy(-1.0), maxDate.shiftedBy(+1.0), 0.25),
                              8, CartesianDerivativesFilter.USE_PV,
                              TestUtils.orbitToQ(orbit, earth, minDate.shiftedBy(-1.0), maxDate.shiftedBy(+1.0), 0.25),
                              2, AngularDerivativesFilter.USE_R).
                addLineSensor(lineSensor).build();
        GeodeticPoint[] gpLine = rugged.directLocation("line", 100);

        for (int i = 0; i < gpLine.length; ++i) {
            GeodeticPoint gpPixel =
                    rugged.directLocation(lineSensor.getDate(100), lineSensor.getPosition(),
                                              lineSensor.getLOS(lineSensor.getDate(100), i));
            Assert.assertEquals(gpLine[i].getLatitude(),  gpPixel.getLatitude(),  1.0e-10);
            Assert.assertEquals(gpLine[i].getLongitude(), gpPixel.getLongitude(), 1.0e-10);
            Assert.assertEquals(gpLine[i].getAltitude(),  gpPixel.getAltitude(),  1.0e-10);
        }

    }

    @Test
    public void testLocationsinglePointNoCorrections()
        throws RuggedException, OrekitException, URISyntaxException {

        int dimension = 200;

        String path = getClass().getClassLoader().getResource("orekit-data").toURI().getPath();
        DataProvidersManager.getInstance().addProvider(new DirectoryCrawler(new File(path)));
        final BodyShape  earth = TestUtils.createEarth();
        final Orbit      orbit = TestUtils.createOrbit(Constants.EIGEN5C_EARTH_MU);

        AbsoluteDate crossing = new AbsoluteDate("2012-01-01T12:30:00.000", TimeScalesFactory.getUTC());

        // one line sensor
        // position: 1.5m in front (+X) and 20 cm above (-Z) of the S/C center of mass
        // los: swath in the (YZ) plane, looking at 50° roll, ±1° aperture
        Vector3D position = new Vector3D(1.5, 0, -0.2);
        TimeDependentLOS los = TestUtils.createLOSPerfectLine(new Rotation(Vector3D.PLUS_I,
                                                                           FastMath.toRadians(50.0),
                                                                           RotationConvention.VECTOR_OPERATOR).applyTo(Vector3D.PLUS_K),
                                                              Vector3D.PLUS_I, FastMath.toRadians(1.0), dimension).build();

        // linear datation model: at reference time we get line 100, and the rate is one line every 1.5ms
        LineDatation lineDatation = new LinearLineDatation(crossing, dimension / 2, 1.0 / 1.5e-3);
        int firstLine = 0;
        int lastLine  = dimension;
        LineSensor lineSensor = new LineSensor("line", lineDatation, position, los);
        AbsoluteDate minDate = lineSensor.getDate(firstLine);
        AbsoluteDate maxDate = lineSensor.getDate(lastLine);

        TileUpdater updater =
                new RandomLandscapeUpdater(0.0, 9000.0, 0.5, 0xf0a401650191f9f6l,
                                           FastMath.toRadians(1.0), 257);

        Rugged rugged = new RuggedBuilder().
                setDigitalElevationModel(updater, 8).
                setAlgorithm(AlgorithmId.DUVENHAGE).
                setEllipsoid(EllipsoidId.WGS84, BodyRotatingFrameId.ITRF).
                setTimeSpan(minDate, maxDate, 0.001, 5.0).
                setTrajectory(InertialFrameId.EME2000,
                              TestUtils.orbitToPV(orbit, earth, minDate.shiftedBy(-1.0), maxDate.shiftedBy(+1.0), 0.25),
                              8, CartesianDerivativesFilter.USE_PV,
                              TestUtils.orbitToQ(orbit, earth, minDate.shiftedBy(-1.0), maxDate.shiftedBy(+1.0), 0.25),
                              2, AngularDerivativesFilter.USE_R).
                setAberrationOfLightCorrection(false).
                setLightTimeCorrection(false).
                addLineSensor(lineSensor).
                build();
        GeodeticPoint[] gpLine = rugged.directLocation("line", 100);

        for (int i = 0; i < gpLine.length; ++i) {
            GeodeticPoint gpPixel =
                    rugged.directLocation(lineSensor.getDate(100), lineSensor.getPosition(),
                                              lineSensor.getLOS(lineSensor.getDate(100), i));
            Assert.assertEquals(gpLine[i].getLatitude(),  gpPixel.getLatitude(),  1.0e-10);
            Assert.assertEquals(gpLine[i].getLongitude(), gpPixel.getLongitude(), 1.0e-10);
            Assert.assertEquals(gpLine[i].getAltitude(),  gpPixel.getAltitude(),  1.0e-10);
        }

    }

    @Test
    public void testBasicScan()
        throws RuggedException, OrekitException, URISyntaxException {

        int dimension = 200;

        String path = getClass().getClassLoader().getResource("orekit-data").toURI().getPath();
        DataProvidersManager.getInstance().addProvider(new DirectoryCrawler(new File(path)));
        final BodyShape  earth = TestUtils.createEarth();
        final Orbit      orbit = TestUtils.createOrbit(Constants.EIGEN5C_EARTH_MU);

        AbsoluteDate crossing = new AbsoluteDate("2012-01-01T12:30:00.000", TimeScalesFactory.getUTC());

        // one line sensor
        // position: 1.5m in front (+X) and 20 cm above (-Z) of the S/C center of mass
        // los: swath in the (YZ) plane, looking at 50° roll, ±1° aperture
        Vector3D position = new Vector3D(1.5, 0, -0.2);
        TimeDependentLOS los = TestUtils.createLOSPerfectLine(new Rotation(Vector3D.PLUS_I,
                                                                           FastMath.toRadians(50.0),
                                                                           RotationConvention.VECTOR_OPERATOR).applyTo(Vector3D.PLUS_K),
                                                              Vector3D.PLUS_I, FastMath.toRadians(1.0), dimension).build();

        // linear datation model: at reference time we get line 100, and the rate is one line every 1.5ms
        LineDatation lineDatation = new LinearLineDatation(crossing, dimension / 2, 1.0 / 1.5e-3);
        int firstLine = 0;
        int lastLine  = dimension;
        LineSensor lineSensor = new LineSensor("line", lineDatation, position, los);
        AbsoluteDate minDate = lineSensor.getDate(firstLine);
        AbsoluteDate maxDate = lineSensor.getDate(lastLine);

        TileUpdater updater =
                new RandomLandscapeUpdater(0.0, 9000.0, 0.5, 0xe12ef744f224cf43l,
                                           FastMath.toRadians(1.0), 257);

        RuggedBuilder builder = new RuggedBuilder().
                setDigitalElevationModel(updater, 8).
                setEllipsoid(EllipsoidId.WGS84, BodyRotatingFrameId.ITRF).
                setTimeSpan(minDate, maxDate, 0.001, 5.0).
                setTrajectory(InertialFrameId.EME2000,
                              TestUtils.orbitToPV(orbit, earth, minDate.shiftedBy(-1.0), maxDate.shiftedBy(+1.0), 0.25),
                              8, CartesianDerivativesFilter.USE_PV,
                              TestUtils.orbitToQ(orbit, earth, minDate.shiftedBy(-1.0), maxDate.shiftedBy(+1.0), 0.25),
                              2, AngularDerivativesFilter.USE_R).
                addLineSensor(lineSensor);
        GeodeticPoint[] gpDuvenhage =
                builder.setAlgorithm(AlgorithmId.DUVENHAGE).build().directLocation("line", 100);

        GeodeticPoint[] gpBasicScan =
                builder.setAlgorithm(AlgorithmId.BASIC_SLOW_EXHAUSTIVE_SCAN_FOR_TESTS_ONLY).build().directLocation("line", 100);

        double[] data = new double[gpDuvenhage.length]; 
        for (int i = 0; i < gpDuvenhage.length; ++i) {
            Vector3D pDuvenhage = earth.transform(gpDuvenhage[i]);
            Vector3D pBasicScan = earth.transform(gpBasicScan[i]);
            data[i] = Vector3D.distance(pDuvenhage, pBasicScan);
        }
        Assert.assertEquals(0.0, new Percentile(99).evaluate(data), 5.1e-4);

    }

    // the following test is disabled by default
    // it is only used to check timings, and also creates a large (38M) temporary file
    @Ignore
    @Test
    public void testInverseLocationTiming()
        throws RuggedException, OrekitException, URISyntaxException {

        long t0       = System.currentTimeMillis();
        int dimension = 2000;
        int nbSensors = 3;

        String path = getClass().getClassLoader().getResource("orekit-data").toURI().getPath();
        DataProvidersManager.getInstance().addProvider(new DirectoryCrawler(new File(path)));
        final BodyShape  earth = TestUtils.createEarth();
        final Orbit      orbit = TestUtils.createOrbit(Constants.EIGEN5C_EARTH_MU);

        AbsoluteDate crossing = new AbsoluteDate("2012-01-01T12:30:00.000", TimeScalesFactory.getUTC());

        List<LineSensor> sensors = new ArrayList<LineSensor>();
        for (int i = 0; i < nbSensors; ++i) {
            // one line sensor
            // position: 1.5m in front (+X) and 20 cm above (-Z) of the S/C center of mass
            // los: swath in the (YZ) plane, looking roughly at 50° roll (sensor-dependent), 2.6" per pixel
            Vector3D position = new Vector3D(1.5, 0, -0.2);
            TimeDependentLOS los = TestUtils.createLOSPerfectLine(new Rotation(Vector3D.PLUS_I,
                                                                               FastMath.toRadians(50.0 - 0.001 * i),
                                                                               RotationConvention.VECTOR_OPERATOR).applyTo(Vector3D.PLUS_K),
                                                                  Vector3D.PLUS_I, FastMath.toRadians(dimension * 2.6 / 3600.0), dimension).build();

            // linear datation model: at reference time we get roughly middle line, and the rate is one line every 1.5ms
            LineDatation lineDatation = new LinearLineDatation(crossing, i + dimension / 2, 1.0 / 1.5e-3);
            sensors.add(new LineSensor("line-" + i, lineDatation, position, los));
        }

        int firstLine = 0;
        int lastLine  = dimension;
        AbsoluteDate minDate = sensors.get(0).getDate(firstLine).shiftedBy(-1.0);
        AbsoluteDate maxDate = sensors.get(sensors.size() - 1).getDate(lastLine).shiftedBy(+1.0);

        TileUpdater updater =
                new RandomLandscapeUpdater(0.0, 9000.0, 0.5, 0xf0a401650191f9f6l,
                                           FastMath.toRadians(1.0), 257);

        RuggedBuilder builder = new RuggedBuilder().
                setDigitalElevationModel(updater, 8).
                setAlgorithm(AlgorithmId.DUVENHAGE).
                setEllipsoid(EllipsoidId.WGS84, BodyRotatingFrameId.ITRF).
                setTimeSpan(minDate, maxDate, 0.001, 5.0).
                setTrajectory(InertialFrameId.EME2000,
                              TestUtils.orbitToPV(orbit, earth, minDate.shiftedBy(-1.0), maxDate.shiftedBy(+1.0), 0.25),
                              8, CartesianDerivativesFilter.USE_PV,
                              TestUtils.orbitToQ(orbit, earth, minDate.shiftedBy(-1.0), maxDate.shiftedBy(+1.0), 0.25),
                              2, AngularDerivativesFilter.USE_R);
        builder.setLightTimeCorrection(true);
        builder.setAberrationOfLightCorrection(true);
        for (LineSensor lineSensor : sensors) {
            builder.addLineSensor(lineSensor);
        }
        Rugged rugged = builder.build();

        double lat0  = FastMath.toRadians(-22.9);
        double lon0  = FastMath.toRadians(142.4);
        double delta = FastMath.toRadians(0.5);

        try {
            long             size   = nbSensors * dimension * dimension * 2l * Integer.SIZE / 8l;
            RandomAccessFile out    = new RandomAccessFile(tempFolder.newFile(), "rw");
            MappedByteBuffer buffer = out.getChannel().map(FileChannel.MapMode.READ_WRITE, 0, size);

            long t1 = System.currentTimeMillis();
            int goodPixels = 0;
            int badPixels  = 0;
            for (final LineSensor lineSensor : sensors) {
                for (int i = 0; i < dimension; ++i) {
                    double latitude  = lat0 + (i * delta) / dimension;
                    if (i %100 == 0) {
                        System.out.println(lineSensor.getName() + " " + i);
                    }
                    for (int j = 0; j < dimension; ++j) {
                        double longitude = lon0 + (j * delta) / dimension;
                        SensorPixel sp = rugged.inverseLocation(lineSensor.getName(), latitude, longitude,
                                                                firstLine, lastLine);
                        if (sp == null) {
                            ++badPixels;
                            buffer.putInt(-1);
                            buffer.putInt(-1);
                        } else {
                            ++goodPixels;
                            final int lineCode  = (int) FastMath.rint(FastMath.scalb(sp.getLineNumber(),  16));
                            final int pixelCode = (int) FastMath.rint(FastMath.scalb(sp.getPixelNumber(), 16));
                            buffer.putInt(lineCode);
                            buffer.putInt(pixelCode);
                        }
                    }
                }
            }

            long t2 = System.currentTimeMillis();
            out.close();
            int sizeM = (int) (size / (1024l * 1024l));
            System.out.format(Locale.US,
                              "%n%n%5dx%5d, %d sensors:%n" +
                              "  Orekit initialization and DEM creation   : %5.1fs%n" +
                              "  inverse location and %3dM grid writing: %5.1fs (%.1f px/s, %.1f%% covered)%n",
                              dimension, dimension, nbSensors,
                              1.0e-3 * (t1 - t0), sizeM, 1.0e-3 * (t2 - t1),
                              (badPixels + goodPixels) / (1.0e-3 * (t2 - t1)),
                              (100.0 * goodPixels) / (goodPixels + badPixels));
        } catch (IOException ioe) {
            Assert.fail(ioe.getLocalizedMessage());
        }
    }

    @Test
    public void testInverseLocation()
        throws RuggedException, OrekitException, URISyntaxException {
        checkInverseLocation(2000, false, false, 4.0e-7, 5.0e-6);
        checkInverseLocation(2000, false, true,  1.0e-5, 2.0e-7);
        checkInverseLocation(2000, true,  false, 4.0e-7, 4.0e-7);
        checkInverseLocation(2000, true,  true,  2.0e-5, 3.0e-7);
    }

    @Test
    public void testDateLocation()
        throws RuggedException, OrekitException, URISyntaxException {
        checkDateLocation(2000, false, false, 7.0e-7);
        checkDateLocation(2000, false, true,  2.0e-5);
        checkDateLocation(2000, true,  false, 8.0e-7);
        checkDateLocation(2000, true,  true,  3.0e-6);
    }
    
    @Test
    public void testLineDatation()
        throws RuggedException, OrekitException, URISyntaxException {
        checkLineDatation(2000, 7.0e-7);
        checkLineDatation(10000, 8.0e-7);
    }


    @Test
    public void testInverseLocNearLineEnd() throws OrekitException, RuggedException, URISyntaxException {

        String path = getClass().getClassLoader().getResource("orekit-data").toURI().getPath();
        DataProvidersManager.getInstance().addProvider(new DirectoryCrawler(new File(path)));
        Vector3D offset = Vector3D.ZERO;
        TimeScale gps = TimeScalesFactory.getGPS();
        Frame eme2000 = FramesFactory.getEME2000();
        Frame itrf = FramesFactory.getITRF(IERSConventions.IERS_2010, true);
        ArrayList<TimeStampedAngularCoordinates> satelliteQList = new ArrayList<TimeStampedAngularCoordinates>();
        ArrayList<TimeStampedPVCoordinates> satellitePVList = new ArrayList<TimeStampedPVCoordinates>();

        TestUtils.addSatelliteQ(gps, satelliteQList, "2009-12-11T16:58:42.592937", -0.340236d, 0.333952d, -0.844012d, -0.245684d);
        TestUtils.addSatelliteQ(gps, satelliteQList, "2009-12-11T16:59:06.592937", -0.354773d, 0.329336d, -0.837871d, -0.252281d);
        TestUtils.addSatelliteQ(gps, satelliteQList, "2009-12-11T16:59:30.592937", -0.369237d, 0.324612d, -0.831445d, -0.258824d);
        TestUtils.addSatelliteQ(gps, satelliteQList, "2009-12-11T16:59:54.592937", -0.3836d, 0.319792d, -0.824743d, -0.265299d);
        TestUtils.addSatelliteQ(gps, satelliteQList, "2009-12-11T17:00:18.592937", -0.397834d, 0.314883d, -0.817777d, -0.271695d);
        TestUtils.addSatelliteQ(gps, satelliteQList, "2009-12-11T17:00:42.592937", -0.411912d, 0.309895d, -0.810561d, -0.278001d);
        TestUtils.addSatelliteQ(gps, satelliteQList, "2009-12-11T17:01:06.592937", -0.42581d, 0.304838d, -0.803111d, -0.284206d);
        TestUtils.addSatelliteQ(gps, satelliteQList, "2009-12-11T17:01:30.592937", -0.439505d, 0.299722d, -0.795442d, -0.290301d);
        TestUtils.addSatelliteQ(gps, satelliteQList, "2009-12-11T17:01:54.592937", -0.452976d, 0.294556d, -0.787571d, -0.296279d);
        TestUtils.addSatelliteQ(gps, satelliteQList, "2009-12-11T17:02:18.592937", -0.466207d, 0.28935d, -0.779516d, -0.302131d);


        TestUtils.addSatellitePV(gps, eme2000, itrf, satellitePVList, "2009-12-11T16:58:42.592937", -726361.466d, -5411878.485d, 4637549.599d, -2068.995d, -4500.601d, -5576.736d);
        TestUtils.addSatellitePV(gps, eme2000, itrf, satellitePVList, "2009-12-11T16:59:04.192937", -779538.267d, -5506500.533d, 4515934.894d, -2058.308d, -4369.521d, -5683.906d);
        TestUtils.addSatellitePV(gps, eme2000, itrf, satellitePVList, "2009-12-11T16:59:25.792937", -832615.368d, -5598184.195d, 4392036.13d, -2046.169d, -4236.279d, -5788.201d);
        TestUtils.addSatellitePV(gps, eme2000, itrf, satellitePVList, "2009-12-11T16:59:47.392937", -885556.748d, -5686883.696d, 4265915.971d, -2032.579d, -4100.944d, -5889.568d);
        TestUtils.addSatellitePV(gps, eme2000, itrf, satellitePVList, "2009-12-11T17:00:08.992937", -938326.32d, -5772554.875d, 4137638.207d, -2017.537d, -3963.59d, -5987.957d);
        TestUtils.addSatellitePV(gps, eme2000, itrf, satellitePVList, "2009-12-11T17:00:30.592937", -990887.942d, -5855155.21d, 4007267.717d, -2001.046d, -3824.291d, -6083.317d);
        TestUtils.addSatellitePV(gps, eme2000, itrf, satellitePVList, "2009-12-11T17:00:52.192937", -1043205.448d, -5934643.836d, 3874870.441d, -1983.107d, -3683.122d, -6175.6d);
        TestUtils.addSatellitePV(gps, eme2000, itrf, satellitePVList, "2009-12-11T17:01:13.792937", -1095242.669d, -6010981.571d, 3740513.34d, -1963.723d, -3540.157d, -6264.76d);
        TestUtils.addSatellitePV(gps, eme2000, itrf, satellitePVList, "2009-12-11T17:01:35.392937", -1146963.457d, -6084130.93d, 3604264.372d, -1942.899d, -3395.473d, -6350.751d);
        TestUtils.addSatellitePV(gps, eme2000, itrf, satellitePVList, "2009-12-11T17:01:56.992937", -1198331.706d, -6154056.146d, 3466192.446d, -1920.64d, -3249.148d, -6433.531d);
        TestUtils.addSatellitePV(gps, eme2000, itrf, satellitePVList, "2009-12-11T17:02:18.592937", -1249311.381d, -6220723.191d, 3326367.397d, -1896.952d, -3101.26d, -6513.056d);

        TileUpdater updater = new RandomLandscapeUpdater(0.0, 9000.0, 0.5, 0xf0a401650191f9f6l, FastMath.toRadians(1.0), 257);
        RuggedBuilder builder = new RuggedBuilder().
                setDigitalElevationModel(updater, 8).
                setAlgorithm(AlgorithmId.DUVENHAGE).
                setEllipsoid(EllipsoidId.WGS84, BodyRotatingFrameId.ITRF).
                setTimeSpan(satellitePVList.get(0).getDate(), satellitePVList.get(satellitePVList.size() - 1).getDate(), 0.001, 5.0).
                setTrajectory(InertialFrameId.EME2000,
                              satellitePVList, 8, CartesianDerivativesFilter.USE_PV,
                              satelliteQList, 8, AngularDerivativesFilter.USE_R);

        List<Vector3D> lineOfSight = new ArrayList<Vector3D>();
        lineOfSight.add(new Vector3D(-0.011204, 0.181530, 1d).normalize());
        lineOfSight.add(new Vector3D(-0.011204, 0.181518, 1d).normalize());
        lineOfSight.add(new Vector3D(-0.011204, 0.181505, 1d).normalize());
        lineOfSight.add(new Vector3D(-0.011204, 0.181492, 1d).normalize());
        lineOfSight.add(new Vector3D(-0.011204, 0.181480, 1d).normalize());
        lineOfSight.add(new Vector3D(-0.011204, 0.181467, 1d).normalize());
        lineOfSight.add(new Vector3D(-0.011204, 0.181455, 1d).normalize());
        lineOfSight.add(new Vector3D(-0.011204, 0.181442, 1d).normalize());
        lineOfSight.add(new Vector3D(-0.011204, 0.181430, 1d).normalize());
        lineOfSight.add(new Vector3D(-0.011204, 0.181417, 1d).normalize());
        lineOfSight.add(new Vector3D(-0.011204, 0.181405, 1d).normalize());
        lineOfSight.add(new Vector3D(-0.011204, 0.181392, 1d).normalize());

        lineOfSight.add(new Vector3D(-0.011204, 0.149762, 1d).normalize());
        lineOfSight.add(new Vector3D(-0.011204, 0.149749, 1d).normalize());
        lineOfSight.add(new Vector3D(-0.011204, 0.149737, 1d).normalize());
        lineOfSight.add(new Vector3D(-0.011204, 0.149724, 1d).normalize());
        lineOfSight.add(new Vector3D(-0.011204, 0.149712, 1d).normalize());
        lineOfSight.add(new Vector3D(-0.011204, 0.149699, 1d).normalize());
        lineOfSight.add(new Vector3D(-0.011204, 0.149686, 1d).normalize());
        lineOfSight.add(new Vector3D(-0.011204, 0.149674, 1d).normalize());
        lineOfSight.add(new Vector3D(-0.011204, 0.149661, 1d).normalize());
        lineOfSight.add(new Vector3D(-0.011204, 0.149649, 1d).normalize());
        lineOfSight.add(new Vector3D(-0.011204, 0.149636, 1d).normalize());
        lineOfSight.add(new Vector3D(-0.011204, 0.149624, 1d).normalize());
        lineOfSight.add(new Vector3D(-0.011204, 0.149611, 1d).normalize());
        lineOfSight.add(new Vector3D(-0.011204, 0.149599, 1d).normalize());
        lineOfSight.add(new Vector3D(-0.011204, 0.149586, 1d).normalize());

        AbsoluteDate absDate = new AbsoluteDate("2009-12-11T16:58:51.593", gps);
        LinearLineDatation lineDatation = new LinearLineDatation(absDate, 1d, 638.5696040868454);
        LineSensor lineSensor = new LineSensor("perfect-line", lineDatation, offset,
                                               new LOSBuilder(lineOfSight).build());
        builder.addLineSensor(lineSensor);

        Rugged rugged = builder.build();
        GeodeticPoint point1 = new GeodeticPoint(0.7053784581520293, -1.7354535645320581, 691.856741468848);
        SensorPixel sensorPixel1 = rugged.inverseLocation(lineSensor.getName(), point1, 1, 131328);
        Assert.assertEquals(   2.01474, sensorPixel1.getLineNumber(), 1.0e-5);
        Assert.assertEquals(   2.09271, sensorPixel1.getPixelNumber(), 1.0e-5);
        GeodeticPoint point2 = new GeodeticPoint(0.704463899881073, -1.7303503789334154, 648.9200602492216);
        SensorPixel sensorPixel2 = rugged.inverseLocation(lineSensor.getName(), point2, 1, 131328);
        Assert.assertEquals(   2.02185, sensorPixel2.getLineNumber(), 1.0e-5);
        Assert.assertEquals(  27.53008, sensorPixel2.getPixelNumber(), 1.0e-5);
        GeodeticPoint point3 = new GeodeticPoint(0.7009593480939814, -1.7314283804521957, 588.3075485689468);
        SensorPixel sensorPixel3 = rugged.inverseLocation(lineSensor.getName(), point3, 1, 131328);
        Assert.assertEquals(2305.26101, sensorPixel3.getLineNumber(),  1.0e-5);
        Assert.assertEquals(  27.18381, sensorPixel3.getPixelNumber(), 1.0e-5);
        GeodeticPoint point4 = new GeodeticPoint(0.7018731669637096, -1.73651769725183, 611.2759403696498);
        SensorPixel sensorPixel4 = rugged.inverseLocation(lineSensor.getName(), point4, 1, 131328);
        Assert.assertEquals(2305.25436, sensorPixel4.getLineNumber(), 1.0e-5);
        Assert.assertEquals(   1.54447, sensorPixel4.getPixelNumber(), 1.0e-5);

    }

    @Test
    public void testInverseLoc() throws OrekitException, RuggedException, URISyntaxException {

        String path = getClass().getClassLoader().getResource("orekit-data").toURI().getPath();
        DataProvidersManager.getInstance().addProvider(new DirectoryCrawler(new File(path)));
        Vector3D offset = Vector3D.ZERO;
        TimeScale gps = TimeScalesFactory.getGPS();
        Frame eme2000 = FramesFactory.getEME2000();
        Frame itrf = FramesFactory.getITRF(IERSConventions.IERS_2010, true);
        ArrayList<TimeStampedAngularCoordinates> satelliteQList = new ArrayList<TimeStampedAngularCoordinates>();
        ArrayList<TimeStampedPVCoordinates> satellitePVList = new ArrayList<TimeStampedPVCoordinates>();

        TestUtils.addSatelliteQ(gps, satelliteQList, "2013-07-07T17:16:27", -0.327993d, -0.715194d, -0.56313d, 0.252592d);
        TestUtils.addSatelliteQ(gps, satelliteQList, "2013-07-07T17:16:29", -0.328628d, -0.71494d, -0.562769d, 0.25329d);
        TestUtils.addSatelliteQ(gps, satelliteQList, "2013-07-07T17:16:31", -0.329263d, -0.714685d, -0.562407d, 0.253988d);
        TestUtils.addSatelliteQ(gps, satelliteQList, "2013-07-07T17:16:33", -0.329898d, -0.714429d, -0.562044d, 0.254685d);
        TestUtils.addSatelliteQ(gps, satelliteQList, "2013-07-07T17:16:35", -0.330532d, -0.714173d, -0.561681d, 0.255383d);
        TestUtils.addSatelliteQ(gps, satelliteQList, "2013-07-07T17:16:37", -0.331166d, -0.713915d, -0.561318d, 0.256079d);
        TestUtils.addSatelliteQ(gps, satelliteQList, "2013-07-07T17:16:39", -0.3318d, -0.713657d, -0.560954d, 0.256776d);
        TestUtils.addSatelliteQ(gps, satelliteQList, "2013-07-07T17:16:41", -0.332434d, -0.713397d, -0.560589d, 0.257472d);
        TestUtils.addSatelliteQ(gps, satelliteQList, "2013-07-07T17:16:43", -0.333067d, -0.713137d, -0.560224d, 0.258168d);
        TestUtils.addSatelliteQ(gps, satelliteQList, "2013-07-07T17:16:45", -0.333699d, -0.712876d, -0.559859d, 0.258864d);
        TestUtils.addSatelliteQ(gps, satelliteQList, "2013-07-07T17:18:17", -0.36244d, -0.699935d, -0.542511d, 0.290533d);
        TestUtils.addSatelliteQ(gps, satelliteQList, "2013-07-07T17:20:27", -0.401688d, -0.678574d, -0.516285d, 0.334116d);
        TestUtils.addSatelliteQ(gps, satelliteQList, "2013-07-07T17:20:29", -0.402278d, -0.678218d, -0.515866d, 0.334776d);
        TestUtils.addSatelliteQ(gps, satelliteQList, "2013-07-07T17:20:31", -0.402868d, -0.677861d, -0.515447d, 0.335435d);
        TestUtils.addSatelliteQ(gps, satelliteQList, "2013-07-07T17:20:33", -0.403457d, -0.677503d, -0.515028d, 0.336093d);
        TestUtils.addSatelliteQ(gps, satelliteQList, "2013-07-07T17:20:35", -0.404046d, -0.677144d, -0.514608d, 0.336752d);
        TestUtils.addSatelliteQ(gps, satelliteQList, "2013-07-07T17:20:37", -0.404634d, -0.676785d, -0.514187d, 0.337409d);
        TestUtils.addSatelliteQ(gps, satelliteQList, "2013-07-07T17:20:39", -0.405222d, -0.676424d, -0.513767d, 0.338067d);
        TestUtils.addSatelliteQ(gps, satelliteQList, "2013-07-07T17:20:41", -0.40581d, -0.676063d, -0.513345d, 0.338724d);
        TestUtils.addSatelliteQ(gps, satelliteQList, "2013-07-07T17:20:43", -0.406397d, -0.675701d, -0.512924d, 0.339381d);
        TestUtils.addSatelliteQ(gps, satelliteQList, "2013-07-07T17:20:45", -0.406983d, -0.675338d, -0.512502d, 0.340038d);

        TestUtils.addSatellitePV(gps, eme2000, itrf, satellitePVList, "2013-07-07T17:16:27.857531", -379110.393d, -5386317.278d, 4708158.61d, -1802.078d, -4690.847d,  -5512.223d);
        TestUtils.addSatellitePV(gps, eme2000, itrf, satellitePVList, "2013-07-07T17:16:36.857531", -398874.476d, -5428039.968d, 4658344.906d, -1801.326d, -4636.91d,  -5557.915d);
        TestUtils.addSatellitePV(gps, eme2000, itrf, satellitePVList, "2013-07-07T17:16:45.857531", -418657.992d, -5469262.453d, 4608122.145d, -1800.345d, -4582.57d,  -5603.119d);
        TestUtils.addSatellitePV(gps, eme2000, itrf, satellitePVList, "2013-07-07T17:16:54.857531", -438458.554d, -5509981.109d, 4557494.737d, -1799.136d, -4527.831d, -5647.831d);
        TestUtils.addSatellitePV(gps, eme2000, itrf, satellitePVList, "2013-07-07T17:17:03.857531", -458273.771d, -5550192.355d, 4506467.128d, -1797.697d, -4472.698d, -5692.046d);
        TestUtils.addSatellitePV(gps, eme2000, itrf, satellitePVList, "2013-07-07T17:17:12.857531", -478101.244d, -5589892.661d, 4455043.798d, -1796.029d, -4417.176d, -5735.762d);
        TestUtils.addSatellitePV(gps, eme2000, itrf, satellitePVList, "2013-07-07T17:17:21.857531", -497938.57d, -5629078.543d, 4403229.263d, -1794.131d, -4361.271d,  -5778.975d);
        TestUtils.addSatellitePV(gps, eme2000, itrf, satellitePVList, "2013-07-07T17:17:30.857531", -517783.34d, -5667746.565d, 4351028.073d, -1792.003d, -4304.987d,  -5821.679d);
        TestUtils.addSatellitePV(gps, eme2000, itrf, satellitePVList, "2013-07-07T17:17:39.857531", -537633.139d, -5705893.34d, 4298444.812d, -1789.644d, -4248.329d,  -5863.873d);
        TestUtils.addSatellitePV(gps, eme2000, itrf, satellitePVList, "2013-07-07T17:17:48.857531", -557485.549d, -5743515.53d, 4245484.097d, -1787.055d, -4191.304d,  -5905.552d);
        TestUtils.addSatellitePV(gps, eme2000, itrf, satellitePVList, "2013-07-07T17:17:57.857531", -577338.146d, -5780609.846d, 4192150.579d, -1784.234d, -4133.916d, -5946.712d);
        TestUtils.addSatellitePV(gps, eme2000, itrf, satellitePVList, "2013-07-07T17:18:06.857531", -597188.502d, -5817173.047d, 4138448.941d, -1781.183d, -4076.171d, -5987.35d);
        TestUtils.addSatellitePV(gps, eme2000, itrf, satellitePVList, "2013-07-07T17:18:15.857531", -617034.185d, -5853201.943d, 4084383.899d, -1777.899d, -4018.073d, -6027.462d);
        TestUtils.addSatellitePV(gps, eme2000, itrf, satellitePVList, "2013-07-07T17:18:24.857531", -636872.759d, -5888693.393d, 4029960.2d, -1774.385d, -3959.629d,   -6067.045d);
        TestUtils.addSatellitePV(gps, eme2000, itrf, satellitePVList, "2013-07-07T17:18:33.857531", -656701.786d, -5923644.307d, 3975182.623d, -1770.638d, -3900.844d, -6106.095d);
        TestUtils.addSatellitePV(gps, eme2000, itrf, satellitePVList, "2013-07-07T17:18:42.857531", -676518.822d, -5958051.645d, 3920055.979d, -1766.659d, -3841.723d, -6144.609d);
        TestUtils.addSatellitePV(gps, eme2000, itrf, satellitePVList, "2013-07-07T17:18:51.857531", -696321.424d, -5991912.417d, 3864585.108d, -1762.449d, -3782.271d, -6182.583d);
        TestUtils.addSatellitePV(gps, eme2000, itrf, satellitePVList, "2013-07-07T17:19:00.857531", -716107.143d, -6025223.686d, 3808774.881d, -1758.006d, -3722.495d, -6220.015d);
        TestUtils.addSatellitePV(gps, eme2000, itrf, satellitePVList, "2013-07-07T17:19:09.857531", -735873.528d, -6057982.563d, 3752630.2d, -1753.332d, -3662.399d,   -6256.9d);
        TestUtils.addSatellitePV(gps, eme2000, itrf, satellitePVList, "2013-07-07T17:19:18.857531", -755618.129d, -6090186.214d, 3696155.993d, -1748.425d, -3601.99d,  -6293.236d);
        TestUtils.addSatellitePV(gps, eme2000, itrf, satellitePVList, "2013-07-07T17:19:27.857531", -775338.49d, -6121831.854d, 3639357.221d, -1743.286d, -3541.272d,  -6329.019d);
        TestUtils.addSatellitePV(gps, eme2000, itrf, satellitePVList, "2013-07-07T17:19:36.857531", -795032.157d, -6152916.751d, 3582238.87d, -1737.915d, -3480.252d,  -6364.246d);
        TestUtils.addSatellitePV(gps, eme2000, itrf, satellitePVList, "2013-07-07T17:19:45.857531", -814696.672d, -6183438.226d, 3524805.957d, -1732.313d, -3418.935d, -6398.915d);
        TestUtils.addSatellitePV(gps, eme2000, itrf, satellitePVList, "2013-07-07T17:19:54.857531", -834329.579d, -6213393.652d, 3467063.525d, -1726.478d, -3357.327d, -6433.022d);
        TestUtils.addSatellitePV(gps, eme2000, itrf, satellitePVList, "2013-07-07T17:20:03.857531", -853928.418d, -6242780.453d, 3409016.644d, -1720.412d, -3295.433d, -6466.563d);
        TestUtils.addSatellitePV(gps, eme2000, itrf, satellitePVList, "2013-07-07T17:20:12.857531", -873490.732d, -6271596.108d, 3350670.411d, -1714.114d, -3233.259d, -6499.537d);
        TestUtils.addSatellitePV(gps, eme2000, itrf, satellitePVList, "2013-07-07T17:20:21.857531", -893014.061d, -6299838.148d, 3292029.951d, -1707.585d, -3170.811d, -6531.941d);
        TestUtils.addSatellitePV(gps, eme2000, itrf, satellitePVList, "2013-07-07T17:20:30.857531", -912495.948d, -6327504.159d, 3233100.411d, -1700.825d, -3108.095d, -6563.77d);
        TestUtils.addSatellitePV(gps, eme2000, itrf, satellitePVList, "2013-07-07T17:20:39.857531", -931933.933d, -6354591.778d, 3173886.968d, -1693.833d, -3045.116d, -6595.024d);

        List<Vector3D> lineOfSight = new ArrayList<Vector3D>();
        lineOfSight.add(new Vector3D(0.0046536264d, -0.1851800945d, 1d));
        lineOfSight.add(new Vector3D(0.0000001251d, -0.0002815246d, 1d));
        lineOfSight.add(new Vector3D(0.0046694108d, 0.1853863933d, 1d));

        AbsoluteDate absDate = new AbsoluteDate("2013-07-07T17:16:36.857", gps);
        LinearLineDatation lineDatation = new LinearLineDatation(absDate, 0.03125d, 19.95565693384045);
        LineSensor lineSensor = new LineSensor("QUICK_LOOK", lineDatation, offset,
                                               new LOSBuilder(lineOfSight).build());
        Rugged rugged = new RuggedBuilder().
                setAlgorithm(AlgorithmId.IGNORE_DEM_USE_ELLIPSOID).
                setEllipsoid(EllipsoidId.WGS84, BodyRotatingFrameId.ITRF).
                setTimeSpan(satellitePVList.get(0).getDate(),
                            satellitePVList.get(satellitePVList.size() - 1).getDate(), 0.1, 10.0).
                setTrajectory(InertialFrameId.EME2000,
                              satellitePVList, 6, CartesianDerivativesFilter.USE_P,
                              satelliteQList, 8, AngularDerivativesFilter.USE_R).
                addLineSensor(lineSensor).
                build();

        GeodeticPoint[] temp = rugged.directLocation("QUICK_LOOK", -250);
        GeodeticPoint first = temp[0];
        double minLon = first.getLongitude();
        double minLat = first.getLatitude();
        temp = rugged.directLocation("QUICK_LOOK", 350);
        GeodeticPoint last = temp[temp.length - 1];
        double maxLon = last.getLongitude();
        double maxLat = last.getLatitude();

        GeodeticPoint gp = new GeodeticPoint((minLat + maxLat) / 2d, (minLon + maxLon) / 2d, 0d);
        SensorPixel sensorPixel = rugged.inverseLocation("QUICK_LOOK", gp, -250, 350);

        Assert.assertNotNull(sensorPixel);

    }

    @Test
    public void testInverseLocCurvedLine()
        throws RuggedException, URISyntaxException, OrekitException {

        String path = getClass().getClassLoader().getResource("orekit-data").toURI().getPath();
        DataProvidersManager.getInstance().addProvider(new DirectoryCrawler(new File(path)));
        final BodyShape  earth = TestUtils.createEarth();
        final Orbit      orbit = TestUtils.createOrbit(Constants.EIGEN5C_EARTH_MU);

        AbsoluteDate crossing = new AbsoluteDate("2012-01-01T12:30:00.000", TimeScalesFactory.getUTC());
        int dimension = 200;

        // one line sensor
        // position: 1.5m in front (+X) and 20 cm above (-Z) of the S/C center of mass
        // los: swath in the (YZ) plane, looking at nadir, 2.6" per pixel, 3" sagitta
        Vector3D position = new Vector3D(1.5, 0, -0.2);
        TimeDependentLOS los = TestUtils.createLOSCurvedLine(Vector3D.PLUS_K, Vector3D.PLUS_I,
                                                             FastMath.toRadians(dimension * 2.6 / 3600.0),
                                                             FastMath.toRadians(3.0 / 3600.0),
                                                             dimension);

        // linear datation model: at reference time we get the middle line, and the rate is one line every 1.5ms
        LineDatation lineDatation = new LinearLineDatation(crossing, dimension / 2, 1.0 / 1.5e-3);
        int firstLine = 0;
        int lastLine  = dimension;
        LineSensor lineSensor = new LineSensor("curved", lineDatation, position, los);
        AbsoluteDate minDate = lineSensor.getDate(firstLine).shiftedBy(-1.0);
        AbsoluteDate maxDate = lineSensor.getDate(lastLine).shiftedBy(+1.0);

        TileUpdater updater =
                new RandomLandscapeUpdater(0.0, 9000.0, 0.3, 0xf0a401650191f9f6l,
                                           FastMath.toRadians(1.0), 257);

        Rugged rugged = new RuggedBuilder().
                setDigitalElevationModel(updater, 8).
                setAlgorithm(AlgorithmId.DUVENHAGE).
                setEllipsoid(EllipsoidId.WGS84, BodyRotatingFrameId.ITRF).
                setTimeSpan(minDate, maxDate, 0.001, 5.0).
                setTrajectory(InertialFrameId.EME2000,
                              TestUtils.orbitToPV(orbit, earth, minDate.shiftedBy(-1.0), maxDate.shiftedBy(+1.0), 0.25),
                              8, CartesianDerivativesFilter.USE_PV,
                              TestUtils.orbitToQ(orbit, earth, minDate.shiftedBy(-1.0), maxDate.shiftedBy(+1.0), 0.25),
                              2, AngularDerivativesFilter.USE_R).
                addLineSensor(lineSensor).
                build();

        int lineNumber = 97;
        GeodeticPoint[] gp = rugged.directLocation("curved", lineNumber);
        for (int i = 0; i < gp.length; ++i) {
            SensorPixel pixel = rugged.inverseLocation("curved", gp[i], firstLine, lastLine);
            Assert.assertEquals(lineNumber, pixel.getLineNumber(),  5.0e-4);
            Assert.assertEquals(i,          pixel.getPixelNumber(), 3.1e-7);
        }

    }

    private void checkInverseLocation(int dimension, boolean lightTimeCorrection, boolean aberrationOfLightCorrection,
                                      double maxLineError, double maxPixelError)
        throws RuggedException, OrekitException, URISyntaxException {

        String path = getClass().getClassLoader().getResource("orekit-data").toURI().getPath();
        DataProvidersManager.getInstance().addProvider(new DirectoryCrawler(new File(path)));
        final BodyShape  earth = TestUtils.createEarth();
        final Orbit      orbit = TestUtils.createOrbit(Constants.EIGEN5C_EARTH_MU);

        AbsoluteDate crossing = new AbsoluteDate("2012-01-01T12:30:00.000", TimeScalesFactory.getUTC());

        // one line sensor
        // position: 1.5m in front (+X) and 20 cm above (-Z) of the S/C center of mass
        // los: swath in the (YZ) plane, looking at 50° roll, 2.6" per pixel
        Vector3D position = new Vector3D(1.5, 0, -0.2);
        TimeDependentLOS los = TestUtils.createLOSPerfectLine(new Rotation(Vector3D.PLUS_I,
                                                                           FastMath.toRadians(50.0),
                                                                           RotationConvention.VECTOR_OPERATOR).applyTo(Vector3D.PLUS_K),
                                                              Vector3D.PLUS_I,
                                                              FastMath.toRadians(dimension * 2.6 / 3600.0), dimension).build();

        // linear datation model: at reference time we get the middle line, and the rate is one line every 1.5ms
        LineDatation lineDatation = new LinearLineDatation(crossing, dimension / 2, 1.0 / 1.5e-3);
        int firstLine = 0;
        int lastLine  = dimension;
        LineSensor lineSensor = new LineSensor("line", lineDatation, position, los);
        AbsoluteDate minDate = lineSensor.getDate(firstLine).shiftedBy(-1.0);
        AbsoluteDate maxDate = lineSensor.getDate(lastLine).shiftedBy(+1.0);

        TileUpdater updater =
                new RandomLandscapeUpdater(0.0, 9000.0, 0.3, 0xf0a401650191f9f6l,
                                           FastMath.toRadians(1.0), 257);

        Rugged rugged = new RuggedBuilder().
                setDigitalElevationModel(updater, 8).
                setAlgorithm(AlgorithmId.DUVENHAGE).
                setEllipsoid(EllipsoidId.WGS84, BodyRotatingFrameId.ITRF).
                setTimeSpan(minDate, maxDate, 0.001, 5.0).
                setTrajectory(InertialFrameId.EME2000,
                              TestUtils.orbitToPV(orbit, earth, minDate.shiftedBy(-1.0), maxDate.shiftedBy(+1.0), 0.25),
                              8, CartesianDerivativesFilter.USE_PV,
                              TestUtils.orbitToQ(orbit, earth, minDate.shiftedBy(-1.0), maxDate.shiftedBy(+1.0), 0.25),
                              2, AngularDerivativesFilter.USE_R).
                setLightTimeCorrection(lightTimeCorrection).
                setAberrationOfLightCorrection(aberrationOfLightCorrection).
                addLineSensor(lineSensor).
                build();

        double referenceLine = 0.87654 * dimension;
        GeodeticPoint[] gp = rugged.directLocation("line", referenceLine);

        for (double p = 0; p < gp.length - 1; p += 1.0) {
            int    i = (int) FastMath.floor(p);
            double d = p - i;
            SensorPixel sp = rugged.inverseLocation("line",
                                                        (1 - d) * gp[i].getLatitude()  + d * gp[i + 1].getLatitude(),
                                                        (1 - d) * gp[i].getLongitude() + d * gp[i + 1].getLongitude(),
                                                        0, dimension);
            Assert.assertEquals(referenceLine, sp.getLineNumber(),  maxLineError);
            Assert.assertEquals(p,             sp.getPixelNumber(), maxPixelError);
        }

        // point out of line (20 pixels before first pixel)
        Assert.assertNull(rugged.inverseLocation("line",
                                                    21 * gp[0].getLatitude()  - 20 * gp[1].getLatitude(),
                                                    21 * gp[0].getLongitude() - 20 * gp[1].getLongitude(),
                                                    0, dimension));

        // point out of line (20 pixels after last pixel)
        Assert.assertNull(rugged.inverseLocation("line",
                                                    -20 * gp[gp.length - 2].getLatitude()  + 21 * gp[gp.length - 1].getLatitude(),
                                                    -20 * gp[gp.length - 2].getLongitude() + 21 * gp[gp.length - 1].getLongitude(),
                                                    0, dimension));

        // point out of line (20 lines before first line)
        GeodeticPoint[] gp0 = rugged.directLocation("line", 0);
        GeodeticPoint[] gp1 = rugged.directLocation("line", 1);
        Assert.assertNull(rugged.inverseLocation("line",
                                                    21 * gp0[dimension / 2].getLatitude()  - 20 * gp1[dimension / 2].getLatitude(),
                                                    21 * gp0[dimension / 2].getLongitude() - 20 * gp1[dimension / 2].getLongitude(),
                                                    0, dimension));

        // point out of line (20 lines after last line)
        GeodeticPoint[] gp2 = rugged.directLocation("line", dimension - 2);
        GeodeticPoint[] gp3 = rugged.directLocation("line", dimension - 1);
        Assert.assertNull(rugged.inverseLocation("line",
                                                    -20 * gp2[dimension / 2].getLatitude()  + 21 * gp3[dimension / 2].getLatitude(),
                                                    -20 * gp2[dimension / 2].getLongitude() + 21 * gp3[dimension / 2].getLongitude(),
                                                    0, dimension));

    }

    @Test
    public void testInverseLocationDerivativesWithoutCorrections()
        throws RuggedException, OrekitException {
        doTestInverseLocationDerivatives(2000, false, false,
                                         8.0e-9, 3.0e-10, 5.0e-12, 9.0e-8);
    }

    @Test
    public void testInverseLocationDerivativesWithLightTimeCorrection()
        throws RuggedException, OrekitException {
        doTestInverseLocationDerivatives(2000, true, false,
                                         3.0e-9, 9.0e-9, 2.1e-12, 9.0e-8);
    }

    @Test
    public void testInverseLocationDerivativesWithAberrationOfLightCorrection()
        throws RuggedException, OrekitException {
        doTestInverseLocationDerivatives(2000, false, true,
                                         4.2e-10, 3.0e-10, 3.4e-12, 7.0e-8);
    }

    @Test
    public void testInverseLocationDerivativesWithAllCorrections()
        throws RuggedException, OrekitException {
        doTestInverseLocationDerivatives(2000, true, true,
                                         3.0e-10, 5.0e-10, 2.0e-12, 7.0e-8);
    }

    private void doTestInverseLocationDerivatives(int dimension,
                                                  boolean lightTimeCorrection,
                                                  boolean aberrationOfLightCorrection,
                                                  double lineTolerance,
                                                  double pixelTolerance,
                                                  double lineDerivativeRelativeTolerance,
                                                  double pixelDerivativeRelativeTolerance)
        throws RuggedException, OrekitException {
        try {

            String path = getClass().getClassLoader().getResource("orekit-data").toURI().getPath();
            DataProvidersManager.getInstance().addProvider(new DirectoryCrawler(new File(path)));
            final BodyShape  earth = TestUtils.createEarth();
            final Orbit      orbit = TestUtils.createOrbit(Constants.EIGEN5C_EARTH_MU);

            AbsoluteDate crossing = new AbsoluteDate("2012-01-01T12:30:00.000", TimeScalesFactory.getUTC());

            // one line sensor
            // position: 1.5m in front (+X) and 20 cm above (-Z) of the S/C center of mass
            // los: swath in the (YZ) plane, looking at 50° roll, 2.6" per pixel
            Vector3D position = new Vector3D(1.5, 0, -0.2);
            LOSBuilder losBuilder =
             TestUtils.createLOSPerfectLine(new Rotation(Vector3D.PLUS_I,
                                                         FastMath.toRadians(50.0),
                                                         RotationConvention.VECTOR_OPERATOR).applyTo(Vector3D.PLUS_K),
                                            Vector3D.PLUS_I,
                                            FastMath.toRadians(dimension * 2.6 / 3600.0), dimension);
            losBuilder.addTransform(new FixedRotation("roll",  Vector3D.MINUS_I, 0.0));
            losBuilder.addTransform(new FixedRotation("pitch", Vector3D.MINUS_J, 0.0));
            TimeDependentLOS los = losBuilder.build();

            // linear datation model: at reference time we get the middle line, and the rate is one line every 1.5ms
            LineDatation lineDatation = new LinearLineDatation(crossing, dimension / 2, 1.0 / 1.5e-3);
            int firstLine = 0;
            int lastLine  = dimension;
            final LineSensor lineSensor = new LineSensor("line", lineDatation, position, los);
            AbsoluteDate minDate = lineSensor.getDate(firstLine).shiftedBy(-1.0);
            AbsoluteDate maxDate = lineSensor.getDate(lastLine).shiftedBy(+1.0);

            TileUpdater updater =
                            new RandomLandscapeUpdater(0.0, 9000.0, 0.3, 0xf0a401650191f9f6l,
                                                       FastMath.toRadians(1.0), 257);

            Rugged rugged = new RuggedBuilder().
                            setDigitalElevationModel(updater, 8).
                            setAlgorithm(AlgorithmId.DUVENHAGE).
                            setEllipsoid(EllipsoidId.WGS84, BodyRotatingFrameId.ITRF).
                            setTimeSpan(minDate, maxDate, 0.001, 5.0).
                            setTrajectory(InertialFrameId.EME2000,
                                          TestUtils.orbitToPV(orbit, earth, minDate.shiftedBy(-1.0), maxDate.shiftedBy(+1.0), 0.25),
                                          8, CartesianDerivativesFilter.USE_PV,
                                          TestUtils.orbitToQ(orbit, earth, minDate.shiftedBy(-1.0), maxDate.shiftedBy(+1.0), 0.25),
                                          2, AngularDerivativesFilter.USE_R).
                            setLightTimeCorrection(lightTimeCorrection).
                            setAberrationOfLightCorrection(aberrationOfLightCorrection).
                            addLineSensor(lineSensor).
                            build();

            // we want to adjust sensor roll and pitch angles
            ParameterDriver rollDriver =
                            lineSensor.getParametersDrivers().
                            filter(driver -> driver.getName().equals("roll")).findFirst().get();
            rollDriver.setSelected(true);
            ParameterDriver pitchDriver =
                            lineSensor.getParametersDrivers().
                            filter(driver -> driver.getName().equals("pitch")).findFirst().get();
            pitchDriver.setSelected(true);

            // prepare generator
            final Observables measures = new Observables(1);
            GroundOptimizationProblemBuilder OptimizationProblembuilder = new GroundOptimizationProblemBuilder(Collections.singletonList(lineSensor),
                                                                                                               measures, rugged);
            DSGenerator generator = OptimizationProblembuilder.getGenerator();

            double referenceLine = 0.87654 * dimension;
            GeodeticPoint[] gp = rugged.directLocation("line", referenceLine);

            Method inverseLoc = Rugged.class.getDeclaredMethod("inverseLocationDerivatives",
                                                               String.class, GeodeticPoint.class,
                                                               Integer.TYPE, Integer.TYPE,
                                                               DSGenerator.class);
            inverseLoc.setAccessible(true);
            int referencePixel = (3 * dimension) / 4;
            DerivativeStructure[] result = 
                            (DerivativeStructure[]) inverseLoc.invoke(rugged,
                                                                      "line", gp[referencePixel], 0, dimension,
                                                                      generator);
            Assert.assertEquals(referenceLine,  result[0].getValue(), lineTolerance);
            Assert.assertEquals(referencePixel, result[1].getValue(), pixelTolerance);
            Assert.assertEquals(2, result[0].getFreeParameters());
            Assert.assertEquals(1, result[0].getOrder());

            // check the partial derivatives
            DSFactory factory = new DSFactory(1, 1);
            double h = 1.0e-6;
            FiniteDifferencesDifferentiator differentiator = new FiniteDifferencesDifferentiator(8, h);

            UnivariateDifferentiableFunction lineVSroll =
                            differentiator.differentiate((double roll) -> {
                                try {
                                    rollDriver.setValue(roll);
                                    pitchDriver.setValue(0);
                                    return rugged.inverseLocation("line", gp[referencePixel], 0, dimension).getLineNumber();
                                } catch (OrekitException e) {
                                    throw new OrekitExceptionWrapper(e);
                                } catch (RuggedException e) {
                                    throw new RuggedExceptionWrapper(e);
                                }
                            });
            double dLdR = lineVSroll.value(factory.variable(0, 0.0)).getPartialDerivative(1);
            Assert.assertEquals(dLdR, result[0].getPartialDerivative(1, 0), dLdR * lineDerivativeRelativeTolerance);

            UnivariateDifferentiableFunction lineVSpitch =
                            differentiator.differentiate((double pitch) -> {
                                try {
                                    rollDriver.setValue(0);
                                    pitchDriver.setValue(pitch);
                                    return rugged.inverseLocation("line", gp[referencePixel], 0, dimension).getLineNumber();
                                } catch (OrekitException e) {
                                    throw new OrekitExceptionWrapper(e);
                                } catch (RuggedException e) {
                                    throw new RuggedExceptionWrapper(e);
                                }
                            });
            double dLdP = lineVSpitch.value(factory.variable(0, 0.0)).getPartialDerivative(1);
            Assert.assertEquals(dLdP, result[0].getPartialDerivative(0, 1), dLdP * lineDerivativeRelativeTolerance);

            UnivariateDifferentiableFunction pixelVSroll =
                            differentiator.differentiate((double roll) -> {
                                try {
                                    rollDriver.setValue(roll);
                                    pitchDriver.setValue(0);
                                    return rugged.inverseLocation("line", gp[referencePixel], 0, dimension).getPixelNumber();
                                } catch (OrekitException e) {
                                    throw new OrekitExceptionWrapper(e);
                                } catch (RuggedException e) {
                                    throw new RuggedExceptionWrapper(e);
                                }
                            });
            double dXdR = pixelVSroll.value(factory.variable(0, 0.0)).getPartialDerivative(1);
            Assert.assertEquals(dXdR, result[1].getPartialDerivative(1, 0), dXdR * pixelDerivativeRelativeTolerance);

            UnivariateDifferentiableFunction pixelVSpitch =
                            differentiator.differentiate((double pitch) -> {
                                try {
                                    rollDriver.setValue(0);
                                    pitchDriver.setValue(pitch);
                                    return rugged.inverseLocation("line", gp[referencePixel], 0, dimension).getPixelNumber();
                                } catch (OrekitException e) {
                                    throw new OrekitExceptionWrapper(e);
                                } catch (RuggedException e) {
                                    throw new RuggedExceptionWrapper(e);
                                }
                            });
            double dXdP = pixelVSpitch.value(factory.variable(0, 0.0)).getPartialDerivative(1);
            Assert.assertEquals(dXdP, result[1].getPartialDerivative(0, 1), dXdP * pixelDerivativeRelativeTolerance);

        } catch (InvocationTargetException | NoSuchMethodException |
                 SecurityException | IllegalAccessException |
                 IllegalArgumentException | URISyntaxException |
                 OrekitExceptionWrapper | RuggedExceptionWrapper e) {
            Assert.fail(e.getLocalizedMessage());
        }
    }



    // TODO add refining tests


    private void checkDateLocation(int dimension, boolean lightTimeCorrection, boolean aberrationOfLightCorrection,
                                       double maxDateError)
        throws RuggedException, OrekitException, URISyntaxException {

        String path = getClass().getClassLoader().getResource("orekit-data").toURI().getPath();
        DataProvidersManager.getInstance().addProvider(new DirectoryCrawler(new File(path)));
        final BodyShape  earth = TestUtils.createEarth();
        final Orbit      orbit = TestUtils.createOrbit(Constants.EIGEN5C_EARTH_MU);

        AbsoluteDate crossing = new AbsoluteDate("2012-01-01T12:30:00.000", TimeScalesFactory.getUTC());

        // one line sensor
        // position: 1.5m in front (+X) and 20 cm above (-Z) of the S/C center of mass
        // los: swath in the (YZ) plane, looking at 50° roll, 2.6" per pixel
        Vector3D position = new Vector3D(1.5, 0, -0.2);
        TimeDependentLOS los = TestUtils.createLOSPerfectLine(new Rotation(Vector3D.PLUS_I,
                                                                           FastMath.toRadians(50.0),
                                                                           RotationConvention.VECTOR_OPERATOR).applyTo(Vector3D.PLUS_K),
                                                              Vector3D.PLUS_I,
                                                              FastMath.toRadians(dimension * 2.6 / 3600.0), dimension).build();

        // linear datation model: at reference time we get line 100, and the rate is one line every 1.5ms
        LineDatation lineDatation = new LinearLineDatation(crossing, dimension / 2, 1.0 / 1.5e-3);
        int firstLine = 0;
        int lastLine  = dimension;
        LineSensor lineSensor = new LineSensor("line", lineDatation, position, los);
        AbsoluteDate minDate = lineSensor.getDate(firstLine).shiftedBy(-1.0);
        AbsoluteDate maxDate = lineSensor.getDate(lastLine).shiftedBy(+1.0);

        TileUpdater updater =
                new RandomLandscapeUpdater(0.0, 9000.0, 0.5, 0xf0a401650191f9f6l,
                                           FastMath.toRadians(1.0), 257);

        Rugged rugged = new RuggedBuilder().
                setDigitalElevationModel(updater, 8).
                setAlgorithm(AlgorithmId.DUVENHAGE).
                setEllipsoid(EllipsoidId.WGS84, BodyRotatingFrameId.ITRF).
                setTimeSpan(minDate, maxDate, 0.001, 5.0).
                setTrajectory(InertialFrameId.EME2000,
                              TestUtils.orbitToPV(orbit, earth, minDate.shiftedBy(-1.0), maxDate.shiftedBy(+1.0), 0.25),
                              8, CartesianDerivativesFilter.USE_PV,
                              TestUtils.orbitToQ(orbit, earth, minDate.shiftedBy(-1.0), maxDate.shiftedBy(+1.0), 0.25),
                              2, AngularDerivativesFilter.USE_R).
                setLightTimeCorrection(lightTimeCorrection).
                setAberrationOfLightCorrection(aberrationOfLightCorrection).
                addLineSensor(lineSensor).
                build();

        double referenceLine = 0.87654 * dimension;
        GeodeticPoint[] gp = rugged.directLocation("line", referenceLine);

        for (double p = 0; p < gp.length - 1; p += 1.0) {
            int    i = (int) FastMath.floor(p);
            double d = p - i;
            AbsoluteDate date = rugged.dateLocation("line",
                                                        (1 - d) * gp[i].getLatitude()  + d * gp[i + 1].getLatitude(),
                                                        (1 - d) * gp[i].getLongitude() + d * gp[i + 1].getLongitude(),
                                                        0, dimension);
            Assert.assertEquals(0.0, date.durationFrom(lineSensor.getDate(referenceLine)),  maxDateError);
        }

        // point out of line (20 lines before first line)
        GeodeticPoint[] gp0 = rugged.directLocation("line", 0);
        GeodeticPoint[] gp1 = rugged.directLocation("line", 1);
        Assert.assertNull(rugged.dateLocation("line",
                                                    21 * gp0[dimension / 2].getLatitude()  - 20 * gp1[dimension / 2].getLatitude(),
                                                    21 * gp0[dimension / 2].getLongitude() - 20 * gp1[dimension / 2].getLongitude(),
                                                    0, dimension));

        // point out of line (20 lines after last line)
        GeodeticPoint[] gp2 = rugged.directLocation("line", dimension - 2);
        GeodeticPoint[] gp3 = rugged.directLocation("line", dimension - 1);
        Assert.assertNull(rugged.dateLocation("line",
                                                    -20 * gp2[dimension / 2].getLatitude()  + 21 * gp3[dimension / 2].getLatitude(),
                                                    -20 * gp2[dimension / 2].getLongitude() + 21 * gp3[dimension / 2].getLongitude(),
                                                    0, dimension));
        
    }

    private void checkLineDatation(int dimension, double maxLineError)
    				throws RuggedException, OrekitException, URISyntaxException {

    	String path = getClass().getClassLoader().getResource("orekit-data").toURI().getPath();
    	DataProvidersManager.getInstance().addProvider(new DirectoryCrawler(new File(path)));

    	AbsoluteDate crossing = new AbsoluteDate("2012-01-01T12:30:00.000", TimeScalesFactory.getUTC());

    	// one line sensor
        // position: 1.5m in front (+X) and 20 cm above (-Z) of the S/C center of mass
        // los: swath in the (YZ) plane, looking at 50° roll, 2.6" per pixel
        Vector3D position = new Vector3D(1.5, 0, -0.2);
        TimeDependentLOS los = TestUtils.createLOSPerfectLine(new Rotation(Vector3D.PLUS_I,
                FastMath.toRadians(50.0),
                RotationConvention.VECTOR_OPERATOR).applyTo(Vector3D.PLUS_K),
        		Vector3D.PLUS_I,
        		FastMath.toRadians(dimension * 2.6 / 3600.0), dimension).build();

        // linear datation model: at reference time we get the middle line, and the rate is one line every 1.5ms
        LineDatation lineDatation = new LinearLineDatation(crossing, dimension / 2, 1.0 / 1.5e-3);
        int firstLine = 0;
        int lastLine  = dimension;
        LineSensor lineSensor = new LineSensor("line", lineDatation, position, los);
        AbsoluteDate minDate = lineSensor.getDate(firstLine).shiftedBy(-1.0);
        AbsoluteDate maxDate = lineSensor.getDate(lastLine).shiftedBy(+1.0);

        // Recompute the lines from the date with the appropriate shift of date
        double recomputedFirstLine = lineSensor.getLine(minDate.shiftedBy(+1.0));
        double recomputedLastLine = lineSensor.getLine(maxDate.shiftedBy(-1.0));

        Assert.assertEquals(firstLine, recomputedFirstLine, maxLineError);
        Assert.assertEquals(lastLine, recomputedLastLine, maxLineError);
    }
}

