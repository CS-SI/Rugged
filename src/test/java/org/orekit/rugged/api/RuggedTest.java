/* Copyright 2002-2014 CS Systèmes d'Information
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
import java.net.URISyntaxException;
import java.nio.MappedByteBuffer;
import java.nio.channels.FileChannel;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.ode.nonstiff.DormandPrince853Integrator;
import org.apache.commons.math3.stat.descriptive.SummaryStatistics;
import org.apache.commons.math3.util.FastMath;
import org.apache.commons.math3.util.Pair;
import org.junit.Assert;
import org.junit.Ignore;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;
import org.orekit.attitudes.AttitudeProvider;
import org.orekit.attitudes.NadirPointing;
import org.orekit.attitudes.YawCompensation;
import org.orekit.bodies.BodyShape;
import org.orekit.bodies.CelestialBodyFactory;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.bodies.OneAxisEllipsoid;
import org.orekit.data.DataProvidersManager;
import org.orekit.data.DirectoryCrawler;
import org.orekit.errors.OrekitException;
import org.orekit.errors.PropagationException;
import org.orekit.forces.gravity.HolmesFeatherstoneAttractionModel;
import org.orekit.forces.gravity.ThirdBodyAttraction;
import org.orekit.forces.gravity.potential.GravityFieldFactory;
import org.orekit.forces.gravity.potential.NormalizedSphericalHarmonicsProvider;
import org.orekit.frames.Frame;
import org.orekit.frames.FramesFactory;
import org.orekit.orbits.CircularOrbit;
import org.orekit.orbits.Orbit;
import org.orekit.orbits.OrbitType;
import org.orekit.orbits.PositionAngle;
import org.orekit.propagation.Propagator;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.analytical.KeplerianPropagator;
import org.orekit.propagation.numerical.NumericalPropagator;
import org.orekit.propagation.sampling.OrekitFixedStepHandler;
import org.orekit.rugged.core.raster.RandomLandscapeUpdater;
import org.orekit.rugged.core.raster.VolcanicConeElevationUpdater;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeScalesFactory;
import org.orekit.utils.Constants;
import org.orekit.utils.IERSConventions;
import org.orekit.utils.PVCoordinates;

public class RuggedTest {

    @Rule
    public TemporaryFolder tempFolder = new TemporaryFolder();

    @Test
    public void testSetContextWithEphemerides()
        throws RuggedException, OrekitException, URISyntaxException {

        String path = getClass().getClassLoader().getResource("orekit-data").toURI().getPath();
        DataProvidersManager.getInstance().addProvider(new DirectoryCrawler(new File(path)));
        AbsoluteDate t0 = new AbsoluteDate("2012-01-01T00:00:00", TimeScalesFactory.getUTC());

        @SuppressWarnings("unchecked")
        List<Pair<AbsoluteDate, PVCoordinates>> pv = Arrays.asList(
            createPV(t0, 0.000, -1545168.478, -7001985.361,       0.000, -1095.152224, 231.344922, -7372.851944),
            createPV(t0, 1.000, -1546262.794, -7001750.226,   -7372.851, -1093.478904, 238.925123, -7372.847995),
            createPV(t0, 2.000, -1547355.435, -7001507.511,  -14745.693, -1091.804408, 246.505033, -7372.836044),
            createPV(t0, 3.000, -1548446.402, -7001257.216,  -22118.520, -1090.128736, 254.084644, -7372.816090),
            createPV(t0, 4.000, -1549535.693, -7000999.342,  -29491.323, -1088.451892, 261.663949, -7372.788133),
            createPV(t0, 5.000, -1550623.306, -7000733.888,  -36864.094, -1086.773876, 269.242938, -7372.752175),
            createPV(t0, 6.000, -1551709.240, -7000460.856,  -44236.825, -1085.094690, 276.821604, -7372.708214),
            createPV(t0, 7.000, -1552793.495, -7000180.245,  -51609.507, -1083.414336, 284.399938, -7372.656251),
            createPV(t0, 8.000, -1553876.068, -6999892.056,  -58982.134, -1081.732817, 291.977932, -7372.596287),
            createPV(t0, 9.000, -1554956.960, -6999596.289,  -66354.697, -1080.050134, 299.555578, -7372.528320),
            createPV(t0,10.000, -1556036.168, -6999292.945,  -73727.188, -1078.366288, 307.132868, -7372.452352),
            createPV(t0,11.000, -1557113.692, -6998982.024,  -81099.599, -1076.681282, 314.709792, -7372.368382),
            createPV(t0,12.000, -1558189.530, -6998663.526,  -88471.922, -1074.995118, 322.286344, -7372.276411),
            createPV(t0,13.000, -1559263.682, -6998337.451,  -95844.150, -1073.307797, 329.862513, -7372.176439),
            createPV(t0,14.000, -1560336.145, -6998003.801, -103216.273, -1071.619321, 337.438294, -7372.068466),
            createPV(t0,15.000, -1561406.920, -6997662.575, -110588.284, -1069.929692, 345.013676, -7371.952492),
            createPV(t0,16.000, -1562476.004, -6997313.774, -117960.175, -1068.238912, 352.588652, -7371.828517),
            createPV(t0,17.000, -1563543.398, -6996957.398, -125331.938, -1066.546983, 360.163213, -7371.696542),
            createPV(t0,18.000, -1564609.098, -6996593.447, -132703.565, -1064.853906, 367.737352, -7371.556566),
            createPV(t0,19.000, -1565673.105, -6996221.923, -140075.049, -1063.159684, 375.311060, -7371.408591),
            createPV(t0,20.000, -1566735.417, -6995842.825, -147446.380, -1061.464319, 382.884328, -7371.252616));

        @SuppressWarnings("unchecked")
        List<Pair<AbsoluteDate, Rotation>> q = Arrays.asList(
            createQ(t0, 0.000, 0.516354347549, -0.400120145429,  0.583012133139,  0.483093065155),
            createQ(t0, 1.000, 0.516659035405, -0.399867643627,  0.582741754688,  0.483302551263),
            createQ(t0, 2.000, 0.516963581177, -0.399615033309,  0.582471217473,  0.483511904409),
            createQ(t0, 3.000, 0.517267984776, -0.399362314553,  0.582200521577,  0.483721124530),
            createQ(t0, 4.000, 0.517572246112, -0.399109487434,  0.581929667081,  0.483930211565),
            createQ(t0, 5.000, 0.517876365096, -0.398856552030,  0.581658654071,  0.484139165451),
            createQ(t0, 6.000, 0.518180341637, -0.398603508416,  0.581387482627,  0.484347986126),
            createQ(t0, 7.000, 0.518484175647, -0.398350356669,  0.581116152834,  0.484556673529),
            createQ(t0, 8.000, 0.518787867035, -0.398097096866,  0.580844664773,  0.484765227599),
            createQ(t0, 9.000, 0.519091415713, -0.397843729083,  0.580573018530,  0.484973648272),
            createQ(t0,10.000, 0.519394821590, -0.397590253397,  0.580301214186,  0.485181935488),
            createQ(t0,11.000, 0.519698084578, -0.397336669885,  0.580029251825,  0.485390089185),
            createQ(t0,12.000, 0.520001204587, -0.397082978623,  0.579757131530,  0.485598109301),
            createQ(t0,13.000, 0.520304181527, -0.396829179688,  0.579484853385,  0.485805995775),
            createQ(t0,14.000, 0.520607015311, -0.396575273158,  0.579212417473,  0.486013748545),
            createQ(t0,15.000, 0.520909705847, -0.396321259108,  0.578939823877,  0.486221367550),
            createQ(t0,16.000, 0.521212253049, -0.396067137616,  0.578667072681,  0.486428852729),
            createQ(t0,17.000, 0.521514656825, -0.395812908759,  0.578394163969,  0.486636204020),
            createQ(t0,18.000, 0.521816917089, -0.395558572613,  0.578121097824,  0.486843421362),
            createQ(t0,19.000, 0.522119033749, -0.395304129256,  0.577847874330,  0.487050504694),
            createQ(t0,20.000, 0.522421006719, -0.395049578765,  0.577574493570,  0.487257453954));

        TileUpdater updater =
                new VolcanicConeElevationUpdater(new GeodeticPoint(FastMath.toRadians(13.25667), FastMath.toRadians(123.685), 2463.0),
                                                 FastMath.toRadians(30.0), 16.0,
                                                 FastMath.toRadians(1.0), 1201);

        Rugged rugged = new Rugged(updater, 8, AlgorithmId.DUVENHAGE,
                                   EllipsoidId.WGS84, InertialFrameId.EME2000, BodyRotatingFrameId.ITRF,
                                   pv, 8, q, 8);

        Assert.assertTrue(rugged.isLightTimeCorrected());
        rugged.setLightTimeCorrection(false);
        Assert.assertFalse(rugged.isLightTimeCorrected());
        Assert.assertTrue(rugged.isAberrationOfLightCorrected());
        rugged.setAberrationOfLightCorrection(false);
        Assert.assertFalse(rugged.isAberrationOfLightCorrected());

    }

    @Test
    public void testSetContextWithPropagator()
        throws RuggedException, OrekitException, URISyntaxException {

        String path = getClass().getClassLoader().getResource("orekit-data").toURI().getPath();
        DataProvidersManager.getInstance().addProvider(new DirectoryCrawler(new File(path)));
        BodyShape  earth                                  = createEarth();
        NormalizedSphericalHarmonicsProvider gravityField = createGravityField();
        Orbit      orbit                                  = createOrbit(gravityField.getMu());
        Propagator propagator                             = createPropagator(earth, gravityField, orbit);

        TileUpdater updater =
                new VolcanicConeElevationUpdater(new GeodeticPoint(FastMath.toRadians(13.25667), FastMath.toRadians(123.685), 2463.0),
                                                 FastMath.toRadians(30.0), 16.0,
                                                 FastMath.toRadians(1.0), 1201);

        Rugged rugged = new Rugged(updater, 8, AlgorithmId.DUVENHAGE,
                                   EllipsoidId.WGS84, InertialFrameId.EME2000, BodyRotatingFrameId.ITRF,
                                   propagator);

        Assert.assertTrue(rugged.isLightTimeCorrected());
        rugged.setLightTimeCorrection(false);
        Assert.assertFalse(rugged.isLightTimeCorrected());
        Assert.assertTrue(rugged.isAberrationOfLightCorrected());
        rugged.setAberrationOfLightCorrection(false);
        Assert.assertFalse(rugged.isAberrationOfLightCorrected());

    }

    // the following test is disabled by default
    // it is only used to check timings, and also create a large (66M) temporary file
    @Test
    @Ignore
    public void testMayonVolcanoTiming()
        throws RuggedException, OrekitException, URISyntaxException {

        long t0 = System.currentTimeMillis();
        int dimension = 2000;

        String path = getClass().getClassLoader().getResource("orekit-data").toURI().getPath();
        DataProvidersManager.getInstance().addProvider(new DirectoryCrawler(new File(path)));
        BodyShape  earth                                  = createEarth();
        NormalizedSphericalHarmonicsProvider gravityField = createGravityField();
        Orbit      orbit                                  = createOrbit(gravityField.getMu());

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
        List<Vector3D> los = createLOS(Vector3D.PLUS_K, Vector3D.PLUS_I,
                                       FastMath.toRadians(10.0), dimension);

        // linear datation model: at reference time we get line 1000, and the rate is one line every 1.5ms
        LineDatation lineDatation = new LinearLineDatation(crossing, dimension / 2, 1.0 / 1.5e-3);
        int firstLine = 0;
        int lastLine  = dimension;
        LineSensor lineSensor = new LineSensor("line", lineDatation, position, los);

        Propagator propagator = createPropagator(earth, gravityField, orbit);
        propagator.propagate(lineDatation.getDate(firstLine).shiftedBy(-1.0));
        propagator.setEphemerisMode();
        propagator.propagate(lineDatation.getDate(lastLine).shiftedBy(+1.0));
        Propagator ephemeris = propagator.getGeneratedEphemeris();

        Rugged rugged = new Rugged(updater, 8, AlgorithmId.DUVENHAGE,
                                   EllipsoidId.WGS84, InertialFrameId.EME2000, BodyRotatingFrameId.ITRF,
                                   ephemeris);
        rugged.setLightTimeCorrection(true);
        rugged.setAberrationOfLightCorrection(true);

        rugged.addLineSensor(lineSensor);

        try {

            int              size   = (lastLine - firstLine) * los.size() * 3 * Integer.SIZE / 8;
            RandomAccessFile out    = new RandomAccessFile(tempFolder.newFile(), "rw");
            MappedByteBuffer buffer = out.getChannel().map(FileChannel.MapMode.READ_WRITE, 0, size);

            long t1 = System.currentTimeMillis();
            int pixels = 0;
            for (double line = firstLine; line < lastLine; line++) {
                GeodeticPoint[] gp = rugged.directLocalization("line", line);
                for (int i = 0; i < gp.length; ++i) {
                    final int latCode = (int) FastMath.rint(FastMath.scalb(gp[i].getLatitude(),  29));
                    final int lonCode = (int) FastMath.rint(FastMath.scalb(gp[i].getLongitude(), 29));
                    final int altCode = (int) FastMath.rint(FastMath.scalb(gp[i].getAltitude(),  17));
                    buffer.putInt(latCode);
                    buffer.putInt(lonCode);
                    buffer.putInt(altCode);
                }
                pixels += los.size();
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
                              "  direct localization and %3dM grid writing: %5.1fs (%.1f px/s)%n",
                              lastLine - firstLine, los.size(),
                              1.0e-3 *(t1 - t0), sizeM, 1.0e-3 *(t2 - t1), pixels / (1.0e-3 * (t2 - t1)));
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
        final BodyShape  earth = createEarth();
        final Orbit      orbit = createOrbit(Constants.EIGEN5C_EARTH_MU);

        AbsoluteDate crossing = new AbsoluteDate("2012-01-07T11:21:15.000", TimeScalesFactory.getUTC());

        // one line sensor
        // position: 1.5m in front (+X) and 20 cm above (-Z) of the S/C center of mass
        // los: swath in the (YZ) plane, centered at +Z, ±10° aperture, 960 pixels
        Vector3D position = new Vector3D(1.5, 0, -0.2);
        List<Vector3D> los = createLOS(Vector3D.PLUS_K, Vector3D.PLUS_I,
                                       FastMath.toRadians(10.0), dimension);

        // linear datation model: at reference time we get line 200, and the rate is one line every 1.5ms
        LineDatation lineDatation = new LinearLineDatation(crossing, dimension / 2, 1.0 / 1.5e-3);
        int firstLine = 0;
        int lastLine  = dimension;
        LineSensor lineSensor = new LineSensor("line", lineDatation, position, los);

        Rugged rugged = new Rugged(null, -1, AlgorithmId.IGNORE_DEM_USE_ELLIPSOID,
                                   EllipsoidId.WGS84, InertialFrameId.EME2000, BodyRotatingFrameId.ITRF,
                                   orbitToPV(orbit, earth, lineDatation, firstLine, lastLine, 0.25), 8,
                                   orbitToQ(orbit, earth, lineDatation, firstLine, lastLine, 0.25), 2);

        rugged.addLineSensor(lineSensor);

        rugged.setLightTimeCorrection(true);
        rugged.setAberrationOfLightCorrection(false);
        GeodeticPoint[] gpWithLightTimeCorrection = rugged.directLocalization("line", 200);

        rugged.setLightTimeCorrection(false);
        rugged.setAberrationOfLightCorrection(false);
        GeodeticPoint[] gpWithoutLightTimeCorrection = rugged.directLocalization("line", 200);

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
        final BodyShape  earth = createEarth();
        final Orbit      orbit = createOrbit(Constants.EIGEN5C_EARTH_MU);

        AbsoluteDate crossing = new AbsoluteDate("2012-01-07T11:46:35.000", TimeScalesFactory.getUTC());

        // one line sensor
        // position: 1.5m in front (+X) and 20 cm above (-Z) of the S/C center of mass
        // los: swath in the (YZ) plane, centered at +Z, ±10° aperture, 960 pixels
        Vector3D position = new Vector3D(1.5, 0, -0.2);
        List<Vector3D> los = createLOS(Vector3D.PLUS_K, Vector3D.PLUS_I,
                                       FastMath.toRadians(10.0), dimension);

        // linear datation model: at reference time we get line 200, and the rate is one line every 1.5ms
        LineDatation lineDatation = new LinearLineDatation(crossing, dimension / 2, 1.0 / 1.5e-3);
        int firstLine = 0;
        int lastLine  = dimension;
        LineSensor lineSensor = new LineSensor("line", lineDatation, position, los);

        Rugged rugged = new Rugged(null, -1, AlgorithmId.IGNORE_DEM_USE_ELLIPSOID,
                                   EllipsoidId.WGS84, InertialFrameId.EME2000, BodyRotatingFrameId.ITRF,
                                   orbitToPV(orbit, earth, lineDatation, firstLine, lastLine, 0.25), 8,
                                   orbitToQ(orbit, earth, lineDatation, firstLine, lastLine, 0.25), 2);

        rugged.addLineSensor(lineSensor);

        rugged.setLightTimeCorrection(false);
        rugged.setAberrationOfLightCorrection(true);
        GeodeticPoint[] gpWithAberrationOfLightCorrection = rugged.directLocalization("line", 200);

        rugged.setLightTimeCorrection(false);
        rugged.setAberrationOfLightCorrection(false);
        GeodeticPoint[] gpWithoutAberrationOfLightCorrection = rugged.directLocalization("line", 200);

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
        final BodyShape  earth = createEarth();
        final Orbit      orbit = createOrbit(Constants.EIGEN5C_EARTH_MU);

        AbsoluteDate crossing = new AbsoluteDate("2012-01-01T12:30:00.000", TimeScalesFactory.getUTC());

        // one line sensor
        // position: 1.5m in front (+X) and 20 cm above (-Z) of the S/C center of mass
        // los: swath in the (YZ) plane, looking at 50° roll, ±1° aperture
        Vector3D position = new Vector3D(1.5, 0, -0.2);
        List<Vector3D> los = createLOS(new Rotation(Vector3D.PLUS_I, FastMath.toRadians(50.0)).applyTo(Vector3D.PLUS_K),
                                       Vector3D.PLUS_I,
                                       FastMath.toRadians(1.0), dimension);

        // linear datation model: at reference time we get line 100, and the rate is one line every 1.5ms
        LineDatation lineDatation = new LinearLineDatation(crossing, dimension / 2, 1.0 / 1.5e-3);
        int firstLine = 0;
        int lastLine  = dimension;
        LineSensor lineSensor = new LineSensor("line", lineDatation, position, los);

        TileUpdater updater =
                new RandomLandscapeUpdater(0.0, 9000.0, 0.5, 0xf0a401650191f9f6l,
                                           FastMath.toRadians(1.0), 257);

        Rugged ruggedFull = new Rugged(updater, 8, AlgorithmId.DUVENHAGE,
                                       EllipsoidId.WGS84, InertialFrameId.EME2000, BodyRotatingFrameId.ITRF,
                                       orbitToPV(orbit, earth, lineDatation, firstLine, lastLine, 0.25), 8,
                                       orbitToQ(orbit, earth, lineDatation, firstLine, lastLine, 0.25), 2);
        ruggedFull.addLineSensor(lineSensor);
        GeodeticPoint[] gpWithFlatBodyCorrection = ruggedFull.directLocalization("line", 100);

        Rugged ruggedFlat = new Rugged(updater, 8, AlgorithmId.DUVENHAGE_FLAT_BODY,
                                       EllipsoidId.WGS84, InertialFrameId.EME2000, BodyRotatingFrameId.ITRF,
                                       orbitToPV(orbit, earth, lineDatation, firstLine, lastLine, 0.25), 8,
                                       orbitToQ(orbit, earth, lineDatation, firstLine, lastLine, 0.25), 2);
        ruggedFlat.addLineSensor(lineSensor);
        GeodeticPoint[] gpWithoutFlatBodyCorrection = ruggedFlat.directLocalization("line", 100);

        SummaryStatistics stats = new SummaryStatistics();
        for (int i = 0; i < gpWithFlatBodyCorrection.length; ++i) {
            Vector3D pWith    = earth.transform(gpWithFlatBodyCorrection[i]);
            Vector3D pWithout = earth.transform(gpWithoutFlatBodyCorrection[i]);
            stats.addValue(Vector3D.distance(pWith, pWithout));
        }
        Assert.assertEquals( 0.005, stats.getMin(),  1.0e-3);
        Assert.assertEquals(39.924, stats.getMax(),  1.0e-3);
        Assert.assertEquals( 4.823, stats.getMean(), 1.0e-3);

    }

    // the following test is disabled by default
    // it is only used to check timings, and also create a small (2M) temporary file
    @Test
    public void testInverseLocalizationTiming()
        throws RuggedException, OrekitException, URISyntaxException {

        long t0 = System.currentTimeMillis();
        int dimension = 2200;

        String path = getClass().getClassLoader().getResource("orekit-data").toURI().getPath();
        DataProvidersManager.getInstance().addProvider(new DirectoryCrawler(new File(path)));
        final BodyShape  earth = createEarth();
        final Orbit      orbit = createOrbit(Constants.EIGEN5C_EARTH_MU);

        AbsoluteDate crossing = new AbsoluteDate("2012-01-01T12:30:00.000", TimeScalesFactory.getUTC());

        // one line sensor
        // position: 1.5m in front (+X) and 20 cm above (-Z) of the S/C center of mass
        // los: swath in the (YZ) plane, looking at 50° roll, 2.6" per pixel
        Vector3D position = new Vector3D(1.5, 0, -0.2);
        List<Vector3D> los = createLOS(new Rotation(Vector3D.PLUS_I, FastMath.toRadians(50.0)).applyTo(Vector3D.PLUS_K),
                                       Vector3D.PLUS_I,
                                       FastMath.toRadians(dimension * 2.6 / 3600.0), dimension);

        // linear datation model: at reference time we get line 100, and the rate is one line every 1.5ms
        LineDatation lineDatation = new LinearLineDatation(crossing, dimension / 2, 1.0 / 1.5e-3);
        int firstLine = 0;
        int lastLine  = dimension;
        LineSensor lineSensor = new LineSensor("line", lineDatation, position, los);

        TileUpdater updater =
                new RandomLandscapeUpdater(0.0, 9000.0, 0.5, 0xf0a401650191f9f6l,
                                           FastMath.toRadians(1.0), 257);

        Rugged rugged = new Rugged(updater, 8, AlgorithmId.DUVENHAGE,
                                   EllipsoidId.WGS84, InertialFrameId.EME2000, BodyRotatingFrameId.ITRF,
                                   orbitToPV(orbit, earth, lineDatation, firstLine, lastLine, 0.25), 8,
                                   orbitToQ(orbit, earth, lineDatation, firstLine, lastLine, 0.25), 2);
        rugged.setLightTimeCorrection(true);
        rugged.setAberrationOfLightCorrection(true);
        rugged.addLineSensor(lineSensor);

        double lat0  = FastMath.toRadians(-22.9);
        double lon0  = FastMath.toRadians(142.4);
        double delta = FastMath.toRadians(0.5);

        try {
            int              size   = dimension * dimension * 2 * Integer.SIZE / 8;
            RandomAccessFile out    = new RandomAccessFile(tempFolder.newFile(), "rw");
            MappedByteBuffer buffer = out.getChannel().map(FileChannel.MapMode.READ_WRITE, 0, size);

            long t1 = System.currentTimeMillis();
            int goodPixels = 0;
            int badPixels  = 0;
            for (int i = 0; i < dimension; ++i) {
                double latitude  = lat0 + (i * delta) / dimension;
                for (int j = 0; j < dimension; ++j) {
                    double longitude = lon0 + (j * delta) / dimension;
                    SensorPixel sp = rugged.inverseLocalization("line", latitude, longitude, 0, dimension);
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

            long t2 = System.currentTimeMillis();
            out.close();
            int sizeM = size / (1024 * 1024);
            System.out.format(Locale.US,
                              "%n%n%5dx%5d:%n" +
                              "  Orekit initialization and DEM creation   : %5.1fs%n" +
                              "  inverse localization and %3dM grid writing: %5.1fs (%.1f px/s, %.1f%% covered)%n",
                              dimension, dimension,
                              1.0e-3 *(t1 - t0), sizeM, 1.0e-3 *(t2 - t1),
                              (badPixels + goodPixels) / (1.0e-3 * (t2 - t1)),
                              (100.0 * goodPixels) / (goodPixels + badPixels));
        } catch (IOException ioe) {
            Assert.fail(ioe.getLocalizedMessage());
        }
    }

    @Test
    public void testInverseLocalization()
        throws RuggedException, OrekitException, URISyntaxException {
        checkInverseLocalization(2000, false, false, 5.0e-5, 8.0e-5);
        checkInverseLocalization(2000, false, true,  5.0e-5, 8.0e-5);
        checkInverseLocalization(2000, true,  false, 5.0e-5, 8.0e-5);
        checkInverseLocalization(2000, true,  true,  5.0e-5, 8.0e-5);
    }

    private void checkInverseLocalization(int dimension, boolean lightTimeCorrection, boolean aberrationOfLightCorrection,
                                          double maxLineError, double maxPixelError)
        throws RuggedException, OrekitException, URISyntaxException {

        String path = getClass().getClassLoader().getResource("orekit-data").toURI().getPath();
        DataProvidersManager.getInstance().addProvider(new DirectoryCrawler(new File(path)));
        final BodyShape  earth = createEarth();
        final Orbit      orbit = createOrbit(Constants.EIGEN5C_EARTH_MU);

        AbsoluteDate crossing = new AbsoluteDate("2012-01-01T12:30:00.000", TimeScalesFactory.getUTC());

        // one line sensor
        // position: 1.5m in front (+X) and 20 cm above (-Z) of the S/C center of mass
        // los: swath in the (YZ) plane, looking at 50° roll, 2.6" per pixel
        Vector3D position = new Vector3D(1.5, 0, -0.2);
        List<Vector3D> los = createLOS(new Rotation(Vector3D.PLUS_I, FastMath.toRadians(50.0)).applyTo(Vector3D.PLUS_K),
                                       Vector3D.PLUS_I,
                                       FastMath.toRadians(dimension * 2.6 / 3600.0), dimension);

        // linear datation model: at reference time we get line 100, and the rate is one line every 1.5ms
        LineDatation lineDatation = new LinearLineDatation(crossing, dimension / 2, 1.0 / 1.5e-3);
        int firstLine = 0;
        int lastLine  = dimension;
        LineSensor lineSensor = new LineSensor("line", lineDatation, position, los);

        TileUpdater updater =
                new RandomLandscapeUpdater(0.0, 9000.0, 0.5, 0xf0a401650191f9f6l,
                                           FastMath.toRadians(1.0), 257);

        Rugged rugged = new Rugged(updater, 8, AlgorithmId.DUVENHAGE,
                                   EllipsoidId.WGS84, InertialFrameId.EME2000, BodyRotatingFrameId.ITRF,
                                   orbitToPV(orbit, earth, lineDatation, firstLine, lastLine, 0.25), 8,
                                   orbitToQ(orbit, earth, lineDatation, firstLine, lastLine, 0.25), 2);
        rugged.setLightTimeCorrection(lightTimeCorrection);
        rugged.setAberrationOfLightCorrection(aberrationOfLightCorrection);
        rugged.addLineSensor(lineSensor);

        double referenceLine = 0.87654 * dimension;
        GeodeticPoint[] gp = rugged.directLocalization("line", referenceLine);

        for (double p = 0; p < gp.length - 1; p += 1.0) {
            int    i = (int) FastMath.floor(p);
            double d = p - i;
            GeodeticPoint g = new GeodeticPoint((1 - d) * gp[i].getLatitude()  + d * gp[i + 1].getLatitude(),
                                                (1 - d) * gp[i].getLongitude() + d * gp[i + 1].getLongitude(),
                                                (1 - d) * gp[i].getAltitude()  + d * gp[i + 1].getAltitude());
            SensorPixel sp = rugged.inverseLocalization("line", g, 0, dimension);
            Assert.assertEquals(referenceLine, sp.getLineNumber(),  maxLineError);
            Assert.assertEquals(p,             sp.getPixelNumber(), maxPixelError);
        }
     }

    private BodyShape createEarth()
       throws OrekitException {
        return new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                    Constants.WGS84_EARTH_FLATTENING,
                                    FramesFactory.getITRF(IERSConventions.IERS_2010, true));
    }

    private NormalizedSphericalHarmonicsProvider createGravityField()
        throws OrekitException {
        return GravityFieldFactory.getNormalizedProvider(12, 12);
    }

    private Orbit createOrbit(double mu)
        throws OrekitException {
        // the following orbital parameters have been computed using
        // Orekit tutorial about phasing, using the following configuration:
        //
        //  orbit.date                          = 2012-01-01T00:00:00.000
        //  phasing.orbits.number               = 143
        //  phasing.days.number                 =  10
        //  sun.synchronous.reference.latitude  = 0
        //  sun.synchronous.reference.ascending = false
        //  sun.synchronous.mean.solar.time     = 10:30:00
        //  gravity.field.degree                = 12
        //  gravity.field.order                 = 12
        AbsoluteDate date = new AbsoluteDate("2012-01-01T00:00:00.000", TimeScalesFactory.getUTC());
        Frame eme2000 = FramesFactory.getEME2000();
        return new CircularOrbit(7173352.811913891,
                                 -4.029194321683225E-4, 0.0013530362644647786,
                                 FastMath.toRadians(98.63218182243709),
                                 FastMath.toRadians(77.55565567747836),
                                 FastMath.PI, PositionAngle.TRUE,
                                 eme2000, date, mu);
    }

    private Propagator createPropagator(BodyShape earth,
                                        NormalizedSphericalHarmonicsProvider gravityField,
                                        Orbit orbit)
        throws OrekitException {

        AttitudeProvider yawCompensation = new YawCompensation(new NadirPointing(earth));
        SpacecraftState state = new SpacecraftState(orbit,
                                                    yawCompensation.getAttitude(orbit,
                                                                                orbit.getDate(),
                                                                                orbit.getFrame()),
                                                    1180.0);

        // numerical model for improving orbit
        OrbitType type = OrbitType.CIRCULAR;
        double[][] tolerances = NumericalPropagator.tolerances(0.1, orbit, type);
        DormandPrince853Integrator integrator =
                new DormandPrince853Integrator(1.0e-4 * orbit.getKeplerianPeriod(),
                                               1.0e-1 * orbit.getKeplerianPeriod(),
                                               tolerances[0], tolerances[1]);
        integrator.setInitialStepSize(1.0e-2 * orbit.getKeplerianPeriod());
        NumericalPropagator numericalPropagator = new NumericalPropagator(integrator);
        numericalPropagator.addForceModel(new HolmesFeatherstoneAttractionModel(earth.getBodyFrame(), gravityField));
        numericalPropagator.addForceModel(new ThirdBodyAttraction(CelestialBodyFactory.getSun()));
        numericalPropagator.addForceModel(new ThirdBodyAttraction(CelestialBodyFactory.getMoon()));
        numericalPropagator.setOrbitType(type);
        numericalPropagator.setInitialState(state);
        numericalPropagator.setAttitudeProvider(yawCompensation);
        return numericalPropagator;

    }

    private List<Vector3D> createLOS(Vector3D center, Vector3D normal, double halfAperture, int n) {
        List<Vector3D> list = new ArrayList<Vector3D>(n);
        for (int i = 0; i < n; ++i) {
            double alpha = (halfAperture * (2 * i + 1 - n)) / (n - 1);
            list.add(new Rotation(normal, alpha).applyTo(center));
        }
        return list;
    }

    private Pair<AbsoluteDate, PVCoordinates> createPV(AbsoluteDate t0, double dt,
                                                       double px, double py, double pz,
                                                       double vx, double vy, double vz) {
        return new Pair<AbsoluteDate, PVCoordinates>(t0.shiftedBy(dt),
                                                     new PVCoordinates(new Vector3D(px, py, pz),
                                                                       new Vector3D(vx, vy, vz)));
    }

    private Pair<AbsoluteDate, Rotation> createQ(AbsoluteDate t0, double dt,
                                                       double q0, double q1, double q2, double q3) {
        return new Pair<AbsoluteDate, Rotation>(t0.shiftedBy(dt), new Rotation(q0, q1, q2, q3, true));
    }

    private List<Pair<AbsoluteDate, PVCoordinates>> orbitToPV(Orbit orbit, BodyShape earth,
                                                              LineDatation lineDatation, int firstLine, int lastLine,
                                                              double step)
        throws PropagationException {
        Propagator propagator = new KeplerianPropagator(orbit);
        propagator.setAttitudeProvider(new YawCompensation(new NadirPointing(earth)));
        propagator.propagate(lineDatation.getDate(firstLine).shiftedBy(-1.0));
        final List<Pair<AbsoluteDate, PVCoordinates>> list = new ArrayList<Pair<AbsoluteDate, PVCoordinates>>();
        propagator.setMasterMode(step, new OrekitFixedStepHandler() {
            public void init(SpacecraftState s0, AbsoluteDate t) {
            }   
            public void handleStep(SpacecraftState currentState, boolean isLast) {
                list.add(new Pair<AbsoluteDate, PVCoordinates>(currentState.getDate(),
                                                               currentState.getPVCoordinates()));
            }
        });
        propagator.propagate(lineDatation.getDate(lastLine).shiftedBy(+1.0));
        return list;
    }

    private List<Pair<AbsoluteDate, Rotation>> orbitToQ(Orbit orbit, BodyShape earth,
                                                        LineDatation lineDatation, int firstLine, int lastLine,
                                                        double step)
        throws PropagationException {
        Propagator propagator = new KeplerianPropagator(orbit);
        propagator.setAttitudeProvider(new YawCompensation(new NadirPointing(earth)));
        propagator.propagate(lineDatation.getDate(firstLine).shiftedBy(-1.0));
        final List<Pair<AbsoluteDate, Rotation>> list = new ArrayList<Pair<AbsoluteDate, Rotation>>();
        propagator.setMasterMode(step, new OrekitFixedStepHandler() {
            public void init(SpacecraftState s0, AbsoluteDate t) {
            }   
            public void handleStep(SpacecraftState currentState, boolean isLast) {
                list.add(new Pair<AbsoluteDate, Rotation>(currentState.getDate(),
                                                          currentState.getAttitude().getRotation()));
            }
        });
        propagator.propagate(lineDatation.getDate(lastLine).shiftedBy(+1.0));
        return list;
    }

}

