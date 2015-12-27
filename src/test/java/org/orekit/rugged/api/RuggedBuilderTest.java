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
package org.orekit.rugged.api;


import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.EOFException;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.StreamCorruptedException;
import java.net.URISyntaxException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.RotationConvention;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.ode.nonstiff.DormandPrince853Integrator;
import org.apache.commons.math3.util.FastMath;
import org.junit.Assert;
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
import org.orekit.forces.gravity.HolmesFeatherstoneAttractionModel;
import org.orekit.forces.gravity.ThirdBodyAttraction;
import org.orekit.forces.gravity.potential.GravityFieldFactory;
import org.orekit.forces.gravity.potential.NormalizedSphericalHarmonicsProvider;
import org.orekit.frames.Frame;
import org.orekit.frames.FramesFactory;
import org.orekit.frames.Transform;
import org.orekit.orbits.CircularOrbit;
import org.orekit.orbits.Orbit;
import org.orekit.orbits.OrbitType;
import org.orekit.orbits.PositionAngle;
import org.orekit.propagation.Propagator;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.analytical.KeplerianPropagator;
import org.orekit.propagation.numerical.NumericalPropagator;
import org.orekit.propagation.sampling.OrekitFixedStepHandler;
import org.orekit.rugged.errors.RuggedException;
import org.orekit.rugged.errors.RuggedMessages;
import org.orekit.rugged.linesensor.LineDatation;
import org.orekit.rugged.linesensor.LineSensor;
import org.orekit.rugged.linesensor.LinearLineDatation;
import org.orekit.rugged.los.LOSBuilder;
import org.orekit.rugged.los.TimeDependentLOS;
import org.orekit.rugged.raster.RandomLandscapeUpdater;
import org.orekit.rugged.raster.TileUpdater;
import org.orekit.rugged.raster.VolcanicConeElevationUpdater;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeScale;
import org.orekit.time.TimeScalesFactory;
import org.orekit.utils.AngularDerivativesFilter;
import org.orekit.utils.CartesianDerivativesFilter;
import org.orekit.utils.Constants;
import org.orekit.utils.IERSConventions;
import org.orekit.utils.PVCoordinates;
import org.orekit.utils.TimeStampedAngularCoordinates;
import org.orekit.utils.TimeStampedPVCoordinates;

public class RuggedBuilderTest {

    @Rule
    public TemporaryFolder tempFolder = new TemporaryFolder();

    @Test
    public void testSetContextWithEphemerides()
        throws RuggedException, OrekitException, URISyntaxException {

        String path = getClass().getClassLoader().getResource("orekit-data").toURI().getPath();
        DataProvidersManager.getInstance().addProvider(new DirectoryCrawler(new File(path)));
        AbsoluteDate t0 = new AbsoluteDate("2012-01-01T00:00:00", TimeScalesFactory.getUTC());

        List<TimeStampedPVCoordinates> pv = Arrays.asList(
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

        List<TimeStampedAngularCoordinates> q = Arrays.asList(
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
                new VolcanicConeElevationUpdater(new GeodeticPoint(FastMath.toRadians(13.25667),
                                                                   FastMath.toRadians(123.685),
                                                                   2463.0),
                                                 FastMath.toRadians(30.0), 16.0,
                                                 FastMath.toRadians(1.0), 1201);

        RuggedBuilder builder = new RuggedBuilder().
                                setDigitalElevationModel(updater, 8).
                                setAlgorithm(AlgorithmId.DUVENHAGE).
                                setEllipsoid(EllipsoidId.GRS80, BodyRotatingFrameId.ITRF).
                                setTimeSpan(pv.get(0).getDate(), pv.get(pv.size() - 1).getDate(), 0.001, 5.0).
                                setTrajectory(InertialFrameId.EME2000,
                                              pv, 8, CartesianDerivativesFilter.USE_PV,
                                              q, 8, AngularDerivativesFilter.USE_R);

        // light time correction and aberration of light correction are enabled by default
        Rugged rugged = builder.build();
        Assert.assertTrue(rugged.isLightTimeCorrected());
        Assert.assertTrue(rugged.isAberrationOfLightCorrected());

        builder.setLightTimeCorrection(false);
        rugged = builder.build();
        Assert.assertFalse(rugged.isLightTimeCorrected());
        Assert.assertTrue(rugged.isAberrationOfLightCorrected());

        builder.setAberrationOfLightCorrection(false);
        rugged = builder.build();
        Assert.assertFalse(rugged.isLightTimeCorrected());
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
                new VolcanicConeElevationUpdater(new GeodeticPoint(FastMath.toRadians(13.25667),
                                                                   FastMath.toRadians(123.685),
                                                                   2463.0),
                                                 FastMath.toRadians(30.0), 16.0,
                                                 FastMath.toRadians(1.0), 1201);

        RuggedBuilder builder = new RuggedBuilder().
                setDigitalElevationModel(updater, 8).
                setAlgorithm(AlgorithmId.DUVENHAGE).
                setEllipsoid(EllipsoidId.IERS96, BodyRotatingFrameId.ITRF).
                setTimeSpan(orbit.getDate().shiftedBy(-10.0), orbit.getDate().shiftedBy(+10.0), 0.001, 5.0).
                setTrajectory(1.0, 8, CartesianDerivativesFilter.USE_PV, AngularDerivativesFilter.USE_R, propagator);

        // light time correction and aberration of light correction are enabled by default
        Rugged rugged = builder.build();
        Assert.assertTrue(rugged.isLightTimeCorrected());
        Assert.assertTrue(rugged.isAberrationOfLightCorrected());

        builder.setLightTimeCorrection(false);
        rugged = builder.build();
        Assert.assertFalse(rugged.isLightTimeCorrected());
        Assert.assertTrue(rugged.isAberrationOfLightCorrected());

        builder.setAberrationOfLightCorrection(false);
        rugged = builder.build();
        Assert.assertFalse(rugged.isLightTimeCorrected());
        Assert.assertFalse(rugged.isAberrationOfLightCorrected());
        Assert.assertEquals(orbit.getDate().shiftedBy(-10.0), rugged.getMinDate());
        Assert.assertEquals(orbit.getDate().shiftedBy(+10.0), rugged.getMaxDate());
        Assert.assertEquals(0, rugged.getLineSensors().size());

    }

    @Test
    public void testOutOfTimeRange()
        throws RuggedException, OrekitException, URISyntaxException {

        String path = getClass().getClassLoader().getResource("orekit-data").toURI().getPath();
        DataProvidersManager.getInstance().addProvider(new DirectoryCrawler(new File(path)));
        AbsoluteDate t0 = new AbsoluteDate("2012-01-01T00:00:00", TimeScalesFactory.getUTC());

        List<TimeStampedPVCoordinates> pv = Arrays.asList(
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
            createPV(t0,10.000, -1556036.168, -6999292.945,  -73727.188, -1078.366288, 307.132868, -7372.452352));

        List<TimeStampedAngularCoordinates> q = Arrays.asList(
            createQ(t0, 4.000, 0.517572246112, -0.399109487434,  0.581929667081,  0.483930211565),
            createQ(t0, 5.000, 0.517876365096, -0.398856552030,  0.581658654071,  0.484139165451),
            createQ(t0, 6.000, 0.518180341637, -0.398603508416,  0.581387482627,  0.484347986126),
            createQ(t0, 7.000, 0.518484175647, -0.398350356669,  0.581116152834,  0.484556673529),
            createQ(t0, 8.000, 0.518787867035, -0.398097096866,  0.580844664773,  0.484765227599));

        TileUpdater updater =
                new VolcanicConeElevationUpdater(new GeodeticPoint(FastMath.toRadians(13.25667),
                                                                   FastMath.toRadians(123.685),
                                                                   2463.0),
                                                 FastMath.toRadians(30.0), 16.0,
                                                 FastMath.toRadians(1.0), 1201);

        Assert.assertNotNull(new RuggedBuilder().
                             setDigitalElevationModel(updater, 8).
                             setAlgorithm(AlgorithmId.DUVENHAGE).
                             setEllipsoid(EllipsoidId.WGS84, BodyRotatingFrameId.ITRF).
                             setTimeSpan(t0.shiftedBy(4), t0.shiftedBy(6), 0.001, 5.0).
                             setTrajectory(InertialFrameId.EME2000,
                                           pv,2, CartesianDerivativesFilter.USE_PV,
                                           q, 2, AngularDerivativesFilter.USE_R).
                             build());
        try {
            new RuggedBuilder().
            setDigitalElevationModel(updater, 8).
            setAlgorithm(AlgorithmId.DUVENHAGE).
            setEllipsoid(EllipsoidId.WGS84, BodyRotatingFrameId.ITRF).
            setTimeSpan(t0.shiftedBy(-1), t0.shiftedBy(6), 0.001, 0.0001).
            setTrajectory(InertialFrameId.EME2000,
                          pv, 2, CartesianDerivativesFilter.USE_PV,
                          q, 2, AngularDerivativesFilter.USE_R);
        } catch (RuggedException re) {
            Assert.assertEquals(RuggedMessages.OUT_OF_TIME_RANGE, re.getSpecifier());
            Assert.assertEquals(t0.shiftedBy(-1), re.getParts()[0]);
        }

        try {
            new RuggedBuilder().
            setDigitalElevationModel(updater, 8).
            setAlgorithm(AlgorithmId.DUVENHAGE).
            setEllipsoid(EllipsoidId.WGS84, BodyRotatingFrameId.ITRF).
            setTimeSpan(t0.shiftedBy(2), t0.shiftedBy(6), 0.001, 0.0001).
            setTrajectory(InertialFrameId.EME2000,
                          pv, 2, CartesianDerivativesFilter.USE_PV,
                          q, 2, AngularDerivativesFilter.USE_R);
        } catch (RuggedException re) {
            Assert.assertEquals(RuggedMessages.OUT_OF_TIME_RANGE, re.getSpecifier());
            Assert.assertEquals(t0.shiftedBy(2), re.getParts()[0]);
        }

        try {
            new RuggedBuilder().
            setDigitalElevationModel(updater, 8).
            setAlgorithm(AlgorithmId.DUVENHAGE).
            setEllipsoid(EllipsoidId.WGS84, BodyRotatingFrameId.ITRF).
            setTimeSpan(t0.shiftedBy(4), t0.shiftedBy(9), 0.001, 0.0001).
            setTrajectory(InertialFrameId.EME2000,
                          pv, 2, CartesianDerivativesFilter.USE_PV,
                          q, 2, AngularDerivativesFilter.USE_R);
        } catch (RuggedException re) {
            Assert.assertEquals(RuggedMessages.OUT_OF_TIME_RANGE, re.getSpecifier());
            Assert.assertEquals(t0.shiftedBy(9), re.getParts()[0]);
        }

        try {
            new RuggedBuilder().
            setDigitalElevationModel(updater, 8).
            setAlgorithm(AlgorithmId.DUVENHAGE).
            setEllipsoid(EllipsoidId.WGS84, BodyRotatingFrameId.ITRF).
            setTimeSpan(t0.shiftedBy(4), t0.shiftedBy(12), 0.001, 0.0001).
            setTrajectory(InertialFrameId.EME2000,
                          pv, 2, CartesianDerivativesFilter.USE_PV,
                          q, 2, AngularDerivativesFilter.USE_R);
        } catch (RuggedException re) {
            Assert.assertEquals(RuggedMessages.OUT_OF_TIME_RANGE, re.getSpecifier());
            Assert.assertEquals(t0.shiftedBy(12), re.getParts()[0]);
        }

    }

    @Test
    public void testInterpolatorDump()
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
        TimeDependentLOS los = createLOSPerfectLine(new Rotation(Vector3D.PLUS_I,
                                                                 FastMath.toRadians(50.0),
                                                                 RotationConvention.VECTOR_OPERATOR).applyTo(Vector3D.PLUS_K),
                                                    Vector3D.PLUS_I, FastMath.toRadians(1.0), dimension);

        // linear datation model: at reference time we get line 100, and the rate is one line every 1.5ms
        LineDatation lineDatation = new LinearLineDatation(crossing, dimension / 2, 1.0 / 1.5e-3);
        int firstLine = 0;
        int lastLine  = dimension;
        LineSensor lineSensor = new LineSensor("line", lineDatation, position, los);
        AbsoluteDate minDate = lineSensor.getDate(firstLine);
        AbsoluteDate maxDate = lineSensor.getDate(lastLine);

        TileUpdater updater =
                new RandomLandscapeUpdater(0.0, 9000.0, 0.5, 0x84186d1344722b8fl,
                                           FastMath.toRadians(1.0), 257);

        RuggedBuilder original = new RuggedBuilder().
                                setDigitalElevationModel(updater, 8).
                                setAlgorithm(AlgorithmId.DUVENHAGE).
                                setEllipsoid(EllipsoidId.WGS84, BodyRotatingFrameId.ITRF).
                                setTimeSpan(minDate, maxDate, 0.001, 5.0).
                                setTrajectory(InertialFrameId.EME2000,
                                              orbitToPV(orbit, earth, minDate.shiftedBy(-1.0), maxDate.shiftedBy(+1.0), 0.25),
                                              8, CartesianDerivativesFilter.USE_PV,
                                              orbitToQ(orbit, earth, minDate.shiftedBy(-1.0), maxDate.shiftedBy(+1.0), 0.25),
                                              2, AngularDerivativesFilter.USE_R).
                                addLineSensor(lineSensor);

        ByteArrayOutputStream bos = new ByteArrayOutputStream();
        original.storeInterpolator(bos);
        Assert.assertTrue(bos.size() > 100000);
        Assert.assertTrue(bos.size() < 200000);

        GeodeticPoint[] gpOriginal = original.build().directLocation("line", 100);

        RuggedBuilder recovered = new RuggedBuilder().
                setDigitalElevationModel(updater, 8).
                setAlgorithm(AlgorithmId.DUVENHAGE).
                setEllipsoid(EllipsoidId.WGS84, BodyRotatingFrameId.ITRF).
                setTrajectoryAndTimeSpan(new ByteArrayInputStream(bos.toByteArray())).
                addLineSensor(lineSensor);
        GeodeticPoint[] gpRecovered = recovered.build().directLocation("line", 100);

        for (int i = 0; i < gpOriginal.length; ++i) {
            Vector3D pOriginal  = earth.transform(gpOriginal[i]);
            Vector3D pRecovered = earth.transform(gpRecovered[i]);
            Assert.assertEquals(0.0, Vector3D.distance(pOriginal, pRecovered), 1.0e-15);
        }

    }

    @Test
    public void testInterpolatorCannotDump()
        throws RuggedException, OrekitException, URISyntaxException, IOException {

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
        TimeDependentLOS los = createLOSPerfectLine(new Rotation(Vector3D.PLUS_I,
                                                                 FastMath.toRadians(50.0),
                                                                 RotationConvention.VECTOR_OPERATOR).applyTo(Vector3D.PLUS_K),
                                                    Vector3D.PLUS_I, FastMath.toRadians(1.0), dimension);

        // linear datation model: at reference time we get line 100, and the rate is one line every 1.5ms
        LineDatation lineDatation = new LinearLineDatation(crossing, dimension / 2, 1.0 / 1.5e-3);
        int firstLine = 0;
        int lastLine  = dimension;
        LineSensor lineSensor = new LineSensor("line", lineDatation, position, los);
        AbsoluteDate minDate = lineSensor.getDate(firstLine);
        AbsoluteDate maxDate = lineSensor.getDate(lastLine);

        RuggedBuilder original = new RuggedBuilder().
                setAlgorithm(AlgorithmId.IGNORE_DEM_USE_ELLIPSOID).
                setEllipsoid(EllipsoidId.WGS84, BodyRotatingFrameId.ITRF).
                setTimeSpan(minDate, maxDate, 0.001, 5.0).
                setTrajectory(InertialFrameId.EME2000,
                              orbitToPV(orbit, earth, minDate.shiftedBy(-1.0), maxDate.shiftedBy(+1.0), 0.25),
                              8, CartesianDerivativesFilter.USE_PV,
                              orbitToQ(orbit, earth, minDate.shiftedBy(-1.0), maxDate.shiftedBy(+1.0), 0.25),
                              2, AngularDerivativesFilter.USE_R);

        FileOutputStream fos = new FileOutputStream(tempFolder.newFile());
        fos.close();
        try {
            original.storeInterpolator(fos);
            Assert.fail("an exception should have been thrown");
        } catch (RuggedException re) {
            Assert.assertEquals(IOException.class, re.getCause().getClass());
        }
    }
    
    @Test
    public void testInterpolatorDumpWrongFrame()
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
        TimeDependentLOS los = createLOSPerfectLine(new Rotation(Vector3D.PLUS_I,
                                                                 FastMath.toRadians(50.0),
                                                                 RotationConvention.VECTOR_OPERATOR).applyTo(Vector3D.PLUS_K),
                                                    Vector3D.PLUS_I, FastMath.toRadians(1.0), dimension);

        // linear datation model: at reference time we get line 100, and the rate is one line every 1.5ms
        LineDatation lineDatation = new LinearLineDatation(crossing, dimension / 2, 1.0 / 1.5e-3);
        int firstLine = 0;
        int lastLine  = dimension;
        LineSensor lineSensor = new LineSensor("line", lineDatation, position, los);
        AbsoluteDate minDate = lineSensor.getDate(firstLine);
        AbsoluteDate maxDate = lineSensor.getDate(lastLine);

        RuggedBuilder original = new RuggedBuilder().
                setDigitalElevationModel(null, -1).
                setAlgorithm(AlgorithmId.IGNORE_DEM_USE_ELLIPSOID).
                setEllipsoid(EllipsoidId.WGS84, BodyRotatingFrameId.ITRF).
                setTimeSpan(minDate, maxDate, 0.001, 5.0).
                setTrajectory(InertialFrameId.EME2000,
                              orbitToPV(orbit, earth, minDate.shiftedBy(-1.0), maxDate.shiftedBy(+1.0), 0.25),
                              8, CartesianDerivativesFilter.USE_PV,
                              orbitToQ(orbit, earth, minDate.shiftedBy(-1.0), maxDate.shiftedBy(+1.0), 0.25),
                              2, AngularDerivativesFilter.USE_R);

        ByteArrayOutputStream bos = new ByteArrayOutputStream();
        original.storeInterpolator(bos);
        Assert.assertTrue(bos.size() > 100000);
        Assert.assertTrue(bos.size() < 200000);

        for (BodyRotatingFrameId bId : Arrays.asList(BodyRotatingFrameId.GTOD,
                                                     BodyRotatingFrameId.ITRF_EQUINOX)) {
            try {
                new RuggedBuilder().
                setAlgorithm(AlgorithmId.IGNORE_DEM_USE_ELLIPSOID).
                setEllipsoid(EllipsoidId.WGS84, bId).
                setTrajectoryAndTimeSpan(new ByteArrayInputStream(bos.toByteArray())).build();
                Assert.fail("an exception should have been thrown");
            } catch (RuggedException re) {
                Assert.assertEquals(RuggedMessages.FRAMES_MISMATCH_WITH_INTERPOLATOR_DUMP,
                                    re.getSpecifier());
            }
        }
    }

    @Test
    public void testInterpolatorNotADump()
        throws RuggedException, OrekitException, URISyntaxException {

        String path = getClass().getClassLoader().getResource("orekit-data").toURI().getPath();
        DataProvidersManager.getInstance().addProvider(new DirectoryCrawler(new File(path)));

        // the following array is a real serialization file corresponding to the following
        // made-up empty class that does not exist in Rugged:
        //        public class NonExistentClass implements java.io.Serializable {
        //            private static final long serialVersionUID = -1;
        //        }
        byte[] nonExistentClass = new byte[] {
            -84, -19,   0,   5, 115, 114,
              0,  16,  78, 111, 110,  69,
            120, 105, 115, 116, 101, 110,
            116,  67, 108,  97, 115, 115,
             -1,  -1,  -1,  -1,  -1,  -1,
             -1,  -1,   2,   0,   0, 120,
            112
        };

        // the following array is a real serialization file of object Integer.valueOf(1)
        byte[] integerOne = new byte[] {
          -84, -19,   0,   5, 115, 114,
            0,  17, 106,  97, 118,  97,
           46, 108,  97, 110, 103,  46,
           73, 110, 116, 101, 103, 101,
          114,  18, -30, -96, -92,  -9,
         -127,-121,  56,   2,   0,   1,
           73,   0,   5, 118,  97, 108,
          117, 101, 120, 114,   0,  16,
          106,  97, 118,  97,  46, 108,
           97, 110, 103,  46,  78, 117,
          109,  98, 101, 114,-122, -84,
         -107,  29,  11,-108, -32,-117,
            2,   0,   0, 120, 112,   0,
            0,   0,   1
        };

        // the following array is a truncation of the previous one
        byte[] truncatedDump = new byte[] {
          -84, -19,   0,   5, 115, 114,
            0,  17, 106,  97, 118,  97
        };

        byte[] notSerialization = new byte[] {
            1, 2, 3, 4, 5, 6
        };

        for (byte[] array : Arrays.asList(nonExistentClass, integerOne, truncatedDump, notSerialization)) {
            try {
                new RuggedBuilder().setTrajectoryAndTimeSpan(new ByteArrayInputStream(array));
                Assert.fail("an exception should have been thrown");
            } catch (RuggedException re) {
                Assert.assertEquals(RuggedMessages.NOT_INTERPOLATOR_DUMP_DATA,
                                    re.getSpecifier());
                if (array == nonExistentClass) {
                    Assert.assertEquals(ClassNotFoundException.class, re.getCause().getClass());
                } else if (array == integerOne) {
                    Assert.assertEquals(ClassCastException.class, re.getCause().getClass());
                } else if (array == truncatedDump) {
                    Assert.assertEquals(EOFException.class, re.getCause().getClass());
                } else if (array == notSerialization) {
                    Assert.assertEquals(StreamCorruptedException.class, re.getCause().getClass());
                }
            }
        }

    }

    protected void addSatellitePV(TimeScale gps, Frame eme2000, Frame itrf,
                                  ArrayList<TimeStampedPVCoordinates> satellitePVList,
                                  String absDate,
                                  double px, double py, double pz, double vx, double vy, double vz)
        throws OrekitException {
        AbsoluteDate ephemerisDate = new AbsoluteDate(absDate, gps);
        Vector3D position = new Vector3D(px, py, pz);
        Vector3D velocity = new Vector3D(vx, vy, vz);
        PVCoordinates pvITRF = new PVCoordinates(position, velocity);
        Transform transform = itrf.getTransformTo(eme2000, ephemerisDate);
        Vector3D pEME2000 = transform.transformPosition(pvITRF.getPosition());
        Vector3D vEME2000 = transform.transformVector(pvITRF.getVelocity());
        satellitePVList.add(new TimeStampedPVCoordinates(ephemerisDate, pEME2000, vEME2000, Vector3D.ZERO));
    }

    protected void addSatelliteQ(TimeScale gps, ArrayList<TimeStampedAngularCoordinates> satelliteQList, String absDate, double q0, double q1, double q2,
            double q3) {
        AbsoluteDate attitudeDate = new AbsoluteDate(absDate, gps);
        Rotation rotation = new Rotation(q0, q1, q2, q3, true);
        TimeStampedAngularCoordinates pair =
                new TimeStampedAngularCoordinates(attitudeDate, rotation, Vector3D.ZERO, Vector3D.ZERO);
        satelliteQList.add(pair);
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

        AttitudeProvider yawCompensation = new YawCompensation(orbit.getFrame(), new NadirPointing(orbit.getFrame(), earth));
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

    private TimeDependentLOS createLOSPerfectLine(Vector3D center, Vector3D normal, double halfAperture, int n) {
        List<Vector3D> list = new ArrayList<Vector3D>(n);
        for (int i = 0; i < n; ++i) {
            double alpha = (halfAperture * (2 * i + 1 - n)) / (n - 1);
            list.add(new Rotation(normal, alpha, RotationConvention.VECTOR_OPERATOR).applyTo(center));
        }
        return new LOSBuilder(list).build();
    }

    private TimeStampedPVCoordinates createPV(AbsoluteDate t0, double dt,
                                              double px, double py, double pz,
                                              double vx, double vy, double vz) {
        return new TimeStampedPVCoordinates(t0.shiftedBy(dt),
                                            new Vector3D(px, py, pz),
                                            new Vector3D(vx, vy, vz),
                                            Vector3D.ZERO);
    }

    private TimeStampedAngularCoordinates createQ(AbsoluteDate t0, double dt,
                                                       double q0, double q1, double q2, double q3) {
        return new TimeStampedAngularCoordinates(t0.shiftedBy(dt),
                                                 new Rotation(q0, q1, q2, q3, true),
                                                 Vector3D.ZERO, Vector3D.ZERO);
    }

    private List<TimeStampedPVCoordinates> orbitToPV(Orbit orbit, BodyShape earth,
                                                     AbsoluteDate minDate, AbsoluteDate maxDate,
                                                     double step)
        throws OrekitException {
        Propagator propagator = new KeplerianPropagator(orbit);
        propagator.setAttitudeProvider(new YawCompensation(orbit.getFrame(), new NadirPointing(orbit.getFrame(), earth)));
        propagator.propagate(minDate);
        final List<TimeStampedPVCoordinates> list = new ArrayList<TimeStampedPVCoordinates>();
        propagator.setMasterMode(step, new OrekitFixedStepHandler() {
            public void init(SpacecraftState s0, AbsoluteDate t) {
            }   
            public void handleStep(SpacecraftState currentState, boolean isLast) {
                list.add(new TimeStampedPVCoordinates(currentState.getDate(),
                                                      currentState.getPVCoordinates().getPosition(),
                                                      currentState.getPVCoordinates().getVelocity(),
                                                      Vector3D.ZERO));
            }
        });
        propagator.propagate(maxDate);
        return list;
    }

    private List<TimeStampedAngularCoordinates> orbitToQ(Orbit orbit, BodyShape earth,
                                                         AbsoluteDate minDate, AbsoluteDate maxDate,
                                                         double step)
        throws OrekitException {
        Propagator propagator = new KeplerianPropagator(orbit);
        propagator.setAttitudeProvider(new YawCompensation(orbit.getFrame(), new NadirPointing(orbit.getFrame(), earth)));
        propagator.propagate(minDate);
        final List<TimeStampedAngularCoordinates> list = new ArrayList<TimeStampedAngularCoordinates>();
        propagator.setMasterMode(step, new OrekitFixedStepHandler() {
            public void init(SpacecraftState s0, AbsoluteDate t) {
            }   
            public void handleStep(SpacecraftState currentState, boolean isLast) {
                list.add(new TimeStampedAngularCoordinates(currentState.getDate(),
                                                           currentState.getAttitude().getRotation(),
                                                           Vector3D.ZERO, Vector3D.ZERO));
            }
        });
        propagator.propagate(maxDate);
        return list;
    }

}

