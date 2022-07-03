/* Copyright 2013-2022 CS GROUP
 * Licensed to CS GROUP (CS) under one or more
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
package org.orekit.rugged.linesensor;

import java.io.File;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.net.URISyntaxException;
import java.util.ArrayList;
import java.util.List;

import org.hipparchus.geometry.euclidean.threed.Line;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.random.RandomGenerator;
import org.hipparchus.random.Well19937a;
import org.hipparchus.util.FastMath;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;
import org.orekit.attitudes.NadirPointing;
import org.orekit.attitudes.YawCompensation;
import org.orekit.bodies.BodyShape;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.bodies.OneAxisEllipsoid;
import org.orekit.data.DataContext;
import org.orekit.data.DirectoryCrawler;
import org.orekit.frames.FramesFactory;
import org.orekit.frames.Transform;
import org.orekit.orbits.CircularOrbit;
import org.orekit.orbits.Orbit;
import org.orekit.orbits.PositionAngle;
import org.orekit.propagation.Propagator;
import org.orekit.propagation.analytical.KeplerianPropagator;
import org.orekit.rugged.TestUtils;
import org.orekit.rugged.errors.RuggedException;
import org.orekit.rugged.errors.RuggedMessages;
import org.orekit.rugged.linesensor.SensorMeanPlaneCrossing.CrossingResult;
import org.orekit.rugged.los.LOSBuilder;
import org.orekit.rugged.utils.SpacecraftToObservedBody;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.AngularDerivativesFilter;
import org.orekit.utils.CartesianDerivativesFilter;
import org.orekit.utils.Constants;
import org.orekit.utils.IERSConventions;
import org.orekit.utils.PVCoordinates;
import org.orekit.utils.TimeStampedAngularCoordinates;
import org.orekit.utils.TimeStampedPVCoordinates;

public class SensorMeanPlaneCrossingTest {

    @Test
    public void testPerfectLine() {

        final Vector3D position  = new Vector3D(1.5, Vector3D.PLUS_I);
        final Vector3D normal    = Vector3D.PLUS_I;
        final Vector3D fovCenter = Vector3D.PLUS_K;
        final Vector3D cross     = Vector3D.crossProduct(normal, fovCenter);

        // build lists of pixels regularly spread on a perfect plane
        final List<Vector3D> los       = new ArrayList<Vector3D>();
        for (int i = -1000; i <= 1000; ++i) {
            final double alpha = i * 0.17 / 1000;
            los.add(new Vector3D(FastMath.cos(alpha), fovCenter, FastMath.sin(alpha), cross));
        }

        final LineSensor sensor = new LineSensor("perfect line",
                                                 new LinearLineDatation(AbsoluteDate.J2000_EPOCH, 0.0, 1.0 / 1.5e-3),
                                                 position, new LOSBuilder(los).build());

        Assert.assertEquals("perfect line", sensor.getName());
        Assert.assertEquals(AbsoluteDate.J2000_EPOCH, sensor.getDate(0.0));
        Assert.assertEquals(0.0, Vector3D.distance(position, sensor.getPosition()), 1.0e-15);

        SensorMeanPlaneCrossing mean = new SensorMeanPlaneCrossing(sensor, createInterpolator(sensor),
                                                                   0, 2000, true, true, 50, 0.01);
        Assert.assertEquals(0.0, Vector3D.angle(normal, mean.getMeanPlaneNormal()), 1.0e-15);

    }

    @Test
    public void testNoisyLine() {

        final RandomGenerator random    = new Well19937a(0xf3ddb33785e12bdal);
        final Vector3D        position  = new Vector3D(1.5, Vector3D.PLUS_I);
        final Vector3D        normal    = Vector3D.PLUS_I;
        final Vector3D        fovCenter = Vector3D.PLUS_K;
        final Vector3D        cross     = Vector3D.crossProduct(normal, fovCenter);

        // build lists of pixels regularly spread on a perfect plane
        final List<Vector3D> los       = new ArrayList<Vector3D>();
        for (int i = -1000; i <= 1000; ++i) {
            final double alpha = i * 0.17 / 10 + 1.0e-5 * random.nextDouble();
            final double delta = 1.0e-5 * random.nextDouble();
            final double cA = FastMath.cos(alpha);
            final double sA = FastMath.sin(alpha);
            final double cD = FastMath.cos(delta);
            final double sD = FastMath.sin(delta);
            los.add(new Vector3D(cA * cD, fovCenter, sA * cD, cross, sD, normal));
        }

        final LineSensor sensor = new LineSensor("noisy line",
                                                 new LinearLineDatation(AbsoluteDate.J2000_EPOCH, 0.0, 1.0 / 1.5e-3),
                                                 position, new LOSBuilder(los).build());

        Assert.assertEquals("noisy line", sensor.getName());
        Assert.assertEquals(AbsoluteDate.J2000_EPOCH, sensor.getDate(0.0));
        Assert.assertEquals(0.0, Vector3D.distance(position, sensor.getPosition()), 1.0e-5);

        SensorMeanPlaneCrossing mean = new SensorMeanPlaneCrossing(sensor, createInterpolator(sensor),
                                                                   0, 2000, true, true, 50, 0.01);
        Assert.assertEquals(0.0, Vector3D.angle(normal, mean.getMeanPlaneNormal()), 8.0e-7);

    }

    @Test
    public void testDerivativeWithoutCorrections() {
        doTestDerivative(false, false, 3.1e-11);
    }

    @Test
    public void testDerivativeLightTimeCorrection() {
        doTestDerivative(true, false, 2.4e-7);
    }

    @Test
    public void testDerivativeAberrationOfLightCorrection() {
        doTestDerivative(false, true, 1.1e-7);
    }

    @Test
    public void testDerivativeWithAllCorrections() {
        doTestDerivative(true, true, 1.4e-7);
    }

    private void doTestDerivative(boolean lightTimeCorrection,
                                  boolean aberrationOfLightCorrection,
                                  double tol) {

        final Vector3D position  = new Vector3D(1.5, Vector3D.PLUS_I);
        final Vector3D normal    = Vector3D.PLUS_I;
        final Vector3D fovCenter = Vector3D.PLUS_K;
        final Vector3D cross     = Vector3D.crossProduct(normal, fovCenter);

        // build lists of pixels regularly spread on a perfect plane
        final List<Vector3D> los       = new ArrayList<Vector3D>();
        for (int i = -1000; i <= 1000; ++i) {
            final double alpha = i * 0.17 / 1000;
            los.add(new Vector3D(FastMath.cos(alpha), fovCenter, FastMath.sin(alpha), cross));
        }

        final LineSensor sensor = new LineSensor("perfect line",
                                                 new LinearLineDatation(AbsoluteDate.J2000_EPOCH, 0.0, 1.0 / 1.5e-3),
                                                 position, new LOSBuilder(los).build());

        Assert.assertEquals("perfect line", sensor.getName());
        Assert.assertEquals(AbsoluteDate.J2000_EPOCH, sensor.getDate(0.0));
        Assert.assertEquals(0.0, Vector3D.distance(position, sensor.getPosition()), 1.0e-15);

        SensorMeanPlaneCrossing mean = new SensorMeanPlaneCrossing(sensor, createInterpolator(sensor),
                                                                   0, 2000, lightTimeCorrection,
                                                                   aberrationOfLightCorrection, 50, 1.0e-6);

        double       refLine = 1200.0;
        AbsoluteDate refDate = sensor.getDate(refLine);
        int          refPixel= 1800;
        Transform    b2i     = mean.getScToBody().getBodyToInertial(refDate);
        Transform    sc2i    = mean.getScToBody().getScToInertial(refDate);
        Transform    sc2b    = new Transform(refDate, sc2i, b2i.getInverse());
        Vector3D     p1      = sc2b.transformPosition(position);
        Vector3D     p2      = sc2b.transformPosition(new Vector3D(1, position,
                                                                   1.0e6, los.get(refPixel)));
        Line         line    = new Line(p1, p2, 0.001);
        BodyShape earth = new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                               Constants.WGS84_EARTH_FLATTENING,
                                               mean.getScToBody().getBodyFrame());
        GeodeticPoint groundPoint = earth.getIntersectionPoint(line, p1, mean.getScToBody().getBodyFrame(), refDate);
        Vector3D      gpCartesian = earth.transform(groundPoint);
        SensorMeanPlaneCrossing.CrossingResult result = mean.find(gpCartesian);

        if (lightTimeCorrection) {
            // applying corrections shifts the point with respect
            // to the reference result computed from a simple model above
            Assert.assertTrue(result.getLine() - refLine > 0.02);
        } else if (aberrationOfLightCorrection) {
            // applying corrections shifts the point with respect
            // to the reference result computed from a simple model above
            Assert.assertTrue(result.getLine() - refLine > 1.9);
        } else {
            // the simple model from which reference results have been compute applies here
            Assert.assertEquals(refLine, result.getLine(), 5.0e-11* refLine);
            Assert.assertEquals(0.0, result.getDate().durationFrom(refDate), 1.0e-9);
            Assert.assertEquals(0.0, Vector3D.angle(los.get(refPixel), result.getTargetDirection()), 5.4e-15);
        }

        double deltaL = 0.5;
        Transform b2scPlus = sc2b.getInverse().shiftedBy(deltaL / sensor.getRate(refLine));
        Vector3D dirPlus = b2scPlus.transformPosition(gpCartesian).subtract(position).normalize();
        Transform b2scMinus = sc2b.getInverse().shiftedBy(-deltaL / sensor.getRate(refLine));
        Vector3D dirMinus = b2scMinus.transformPosition(gpCartesian).subtract(position).normalize();
        Vector3D dirDer = new Vector3D(1.0 / (2 * deltaL), dirPlus.subtract(dirMinus));
        Assert.assertEquals(0.0,
                            Vector3D.distance(result.getTargetDirectionDerivative(), dirDer),
                            tol * dirDer.getNorm());

        try {
            mean.getScToBody().getBodyToInertial(refDate.shiftedBy(-Constants.JULIAN_CENTURY));
            Assert.fail("an exception should have been thrown");
        } catch (RuggedException re) {
            Assert.assertEquals(RuggedMessages.OUT_OF_TIME_RANGE, re.getSpecifier());
        }
        try {
            mean.getScToBody().getBodyToInertial(refDate.shiftedBy(Constants.JULIAN_CENTURY));
            Assert.fail("an exception should have been thrown");
        } catch (RuggedException re) {
            Assert.assertEquals(RuggedMessages.OUT_OF_TIME_RANGE, re.getSpecifier());
        }
        Assert.assertNotNull(mean.getScToBody().getBodyToInertial(refDate));

    }

    @Test
    public void testSlowFind()
        throws NoSuchMethodException,
               SecurityException, IllegalAccessException, IllegalArgumentException,
               InvocationTargetException {

        final Vector3D position  = new Vector3D(1.5, Vector3D.PLUS_I);
        final Vector3D normal    = Vector3D.PLUS_I;
        final Vector3D fovCenter = Vector3D.PLUS_K;
        final Vector3D cross     = Vector3D.crossProduct(normal, fovCenter);

        // build lists of pixels regularly spread on a perfect plane
        final List<Vector3D> los       = new ArrayList<Vector3D>();
        for (int i = -1000; i <= 1000; ++i) {
            final double alpha = i * 0.17 / 1000;
            los.add(new Vector3D(FastMath.cos(alpha), fovCenter, FastMath.sin(alpha), cross));
        }

        final LineSensor sensor = new LineSensor("perfect line",
                                                 new LinearLineDatation(AbsoluteDate.J2000_EPOCH, 0.0, 1.0 / 1.5e-3),
                                                 position, new LOSBuilder(los).build());

        Assert.assertEquals("perfect line", sensor.getName());
        Assert.assertEquals(AbsoluteDate.J2000_EPOCH, sensor.getDate(0.0));
        Assert.assertEquals(0.0, Vector3D.distance(position, sensor.getPosition()), 1.0e-15);

        SensorMeanPlaneCrossing mean = new SensorMeanPlaneCrossing(sensor, createInterpolator(sensor),
                                                                   0, 2000, true, true, 50, 1.0e-6);

        double       refLine = 1200.0;
        AbsoluteDate refDate = sensor.getDate(refLine);
        int          refPixel= 1800;
        Transform    b2i     = mean.getScToBody().getBodyToInertial(refDate);
        Transform    sc2i    = mean.getScToBody().getScToInertial(refDate);
        Transform    sc2b    = new Transform(refDate, sc2i, b2i.getInverse());
        Vector3D     p1      = sc2b.transformPosition(position);
        Vector3D     p2      = sc2b.transformPosition(new Vector3D(1, position,
                                                                   1.0e6, los.get(refPixel)));
        Line         line    = new Line(p1, p2, 0.001);
        BodyShape earth = new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                               Constants.WGS84_EARTH_FLATTENING,
                                               mean.getScToBody().getBodyFrame());
        GeodeticPoint groundPoint = earth.getIntersectionPoint(line, p1, mean.getScToBody().getBodyFrame(), refDate);
        Vector3D      gpCartesian = earth.transform(groundPoint);
        SensorMeanPlaneCrossing.CrossingResult result = mean.find(gpCartesian);

        Method slowFind = SensorMeanPlaneCrossing.class.getDeclaredMethod("slowFind",
                                                                          PVCoordinates.class,
                                                                          Double.TYPE);
        slowFind.setAccessible(true);
        SensorMeanPlaneCrossing.CrossingResult slowResult =
                        (CrossingResult) slowFind.invoke(mean,
                        new PVCoordinates(gpCartesian, Vector3D.ZERO),
                        400.0);

        Assert.assertEquals(result.getLine(), slowResult.getLine(), 2.0e-8);
        Assert.assertEquals(0.0,
                            Vector3D.distance(result.getTargetDirection(),
                                              slowResult.getTargetDirection()),
                            2.0e-13);
        Assert.assertEquals(0.0,
                            Vector3D.distance(result.getTargetDirectionDerivative(),
                                              slowResult.getTargetDirectionDerivative()),
                            1.0e-15);
    }

    private SpacecraftToObservedBody createInterpolator(LineSensor sensor) {
        
        Orbit orbit = new CircularOrbit(7173352.811913891,
                                        -4.029194321683225E-4, 0.0013530362644647786,
                                        FastMath.toRadians(98.63218182243709),
                                        FastMath.toRadians(77.55565567747836),
                                        FastMath.PI, PositionAngle.TRUE,
                                        FramesFactory.getEME2000(), sensor.getDate(1000),
                                        Constants.EIGEN5C_EARTH_MU);
        BodyShape earth = new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                               Constants.WGS84_EARTH_FLATTENING,
                                               FramesFactory.getITRF(IERSConventions.IERS_2010, true));
        AbsoluteDate minDate = sensor.getDate(0);
        AbsoluteDate maxDate = sensor.getDate(2000);
        return new SpacecraftToObservedBody(orbit.getFrame(), earth.getBodyFrame(),
                                            minDate, maxDate, 0.01,
                                            5.0,
                                            orbitToPV(orbit, earth, minDate, maxDate, 0.25), 2,
                                            CartesianDerivativesFilter.USE_P,
                                            orbitToQ(orbit, earth, minDate, maxDate, 0.25), 2,
                                            AngularDerivativesFilter.USE_R);
    }

    private List<TimeStampedPVCoordinates> orbitToPV(Orbit orbit, BodyShape earth,
                                                     AbsoluteDate minDate, AbsoluteDate maxDate,
                                                     double step) {
        
        Propagator propagator = new KeplerianPropagator(orbit);
        propagator.setAttitudeProvider(new YawCompensation(orbit.getFrame(), new NadirPointing(orbit.getFrame(), earth)));
        propagator.propagate(minDate);
        final List<TimeStampedPVCoordinates> list = new ArrayList<TimeStampedPVCoordinates>();
        propagator.getMultiplexer().add(step, currentState -> list.add(new TimeStampedPVCoordinates(currentState.getDate(),
                                                                                                    currentState.getPVCoordinates().getPosition(),
                                                                                                    currentState.getPVCoordinates().getVelocity(),
                                                                                                    Vector3D.ZERO)));
        propagator.propagate(maxDate);
        return list;
    }

    private List<TimeStampedAngularCoordinates> orbitToQ(Orbit orbit, BodyShape earth,
                                                         AbsoluteDate minDate, AbsoluteDate maxDate,
                                                         double step) {
        
        Propagator propagator = new KeplerianPropagator(orbit);
        propagator.setAttitudeProvider(new YawCompensation(orbit.getFrame(), new NadirPointing(orbit.getFrame(), earth)));
        propagator.propagate(minDate);
        final List<TimeStampedAngularCoordinates> list = new ArrayList<TimeStampedAngularCoordinates>();
        propagator.getMultiplexer().add(step, currentState -> list.add(new TimeStampedAngularCoordinates(currentState.getDate(),
                                                                                                         currentState.getAttitude().getRotation(),
                                                                                                         Vector3D.ZERO, Vector3D.ZERO)));
        propagator.propagate(maxDate);
        return list;
    }

    @Before
    public void setUp() throws URISyntaxException {
        TestUtils.clearFactories();
        String path = getClass().getClassLoader().getResource("orekit-data").toURI().getPath();
        DataContext.getDefault().getDataProvidersManager().addProvider(new DirectoryCrawler(new File(path)));
    }

}
