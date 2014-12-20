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
package org.orekit.rugged.linesensor;

import java.io.File;
import java.net.URISyntaxException;
import java.util.ArrayList;
import java.util.List;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.random.RandomGenerator;
import org.apache.commons.math3.random.Well19937a;
import org.apache.commons.math3.util.FastMath;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;
import org.orekit.attitudes.NadirPointing;
import org.orekit.attitudes.YawCompensation;
import org.orekit.bodies.BodyShape;
import org.orekit.bodies.OneAxisEllipsoid;
import org.orekit.data.DataProvidersManager;
import org.orekit.data.DirectoryCrawler;
import org.orekit.errors.OrekitException;
import org.orekit.errors.PropagationException;
import org.orekit.frames.FramesFactory;
import org.orekit.orbits.CircularOrbit;
import org.orekit.orbits.Orbit;
import org.orekit.orbits.PositionAngle;
import org.orekit.propagation.Propagator;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.analytical.KeplerianPropagator;
import org.orekit.propagation.sampling.OrekitFixedStepHandler;
import org.orekit.rugged.errors.RuggedException;
import org.orekit.rugged.linesensor.LineSensor;
import org.orekit.rugged.linesensor.LinearLineDatation;
import org.orekit.rugged.linesensor.SensorMeanPlaneCrossing;
import org.orekit.rugged.los.LOSBuilder;
import org.orekit.rugged.utils.SpacecraftToObservedBody;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.AngularDerivativesFilter;
import org.orekit.utils.CartesianDerivativesFilter;
import org.orekit.utils.Constants;
import org.orekit.utils.IERSConventions;
import org.orekit.utils.TimeStampedAngularCoordinates;
import org.orekit.utils.TimeStampedPVCoordinates;

public class SensorMeanPlaneCrossingTest {

    @Test
    public void testPerfectLine() throws RuggedException, OrekitException {

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
    public void testNoisyLine() throws RuggedException, OrekitException {

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

    private SpacecraftToObservedBody createInterpolator(LineSensor sensor)
        throws RuggedException, OrekitException {
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
                                                     double step)
        throws PropagationException {
        Propagator propagator = new KeplerianPropagator(orbit);
        propagator.setAttitudeProvider(new YawCompensation(new NadirPointing(earth)));
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
        throws PropagationException {
        Propagator propagator = new KeplerianPropagator(orbit);
        propagator.setAttitudeProvider(new YawCompensation(new NadirPointing(earth)));
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

    @Before
    public void setUp() throws OrekitException, URISyntaxException {
        String path = getClass().getClassLoader().getResource("orekit-data").toURI().getPath();
        DataProvidersManager.getInstance().addProvider(new DirectoryCrawler(new File(path)));
    }

}
