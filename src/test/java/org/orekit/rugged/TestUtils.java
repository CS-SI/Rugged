/* Copyright 2013-2025 CS GROUP
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
package org.orekit.rugged;


import static org.junit.jupiter.api.Assertions.assertEquals;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import org.hipparchus.geometry.euclidean.threed.Rotation;
import org.hipparchus.geometry.euclidean.threed.RotationConvention;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.ode.nonstiff.DormandPrince853Integrator;
import org.hipparchus.util.FastMath;
import org.junit.jupiter.api.Assertions;
import org.orekit.attitudes.AttitudeProvider;
import org.orekit.attitudes.NadirPointing;
import org.orekit.attitudes.YawCompensation;
import org.orekit.bodies.BodyShape;
import org.orekit.bodies.CelestialBodyFactory;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.bodies.OneAxisEllipsoid;
import org.orekit.data.DataContext;
import org.orekit.forces.gravity.HolmesFeatherstoneAttractionModel;
import org.orekit.forces.gravity.ThirdBodyAttraction;
import org.orekit.forces.gravity.potential.GravityFieldFactory;
import org.orekit.forces.gravity.potential.NormalizedSphericalHarmonicsProvider;
import org.orekit.frames.Frame;
import org.orekit.frames.FramesFactory;
import org.orekit.frames.Transform;
import org.orekit.orbits.CircularOrbit;
import org.orekit.orbits.FieldCartesianOrbit;
import org.orekit.orbits.FieldCircularOrbit;
import org.orekit.orbits.FieldEquinoctialOrbit;
import org.orekit.orbits.FieldKeplerianOrbit;
import org.orekit.orbits.Orbit;
import org.orekit.orbits.OrbitType;
import org.orekit.orbits.PositionAngleType;
import org.orekit.propagation.CartesianToleranceProvider;
import org.orekit.propagation.Propagator;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.ToleranceProvider;
import org.orekit.propagation.analytical.KeplerianPropagator;
import org.orekit.propagation.numerical.NumericalPropagator;
import org.orekit.propagation.semianalytical.dsst.utilities.JacobiPolynomials;
import org.orekit.propagation.semianalytical.dsst.utilities.NewcombOperators;
import org.orekit.rugged.linesensor.SensorPixel;
import org.orekit.rugged.los.LOSBuilder;
import org.orekit.rugged.los.TimeDependentLOS;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeScale;
import org.orekit.time.TimeScalesFactory;
import org.orekit.utils.Constants;
import org.orekit.utils.IERSConventions;
import org.orekit.utils.PVCoordinates;
import org.orekit.utils.TimeStampedAngularCoordinates;
import org.orekit.utils.TimeStampedPVCoordinates;

/**
 * @author Luc Maisonobe
 * @author Guylaine Prat
 */
public class TestUtils {
    
    /**
     * Clean up of factories for JUnit 
     * TBN: copied from Utils of Test suite of Orekit 9.0
     * @since 2.0
     */
    public static void clearFactories() {

        clearFactoryMaps(CelestialBodyFactory.class);
        CelestialBodyFactory.clearCelestialBodyLoaders();
        clearFactoryMaps(FramesFactory.class);
        clearFactoryMaps(TimeScalesFactory.class);
        clearFactory(TimeScalesFactory.class, TimeScale.class);
        clearFactoryMaps(FieldCartesianOrbit.class);
        clearFactoryMaps(FieldKeplerianOrbit.class);
        clearFactoryMaps(FieldCircularOrbit.class);
        clearFactoryMaps(FieldEquinoctialOrbit.class);
        clearFactoryMaps(JacobiPolynomials.class);
        clearFactoryMaps(NewcombOperators.class);
        for (final Class<?> c : NewcombOperators.class.getDeclaredClasses()) {
            if (c.getName().endsWith("PolynomialsGenerator")) {
                clearFactoryMaps(c);
            }
        }
        FramesFactory.clearEOPHistoryLoaders();
        FramesFactory.setEOPContinuityThreshold(5 * Constants.JULIAN_DAY);
        TimeScalesFactory.clearUTCTAIOffsetsLoaders();
        GravityFieldFactory.clearPotentialCoefficientsReaders();
        GravityFieldFactory.clearOceanTidesReaders();
        DataContext.getDefault().getDataProvidersManager().clearProviders();
        DataContext.getDefault().getDataProvidersManager().clearLoadedDataNames();
    }

    /** Clean up of factory map
     * @param factoryClass
     * @since 2.0
     */
    private static void clearFactoryMaps(Class<?> factoryClass) {
        try {
            
            for (Field field : factoryClass.getDeclaredFields()) {
                if (Modifier.isStatic(field.getModifiers()) &&
                    Map.class.isAssignableFrom(field.getType())) {
                    field.setAccessible(true);
                    ((Map<?, ?>) field.get(null)).clear();
                }
            }
        } catch (IllegalAccessException iae) {
            Assertions.fail(iae.getMessage());
        }
    }
    
    /** Clean up of a factory
     * @param factoryClass
     * @param cachedFieldsClass
     * @since 2.0
     */
    private static void clearFactory(Class<?> factoryClass, Class<?> cachedFieldsClass) {
        try {
            
            for (Field field : factoryClass.getDeclaredFields()) {
                if (Modifier.isStatic(field.getModifiers()) &&
                        cachedFieldsClass.isAssignableFrom(field.getType())) {
                    field.setAccessible(true);
                    field.set(null, null);
                }
            }
        } catch (IllegalAccessException iae) {
            Assertions.fail(iae.getMessage());
        }
    }
    
    
    /**
     * Generate satellite ephemeris.
     */
    public static void addSatellitePV(TimeScale gps, Frame eme2000, Frame itrf,
                                      ArrayList<TimeStampedPVCoordinates> satellitePVList,
                                      String absDate,
                                      double px, double py, double pz, double vx, double vy, double vz) {
        
        AbsoluteDate ephemerisDate = new AbsoluteDate(absDate, gps);
        Vector3D position = new Vector3D(px, py, pz);
        Vector3D velocity = new Vector3D(vx, vy, vz);
        PVCoordinates pvITRF = new PVCoordinates(position, velocity);
        Transform transform = itrf.getTransformTo(eme2000, ephemerisDate);
        Vector3D pEME2000 = transform.transformPosition(pvITRF.getPosition());
        Vector3D vEME2000 = transform.transformVector(pvITRF.getVelocity());
        satellitePVList.add(new TimeStampedPVCoordinates(ephemerisDate, pEME2000, vEME2000, Vector3D.ZERO));
    }

    /**
     * Generate satellite attitudes.
     */
    public static void addSatelliteQ(TimeScale gps, ArrayList<TimeStampedAngularCoordinates> satelliteQList,
                                     String absDate, double q0, double q1, double q2, double q3) {
        
        AbsoluteDate attitudeDate = new AbsoluteDate(absDate, gps);
        Rotation rotation = new Rotation(q0, q1, q2, q3, true);
        TimeStampedAngularCoordinates pair =
                new TimeStampedAngularCoordinates(attitudeDate, rotation, Vector3D.ZERO, Vector3D.ZERO);
        satelliteQList.add(pair);
    }

    /** Create an Earth for Junit tests.
     */
    public static BodyShape createEarth() {
        
        return new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                    Constants.WGS84_EARTH_FLATTENING,
                                    FramesFactory.getITRF(IERSConventions.IERS_2010, true));
    }

    /** Created a gravity field.
     * @return normalized spherical harmonics coefficients
     */
    public static NormalizedSphericalHarmonicsProvider createGravityField() {
        
        return GravityFieldFactory.getNormalizedProvider(12, 12);
    }

    /** Create an orbit.
     * @param mu Earth gravitational constant
     * @return the orbit
     */
    public static Orbit createOrbit(double mu) {
        
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
        
        // Observation satellite about 800km above ground
        return new CircularOrbit(7173352.811913891,
                                 -4.029194321683225E-4, 0.0013530362644647786,
                                 FastMath.toRadians(98.63218182243709),
                                 FastMath.toRadians(77.55565567747836),
                                 FastMath.PI, PositionAngleType.TRUE,
                                 eme2000, date, mu);
    }
 
    /** Create the propagator of an orbit.
     * @return propagator of the orbit
     */
    public static Propagator createPropagator(BodyShape earth,
                                              NormalizedSphericalHarmonicsProvider gravityField,
                                              Orbit orbit) {

        AttitudeProvider yawCompensation = new YawCompensation(orbit.getFrame(), new NadirPointing(orbit.getFrame(), earth));
        SpacecraftState state = new SpacecraftState(orbit,
                                                    yawCompensation.getAttitude(orbit,
                                                                                orbit.getDate(),
                                                                                orbit.getFrame())).withMass(1180.0);

        // numerical model for improving orbit
        OrbitType type = OrbitType.CIRCULAR;
        double[][] tolerances = ToleranceProvider.of(CartesianToleranceProvider.of(0.1)).getTolerances(orbit, type);
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

    /** Create a perfect Line Of Sight list
     * @return the perfect LOS list
     */
    public static LOSBuilder createLOSPerfectLine(Vector3D center, Vector3D normal, double halfAperture, int n) {

        List<Vector3D> list = new ArrayList<Vector3D>(n);
        for (int i = 0; i < n; ++i) {
            double alpha = (halfAperture * (2 * i + 1 - n)) / (n - 1);
            list.add(new Rotation(normal, alpha, RotationConvention.VECTOR_OPERATOR).applyTo(center));
        }
        return new LOSBuilder(list);
    }

    /** Create a Line Of Sight which depends on time.
     * @return the dependent of time LOS
     */
    public static TimeDependentLOS createLOSCurvedLine(Vector3D center, Vector3D normal,
                                                 double halfAperture, double sagitta, int n) {
        
        Vector3D u = Vector3D.crossProduct(center, normal);
        List<Vector3D> list = new ArrayList<Vector3D>(n);
        for (int i = 0; i < n; ++i) {
            double x = (2.0 * i + 1.0 - n) / (n - 1);
            double alpha = x * halfAperture;
            double beta  = x * x * sagitta;
            list.add(new Rotation(normal, alpha, RotationConvention.VECTOR_OPERATOR).
                     applyTo(new Rotation(u, beta, RotationConvention.VECTOR_OPERATOR).
                     applyTo(center)));
        }
        return new LOSBuilder(list).build();
    }

    /** Propagate an orbit between 2 given dates
     * @return a list of TimeStampedPVCoordinates
     */
    public static List<TimeStampedPVCoordinates> orbitToPV(Orbit orbit, BodyShape earth,
                                                           AbsoluteDate minDate, AbsoluteDate maxDate,
                                                           double step)  {
        
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

    /** Propagate an attitude between 2 given dates
     * @return a list of TimeStampedAngularCoordinates
     */
    public static List<TimeStampedAngularCoordinates> orbitToQ(Orbit orbit, BodyShape earth,
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
    
    /**
     * Asserts that two SensorPixel are equal to within a positive delta for each component.
     * For more details see: 
     * org.junit.assert.assertEquals(double expected, double actual, double delta)
     */
    static public void sensorPixelAssertEquals(SensorPixel expected, SensorPixel result, double delta) {
        
        assertEquals(expected.getLineNumber(), result.getLineNumber(), delta);
        assertEquals(expected.getPixelNumber(), result.getPixelNumber(), delta);
    }

    /**
     * Tell if two SensorPixel are the same, where each components are equal to within a positive delta.
     * For more details see: 
     * org.junit.assert.assertEquals(String message, double expected, double actual, double delta)
     * @param expected expected sensor pixel
     * @param result actual sensor pixel
     * @param delta delta within two values are consider equal
     * @return true if the two SensorPixel are the same, false otherwise
     */
    static public Boolean sameSensorPixels(SensorPixel expected, SensorPixel result, double delta) {
        
        Boolean sameSensorPixel = false;
        
        // To have same pixel (true)
        sameSensorPixel = !(isDifferent(expected.getLineNumber(), result.getLineNumber(), delta) ||
                            isDifferent(expected.getPixelNumber(), result.getPixelNumber(), delta));
        
        return sameSensorPixel;
    }
    
    /**
     * Tell if two SensorPixel are the same, where each components are equal to within a positive delta.
     * For more details see: 
     * org.junit.assert.assertEquals(String message, double expected, double actual, double delta)
     * @param expected expected sensor pixel
     * @param result actual sensor pixel
     * @param delta delta within two values are consider equal
     * @return true if the two SensorPixel are the same, false otherwise
     */
    static public Boolean sameGeodeticPoints(GeodeticPoint expected, GeodeticPoint result, double delta) {
        
        Boolean sameGeodeticPoint = false;
        
        // To have same GeodeticPoint (true)
        sameGeodeticPoint = !(isDifferent(expected.getLatitude(), result.getLatitude(), delta) ||
                            isDifferent(expected.getLongitude(), result.getLongitude(), delta) ||
                            isDifferent(expected.getAltitude(), result.getAltitude(), delta));
        
        return sameGeodeticPoint;
    }

    
    /** Return true if two double values are different within a positive delta.
     * @param val1 first value to compare
     * @param val2 second value to compare 
     * @param delta delta within the two values are consider equal
     * @return true is different, false if equal within the positive delta
     */
    static private boolean isDifferent(double val1, double val2, double delta) {
        
        if (Double.compare(val1, val2) == 0) {
            return false;
        }
        if ((FastMath.abs(val1 - val2) <= delta)) {
            return false;
        }
        return true;
    }


}

