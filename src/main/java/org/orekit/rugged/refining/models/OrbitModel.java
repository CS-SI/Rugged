/*
 * Copyright 2013-2016 CS Systèmes d'Information Licensed to CS Systèmes
 * d'Information (CS) under one or more contributor license agreements. See the
 * NOTICE file distributed with this work for additional information regarding
 * copyright ownership. CS licenses this file to You under the Apache License,
 * Version 2.0 (the "License"); you may not use this file except in compliance
 * with the License. You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0 Unless required by applicable law
 * or agreed to in writing, software distributed under the License is
 * distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied. See the License for the specific language
 * governing permissions and limitations under the License.
 */
package org.orekit.rugged.refining.models;

import org.hipparchus.geometry.euclidean.threed.Rotation;
import org.hipparchus.geometry.euclidean.threed.RotationOrder;
import org.hipparchus.geometry.euclidean.threed.RotationConvention;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.ode.nonstiff.DormandPrince853Integrator;
import org.hipparchus.util.FastMath;
import java.util.ArrayList;
import java.util.List;
import org.orekit.attitudes.AttitudeProvider;
import org.orekit.attitudes.NadirPointing;
import org.orekit.attitudes.YawCompensation;
import org.orekit.attitudes.LofOffset;
import org.orekit.attitudes.TabulatedLofOffset;
import org.orekit.bodies.BodyShape;
import org.orekit.bodies.CelestialBodyFactory;
import org.orekit.bodies.OneAxisEllipsoid;
import org.orekit.errors.OrekitException;
import org.orekit.forces.gravity.HolmesFeatherstoneAttractionModel;
import org.orekit.forces.gravity.ThirdBodyAttraction;
import org.orekit.forces.gravity.potential.GravityFieldFactory;
import org.orekit.forces.gravity.potential.NormalizedSphericalHarmonicsProvider;
import org.orekit.frames.Frame;
import org.orekit.frames.FramesFactory;
import org.orekit.frames.Transform;
import org.orekit.frames.LOFType;
import org.orekit.orbits.CircularOrbit;
import org.orekit.orbits.Orbit;
import org.orekit.orbits.OrbitType;
import org.orekit.orbits.PositionAngle;
import org.orekit.propagation.Propagator;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.analytical.KeplerianPropagator;
import org.orekit.propagation.numerical.NumericalPropagator;
import org.orekit.propagation.sampling.OrekitFixedStepHandler;
import org.orekit.time.AbsoluteDate;

import org.orekit.time.TimeScale;
import org.orekit.utils.Constants;
import org.orekit.utils.IERSConventions;
import org.orekit.utils.PVCoordinates;
import org.orekit.utils.TimeStampedAngularCoordinates;
import org.orekit.utils.TimeStampedPVCoordinates;
import org.orekit.utils.AngularDerivativesFilter;

/**
 * Orbit Model class to generate PV and Q
 * 
 * @author jguinet
 */

public class OrbitModel {
    
    
    /** Flag to change Attitude Law (by default Nadir Pointing Yaw Compensation). */
    private boolean UserDefinedLOFTransform; 

    /** User defined Roll law. */
    private double[] LOFTransformRollPoly;

    /** User defined Pitch law. */
    private double[] LOFTransformPitchPoly;

    /** User defined Yaw law. */
    private double[] LOFTransformYawPoly;

    /** reference date */
    private AbsoluteDate refDate;
    
    /** attitude Provider */
    AttitudeProvider attitudeProvider;
    
    public OrbitModel() {
        UserDefinedLOFTransform = false;
    }

    public static void addSatellitePV(TimeScale gps, Frame eme2000, Frame itrf,
                                      ArrayList<TimeStampedPVCoordinates> satellitePVList,
                                      String absDate, double px, double py,
                                      double pz, double vx, double vy,
                                      double vz)
        throws OrekitException {
        AbsoluteDate ephemerisDate = new AbsoluteDate(absDate, gps);
        Vector3D position = new Vector3D(px, py, pz);
        Vector3D velocity = new Vector3D(vx, vy, vz);
        PVCoordinates pvITRF = new PVCoordinates(position, velocity);
        Transform transform = itrf.getTransformTo(eme2000, ephemerisDate);
        Vector3D pEME2000 = transform.transformPosition(pvITRF.getPosition());
        Vector3D vEME2000 = transform.transformVector(pvITRF.getVelocity());
        satellitePVList
            .add(new TimeStampedPVCoordinates(ephemerisDate, pEME2000, vEME2000,
                                              Vector3D.ZERO));
    }

    public void addSatelliteQ(TimeScale gps,
                              ArrayList<TimeStampedAngularCoordinates> satelliteQList,
                              String absDate, double q0, double q1, double q2,
                              double q3) {
        AbsoluteDate attitudeDate = new AbsoluteDate(absDate, gps);
        Rotation rotation = new Rotation(q0, q1, q2, q3, true);
        TimeStampedAngularCoordinates pair = new TimeStampedAngularCoordinates(attitudeDate,
                                                                               rotation,
                                                                               Vector3D.ZERO,
                                                                               Vector3D.ZERO);
        satelliteQList.add(pair);
    }

    public BodyShape createEarth()
        throws OrekitException {
        return new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                    Constants.WGS84_EARTH_FLATTENING,
                                    FramesFactory.getITRF(
                                                          IERSConventions.IERS_2010, true));
    }

    public NormalizedSphericalHarmonicsProvider createGravityField()
        throws OrekitException {
        return GravityFieldFactory.getNormalizedProvider(12, 12);
    }

    public Orbit createOrbit(double mu, AbsoluteDate date)
        throws OrekitException {
        // the following orbital parameters have been computed using
        // Orekit tutorial about phasing, using the following configuration:
        //
        // orbit.date = 2012-01-01T00:00:00.000
        // phasing.orbits.number = 143
        // phasing.days.number = 10
        // sun.synchronous.reference.latitude = 0
        // sun.synchronous.reference.ascending = false
        // sun.synchronous.mean.solar.time = 10:30:00
        // gravity.field.degree = 12
        // gravity.field.order = 12

        Frame eme2000 = FramesFactory.getEME2000();
        // return new CircularOrbit(7173352.811913891,
        return new CircularOrbit(694000.0 +
                                 Constants.WGS84_EARTH_EQUATORIAL_RADIUS, // ajouter
                                                                          // Rayon
                                 -4.029194321683225E-4, 0.0013530362644647786,
                                 FastMath.toRadians(98.2), // pleiades
                                                           // inclinaison 98.2
                                 // FastMath.toRadians(-86.47),
                                 FastMath.toRadians(-86.47 + 180),
                                 FastMath.toRadians(135.9 + 0.3),
                                 PositionAngle.TRUE, eme2000, date, mu);
    }

    public void setLOFTransform(double[] rollPoly, double[] pitchPoly,
                                double[] yawPoly, AbsoluteDate refDate) {
        
        this.UserDefinedLOFTransform = true;
        LOFTransformRollPoly = rollPoly.clone();
        LOFTransformPitchPoly = pitchPoly.clone();
        LOFTransformYawPoly = yawPoly.clone();
        this.refDate = refDate;
    }

    private double getPoly(double[] poly, double shift) {
        double val = 0.0;
        for (int coef = 0; coef < poly.length; coef++) {
            val = val + poly[coef] * Math.pow(shift, coef);
        }
        return val;
    }

    private Rotation getOffset(BodyShape earth, Orbit orbit, double shift)
        throws OrekitException {
        LOFType type = LOFType.VVLH;
        double roll = getPoly(LOFTransformRollPoly, shift);
        double pitch = getPoly(LOFTransformPitchPoly, shift);
        double yaw = getPoly(LOFTransformYawPoly, shift);
        /**System.out.format("at shift %f roll %f pitch %f yaw %f \n ", shift,
                          roll, pitch, yaw);*/

        LofOffset law = new LofOffset(orbit.getFrame(), type, RotationOrder.XYZ,
                                      roll, pitch, yaw);
        Rotation offsetAtt = law
            .getAttitude(orbit, orbit.getDate().shiftedBy(shift),
                         orbit.getFrame())
            .getRotation();
        NadirPointing nadirPointing = new NadirPointing(orbit.getFrame(),
                                                        earth);
        Rotation nadirAtt = nadirPointing
            .getAttitude(orbit, orbit.getDate().getDate().shiftedBy(shift),
                         orbit.getFrame())
            .getRotation();
        Rotation offsetProper = offsetAtt
            .compose(nadirAtt.revert(), RotationConvention.VECTOR_OPERATOR);
        return offsetProper;
    }

    public AttitudeProvider createAttitudeProvider(BodyShape earth, Orbit orbit)
        throws OrekitException {

        if (this.UserDefinedLOFTransform) {
            LOFType type = LOFType.VVLH;

            ArrayList<TimeStampedAngularCoordinates> list = new ArrayList<TimeStampedAngularCoordinates>();

            for (double shift = -10.0; shift < +10.0; shift += 1e-2) {
                list.add(new TimeStampedAngularCoordinates(refDate
                    .shiftedBy(shift), getOffset(earth, orbit, shift),
                                                           Vector3D.ZERO,
                                                           Vector3D.ZERO));
            }

            TabulatedLofOffset tabulated = new TabulatedLofOffset(orbit
                .getFrame(), type, list, 2, AngularDerivativesFilter.USE_R);

            return tabulated;
        } else {
            return new YawCompensation(orbit.getFrame(), new NadirPointing(orbit.getFrame(), earth));
        }
    }

    public Propagator createPropagator(BodyShape earth,
                                       NormalizedSphericalHarmonicsProvider gravityField,
                                       Orbit orbit)
        throws OrekitException {

        AttitudeProvider attitudeProvider = createAttitudeProvider(earth,orbit);

        SpacecraftState state = new SpacecraftState(orbit, attitudeProvider
            .getAttitude(orbit, orbit.getDate(), orbit.getFrame()), 1180.0);

        // numerical model for improving orbit
        OrbitType type = OrbitType.CIRCULAR;
        double[][] tolerances = NumericalPropagator.tolerances(0.1, orbit,
                                                               type);
        DormandPrince853Integrator integrator = new DormandPrince853Integrator(1.0e-4 *
                                                                               orbit
                                                                                   .getKeplerianPeriod(),
                                                                               1.0e-1 * orbit
                                                                                   .getKeplerianPeriod(),
                                                                               tolerances[0],
                                                                               tolerances[1]);
        integrator.setInitialStepSize(1.0e-2 * orbit.getKeplerianPeriod());
        NumericalPropagator numericalPropagator = new NumericalPropagator(integrator);
        numericalPropagator
            .addForceModel(new HolmesFeatherstoneAttractionModel(earth
                .getBodyFrame(), gravityField));
        numericalPropagator
            .addForceModel(new ThirdBodyAttraction(CelestialBodyFactory
                .getSun()));
        numericalPropagator
            .addForceModel(new ThirdBodyAttraction(CelestialBodyFactory
                .getMoon()));
        numericalPropagator.setOrbitType(type);
        numericalPropagator.setInitialState(state);
        numericalPropagator.setAttitudeProvider(attitudeProvider);
        return numericalPropagator;

    }

    public List<TimeStampedPVCoordinates>
        orbitToPV(Orbit orbit, BodyShape earth, AbsoluteDate minDate,
                  AbsoluteDate maxDate, double step)
            throws OrekitException {
        Propagator propagator = new KeplerianPropagator(orbit);
        
        propagator.setAttitudeProvider(createAttitudeProvider(earth, orbit));
        
        propagator.propagate(minDate);
        final List<TimeStampedPVCoordinates> list = new ArrayList<TimeStampedPVCoordinates>();
        propagator.setMasterMode(step, new OrekitFixedStepHandler() {

            public void init(SpacecraftState s0, AbsoluteDate t) {
            }

            public void handleStep(SpacecraftState currentState,
                                   boolean isLast) {
                list.add(new TimeStampedPVCoordinates(currentState
                    .getDate(), currentState.getPVCoordinates().getPosition(),
                                                      currentState
                                                          .getPVCoordinates()
                                                          .getVelocity(),
                                                      Vector3D.ZERO));
            }
        });
        propagator.propagate(maxDate);
        return list;
    }

    public List<TimeStampedAngularCoordinates>
        orbitToQ(Orbit orbit, BodyShape earth, AbsoluteDate minDate,
                 AbsoluteDate maxDate, double step)
            throws OrekitException {
        Propagator propagator = new KeplerianPropagator(orbit);
        propagator.setAttitudeProvider(createAttitudeProvider(earth, orbit));
        propagator.propagate(minDate);
        final List<TimeStampedAngularCoordinates> list = new ArrayList<TimeStampedAngularCoordinates>();
        propagator.setMasterMode(step, new OrekitFixedStepHandler() {

            public void init(SpacecraftState s0, AbsoluteDate t) {
            }

            public void handleStep(SpacecraftState currentState,
                                   boolean isLast) {
                list.add(new TimeStampedAngularCoordinates(currentState
                    .getDate(), currentState.getAttitude().getRotation(),
                                                           Vector3D.ZERO,
                                                           Vector3D.ZERO));
            }
        });
        propagator.propagate(maxDate);
        return list;
    }

}
