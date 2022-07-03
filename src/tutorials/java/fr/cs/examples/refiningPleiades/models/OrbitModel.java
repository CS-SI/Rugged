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
package fr.cs.examples.refiningPleiades.models;

import java.util.ArrayList;
import java.util.List;

import org.hipparchus.geometry.euclidean.threed.Rotation;
import org.hipparchus.geometry.euclidean.threed.RotationConvention;
import org.hipparchus.geometry.euclidean.threed.RotationOrder;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.util.FastMath;
import org.orekit.attitudes.AttitudeProvider;
import org.orekit.attitudes.LofOffset;
import org.orekit.attitudes.NadirPointing;
import org.orekit.attitudes.TabulatedLofOffset;
import org.orekit.attitudes.YawCompensation;
import org.orekit.bodies.BodyShape;
import org.orekit.bodies.OneAxisEllipsoid;
import org.orekit.forces.gravity.potential.GravityFieldFactory;
import org.orekit.forces.gravity.potential.NormalizedSphericalHarmonicsProvider;
import org.orekit.frames.Frame;
import org.orekit.frames.FramesFactory;
import org.orekit.frames.LOFType;
import org.orekit.frames.Transform;
import org.orekit.orbits.CircularOrbit;
import org.orekit.orbits.Orbit;
import org.orekit.orbits.PositionAngle;
import org.orekit.propagation.Propagator;
import org.orekit.propagation.analytical.KeplerianPropagator;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeScale;
import org.orekit.utils.AngularDerivativesFilter;
import org.orekit.utils.Constants;
import org.orekit.utils.IERSConventions;
import org.orekit.utils.PVCoordinates;
import org.orekit.utils.TimeStampedAngularCoordinates;
import org.orekit.utils.TimeStampedPVCoordinates;

/**
 * Orbit Model class to generate positions-velocities and attitude quaternions.
 * <p>
 * the aim of this class is to simulate the orbit model of an LEO satellite
 * </p>
 * @author Jonathan Guinet
 * @author Guylaine Prat
 * @since 2.0 
 */

public class OrbitModel {

    /** Flag to change Attitude Law (by default Nadir Pointing Yaw Compensation). */
    private boolean userDefinedLOFTransform;

    /** User defined Roll law. */
    private double[] lofTransformRollPoly;

    /** User defined Pitch law. */
    private double[] lofTransformPitchPoly;

    /** User defined Yaw law. */
    private double[] lofTransformYawPoly;

    /** Reference date. */
    private AbsoluteDate refDate;

    /** Default constructor.
     */
    public OrbitModel() {
        userDefinedLOFTransform = false;
    }

    /** Generate satellite ephemeris.
     */
    public static void addSatellitePV(final TimeScale gps, final Frame eme2000, final Frame itrf,
                                      final List<TimeStampedPVCoordinates> satellitePVList,
                                      final String absDate,
                                      final double px, final double py, final double pz,
                                      final double vx, final double vy, final double vz) {
    	
        final AbsoluteDate ephemerisDate = new AbsoluteDate(absDate, gps);
        final Vector3D position = new Vector3D(px, py, pz);
        final Vector3D velocity = new Vector3D(vx, vy, vz);
        final PVCoordinates pvITRF = new PVCoordinates(position, velocity);
        final Transform transform = itrf.getTransformTo(eme2000, ephemerisDate);
        final Vector3D pEME2000 = transform.transformPosition(pvITRF.getPosition());
        final Vector3D vEME2000 = transform.transformVector(pvITRF.getVelocity());
        satellitePVList.add(new TimeStampedPVCoordinates(ephemerisDate, pEME2000, vEME2000, Vector3D.ZERO));
    }

    /** Generate satellite attitude.
     */
    public void addSatelliteQ(final TimeScale gps,
                              final List<TimeStampedAngularCoordinates> satelliteQList,
                              final String absDate,
                              final double q0, final double q1, final double q2, final double q3) {
    	
        final AbsoluteDate attitudeDate = new AbsoluteDate(absDate, gps);
        final Rotation rotation = new Rotation(q0, q1, q2, q3, true);
        final TimeStampedAngularCoordinates pair = 
                new TimeStampedAngularCoordinates(attitudeDate, rotation, Vector3D.ZERO, Vector3D.ZERO);
        satelliteQList.add(pair);
    }

    /** Create an Earth.
     * @return the Earth as the WGS84 ellipsoid
     */
    public BodyShape createEarth() {
    	
        return new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                    Constants.WGS84_EARTH_FLATTENING,
                                    FramesFactory.getITRF(IERSConventions.IERS_2010, true));
    }

    /** Created a gravity field.
     * @return normalized spherical harmonics coefficients
     */
    public NormalizedSphericalHarmonicsProvider createGravityField() {
    	
        return GravityFieldFactory.getNormalizedProvider(12, 12);
    }

    /** Create an orbit at a chosen date.
     * @param mu Earth gravitational constant
     * @return the orbit
     */
    public Orbit createOrbit(final double mu, final AbsoluteDate date) {
    	
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

        final Frame eme2000 = FramesFactory.getEME2000();
        return new CircularOrbit(694000.0 + Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                 -4.029194321683225E-4,
                                 0.0013530362644647786,
                                 FastMath.toRadians(98.2), // Pleiades inclination 98.2 deg
                                 FastMath.toRadians(-86.47 + 180),
                                 FastMath.toRadians(135.9 + 0.3),
                                 PositionAngle.TRUE,
                                 eme2000,
                                 date,
                                 mu);
    }

    /** Set the Local Orbital Frame characteristics.
     */
    public void setLOFTransform(final double[] rollPoly, final double[] pitchPoly,
                                final double[] yawPoly, final AbsoluteDate date) {

        this.userDefinedLOFTransform = true;
        lofTransformRollPoly = rollPoly.clone();
        lofTransformPitchPoly = pitchPoly.clone();
        lofTransformYawPoly = yawPoly.clone();
        this.refDate = date;
    }

    /** Recompute the polynom coefficients with shift.
     */
    private double getPoly(final double[] poly, final double shift) {
    	
        double val = 0.0;
        for (int coef = 0; coef < poly.length; coef++) {
            val = val + poly[coef] * FastMath.pow(shift, coef);
        }
        return val;
    }

    /** Get the offset.
     */
   private Rotation getOffset(final BodyShape earth, final Orbit orbit, final double shift){
    	
        final LOFType type = LOFType.VVLH;
        final double roll = getPoly(lofTransformRollPoly, shift);
        final double pitch = getPoly(lofTransformPitchPoly, shift);
        final double yaw = getPoly(lofTransformYawPoly, shift);

        final LofOffset law = new LofOffset(orbit.getFrame(), type, RotationOrder.XYZ,
                                      roll, pitch, yaw);
        final Rotation offsetAtt = law.
                                   getAttitude(orbit, orbit.getDate().shiftedBy(shift), orbit.getFrame()).
                                   getRotation();
        final NadirPointing nadirPointing = new NadirPointing(orbit.getFrame(), earth);
        final Rotation nadirAtt = nadirPointing.
                                  getAttitude(orbit, orbit.getDate().getDate().shiftedBy(shift), orbit.getFrame()).
                                  getRotation();
        final Rotation offsetProper = offsetAtt.compose(nadirAtt.revert(), RotationConvention.VECTOR_OPERATOR);

        return offsetProper;
    }

   /** Create the attitude provider.
    */
    public AttitudeProvider createAttitudeProvider(final BodyShape earth, final Orbit orbit) {

        if (userDefinedLOFTransform) {
            final LOFType type = LOFType.VVLH;

            final List<TimeStampedAngularCoordinates> list = new ArrayList<TimeStampedAngularCoordinates>();

            for (double shift = -10.0; shift < +10.0; shift += 1e-2) {
                list.add(new TimeStampedAngularCoordinates(refDate
                    .shiftedBy(shift), getOffset(earth, orbit, shift),
                                                           Vector3D.ZERO,
                                                           Vector3D.ZERO));
            }

            final TabulatedLofOffset tabulated = new TabulatedLofOffset(orbit.getFrame(), type, list,
                                                                        2, AngularDerivativesFilter.USE_R);

            return tabulated;
        } else {
            return new YawCompensation(orbit.getFrame(), new NadirPointing(orbit.getFrame(), earth));
        }
    }

   /** Generate the orbit.
    */
   public List<TimeStampedPVCoordinates> orbitToPV(final Orbit orbit, final BodyShape earth,
    		                                        final AbsoluteDate minDate, final AbsoluteDate maxDate,
    		                                        final double step) {
    	
        final Propagator propagator = new KeplerianPropagator(orbit);

        propagator.setAttitudeProvider(createAttitudeProvider(earth, orbit));

        propagator.propagate(minDate);
        final List<TimeStampedPVCoordinates> list = new ArrayList<TimeStampedPVCoordinates>();
        propagator.getMultiplexer().add(step,
        		                        currentState ->
                                        list.add(new TimeStampedPVCoordinates(currentState.getDate(),
                                                                              currentState.getPVCoordinates().getPosition(),
                                                                              currentState.getPVCoordinates().getVelocity(),
                                                                              Vector3D.ZERO)));
        propagator.propagate(maxDate);

        return list;
    }

   /** Generate the attitude.
    */
   public List<TimeStampedAngularCoordinates> orbitToQ(final Orbit orbit, final BodyShape earth,
    		                                            final AbsoluteDate minDate, final AbsoluteDate maxDate,
    		                                            final double step) {
    	
        final Propagator propagator = new KeplerianPropagator(orbit);
        propagator.setAttitudeProvider(createAttitudeProvider(earth, orbit));
        propagator.propagate(minDate);
        final List<TimeStampedAngularCoordinates> list = new ArrayList<>();
        propagator.getMultiplexer().add(step,
                                        currentState ->
                                        list.add(new TimeStampedAngularCoordinates(currentState.getDate(),
                                                                                   currentState.getAttitude().getRotation(),
                                                                                   Vector3D.ZERO,
                                                                                   Vector3D.ZERO)));
        propagator.propagate(maxDate);

        return list;
    }
}
