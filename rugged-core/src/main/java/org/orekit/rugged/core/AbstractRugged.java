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
package org.orekit.rugged.core;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.orekit.attitudes.Attitude;
import org.orekit.attitudes.AttitudeProvider;
import org.orekit.attitudes.TabulatedProvider;
import org.orekit.bodies.BodyShape;
import org.orekit.bodies.OneAxisEllipsoid;
import org.orekit.data.DataProvidersManager;
import org.orekit.data.DirectoryCrawler;
import org.orekit.errors.OrekitException;
import org.orekit.frames.Frame;
import org.orekit.frames.FramesFactory;
import org.orekit.orbits.CartesianOrbit;
import org.orekit.orbits.Orbit;
import org.orekit.propagation.Propagator;
import org.orekit.rugged.api.GroundPoint;
import org.orekit.rugged.api.Rugged;
import org.orekit.rugged.api.RuggedException;
import org.orekit.rugged.api.RuggedMessages;
import org.orekit.rugged.api.SatellitePV;
import org.orekit.rugged.api.SatelliteQ;
import org.orekit.rugged.api.SensorPixel;
import org.orekit.rugged.api.Rugged.BodyRotatingFrame;
import org.orekit.rugged.api.Rugged.Ellipsoid;
import org.orekit.rugged.api.Rugged.InertialFrame;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeScale;
import org.orekit.time.TimeScalesFactory;
import org.orekit.utils.Constants;
import org.orekit.utils.IERSConventions;
import org.orekit.utils.ImmutableTimeStampedCache;
import org.orekit.utils.PVCoordinates;
import org.orekit.utils.PVCoordinatesProvider;

/** Boilerplate part for direct and inverse localization.
 * @author Luc Maisonobe
 */
public abstract class AbstractRugged implements Rugged {

    /** UTC time scale. */
    private TimeScale utc;

    /** Inertial frame. */
    private Frame frame;

    /** Reference ellipsoid. */
    private BodyShape shape;

    /** Orbit propagator/interpolator. */
    private PVCoordinatesProvider pvProvider;

    /** Attitude propagator/interpolator. */
    private AttitudeProvider aProvider;

    /** Simple constructor.
     */
    protected AbstractRugged() {
    }

    /** {@inheritDoc} */
    @Override
    public  void setGeneralContext(final File orekitDataDir,
                                   final Ellipsoid ellipsoid,
                                   final InertialFrame inertialFrame,
                                   final BodyRotatingFrame bodyRotatingFrame,
                                   final List<SatellitePV> positionsVelocities, final int pvInterpolationOrder,
                                   final List<SatelliteQ> quaternions, final int aInterpolationOrder)
        throws RuggedException {
        try {
            utc        = selectTimeScale(orekitDataDir);
            frame      = selectInertialFrame(inertialFrame);
            shape      = selectEllipsoid(ellipsoid, selectBodyRotatingFrame(bodyRotatingFrame));
            pvProvider = selectPVCoordinatesProvider(positionsVelocities, pvInterpolationOrder);
            aProvider  = selectAttitudeProvider(quaternions, aInterpolationOrder);
        } catch (OrekitException oe) {
            throw new RuggedException(oe, oe.getSpecifier(), oe.getParts().clone());
        }
    }

    /** Set up general context.
     * <p>
     * This method is the first one that must be called, otherwise the
     * other methods will fail due to uninitialized context.
     * </p>
     * @param orekitDataDir top directory for Orekit data
     * @param ellipsoid reference ellipsoid
     * @param inertialFrameName inertial frame
     * @param bodyRotatingFrame body rotating frame
     * @param propagator global propagator
     * @exception RuggedException if data needed for some frame cannot be loaded
     */
    public void setGeneralContext(final File orekitDataDir, final Ellipsoid ellipsoid,
                                  final InertialFrame inertialFrame,
                                  final BodyRotatingFrame bodyRotatingFrame,
                                  final Propagator propagator)
        throws RuggedException {
        try {
            utc        = selectTimeScale(orekitDataDir);
            frame      = selectInertialFrame(inertialFrame);
            shape      = selectEllipsoid(ellipsoid, selectBodyRotatingFrame(bodyRotatingFrame));
            pvProvider = propagator;
            aProvider  = propagator.getAttitudeProvider();
        } catch (OrekitException oe) {
            throw new RuggedException(oe, oe.getSpecifier(), oe.getParts().clone());
        }
    }

    /** Select time scale Orekit data.
     * @param orekitDataDir top directory for Orekit data
     * @return utc time scale
     * @exception OrekitException if data needed for some frame cannot be loaded
     */
    private TimeScale selectTimeScale(final File orekitDataDir)
        throws OrekitException {

        // set up Orekit data
        DataProvidersManager.getInstance().addProvider(new DirectoryCrawler(orekitDataDir));
        return TimeScalesFactory.getUTC();

    }

    /** Select inertial frame.
     * @param inertialFrameName inertial frame
     * @return inertial frame
     * @exception OrekitException if data needed for some frame cannot be loaded
     */
    private Frame selectInertialFrame(final InertialFrame inertialFrame)
        throws OrekitException {

        // set up the inertial frame
        switch (inertialFrame) {
            case GCRF :
                return FramesFactory.getGCRF();
            case EME2000 :
                return FramesFactory.getEME2000();
            case MOD :
                return FramesFactory.getMOD(IERSConventions.IERS_1996);
            case TOD :
                return FramesFactory.getTOD(IERSConventions.IERS_1996, true);
            case VEIS1950 :
                return FramesFactory.getVeis1950();
            default :
                // this should never happen
                throw RuggedException.createInternalError(null);
        }

    }

    /** Select body rotating frame.
     * @param bodyRotatingFrame body rotating frame
     * @return body rotating frame
     * @exception OrekitException if data needed for some frame cannot be loaded
     */
    private Frame selectBodyRotatingFrame(final BodyRotatingFrame bodyRotatingFrame)
        throws OrekitException {
        
        // set up the rotating frame
        switch (bodyRotatingFrame) {
            case ITRF :
                return FramesFactory.getITRF(IERSConventions.IERS_2010, true);
            case GTOD :
                return FramesFactory.getGTOD(IERSConventions.IERS_1996, true);
            default :
                // this should never happen
                throw RuggedException.createInternalError(null);
        }

    }

    /** Select ellipsoid.
     * @param ellipsoid reference ellipsoid
     * @param bodyFrame body rotating frame
     * @exception OrekitException if data needed for some frame cannot be loaded
     */
    private BodyShape selectEllipsoid(final Ellipsoid ellipsoid, final Frame bodyFrame)
        throws OrekitException {
        
        // set up the ellipsoid
        switch (ellipsoid) {
            case GRS80 :
                return new OneAxisEllipsoid(6378137.0, 1.0 / 298.257222101, bodyFrame);
            case WGS84 :
                return new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                             Constants.WGS84_EARTH_FLATTENING,
                                             bodyFrame);
            case IERS96 :
                return new OneAxisEllipsoid(6378136.49, 1.0 / 298.25645, bodyFrame);
            case IERS2003 :
                return new OneAxisEllipsoid(6378136.6, 1.0 / 298.25642, bodyFrame);
            default :
                // this should never happen
                throw RuggedException.createInternalError(null);
        }

    }

    /** Select attitude provider.
     * @param quaternions satellite quaternions
     * @param interpolationOrder order to use for interpolation
     * @exception OrekitException if data needed for some frame cannot be loaded
     */
    private AttitudeProvider selectAttitudeProvider(final List<SatelliteQ> quaternions,
                                                    final int interpolationOrder)
        throws OrekitException {

        // set up the attitude provider
        final List<Attitude> attitudes = new ArrayList<Attitude>(quaternions.size());
        for (final SatelliteQ sq : quaternions) {
            final AbsoluteDate date = new AbsoluteDate(sq.getDate(), utc);
            final Rotation rotation = new Rotation(sq.getQ0(), sq.getQ1(), sq.getQ2(), sq.getQ3(), true);
            attitudes.add(new Attitude(date, frame, rotation, Vector3D.ZERO));
        }
        return new TabulatedProvider(attitudes, interpolationOrder, false);

    }

    /** Select position/velocity provider.
     * @param positionsVelocities satellite position and velocity
     * @param interpolationOrder order to use for interpolation
     * @exception OrekitException if data needed for some frame cannot be loaded
     */
    private PVCoordinatesProvider selectPVCoordinatesProvider(final List<SatellitePV> positionsVelocities,
                                                              final int interpolationOrder)
        throws OrekitException {

        // set up the ephemeris
        final List<Orbit> orbits = new ArrayList<Orbit>(positionsVelocities.size());
        for (final SatellitePV pv : positionsVelocities) {
            final AbsoluteDate date    = new AbsoluteDate(pv.getDate(), utc);
            final Vector3D position    = new Vector3D(pv.getPx(), pv.getPy(), pv.getPz());
            final Vector3D velocity    = new Vector3D(pv.getVx(), pv.getVy(), pv.getVz());
            final CartesianOrbit orbit = new CartesianOrbit(new PVCoordinates(position, velocity),
                                                            frame, date, Constants.EIGEN5C_EARTH_MU);
            orbits.add(orbit);
        }

        final ImmutableTimeStampedCache<Orbit> cache =
                new ImmutableTimeStampedCache<Orbit>(interpolationOrder, orbits);
        return new PVCoordinatesProvider() {

            /** {@inhritDoc} */
            @Override
            public PVCoordinates getPVCoordinates(final AbsoluteDate date, final Frame frame)
                throws OrekitException {
                final List<Orbit> sample = cache.getNeighbors(date);
                final Orbit interpolated = sample.get(0).interpolate(date, sample);
                return interpolated.getPVCoordinates(date, frame);
            }

        };

    }

    /** {@inheritDoc} */
    @Override
    public GroundPoint[] directLocalization(String sensorName, int lineNumber)
        throws RuggedException {

        checkContext();

        // TODO: implement direct localization
        throw RuggedException.createInternalError(null);

    }

    /** {@inheritDoc} */
    @Override
    public SensorPixel inverseLocalization(String sensorName, GroundPoint groundPoint)
        throws RuggedException {

        checkContext();

        // TODO: implement direct localization
        throw RuggedException.createInternalError(null);

    }

    /** Check if context has been initialized.
     * @exception RuggedException if context has not been initialized
     */
    private void checkContext() throws RuggedException {
        if (frame == null) {
            throw new RuggedException(RuggedMessages.UNINITIALIZED_CONTEXT);
        }
    }

}
