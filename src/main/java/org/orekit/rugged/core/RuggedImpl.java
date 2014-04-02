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
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.orekit.attitudes.Attitude;
import org.orekit.attitudes.AttitudeProvider;
import org.orekit.attitudes.TabulatedProvider;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.data.DataProvidersManager;
import org.orekit.data.DirectoryCrawler;
import org.orekit.errors.OrekitException;
import org.orekit.frames.Frame;
import org.orekit.frames.FramesFactory;
import org.orekit.frames.Transform;
import org.orekit.orbits.CartesianOrbit;
import org.orekit.orbits.Orbit;
import org.orekit.propagation.Propagator;
import org.orekit.rugged.api.GroundPoint;
import org.orekit.rugged.api.LineDatation;
import org.orekit.rugged.api.PixelLOS;
import org.orekit.rugged.api.Rugged;
import org.orekit.rugged.api.RuggedException;
import org.orekit.rugged.api.RuggedMessages;
import org.orekit.rugged.api.SatellitePV;
import org.orekit.rugged.api.SatelliteQ;
import org.orekit.rugged.api.SensorPixel;
import org.orekit.rugged.api.TileUpdater;
import org.orekit.rugged.core.duvenhage.DuvenhageAlgorithm;
import org.orekit.rugged.core.raster.IntersectionAlgorithm;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeScale;
import org.orekit.time.TimeScalesFactory;
import org.orekit.utils.Constants;
import org.orekit.utils.IERSConventions;
import org.orekit.utils.ImmutableTimeStampedCache;
import org.orekit.utils.PVCoordinates;
import org.orekit.utils.PVCoordinatesProvider;

/** Top level Rugged class.
 * @author Luc Maisonobe
 */
public class RuggedImpl implements Rugged {

    /** UTC time scale. */
    private TimeScale utc;

    /** Reference date. */
    private AbsoluteDate referenceDate;

    /** Inertial frame. */
    private Frame frame;

    /** Reference ellipsoid. */
    private ExtendedEllipsoid ellipsoid;

    /** Converter between spacecraft and body. */
    private SpacecraftToObservedBody scToBody;

    /** Sensors. */
    private final Map<String, Sensor> sensors;

    /** DEM intersection algorithm. */
    private IntersectionAlgorithm algorithm;

    /** Simple constructor.
     */
    protected RuggedImpl() {
        sensors = new HashMap<String, Sensor>();
    }

    /** {@inheritDoc} */
    @Override
    public  void setGeneralContext(final File orekitDataDir, final String newReferenceDate,
                                   final Algorithm algorithmID, final Ellipsoid ellipsoidID,
                                   final InertialFrame inertialFrameID,
                                   final BodyRotatingFrame bodyRotatingFrameID,
                                   final List<SatellitePV> positionsVelocities, final int pvInterpolationOrder,
                                   final List<SatelliteQ> quaternions, final int aInterpolationOrder)
        throws RuggedException {
        try {

            // time reference
            utc                = selectTimeScale(orekitDataDir);
            this.referenceDate = new AbsoluteDate(newReferenceDate, utc);

            // space reference
            frame = selectInertialFrame(inertialFrameID);
            ellipsoid = selectEllipsoid(ellipsoidID, selectBodyRotatingFrame(bodyRotatingFrameID));

            // orbit/attitude to body converter
            final PVCoordinatesProvider pvProvider = selectPVCoordinatesProvider(positionsVelocities, pvInterpolationOrder);
            final AttitudeProvider aProvider = selectAttitudeProvider(quaternions, aInterpolationOrder);
            scToBody = new SpacecraftToObservedBody(frame, ellipsoid.getBodyFrame(), pvProvider, aProvider);

            // intersection algorithm
            algorithm = selectAlgorithm(algorithmID);

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
     * @param newReferenceDate reference date from which all other dates are computed
     * @param algorithmID identifier of algorithm to use for Digital Elevation Model intersection
     * @param ellipsoidID identifier of reference ellipsoid
     * @param inertialFrameID identifier of inertial frame
     * @param bodyRotatingFrameID identifier of body rotating frame
     * @param propagator global propagator
     * @exception RuggedException if data needed for some frame cannot be loaded
     */
    public void setGeneralContext(final File orekitDataDir, final AbsoluteDate newReferenceDate,
                                  final Algorithm algorithmID, final Ellipsoid ellipsoidID,
                                  final InertialFrame inertialFrameID,
                                  final BodyRotatingFrame bodyRotatingFrameID,
                                  final Propagator propagator)
        throws RuggedException {
        try {

            // time reference
            utc                = selectTimeScale(orekitDataDir);
            this.referenceDate = newReferenceDate;

            // space reference
            frame = selectInertialFrame(inertialFrameID);
            ellipsoid = selectEllipsoid(ellipsoidID, selectBodyRotatingFrame(bodyRotatingFrameID));

            // orbit/attitude to body converter
            scToBody = new SpacecraftToObservedBody(frame, ellipsoid.getBodyFrame(),
                                                    propagator, propagator.getAttitudeProvider());

            // intersection algorithm
            algorithm = selectAlgorithm(algorithmID);

        } catch (OrekitException oe) {
            throw new RuggedException(oe, oe.getSpecifier(), oe.getParts().clone());
        }
    }

    /** Get the reference date.
     * @return reference date
     */
    public AbsoluteDate getReferenceDate() {
        return referenceDate;
    }

    /** {@inheritDoc} */
    @Override
    public void setUpTilesManagement(final TileUpdater updater, final int maxCachedTiles) {
        algorithm.setUpTilesManagement(updater, maxCachedTiles);
    }

    /** {@inheritDoc} */
    @Override
    public void setLineSensor(final String sensorName, final List<PixelLOS> linesOfSigth, final LineDatation datationModel) {
        final List<Vector3D> positions = new ArrayList<Vector3D>(linesOfSigth.size());
        final List<Vector3D> los       = new ArrayList<Vector3D>(linesOfSigth.size());
        for (final PixelLOS plos : linesOfSigth) {
            positions.add(new Vector3D(plos.getPx(), plos.getPy(), plos.getPz()));
            los.add(new Vector3D(plos.getDx(), plos.getDy(), plos.getDz()));
        }
        final Sensor sensor = new Sensor(sensorName, referenceDate, datationModel, positions, los);
        sensors.put(sensor.getName(), sensor);
    }

    /** Select time scale Orekit data.
     * @param orekitDataDir top directory for Orekit data (if null, Orekit has already been configured)
     * @return utc time scale
     * @exception OrekitException if data needed for some frame cannot be loaded
     */
    private TimeScale selectTimeScale(final File orekitDataDir)
        throws OrekitException {

        if (orekitDataDir != null) {
            // set up Orekit data
            DataProvidersManager.getInstance().addProvider(new DirectoryCrawler(orekitDataDir));
        }

        return TimeScalesFactory.getUTC();

    }

    /** Select inertial frame.
     * @param inertialFrame inertial frame identifier
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
     * @param bodyRotatingFrame body rotating frame identifier
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
     * @param ellipsoidID reference ellipsoid identifier
     * @param bodyFrame body rotating frame
     * @return selected ellipsoid
     * @exception OrekitException if data needed for some frame cannot be loaded
     */
    private ExtendedEllipsoid selectEllipsoid(final Ellipsoid ellipsoidID, final Frame bodyFrame)
        throws OrekitException {

        // set up the ellipsoid
        switch (ellipsoidID) {
        case GRS80 :
            return new ExtendedEllipsoid(6378137.0, 1.0 / 298.257222101, bodyFrame);
        case WGS84 :
            return new ExtendedEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                         Constants.WGS84_EARTH_FLATTENING,
                                         bodyFrame);
        case IERS96 :
            return new ExtendedEllipsoid(6378136.49, 1.0 / 298.25645, bodyFrame);
        case IERS2003 :
            return new ExtendedEllipsoid(6378136.6, 1.0 / 298.25642, bodyFrame);
        default :
            // this should never happen
            throw RuggedException.createInternalError(null);
        }

    }

    /** Select attitude provider.
     * @param quaternions satellite quaternions
     * @param interpolationOrder order to use for interpolation
     * @return selected attitude provider
     * @exception OrekitException if data needed for some frame cannot be loaded
     */
    private AttitudeProvider selectAttitudeProvider(final List<SatelliteQ> quaternions,
                                                    final int interpolationOrder)
        throws OrekitException {

        // set up the attitude provider
        final List<Attitude> attitudes = new ArrayList<Attitude>(quaternions.size());
        for (final SatelliteQ sq : quaternions) {
            final AbsoluteDate date = referenceDate.shiftedBy(sq.getDate());
            final Rotation rotation = new Rotation(sq.getQ0(), sq.getQ1(), sq.getQ2(), sq.getQ3(), true);
            attitudes.add(new Attitude(date, frame, rotation, Vector3D.ZERO));
        }
        return new TabulatedProvider(attitudes, interpolationOrder, false);

    }

    /** Select position/velocity provider.
     * @param positionsVelocities satellite position and velocity
     * @param interpolationOrder order to use for interpolation
     * @return selected position/velocity provider
     * @exception OrekitException if data needed for some frame cannot be loaded
     */
    private PVCoordinatesProvider selectPVCoordinatesProvider(final List<SatellitePV> positionsVelocities,
                                                              final int interpolationOrder)
        throws OrekitException {

        // set up the ephemeris
        final List<Orbit> orbits = new ArrayList<Orbit>(positionsVelocities.size());
        for (final SatellitePV pv : positionsVelocities) {
            final AbsoluteDate date    = referenceDate.shiftedBy(pv.getDate());
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
            public PVCoordinates getPVCoordinates(final AbsoluteDate date, final Frame f)
                throws OrekitException {
                final List<Orbit> sample = cache.getNeighbors(date);
                final Orbit interpolated = sample.get(0).interpolate(date, sample);
                return interpolated.getPVCoordinates(date, f);
            }

        };

    }

    /** Select DEM intersection algorithm.
     * @param algorithmID intersection algorithm identifier
     * @return selected algorithm
     */
    private IntersectionAlgorithm selectAlgorithm(final Algorithm algorithmID) {

        // set up the algorithm
        switch (algorithmID) {
        case DUVENHAGE :
            return new DuvenhageAlgorithm();
        case BASIC_SLOW_EXHAUSTIVE_SCAN_FOR_TESTS_ONLY :
            return new BasicScanAlgorithm();
        case IGNORE_DEM_USE_ELLIPSOID :
            return new IgnoreDEMAlgorithm();
        default :
            // this should never happen
            throw RuggedException.createInternalError(null);
        }

    }

    /** {@inheritDoc} */
    @Override
    public GroundPoint[] directLocalization(final String sensorName, final double lineNumber)
        throws RuggedException {
        try {

            checkContext();

            // select the sensor
            final Sensor sensor = getSensor(sensorName);

            // compute the approximate transform between spacecraft and observed body
            final AbsoluteDate date        = sensor.getDate(lineNumber);
            final Transform    scToInert   = scToBody.getScToInertial(date);
            final Transform    inertToBody = scToBody.getInertialToBody(date);
            final Transform    approximate = new Transform(date, scToInert, inertToBody);

            // compute localization of each pixel
            final GroundPoint[] gp = new GroundPoint[sensor.getNbPixels()];
            for (int i = 0; i < gp.length; ++i) {

                // fix light travel time
                final Vector3D  sP     = approximate.transformPosition(sensor.getPosition(i));
                final Vector3D  sL     = approximate.transformVector(sensor.getLos(i));
                final Vector3D  eP     = ellipsoid.transform(ellipsoid.pointOnGround(sP, sL));
                final double    deltaT = eP.distance(sP) / Constants.SPEED_OF_LIGHT;
                final Transform fixed  = new Transform(date, scToInert, inertToBody.shiftedBy(-deltaT));

                final GeodeticPoint geodetic =
                        algorithm.intersection(ellipsoid,
                                               fixed.transformPosition(sensor.getPosition(i)),
                                               fixed.transformVector(sensor.getLos(i)));
                gp[i] = new GroundPoint(geodetic.getLatitude(),
                                        geodetic.getLongitude(),
                                        geodetic.getAltitude());
            }

            return gp;

        } catch (OrekitException oe) {
            throw new RuggedException(oe, oe.getSpecifier(), oe.getParts());
        }
    }

    /** {@inheritDoc} */
    @Override
    public SensorPixel inverseLocalization(final String sensorName, final GroundPoint groundPoint)
        throws RuggedException {

        checkContext();
        final Sensor sensor = getSensor(sensorName);

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

    /** Get a sensor.
     * @param sensorName sensor name
     * @return selected sensor
     * @exception RuggedException if sensor is not known
     */
    private Sensor getSensor(final String sensorName) throws RuggedException {
        final Sensor sensor = sensors.get(sensorName);
        if (sensor == null) {
            throw new RuggedException(RuggedMessages.UNKNOWN_SENSOR, sensorName);
        }
        return sensor;
    }

}
