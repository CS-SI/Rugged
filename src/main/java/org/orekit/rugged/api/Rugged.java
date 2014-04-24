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
package org.orekit.rugged.api;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.util.Pair;
import org.orekit.attitudes.Attitude;
import org.orekit.attitudes.AttitudeProvider;
import org.orekit.attitudes.TabulatedProvider;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.bodies.OneAxisEllipsoid;
import org.orekit.errors.OrekitException;
import org.orekit.frames.Frame;
import org.orekit.frames.FramesFactory;
import org.orekit.frames.Transform;
import org.orekit.orbits.CartesianOrbit;
import org.orekit.orbits.Orbit;
import org.orekit.propagation.Propagator;
import org.orekit.rugged.core.BasicScanAlgorithm;
import org.orekit.rugged.core.ExtendedEllipsoid;
import org.orekit.rugged.core.IgnoreDEMAlgorithm;
import org.orekit.rugged.core.Sensor;
import org.orekit.rugged.core.SpacecraftToObservedBody;
import org.orekit.rugged.core.duvenhage.DuvenhageAlgorithm;
import org.orekit.rugged.core.raster.IntersectionAlgorithm;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.Constants;
import org.orekit.utils.IERSConventions;
import org.orekit.utils.ImmutableTimeStampedCache;
import org.orekit.utils.PVCoordinates;
import org.orekit.utils.PVCoordinatesProvider;

/** Main class of Rugged library API.
 * @author Luc Maisonobe
 */
public class Rugged {

    /** Reference date. */
    private final AbsoluteDate referenceDate;

    /** Inertial frame. */
    private final Frame inertialFrame;

    /** Reference ellipsoid. */
    private final ExtendedEllipsoid ellipsoid;

    /** Converter between spacecraft and body. */
    private final SpacecraftToObservedBody scToBody;

    /** Sensors. */
    private final Map<String, Sensor> sensors;

    /** DEM intersection algorithm. */
    private final IntersectionAlgorithm algorithm;

    /** Flag for light time correction. */
    private boolean lightTimeCorrection;

    /** Flag for aberration of light correction. */
    private boolean aberrationOfLightCorrection;

    /** Build a configured instance.
     * <p>
     * By default, the instance performs both light time correction (which refers
     * to ground point motion with respect to inertial frame) and aberration of
     * light correction (which refers to spacecraft proper velocity). Explicit calls
     * to {@link #setLightTimeCorrection(boolean) setLightTimeCorrection}
     * and {@link #setAberrationOfLightCorrection(boolean) setAberrationOfLightCorrection}
     * can be made after construction if these phenomena should not be corrected.
     * </p>
     * @param referenceDate reference date from which all other dates are computed
     * @param updater updater used to load Digital Elevation Model tiles
     * @param maxCachedTiles maximum number of tiles stored in the cache
     * @param algorithmID identifier of algorithm to use for Digital Elevation Model intersection
     * @param ellipsoidID identifier of reference ellipsoid
     * @param inertialFrameID identifier of inertial frame
     * @param bodyRotatingFrameID identifier of body rotating frame
     * @param positionsVelocities satellite position and velocity
     * @param pvInterpolationOrder order to use for position/velocity interpolation
     * @param quaternions satellite quaternions
     * @param aInterpolationOrder order to use for attitude interpolation
     * @exception RuggedException if data needed for some frame cannot be loaded
     */
    public Rugged(final AbsoluteDate referenceDate,
                  final TileUpdater updater, final int maxCachedTiles,
                  final AlgorithmId algorithmID, final EllipsoidId ellipsoidID,
                  final InertialFrameId inertialFrameID, final BodyRotatingFrameId bodyRotatingFrameID,
                  final List<Pair<AbsoluteDate, PVCoordinates>> positionsVelocities,
                  final int pvInterpolationOrder, final List<Pair<AbsoluteDate, Rotation>> quaternions,
                  final int aInterpolationOrder)
        throws RuggedException {
        this(referenceDate, updater, maxCachedTiles, algorithmID,
             selectEllipsoid(ellipsoidID, selectBodyRotatingFrame(bodyRotatingFrameID)),
             selectInertialFrame(inertialFrameID),
             positionsVelocities, pvInterpolationOrder, quaternions, aInterpolationOrder);
    }

    /** Build a configured instance.
     * <p>
     * By default, the instance performs both light time correction (which refers
     * to ground point motion with respect to inertial frame) and aberration of
     * light correction (which refers to spacecraft proper velocity). Explicit calls
     * to {@link #setLightTimeCorrection(boolean) setLightTimeCorrection}
     * and {@link #setAberrationOfLightCorrection(boolean) setAberrationOfLightCorrection}
     * can be made after construction if these phenomena should not be corrected.
     * </p>
     * @param referenceDate reference date from which all other dates are computed
     * @param updater updater used to load Digital Elevation Model tiles
     * @param maxCachedTiles maximum number of tiles stored in the cache
     * @param algorithmID identifier of algorithm to use for Digital Elevation Model intersection
     * @param ellipsoidID identifier of reference ellipsoid
     * @param inertialFrameID identifier of inertial frame
     * @param bodyRotatingFrameID identifier of body rotating frame
     * @param positionsVelocities satellite position and velocity
     * @param pvInterpolationOrder order to use for position/velocity interpolation
     * @param quaternions satellite quaternions
     * @param aInterpolationOrder order to use for attitude interpolation
     * @exception RuggedException if data needed for some frame cannot be loaded
     */
    public Rugged(final AbsoluteDate referenceDate,
                  final TileUpdater updater, final int maxCachedTiles,
                  final AlgorithmId algorithmID, final OneAxisEllipsoid ellipsoid, final Frame inertialFrame,
                  final List<Pair<AbsoluteDate, PVCoordinates>> positionsVelocities,
                  final int pvInterpolationOrder, final List<Pair<AbsoluteDate, Rotation>> quaternions,
                  final int aInterpolationOrder)
        throws RuggedException {
        this(referenceDate, updater, maxCachedTiles, algorithmID,
             extend(ellipsoid), inertialFrame,
             selectPVCoordinatesProvider(positionsVelocities, pvInterpolationOrder, inertialFrame),
             selectAttitudeProvider(quaternions, aInterpolationOrder, inertialFrame));
    }

    /** Build a configured instance.
     * <p>
     * By default, the instance performs both light time correction (which refers
     * to ground point motion with respect to inertial frame) and aberration of
     * light correction (which refers to spacecraft proper velocity). Explicit calls
     * to {@link #setLightTimeCorrection(boolean) setLightTimeCorrection}
     * and {@link #setAberrationOfLightCorrection(boolean) setAberrationOfLightCorrection}
     * can be made after construction if these phenomena should not be corrected.
     * </p>
     * @param referenceDate reference date from which all other dates are computed
     * @param updater updater used to load Digital Elevation Model tiles
     * @param maxCachedTiles maximum number of tiles stored in the cache
     * @param algorithmID identifier of algorithm to use for Digital Elevation Model intersection
     * @param ellipsoidID identifier of reference ellipsoid
     * @param inertialFrameID identifier of inertial frame
     * @param bodyRotatingFrameID identifier of body rotating frame
     * @param propagator global propagator
     * @exception RuggedException if data needed for some frame cannot be loaded
     */
    public Rugged(final AbsoluteDate referenceDate,
                  final TileUpdater updater, final int maxCachedTiles,
                  final AlgorithmId algorithmID, final EllipsoidId ellipsoidID,
                  final InertialFrameId inertialFrameID, final BodyRotatingFrameId bodyRotatingFrameID,
                  final Propagator propagator)
        throws RuggedException {
        this(referenceDate, updater, maxCachedTiles, algorithmID,
             selectEllipsoid(ellipsoidID, selectBodyRotatingFrame(bodyRotatingFrameID)),
             selectInertialFrame(inertialFrameID), propagator);
    }

    /** Build a configured instance.
     * <p>
     * By default, the instance performs both light time correction (which refers
     * to ground point motion with respect to inertial frame) and aberration of
     * light correction (which refers to spacecraft proper velocity). Explicit calls
     * to {@link #setLightTimeCorrection(boolean) setLightTimeCorrection}
     * and {@link #setAberrationOfLightCorrection(boolean) setAberrationOfLightCorrection}
     * can be made after construction if these phenomena should not be corrected.
     * </p>
     * @param referenceDate reference date from which all other dates are computed
     * @param updater updater used to load Digital Elevation Model tiles
     * @param maxCachedTiles maximum number of tiles stored in the cache
     * @param algorithmID identifier of algorithm to use for Digital Elevation Model intersection
     * @param ellipsoid f reference ellipsoid
     * @param inertialFrame inertial frame
     * @param propagator global propagator
     * @exception RuggedException if data needed for some frame cannot be loaded
     */
    public Rugged(final AbsoluteDate referenceDate,
                  final TileUpdater updater, final int maxCachedTiles,
                  final AlgorithmId algorithmID, final OneAxisEllipsoid ellipsoid,
                  final Frame inertialFrame, final Propagator propagator)
        throws RuggedException {
        this(referenceDate, updater, maxCachedTiles, algorithmID,
             extend(ellipsoid), inertialFrame,
             propagator, propagator.getAttitudeProvider());
    }

    /** Low level private constructor.
     * @param referenceDate reference date from which all other dates are computed
     * @param updater updater used to load Digital Elevation Model tiles
     * @param maxCachedTiles maximum number of tiles stored in the cache
     * @param algorithmID identifier of algorithm to use for Digital Elevation Model intersection
     * @param ellipsoid f reference ellipsoid
     * @param inertialFrame inertial frame
     * @param pvProvider orbit propagator/interpolator
     * @param aProvider attitude propagator/interpolator
     * @exception RuggedException if data needed for some frame cannot be loaded
     */
    private Rugged(final AbsoluteDate referenceDate,
                  final TileUpdater updater, final int maxCachedTiles, final AlgorithmId algorithmID,
                  final ExtendedEllipsoid ellipsoid, final Frame inertialFrame,
                  final PVCoordinatesProvider pvProvider, final AttitudeProvider aProvider)
        throws RuggedException {

        // time reference
        this.referenceDate = referenceDate;

        // space reference
        this.inertialFrame = inertialFrame;
        this.ellipsoid     = ellipsoid;

        // orbit/attitude to body converter
        this.scToBody = new SpacecraftToObservedBody(inertialFrame, ellipsoid.getBodyFrame(), pvProvider, aProvider);

        // intersection algorithm
        this.algorithm = selectAlgorithm(algorithmID, updater, maxCachedTiles);

        this.sensors = new HashMap<String, Sensor>();
        setLightTimeCorrection(true);
        setAberrationOfLightCorrection(true);

    }

    /** Get the reference date.
     * @return reference date
     */
    public AbsoluteDate getReferenceDate() {
        return referenceDate;
    }

    /** Set flag for light time correction.
     * <p>
     * This methods set the flag for compensating or not light time between
     * ground and spacecraft. Compensating this delay improves localization
     * accuracy and is enabled by default. Not compensating it is mainly useful
     * for validation purposes against system that do not compensate it.
     * </p>
     * @param lightTimeCorrection if true, the light travel time between ground
     * and spacecraft is compensated for more accurate localization
     * @see #isLightTimeCorrected()
     */
    public void setLightTimeCorrection(final boolean lightTimeCorrection) {
        this.lightTimeCorrection = lightTimeCorrection;
    }

    /** Get flag for light time correction.
     * @return true if the light time between ground and spacecraft is
     * compensated for more accurate localization
     * @see #setLightTimeCorrection(boolean)
     */
    public boolean isLightTimeCorrected() {
        return lightTimeCorrection;
    }

    /** Set flag for aberration of light correction.
     * <p>
     * This methods set the flag for compensating or not aberration of light,
     * which is velocity composition between light and spacecraft when the
     * light from ground points reaches the sensor.
     * Compensating this velocity composition improves localization
     * accuracy and is enabled by default. Not compensating it is useful
     * in two cases: for validation purposes against system that do not
     * compensate it or when the pixels line of sight already include the
     * correction.
     * </p>
     * @param aberrationOfLightCorrection if true, the aberration of light
     * is corrected for more accurate localization
     * @see #isAberrationOfLightCorrected()
     * @see #setLineSensor(String, List, LineDatation)
     */
    public void setAberrationOfLightCorrection(final boolean aberrationOfLightCorrection) {
        this.aberrationOfLightCorrection = aberrationOfLightCorrection;
    }

    /** Get flag for aberration of light correction.
     * @return true if the aberration of light time is corrected
     * for more accurate localization
     * @see #setAberrationOfLightCorrection(boolean)
     */
    public boolean isAberrationOfLightCorrected() {
        return aberrationOfLightCorrection;
    }

    /** Set up line sensor model.
     * @param sensorName name of the line sensor.
     * @param linesOfSigth lines of sight for each pixels
     * @param datationModel model to use for dating sensor lines
     */
    public void setLineSensor(final String sensorName, final List<PixelLOS> linesOfSigth, final LineDatation datationModel) {
        final List<Vector3D> positions = new ArrayList<Vector3D>(linesOfSigth.size());
        final List<Vector3D> los       = new ArrayList<Vector3D>(linesOfSigth.size());
        for (final PixelLOS plos : linesOfSigth) {
            positions.add(new Vector3D(plos.getPx(), plos.getPy(), plos.getPz()));
            los.add(new Vector3D(plos.getDx(), plos.getDy(), plos.getDz()).normalize());
        }
        final Sensor sensor = new Sensor(sensorName, referenceDate, datationModel, positions, los);
        sensors.put(sensor.getName(), sensor);
    }

    /** Select inertial frame.
     * @param inertialFrameId inertial frame identifier
     * @return inertial frame
     * @exception RuggedException if data needed for some frame cannot be loaded
     */
    private static Frame selectInertialFrame(final InertialFrameId inertialFrameId)
        throws RuggedException {

        try {
            // set up the inertial frame
            switch (inertialFrameId) {
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
        } catch (OrekitException oe) {
            throw new RuggedException(oe, oe.getSpecifier(), oe.getParts().clone());
        }

    }

    /** Select body rotating frame.
     * @param bodyRotatingFrame body rotating frame identifier
     * @return body rotating frame
     * @exception RuggedException if data needed for some frame cannot be loaded
     */
    private static Frame selectBodyRotatingFrame(final BodyRotatingFrameId bodyRotatingFrame)
        throws RuggedException {

        try {
            // set up the rotating frame
            switch (bodyRotatingFrame) {
            case ITRF :
                return FramesFactory.getITRF(IERSConventions.IERS_2010, true);
            case ITRF_EQUINOX :
                return FramesFactory.getITRFEquinox(IERSConventions.IERS_1996, true);
            case GTOD :
                return FramesFactory.getGTOD(IERSConventions.IERS_1996, true);
            default :
                // this should never happen
                throw RuggedException.createInternalError(null);
            }
        } catch (OrekitException oe) {
            throw new RuggedException(oe, oe.getSpecifier(), oe.getParts().clone());
        }

    }

    /** Select ellipsoid.
     * @param ellipsoidID reference ellipsoid identifier
     * @param bodyFrame body rotating frame
     * @return selected ellipsoid
     */
    private static OneAxisEllipsoid selectEllipsoid(final EllipsoidId ellipsoidID, final Frame bodyFrame) {

        // set up the ellipsoid
        switch (ellipsoidID) {
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

    /** Convert a {@link OneAxisEllipsoid} into a {@link ExtendedEllipsoid}
     * @param ellipsoid ellpsoid to extend
     * @return extended ellipsoid
     */
    private static ExtendedEllipsoid extend(final OneAxisEllipsoid ellipsoid) {
        return new ExtendedEllipsoid(ellipsoid.getEquatorialRadius(),
                                     ellipsoid.getFlattening(),
                                     ellipsoid.getBodyFrame());
    }

    /** Select attitude provider.
     * @param quaternions satellite quaternions
     * @param interpolationOrder order to use for interpolation
     * @param inertialFrame inertial frame where position and velocity are defined
     * @return selected attitude provider
     */
    private static AttitudeProvider selectAttitudeProvider(final List<Pair<AbsoluteDate, Rotation>> quaternions,
                                                           final int interpolationOrder, final Frame inertialFrame) {

        // set up the attitude provider
        final List<Attitude> attitudes = new ArrayList<Attitude>(quaternions.size());
        for (final Pair<AbsoluteDate, Rotation> q : quaternions) {
            attitudes.add(new Attitude(q.getFirst(), inertialFrame, q.getSecond(), Vector3D.ZERO));
        }
        return new TabulatedProvider(attitudes, interpolationOrder, false);

    }

    /** Select position/velocity provider.
     * @param positionsVelocities satellite position and velocity
     * @param interpolationOrder order to use for interpolation
     * @param inertialFrame inertial frame where position and velocity are defined
     * @return selected position/velocity provider
     */
    private static PVCoordinatesProvider selectPVCoordinatesProvider(final List<Pair<AbsoluteDate, PVCoordinates>> positionsVelocities,
                                                                     final int interpolationOrder, final Frame inertialFrame) {

        // set up the ephemeris
        final List<Orbit> orbits = new ArrayList<Orbit>(positionsVelocities.size());
        for (final Pair<AbsoluteDate, PVCoordinates> pv : positionsVelocities) {
            final CartesianOrbit orbit = new CartesianOrbit(pv.getSecond(), inertialFrame, pv.getFirst(), Constants.EIGEN5C_EARTH_MU);
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
     * @param updater updater used to load Digital Elevation Model tiles
     * @param maxCachedTiles maximum number of tiles stored in the cache
     * @return selected algorithm
     */
    private IntersectionAlgorithm selectAlgorithm(final AlgorithmId algorithmID,
                                                  final TileUpdater updater, final int maxCachedTiles) {

        // set up the algorithm
        switch (algorithmID) {
        case DUVENHAGE :
            return new DuvenhageAlgorithm(updater, maxCachedTiles);
        case BASIC_SLOW_EXHAUSTIVE_SCAN_FOR_TESTS_ONLY :
            return new BasicScanAlgorithm(updater, maxCachedTiles);
        case IGNORE_DEM_USE_ELLIPSOID :
            return new IgnoreDEMAlgorithm();
        default :
            // this should never happen
            throw RuggedException.createInternalError(null);
        }

    }

    /** Direct localization of a sensor line.
     * @param sensorName name of the line sensor
     * @param lineNumber number of the line to localize on ground
     * @return ground position of all pixels of the specified sensor line
     * @exception RuggedException if line cannot be localized, or sensor is unknown
     */
    public GeodeticPoint[] directLocalization(final String sensorName, final double lineNumber)
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
            final GeodeticPoint[] gp = new GeodeticPoint[sensor.getNbPixels()];
            for (int i = 0; i < gp.length; ++i) {

                final Vector3D pInert = scToInert.transformPosition(sensor.getPosition(i));
                final Vector3D lInert;
                if (aberrationOfLightCorrection) {
                    // apply aberration of light correction
                    // as the spacecraft velocity is small with respect to speed of light,
                    // we use classical velocity addition and not relativistic velocity addition
                    final Vector3D spacecraftVelocity = scToInert.transformPVCoordinates(PVCoordinates.ZERO).getVelocity();
                    lInert = new Vector3D(Constants.SPEED_OF_LIGHT, scToInert.transformVector(sensor.getLos(i)),
                                          1.0, spacecraftVelocity).normalize();
                } else {
                    lInert = scToInert.transformVector(sensor.getLos(i));
                    // don't apply aberration of light correction
                }

                final Vector3D pBody;
                final Vector3D lBody;
                if (lightTimeCorrection) {
                    // apply light time correction
                    final Vector3D sP     = approximate.transformPosition(sensor.getPosition(i));
                    final Vector3D sL     = approximate.transformVector(sensor.getLos(i));
                    final Vector3D eP     = ellipsoid.transform(ellipsoid.pointOnGround(sP, sL));
                    final double   deltaT = eP.distance(sP) / Constants.SPEED_OF_LIGHT;
                    final Transform shifted = inertToBody.shiftedBy(-deltaT);
                    pBody = shifted.transformPosition(pInert);
                    lBody = shifted.transformVector(lInert);
                } else {
                    // don't apply light time correction
                    pBody = inertToBody.transformPosition(pInert);
                    lBody = inertToBody.transformVector(lInert);
                }

                // compute DEM intersection
                gp[i] = algorithm.intersection(ellipsoid, pBody, lBody);

            }

            return gp;

        } catch (OrekitException oe) {
            throw new RuggedException(oe, oe.getSpecifier(), oe.getParts());
        }
    }

    /** Inverse localization of a ground point.
     * @param sensorName name of the line  sensor
     * @param groundPoint ground point to localize
     * @return sensor pixel seeing ground point
     * @exception RuggedException if line cannot be localized, or sensor is unknown
     */
    public SensorPixel inverseLocalization(final String sensorName, final GeodeticPoint groundPoint)
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
        if (inertialFrame == null) {
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
