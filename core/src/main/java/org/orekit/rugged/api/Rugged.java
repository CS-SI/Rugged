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

import org.apache.commons.math3.analysis.differentiation.DerivativeStructure;
import org.apache.commons.math3.geometry.euclidean.threed.FieldVector3D;
import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.util.FastMath;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.bodies.OneAxisEllipsoid;
import org.orekit.errors.OrekitException;
import org.orekit.errors.PropagationException;
import org.orekit.frames.Frame;
import org.orekit.frames.FramesFactory;
import org.orekit.frames.Transform;
import org.orekit.propagation.Propagator;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.sampling.OrekitFixedStepHandler;
import org.orekit.rugged.intersection.BasicScanAlgorithm;
import org.orekit.rugged.intersection.IgnoreDEMAlgorithm;
import org.orekit.rugged.intersection.IntersectionAlgorithm;
import org.orekit.rugged.intersection.duvenhage.DuvenhageAlgorithm;
import org.orekit.rugged.raster.TileUpdater;
import org.orekit.rugged.utils.ExtendedEllipsoid;
import org.orekit.rugged.utils.SpacecraftToObservedBody;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.Constants;
import org.orekit.utils.IERSConventions;
import org.orekit.utils.PVCoordinates;
import org.orekit.utils.TimeStampedAngularCoordinates;
import org.orekit.utils.TimeStampedPVCoordinates;

/** Main class of Rugged library API.
 * @author Luc Maisonobe
 */
public class Rugged {

    /** Accuracy to use in the first stage of inverse localization.
     * <p>
     * This accuracy is only used to locate the point within one
     * pixel, hence there is no point in choosing a too small value here.
     * </p>
     */
    private static final double COARSE_INVERSE_LOCALIZATION_ACCURACY = 0.01;

    /** Maximum number of evaluations. */
    private static final int MAX_EVAL = 50;

    /** Time step for frames transforms interpolations. */
    private static final double FRAMES_TRANSFORMS_INTERPOLATION_STEP = 0.01;

    /** Reference ellipsoid. */
    private final ExtendedEllipsoid ellipsoid;

    /** Converter between spacecraft and body. */
    private final SpacecraftToObservedBody scToBody;

    /** Sensors. */
    private final Map<String, LineSensor> sensors;

    /** Mean plane crossing finders. */
    private final Map<String, SensorMeanPlaneCrossing> finders;

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
     * @param updater updater used to load Digital Elevation Model tiles
     * @param maxCachedTiles maximum number of tiles stored in the cache
     * @param algorithmID identifier of algorithm to use for Digital Elevation Model intersection
     * @param ellipsoidID identifier of reference ellipsoid
     * @param inertialFrameID identifier of inertial frame
     * @param bodyRotatingFrameID identifier of body rotating frame
     * @param minDate start of search time span
     * @param maxDate end of search time span
     * @param positionsVelocities satellite position and velocity
     * @param pvInterpolationOrder order to use for position/velocity interpolation
     * @param quaternions satellite quaternions
     * @param aInterpolationOrder order to use for attitude interpolation
     * @exception RuggedException if data needed for some frame cannot be loaded or if position
     * or attitude samples do not fully cover the [{@code minDate}, {@code maxDate}] search time span
     */
    public Rugged(final TileUpdater updater, final int maxCachedTiles,
                  final AlgorithmId algorithmID, final EllipsoidId ellipsoidID,
                  final InertialFrameId inertialFrameID, final BodyRotatingFrameId bodyRotatingFrameID,
                  final AbsoluteDate minDate, final AbsoluteDate maxDate,
                  final List<TimeStampedPVCoordinates> positionsVelocities, final int pvInterpolationOrder,
                  final List<TimeStampedAngularCoordinates> quaternions, final int aInterpolationOrder)
        throws RuggedException {
        this(updater, maxCachedTiles, algorithmID,
             selectEllipsoid(ellipsoidID, selectBodyRotatingFrame(bodyRotatingFrameID)),
             selectInertialFrame(inertialFrameID), minDate, maxDate,
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
     * @param updater updater used to load Digital Elevation Model tiles
     * @param maxCachedTiles maximum number of tiles stored in the cache
     * @param algorithmID identifier of algorithm to use for Digital Elevation Model intersection
     * @param ellipsoid reference ellipsoid
     * @param inertialFrame inertial frame
     * @param minDate start of search time span
     * @param maxDate end of search time span
     * @param positionsVelocities satellite position and velocity
     * @param pvInterpolationOrder order to use for position/velocity interpolation
     * @param quaternions satellite quaternions
     * @param aInterpolationOrder order to use for attitude interpolation
     * @exception RuggedException if data needed for some frame cannot be loaded or if position
     * or attitude samples do not fully cover the [{@code minDate}, {@code maxDate}] search time span
     */
    public Rugged(final TileUpdater updater, final int maxCachedTiles,
                  final AlgorithmId algorithmID, final OneAxisEllipsoid ellipsoid, final Frame inertialFrame,
                  final AbsoluteDate minDate, final AbsoluteDate maxDate,
                  final List<TimeStampedPVCoordinates> positionsVelocities, final int pvInterpolationOrder,
                  final List<TimeStampedAngularCoordinates> quaternions, final int aInterpolationOrder)
        throws RuggedException {

        // space reference
        this.ellipsoid     = extend(ellipsoid);

        // orbit/attitude to body converter
        this.scToBody = new SpacecraftToObservedBody(inertialFrame, ellipsoid.getBodyFrame(),
                                                     minDate, maxDate,
                                                     positionsVelocities, pvInterpolationOrder,
                                                     quaternions, aInterpolationOrder,
                                                     FRAMES_TRANSFORMS_INTERPOLATION_STEP);

        // intersection algorithm
        this.algorithm = selectAlgorithm(algorithmID, updater, maxCachedTiles);

        this.sensors = new HashMap<String, LineSensor>();
        this.finders = new HashMap<String, SensorMeanPlaneCrossing>();

        setLightTimeCorrection(true);
        setAberrationOfLightCorrection(true);

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
     * @param updater updater used to load Digital Elevation Model tiles
     * @param maxCachedTiles maximum number of tiles stored in the cache
     * @param algorithmID identifier of algorithm to use for Digital Elevation Model intersection
     * @param ellipsoidID identifier of reference ellipsoid
     * @param inertialFrameID identifier of inertial frame
     * @param bodyRotatingFrameID identifier of body rotating frame
     * @param minDate start of search time span
     * @param maxDate end of search time span
     * @param interpolationStep step to use for inertial/Earth/spacraft transforms interpolations
     * @param interpolationOrder order to use for inertial/Earth/spacraft transforms interpolations
     * @param propagator global propagator
     * @exception RuggedException if data needed for some frame cannot be loaded
     */
    public Rugged(final TileUpdater updater, final int maxCachedTiles,
                  final AlgorithmId algorithmID, final EllipsoidId ellipsoidID,
                  final InertialFrameId inertialFrameID, final BodyRotatingFrameId bodyRotatingFrameID,
                  final AbsoluteDate minDate, final AbsoluteDate maxDate,
                  final double interpolationStep, final int interpolationOrder, final Propagator propagator)
        throws RuggedException {
        this(updater, maxCachedTiles, algorithmID,
             selectEllipsoid(ellipsoidID, selectBodyRotatingFrame(bodyRotatingFrameID)),
             selectInertialFrame(inertialFrameID), minDate, maxDate,
             interpolationStep, interpolationOrder, propagator);
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
     * @param updater updater used to load Digital Elevation Model tiles
     * @param maxCachedTiles maximum number of tiles stored in the cache
     * @param algorithmID identifier of algorithm to use for Digital Elevation Model intersection
     * @param ellipsoid f reference ellipsoid
     * @param inertialFrame inertial frame
     * @param minDate start of search time span
     * @param maxDate end of search time span
     * @param interpolationStep step to use for inertial/Earth/spacraft transforms interpolations
     * @param interpolationOrder order to use for inertial/Earth/spacraft transforms interpolations
     * @param propagator global propagator
     * @exception RuggedException if data needed for some frame cannot be loaded
     */
    public Rugged(final TileUpdater updater, final int maxCachedTiles,
                  final AlgorithmId algorithmID, final OneAxisEllipsoid ellipsoid, final Frame inertialFrame,
                  final AbsoluteDate minDate, final AbsoluteDate maxDate,
                  final double interpolationStep, final int interpolationOrder, final Propagator propagator)
        throws RuggedException {
        try {

            // space reference
            this.ellipsoid = extend(ellipsoid);

            // extract position/attitude samples from propagator
            final List<TimeStampedPVCoordinates> positionsVelocities =
                    new ArrayList<TimeStampedPVCoordinates>();
            final List<TimeStampedAngularCoordinates> quaternions =
                    new ArrayList<TimeStampedAngularCoordinates>();
            propagator.setMasterMode(interpolationStep, new OrekitFixedStepHandler() {

                /** {@inheritDoc} */
                @Override
                public void init(final SpacecraftState s0, final AbsoluteDate t) {
                }

                /** {@inheritDoc} */
                @Override
                public void handleStep(final SpacecraftState currentState, final boolean isLast)
                    throws PropagationException {
                    try {
                        final AbsoluteDate  date = currentState.getDate();
                        final PVCoordinates pv   = currentState.getPVCoordinates(inertialFrame);
                        final Rotation      q    = currentState.getAttitude().getRotation();
                        positionsVelocities.add(new TimeStampedPVCoordinates(date, pv.getPosition(), pv.getVelocity()));
                        quaternions.add(new TimeStampedAngularCoordinates(date, q, Vector3D.ZERO));
                    } catch (OrekitException oe) {
                        throw new PropagationException(oe);
                    }
                }

            });
            propagator.propagate(minDate.shiftedBy(-interpolationStep), maxDate.shiftedBy(interpolationStep));

            // orbit/attitude to body converter
            this.scToBody = new SpacecraftToObservedBody(inertialFrame, ellipsoid.getBodyFrame(),
                                                         minDate, maxDate,
                                                         positionsVelocities, interpolationOrder,
                                                         quaternions, interpolationOrder,
                                                         FRAMES_TRANSFORMS_INTERPOLATION_STEP);

            // intersection algorithm
            this.algorithm = selectAlgorithm(algorithmID, updater, maxCachedTiles);

            this.sensors = new HashMap<String, LineSensor>();
            this.finders = new HashMap<String, SensorMeanPlaneCrossing>();

            setLightTimeCorrection(true);
            setAberrationOfLightCorrection(true);
        } catch (PropagationException pe) {
            throw new RuggedException(pe, pe.getSpecifier(), pe.getParts());
        }
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
     * @see #setAberrationOfLightCorrection(boolean)
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
     * @see #setLightTimeCorrection(boolean)
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
     * @param lineSensor line sensor model
     */
    public void addLineSensor(final LineSensor lineSensor) {
        sensors.put(lineSensor.getName(), lineSensor);
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

    /** Convert a {@link OneAxisEllipsoid} into a {@link ExtendedEllipsoid}.
     * @param ellipsoid ellpsoid to extend
     * @return extended ellipsoid
     */
    private static ExtendedEllipsoid extend(final OneAxisEllipsoid ellipsoid) {
        return new ExtendedEllipsoid(ellipsoid.getEquatorialRadius(),
                                     ellipsoid.getFlattening(),
                                     ellipsoid.getBodyFrame());
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
            return new DuvenhageAlgorithm(updater, maxCachedTiles, false);
        case DUVENHAGE_FLAT_BODY :
            return new DuvenhageAlgorithm(updater, maxCachedTiles, true);
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

            // compute the approximate transform between spacecraft and observed body
            final LineSensor   sensor      = getLineSensor(sensorName);
            final AbsoluteDate date        = sensor.getDate(lineNumber);
            final Transform    scToInert   = scToBody.getScToInertial(date);
            final Transform    inertToBody = scToBody.getInertialToBody(date);
            final Transform    approximate = new Transform(date, scToInert, inertToBody);

            final Vector3D spacecraftVelocity =
                    scToInert.transformPVCoordinates(PVCoordinates.ZERO).getVelocity();

            // compute localization of each pixel
            final Vector3D pInert    = scToInert.transformPosition(sensor.getPosition());
            final GeodeticPoint[] gp = new GeodeticPoint[sensor.getNbPixels()];
            for (int i = 0; i < sensor.getNbPixels(); ++i) {

                final Vector3D obsLInert = scToInert.transformVector(sensor.getLos(i));
                final Vector3D lInert;
                if (aberrationOfLightCorrection) {
                    // apply aberration of light correction
                    // as the spacecraft velocity is small with respect to speed of light,
                    // we use classical velocity addition and not relativistic velocity addition
                    // we look for a positive k such that: c * lInert + vsat = k * obsLInert
                    // with lInert normalized
                    final double a = obsLInert.getNormSq();
                    final double b = -Vector3D.dotProduct(obsLInert, spacecraftVelocity);
                    final double c = spacecraftVelocity.getNormSq() - Constants.SPEED_OF_LIGHT * Constants.SPEED_OF_LIGHT;
                    final double s = FastMath.sqrt(b * b - a * c);
                    final double k = (b > 0) ? -c / (s + b) : (s - b) / a;
                    lInert = new Vector3D( k   / Constants.SPEED_OF_LIGHT, obsLInert,
                                          -1.0 / Constants.SPEED_OF_LIGHT, spacecraftVelocity);
                } else {
                    // don't apply aberration of light correction
                    lInert = obsLInert;
                }

                if (lightTimeCorrection) {
                    // compute DEM intersection with light time correction
                    final Vector3D  sP       = approximate.transformPosition(sensor.getPosition());
                    final Vector3D  sL       = approximate.transformVector(sensor.getLos(i));
                    final Vector3D  eP1      = ellipsoid.transform(ellipsoid.pointOnGround(sP, sL));
                    final double    deltaT1  = eP1.distance(sP) / Constants.SPEED_OF_LIGHT;
                    final Transform shifted1 = inertToBody.shiftedBy(-deltaT1);
                    final GeodeticPoint gp1  = algorithm.intersection(ellipsoid,
                                                                      shifted1.transformPosition(pInert),
                                                                      shifted1.transformVector(lInert));

                    final Vector3D  eP2      = ellipsoid.transform(gp1);
                    final double    deltaT2  = eP2.distance(sP) / Constants.SPEED_OF_LIGHT;
                    final Transform shifted2 = inertToBody.shiftedBy(-deltaT2);
                    gp[i] = algorithm.refineIntersection(ellipsoid,
                                                         shifted2.transformPosition(pInert),
                                                         shifted2.transformVector(lInert),
                                                         gp1);

                } else {
                    // compute DEM intersection without light time correction
                    final Vector3D pBody = inertToBody.transformPosition(pInert);
                    final Vector3D lBody = inertToBody.transformVector(lInert);
                    gp[i] = algorithm.refineIntersection(ellipsoid, pBody, lBody,
                                                         algorithm.intersection(ellipsoid, pBody, lBody));
                }

            }

            return gp;

        } catch (OrekitException oe) {
            throw new RuggedException(oe, oe.getSpecifier(), oe.getParts());
        }
    }

    /** Inverse localization of a ground point.
     * <p>
     * The point is given only by its latitude and longitude, the elevation is
     * computed from the Digital Elevation Model.
     * </p>
     * @param sensorName name of the line  sensor
     * @param latitude ground point latitude
     * @param longitude ground point longitude
     * @param minLine minimum line number
     * @param maxLine maximum line number
     * @return sensor pixel seeing ground point, or null if ground point cannot
     * be seen between the prescribed line numbers
     * @exception RuggedException if line cannot be localized, or sensor is unknown
     */
    public SensorPixel inverseLocalization(final String sensorName,
                                           final double latitude, final double longitude,
                                           final int minLine,  final int maxLine)
        throws RuggedException {
        final GeodeticPoint groundPoint =
                new GeodeticPoint(latitude, longitude, algorithm.getElevation(latitude, longitude));
        return inverseLocalization(sensorName, groundPoint, minLine, maxLine);
    }

    /** Inverse localization of a point.
     * @param sensorName name of the line  sensor
     * @param point point to localize
     * @param minLine minimum line number
     * @param maxLine maximum line number
     * @return sensor pixel seeing point, or null if point cannot be seen between the
     * prescribed line numbers
     * @exception RuggedException if line cannot be localized, or sensor is unknown
     */
    public SensorPixel inverseLocalization(final String sensorName, final GeodeticPoint point,
                                           final int minLine, final int maxLine)
        throws RuggedException {

        final LineSensor sensor = getLineSensor(sensorName);
        SensorMeanPlaneCrossing planeCrossing = finders.get(sensorName);
        if (planeCrossing == null ||
            planeCrossing.getMinLine() != minLine ||
            planeCrossing.getMaxLine() != maxLine) {

            // create a new finder for the specified sensor and range
            planeCrossing = new SensorMeanPlaneCrossing(sensor, scToBody, minLine, maxLine,
                                                        lightTimeCorrection, aberrationOfLightCorrection,
                                                        MAX_EVAL, COARSE_INVERSE_LOCALIZATION_ACCURACY);

            // store the finder, in order to reuse it
            // (and save some computation done in its constructor)
            finders.put(sensorName, planeCrossing);

        }

        // find approximately the sensor line at which ground point crosses sensor mean plane
        final Vector3D   target = ellipsoid.transform(point);
        final SensorMeanPlaneCrossing.CrossingResult crossingResult = planeCrossing.find(target);
        if (crossingResult == null) {
            // target is out of search interval
            return null;
        }

        // find approximately the pixel along this sensor line
        final SensorPixelCrossing pixelCrossing =
                new SensorPixelCrossing(sensor, crossingResult.getTargetDirection().toVector3D(),
                                        MAX_EVAL, COARSE_INVERSE_LOCALIZATION_ACCURACY);
        final double coarsePixel = pixelCrossing.locatePixel();
        if (Double.isNaN(coarsePixel)) {
            // target is out of search interval
            return null;
        }

        // fix line by considering the closest pixel exact position and line-of-sight
        // (this pixel might point towards a direction slightly above or below the mean sensor plane)
        final int      closestIndex    = (int) FastMath.rint(coarsePixel);
        final DerivativeStructure beta = FieldVector3D.angle(crossingResult.getTargetDirection(), sensor.getMeanPlaneNormal());
        final double   deltaL          = (0.5 * FastMath.PI - beta.getValue()) / beta.getPartialDerivative(1);
        final double   fixedLine       = crossingResult.getLine() + deltaL;
        final Vector3D fixedDirection  = new Vector3D(crossingResult.getTargetDirection().getX().taylor(deltaL),
                                                      crossingResult.getTargetDirection().getY().taylor(deltaL),
                                                      crossingResult.getTargetDirection().getZ().taylor(deltaL)).normalize();

        // fix pixel
        final double alpha      = sensor.getAzimuth(fixedDirection, closestIndex);
        final double pixelWidth = sensor.getWidth(closestIndex);
        final double fixedPixel = closestIndex + alpha / pixelWidth;

        return new SensorPixel(fixedLine, fixedPixel);

    }

    /** Get a sensor.
     * @param sensorName sensor name
     * @return selected sensor
     * @exception RuggedException if sensor is not known
     */
    private LineSensor getLineSensor(final String sensorName) throws RuggedException {
        final LineSensor sensor = sensors.get(sensorName);
        if (sensor == null) {
            throw new RuggedException(RuggedMessages.UNKNOWN_SENSOR, sensorName);
        }
        return sensor;
    }

}
