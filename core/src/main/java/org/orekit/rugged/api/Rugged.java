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

import java.io.IOException;
import java.io.InputStream;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.OutputStream;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.apache.commons.math3.analysis.differentiation.DerivativeStructure;
import org.apache.commons.math3.exception.util.LocalizedFormats;
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
import org.orekit.rugged.utils.NormalizedGeodeticPoint;
import org.orekit.rugged.utils.SpacecraftToObservedBody;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.AngularDerivativesFilter;
import org.orekit.utils.CartesianDerivativesFilter;
import org.orekit.utils.Constants;
import org.orekit.utils.IERSConventions;
import org.orekit.utils.PVCoordinates;
import org.orekit.utils.TimeStampedAngularCoordinates;
import org.orekit.utils.TimeStampedPVCoordinates;

/** Main class of Rugged library API.
 * @author Luc Maisonobe
 */
public class Rugged {

    /** Accuracy to use in the first stage of inverse location.
     * <p>
     * This accuracy is only used to locate the point within one
     * pixel, hence there is no point in choosing a too small value here.
     * </p>
     */
    private static final double COARSE_INVERSE_LOCATION_ACCURACY = 0.01;

    /** Maximum number of evaluations. */
    private static final int MAX_EVAL = 50;

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
     * @param inertialFrameID identifier of inertial frame used for spacecraft positions/velocities/quaternions
     * @param bodyRotatingFrameID identifier of body rotating frame for observed ground points
     * @param minDate start of search time span
     * @param maxDate end of search time span
     * @param tStep step to use for inertial frame to body frame transforms cache computations
     * @param overshootTolerance tolerance in seconds allowed for {@code minDate} and {@code maxDate} overshooting
     * @param positionsVelocities satellite position and velocity (m and m/s in inertial frame)
     * @param pvInterpolationNumber number of points to use for position/velocity interpolation
     * @param pvFilter filter for derivatives from the sample to use in position/velocity interpolation
     * @param quaternions satellite quaternions with respect to inertial frame
     * @param aInterpolationNumber number of points to use for attitude interpolation
     * @param aFilter filter for derivatives from the sample to use in attitude interpolation
     * @exception RuggedException if data needed for some frame cannot be loaded or if position
     * or attitude samples do not fully cover the [{@code minDate}, {@code maxDate}] search time span
     */
    public Rugged(final TileUpdater updater, final int maxCachedTiles,
                  final AlgorithmId algorithmID, final EllipsoidId ellipsoidID,
                  final InertialFrameId inertialFrameID, final BodyRotatingFrameId bodyRotatingFrameID,
                  final AbsoluteDate minDate, final AbsoluteDate maxDate, final double tStep,
                  final double overshootTolerance,
                  final List<TimeStampedPVCoordinates> positionsVelocities, final int pvInterpolationNumber,
                  final CartesianDerivativesFilter pvFilter,
                  final List<TimeStampedAngularCoordinates> quaternions, final int aInterpolationNumber,
                  final AngularDerivativesFilter aFilter)
        throws RuggedException {
        this(updater, maxCachedTiles, algorithmID,
             selectEllipsoid(ellipsoidID, selectBodyRotatingFrame(bodyRotatingFrameID)),
             selectInertialFrame(inertialFrameID),
             minDate, maxDate, tStep, overshootTolerance,
             positionsVelocities, pvInterpolationNumber, pvFilter,
             quaternions, aInterpolationNumber, aFilter);
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
     * @param inertialFrame inertial frame used for spacecraft positions/velocities/quaternions
     * @param minDate start of search time span
     * @param maxDate end of search time span
     * @param tStep step to use for inertial frame to body frame transforms cache computations
     * @param overshootTolerance tolerance in seconds allowed for {@code minDate} and {@code maxDate} overshooting
     * @param positionsVelocities satellite position and velocity (m and m/s in inertial frame)
     * @param pvInterpolationNumber number of points to use for position/velocity interpolation
     * @param pvFilter filter for derivatives from the sample to use in position/velocity interpolation
     * @param quaternions satellite quaternions with respect to inertial frame
     * @param aInterpolationNumber number of points to use for attitude interpolation
     * @param aFilter filter for derivatives from the sample to use in attitude interpolation
     * @exception RuggedException if data needed for some frame cannot be loaded or if position
     * or attitude samples do not fully cover the [{@code minDate}, {@code maxDate}] search time span
     */
    public Rugged(final TileUpdater updater, final int maxCachedTiles,
                  final AlgorithmId algorithmID, final OneAxisEllipsoid ellipsoid, final Frame inertialFrame,
                  final AbsoluteDate minDate, final AbsoluteDate maxDate, final double tStep,
                  final double overshootTolerance,
                  final List<TimeStampedPVCoordinates> positionsVelocities, final int pvInterpolationNumber,
                  final CartesianDerivativesFilter pvFilter,
                  final List<TimeStampedAngularCoordinates> quaternions, final int aInterpolationNumber,
                  final AngularDerivativesFilter aFilter)
        throws RuggedException {
        this(updater, maxCachedTiles, algorithmID, ellipsoid,
             createInterpolator(inertialFrame, ellipsoid.getBodyFrame(),
                                minDate, maxDate, tStep, overshootTolerance,
                                positionsVelocities, pvInterpolationNumber, pvFilter,
                                quaternions, aInterpolationNumber, aFilter));
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
     * @param inertialFrameID identifier of inertial frame for spacecraft positions/velocities/quaternions
     * @param bodyRotatingFrameID identifier of body rotating frame for observed ground points
     * @param minDate start of search time span
     * @param maxDate end of search time span
     * @param tStep step to use for inertial frame to body frame transforms cache computations
     * @param overshootTolerance tolerance in seconds allowed for {@code minDate} and {@code maxDate} overshooting
     * @param interpolationStep step to use for inertial/Earth/spacraft transforms interpolations
     * @param interpolationNumber number of points to use for inertial/Earth/spacraft transforms interpolations
     * @param pvFilter filter for derivatives from the sample to use in position/velocity interpolation
     * @param aFilter filter for derivatives from the sample to use in attitude interpolation
     * @param propagator global propagator
     * @exception RuggedException if data needed for some frame cannot be loaded
     */
    public Rugged(final TileUpdater updater, final int maxCachedTiles,
                  final AlgorithmId algorithmID, final EllipsoidId ellipsoidID,
                  final InertialFrameId inertialFrameID, final BodyRotatingFrameId bodyRotatingFrameID,
                  final AbsoluteDate minDate, final AbsoluteDate maxDate, final double tStep,
                  final double overshootTolerance,
                  final double interpolationStep, final int interpolationNumber,
                  final CartesianDerivativesFilter pvFilter, final AngularDerivativesFilter aFilter,
                  final Propagator propagator)
        throws RuggedException {
        this(updater, maxCachedTiles, algorithmID,
             selectEllipsoid(ellipsoidID, selectBodyRotatingFrame(bodyRotatingFrameID)),
             selectInertialFrame(inertialFrameID),
             minDate, maxDate, tStep, overshootTolerance,
             interpolationStep, interpolationNumber, pvFilter, aFilter, propagator);
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
     * @param inertialFrame inertial frame for spacecraft positions/velocities/quaternions
     * @param minDate start of search time span
     * @param maxDate end of search time span
     * @param tStep step to use for inertial frame to body frame transforms cache computations
     * @param overshootTolerance tolerance in seconds allowed for {@code minDate} and {@code maxDate} overshooting
     * @param interpolationStep step to use for inertial/Earth/spacraft transforms interpolations
     * @param interpolationNumber number of points of to use for inertial/Earth/spacraft transforms interpolations
     * @param pvFilter filter for derivatives from the sample to use in position/velocity interpolation
     * @param aFilter filter for derivatives from the sample to use in attitude interpolation
     * @param propagator global propagator
     * @exception RuggedException if data needed for some frame cannot be loaded
     */
    public Rugged(final TileUpdater updater, final int maxCachedTiles,
                  final AlgorithmId algorithmID, final OneAxisEllipsoid ellipsoid, final Frame inertialFrame,
                  final AbsoluteDate minDate, final AbsoluteDate maxDate, final double tStep,
                  final double overshootTolerance,
                  final double interpolationStep, final int interpolationNumber,
                  final CartesianDerivativesFilter pvFilter, final AngularDerivativesFilter aFilter,
                  final Propagator propagator)
        throws RuggedException {
        this(updater, maxCachedTiles, algorithmID, ellipsoid,
             createInterpolator(inertialFrame, ellipsoid.getBodyFrame(),
                                minDate, maxDate, tStep, overshootTolerance,
                                interpolationStep, interpolationNumber, pvFilter, aFilter, propagator));
    }

    /** Build a configured instance, reusing the interpolator dumped from a previous instance.
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
     * @param inertialFrameID identifier of inertial frame for spacecraft positions/velocities/quaternions
     * @param bodyRotatingFrameID identifier of body rotating frame for observed ground points
     * @param dumpStream stream from where to read previous instance dumped interpolator
     * (caller opened it and remains responsible for closing it)
     * @exception RuggedException if dump file cannot be loaded
     * or if frames do not match the ones referenced in the dump file
     * @see #dumpInterpolator(OutputStream)
     */
    public Rugged(final TileUpdater updater, final int maxCachedTiles,
                  final AlgorithmId algorithmID, final EllipsoidId ellipsoidID,
                  final InertialFrameId inertialFrameID, final BodyRotatingFrameId bodyRotatingFrameID,
                  final InputStream dumpStream)
        throws RuggedException {
        this(updater, maxCachedTiles, algorithmID,
             selectEllipsoid(ellipsoidID, selectBodyRotatingFrame(bodyRotatingFrameID)),
             selectInertialFrame(inertialFrameID), dumpStream);
    }

    /** Build a configured instance, reusing the interpolator dumped from a previous instance.
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
     * @param inertialFrame inertial frame for spacecraft positions/velocities/quaternions
     * @param dumpStream stream from where to read previous instance dumped interpolator
     * (caller opened it and remains responsible for closing it)
     * @exception RuggedException if dump file cannot be loaded
     * or if frames do not match the ones referenced in the dump file
     * @see #dumpInterpolator(OutputStream)
     */
    public Rugged(final TileUpdater updater, final int maxCachedTiles,
                  final AlgorithmId algorithmID, final OneAxisEllipsoid ellipsoid, final Frame inertialFrame,
                  final InputStream dumpStream)
        throws RuggedException {
        this(updater, maxCachedTiles, algorithmID, ellipsoid,
             createInterpolator(inertialFrame, ellipsoid.getBodyFrame(), dumpStream));
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
     * @param scToBody transforms interpolator
     */
    private Rugged(final TileUpdater updater, final int maxCachedTiles,
                  final AlgorithmId algorithmID, final OneAxisEllipsoid ellipsoid,
                  final SpacecraftToObservedBody scToBody) {

        // space reference
        this.ellipsoid = extend(ellipsoid);

        // orbit/attitude to body converter
        this.scToBody = scToBody;

        // intersection algorithm
        this.algorithm = selectAlgorithm(algorithmID, updater, maxCachedTiles);

        this.sensors = new HashMap<String, LineSensor>();
        this.finders = new HashMap<String, SensorMeanPlaneCrossing>();

        setLightTimeCorrection(true);
        setAberrationOfLightCorrection(true);

    }

    /** Create a transform interpolator from positions and quaternions lists.
     * @param inertialFrame inertial frame
     * @param bodyFrame observed body frame
     * @param minDate start of search time span
     * @param maxDate end of search time span
     * @param tStep step to use for inertial frame to body frame transforms cache computations
     * @param overshootTolerance tolerance in seconds allowed for {@code minDate} and {@code maxDate} overshooting
     * @param positionsVelocities satellite position and velocity
     * @param pvInterpolationNumber number of points to use for position/velocity interpolation
     * @param pvFilter filter for derivatives from the sample to use in position/velocity interpolation
     * @param quaternions satellite quaternions
     * @param aInterpolationNumber number of points to use for attitude interpolation
     * @param aFilter filter for derivatives from the sample to use in attitude interpolation
     * @return transforms interpolator
     * @exception RuggedException if data needed for some frame cannot be loaded or if position
     * or attitude samples do not fully cover the [{@code minDate}, {@code maxDate}] search time span
     */
    private static SpacecraftToObservedBody createInterpolator(final Frame inertialFrame, final Frame bodyFrame,
                                                               final AbsoluteDate minDate, final AbsoluteDate maxDate,
                                                               final double tStep, final double overshootTolerance,
                                                               final List<TimeStampedPVCoordinates> positionsVelocities,
                                                               final int pvInterpolationNumber,
                                                               final CartesianDerivativesFilter pvFilter,
                                                               final List<TimeStampedAngularCoordinates> quaternions,
                                                               final int aInterpolationNumber,
                                                               final AngularDerivativesFilter aFilter)
        throws RuggedException {
        return new SpacecraftToObservedBody(inertialFrame, bodyFrame,
                                            minDate, maxDate, tStep,
                                            overshootTolerance, positionsVelocities, pvInterpolationNumber,
                                            pvFilter, quaternions, aInterpolationNumber,
                                            aFilter);
    }

    /** Create a transform interpolator from a propagator.
     * @param inertialFrame inertial frame
     * @param bodyFrame observed body frame
     * @param minDate start of search time span
     * @param maxDate end of search time span
     * @param tStep step to use for inertial frame to body frame transforms cache computations
     * @param overshootTolerance tolerance in seconds allowed for {@code minDate} and {@code maxDate} overshooting
     * @param interpolationStep step to use for inertial/Earth/spacraft transforms interpolations
     * @param interpolationNumber number of points of to use for inertial/Earth/spacraft transforms interpolations
     * @param pvFilter filter for derivatives from the sample to use in position/velocity interpolation
     * @param aFilter filter for derivatives from the sample to use in attitude interpolation
     * @param propagator global propagator
     * @return transforms interpolator
     * @exception RuggedException if data needed for some frame cannot be loaded
     */
    private static SpacecraftToObservedBody createInterpolator(final Frame inertialFrame, final Frame bodyFrame,
                                                               final AbsoluteDate minDate, final AbsoluteDate maxDate,
                                                               final double tStep, final double overshootTolerance,
                                                               final double interpolationStep, final int interpolationNumber,
                                                               final CartesianDerivativesFilter pvFilter,
                                                               final AngularDerivativesFilter aFilter,
                                                               final Propagator propagator)
        throws RuggedException {
        try {

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
                        positionsVelocities.add(new TimeStampedPVCoordinates(date, pv.getPosition(), pv.getVelocity(), Vector3D.ZERO));
                        quaternions.add(new TimeStampedAngularCoordinates(date, q, Vector3D.ZERO, Vector3D.ZERO));
                    } catch (OrekitException oe) {
                        throw new PropagationException(oe);
                    }
                }

            });
            propagator.propagate(minDate.shiftedBy(-interpolationStep), maxDate.shiftedBy(interpolationStep));

            // orbit/attitude to body converter
            return createInterpolator(inertialFrame, bodyFrame,
                                      minDate, maxDate, tStep,
                                      overshootTolerance, positionsVelocities, interpolationNumber,
                                      pvFilter, quaternions, interpolationNumber,
                                      aFilter);

        } catch (PropagationException pe) {
            throw new RuggedException(pe, pe.getSpecifier(), pe.getParts());
        }
    }

    /** Create a transform interpolator from positions and quaternions lists.
     * @param inertialFrame inertial frame
     * @param bodyFrame observed body frame
     * @param dumpStream stream from where to read previous instance dumped interpolator
     * (caller opened it and remains responsible for closing it)
     * @return recovered interpolator from dump
     * @exception RuggedException if dump file cannot be loaded or if
     * frames do not match the ones referenced in the dump file
     */
    private static SpacecraftToObservedBody createInterpolator(final Frame inertialFrame, final Frame bodyFrame,
                                                               final InputStream dumpStream)
        throws RuggedException {
        try {
            final ObjectInputStream ois = new ObjectInputStream(dumpStream);
            final SpacecraftToObservedBody scToBody = (SpacecraftToObservedBody) ois.readObject();
            if (!inertialFrame.getName().equals(scToBody.getInertialFrameName())) {
                throw new RuggedException(RuggedMessages.FRAMES_MISMATCH_WITH_INTERPOLATOR_DUMP,
                                          inertialFrame.getName(), scToBody.getInertialFrameName());
            }
            if (!bodyFrame.getName().equals(scToBody.getBodyFrameName())) {
                throw new RuggedException(RuggedMessages.FRAMES_MISMATCH_WITH_INTERPOLATOR_DUMP,
                                          bodyFrame.getName(), scToBody.getBodyFrameName());
            }
            return scToBody;
        } catch (ClassNotFoundException cnfe) {
            throw new RuggedException(cnfe, RuggedMessages.NOT_INTERPOLATOR_DUMP_DATA);
        } catch (ClassCastException cce) {
            throw new RuggedException(cce, RuggedMessages.NOT_INTERPOLATOR_DUMP_DATA);
        } catch (IOException ioe) {
            throw new RuggedException(ioe, RuggedMessages.NOT_INTERPOLATOR_DUMP_DATA);
        }
    }

    /** Dump frames transform interpolator.
     * <p>
     * This method allows to reuse the interpolator built in one instance, to build
     * another instance by calling {@link #Rugged(TileUpdater, int, AlgorithmId, OneAxisEllipsoid, Frame, InputStream)}
     * or {@link #Rugged(TileUpdater, int, AlgorithmId, EllipsoidId, InertialFrameId, BodyRotatingFrameId, InputStream)}.
     * This reduces Rugged initialization time as setting up the interpolator can be long, it is
     * mainly intended to be used when several runs are done (for example in an image processing chain)
     * with the same configuration.
     * </p>
     * @param dumpStream stream where to dump the interpolator
     * (caller opened it and remains responsible for closing it)
     * @exception RuggedException if interpolator cannot be written to file
     * @see #Rugged(TileUpdater, int, AlgorithmId, OneAxisEllipsoid, Frame, InputStream)
     * @see #Rugged(TileUpdater, int, AlgorithmId, EllipsoidId, InertialFrameId, BodyRotatingFrameId, InputStream)
     */
    public void dumpInterpolator(final OutputStream dumpStream) throws RuggedException {
        try {
            final ObjectOutputStream oos = new ObjectOutputStream(dumpStream);
            oos.writeObject(scToBody);
        } catch (IOException ioe) {
            throw new RuggedException(ioe, LocalizedFormats.SIMPLE_MESSAGE, ioe.getMessage());
        }
    }

    /** Set flag for light time correction.
     * <p>
     * This methods set the flag for compensating or not light time between
     * ground and spacecraft. Compensating this delay improves location
     * accuracy and is enabled by default. Not compensating it is mainly useful
     * for validation purposes against system that do not compensate it.
     * </p>
     * @param lightTimeCorrection if true, the light travel time between ground
     * and spacecraft is compensated for more accurate location
     * @see #isLightTimeCorrected()
     * @see #setAberrationOfLightCorrection(boolean)
     */
    public void setLightTimeCorrection(final boolean lightTimeCorrection) {
        this.lightTimeCorrection = lightTimeCorrection;
    }

    /** Get flag for light time correction.
     * @return true if the light time between ground and spacecraft is
     * compensated for more accurate location
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
     * Compensating this velocity composition improves location
     * accuracy and is enabled by default. Not compensating it is useful
     * in two cases: for validation purposes against system that do not
     * compensate it or when the pixels line of sight already include the
     * correction.
     * </p>
     * @param aberrationOfLightCorrection if true, the aberration of light
     * is corrected for more accurate location
     * @see #isAberrationOfLightCorrected()
     * @see #setLightTimeCorrection(boolean)
     */
    public void setAberrationOfLightCorrection(final boolean aberrationOfLightCorrection) {
        this.aberrationOfLightCorrection = aberrationOfLightCorrection;
    }

    /** Get flag for aberration of light correction.
     * @return true if the aberration of light time is corrected
     * for more accurate location
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

    /** Get the line sensors.
     * @return line sensors
     */
    public Collection<LineSensor> getLineSensors() {
        return sensors.values();
    }

    /** Get the start of search time span.
     * @return start of search time span
     */
    public AbsoluteDate getMinDate() {
        return scToBody.getMinDate();
    }

    /** Get the end of search time span.
     * @return end of search time span
     */
    public AbsoluteDate getMaxDate() {
        return scToBody.getMaxDate();
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

    /** Check if a date is in the supported range.
     * <p>
     * The supporte range is given by the {@code minDate} and
     * {@code maxDate} construction parameters, with an {@code
     * overshootTolerance} margin accepted (i.e. a date slightly
     * before {@code minDate} or slightly after {@code maxDate}
     * will be considered in range if the overshoot does not exceed
     * the tolerance set at construction).
     * </p>
     * @param date date to check
     * @return true if date is in the supported range
     */
    public boolean isInRange(final AbsoluteDate date) {
        return scToBody.isInRange(date);
    }

    /** Direct location of a sensor line.
     * @param sensorName name of the line sensor
     * @param lineNumber number of the line to localize on ground
     * @return ground position of all pixels of the specified sensor line
     * @exception RuggedException if line cannot be localized, or sensor is unknown
     */
    public GeodeticPoint[] directLocation(final String sensorName, final double lineNumber)
        throws RuggedException {

        // compute the approximate transform between spacecraft and observed body
        final LineSensor   sensor      = getLineSensor(sensorName);
        final AbsoluteDate date        = sensor.getDate(lineNumber);
        final Transform    scToInert   = scToBody.getScToInertial(date);
        final Transform    inertToBody = scToBody.getInertialToBody(date);
        final Transform    approximate = new Transform(date, scToInert, inertToBody);

        final Vector3D spacecraftVelocity =
                scToInert.transformPVCoordinates(PVCoordinates.ZERO).getVelocity();

        // compute location of each pixel
        final Vector3D pInert    = scToInert.transformPosition(sensor.getPosition());
        final GeodeticPoint[] gp = new GeodeticPoint[sensor.getNbPixels()];
        for (int i = 0; i < sensor.getNbPixels(); ++i) {

            final Vector3D obsLInert = scToInert.transformVector(sensor.getLos(date, i));
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
                final Vector3D  sL       = approximate.transformVector(sensor.getLos(date, i));
                final Vector3D  eP1      = ellipsoid.transform(ellipsoid.pointOnGround(sP, sL, 0.0));
                final double    deltaT1  = eP1.distance(sP) / Constants.SPEED_OF_LIGHT;
                final Transform shifted1 = inertToBody.shiftedBy(-deltaT1);
                final NormalizedGeodeticPoint gp1  = algorithm.intersection(ellipsoid,
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

    }

    /** Direct location of a single line-of-sight.
     * @param date date of the location
     * @param position pixel position in spacecraft frame
     * @param los normalized line-of-sight in spacecraft frame
     * @return ground position of all pixels of the specified sensor line
     * @exception RuggedException if line cannot be localized, or sensor is unknown
     */
    public GeodeticPoint directLocation(final AbsoluteDate date, final Vector3D position, final Vector3D los)
        throws RuggedException {

        // compute the approximate transform between spacecraft and observed body
        final Transform    scToInert   = scToBody.getScToInertial(date);
        final Transform    inertToBody = scToBody.getInertialToBody(date);
        final Transform    approximate = new Transform(date, scToInert, inertToBody);

        final Vector3D spacecraftVelocity =
                scToInert.transformPVCoordinates(PVCoordinates.ZERO).getVelocity();

        // compute location of specified pixel
        final Vector3D pInert    = scToInert.transformPosition(position);

        final Vector3D obsLInert = scToInert.transformVector(los);
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
            final Vector3D  sP       = approximate.transformPosition(position);
            final Vector3D  sL       = approximate.transformVector(los);
            final Vector3D  eP1      = ellipsoid.transform(ellipsoid.pointOnGround(sP, sL, 0.0));
            final double    deltaT1  = eP1.distance(sP) / Constants.SPEED_OF_LIGHT;
            final Transform shifted1 = inertToBody.shiftedBy(-deltaT1);
            final NormalizedGeodeticPoint gp1  = algorithm.intersection(ellipsoid,
                                                                        shifted1.transformPosition(pInert),
                                                                        shifted1.transformVector(lInert));

            final Vector3D  eP2      = ellipsoid.transform(gp1);
            final double    deltaT2  = eP2.distance(sP) / Constants.SPEED_OF_LIGHT;
            final Transform shifted2 = inertToBody.shiftedBy(-deltaT2);
            return algorithm.refineIntersection(ellipsoid,
                                                shifted2.transformPosition(pInert),
                                                shifted2.transformVector(lInert),
                                                gp1);

        } else {
            // compute DEM intersection without light time correction
            final Vector3D pBody = inertToBody.transformPosition(pInert);
            final Vector3D lBody = inertToBody.transformVector(lInert);
            return algorithm.refineIntersection(ellipsoid, pBody, lBody,
                                                algorithm.intersection(ellipsoid, pBody, lBody));
        }

    }

    /** Find the date at which sensor sees a ground point.
     * <p>
     * This method is a partial {@link #inverseLocation(String,
     * GeodeticPoint, int, int) inverse location} focusing only on date.
     * </p>
     * <p>
     * The point is given only by its latitude and longitude, the elevation is
     * computed from the Digital Elevation Model.
     * </p>
     * @param sensorName name of the line  sensor
     * @param latitude ground point latitude
     * @param longitude ground point longitude
     * @param minLine minimum line number
     * @param maxLine maximum line number
     * @return date at which ground point is seen by line sensor
     * @exception RuggedException if line cannot be localized, or sensor is unknown
     * @see #inverseLocation(String, double, double, int, int)
     */
    public AbsoluteDate dateLocation(final String sensorName,
                                     final double latitude, final double longitude,
                                     final int minLine, final int maxLine)
        throws RuggedException {
        final GeodeticPoint groundPoint =
                new GeodeticPoint(latitude, longitude, algorithm.getElevation(latitude, longitude));
        return dateLocation(sensorName, groundPoint, minLine, maxLine);
    }

    /** Find the date at which sensor sees a ground point.
     * <p>
     * This method is a partial {@link #inverseLocation(String,
     * GeodeticPoint, int, int) inverse location} focusing only on date.
     * </p>
     * @param sensorName name of the line  sensor
     * @param point point to localize
     * @param minLine minimum line number
     * @param maxLine maximum line number
     * @return date at which ground point is seen by line sensor
     * @exception RuggedException if line cannot be localized, or sensor is unknown
     * @see #inverseLocation(String, GeodeticPoint, int, int)
     */
    public AbsoluteDate dateLocation(final String sensorName, final GeodeticPoint point,
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
                                                        MAX_EVAL, COARSE_INVERSE_LOCATION_ACCURACY);

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
        } else {
            return sensor.getDate(crossingResult.getLine());
        }

    }

    /** Inverse location of a ground point.
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
    public SensorPixel inverseLocation(final String sensorName,
                                       final double latitude, final double longitude,
                                       final int minLine,  final int maxLine)
        throws RuggedException {
        final GeodeticPoint groundPoint =
                new GeodeticPoint(latitude, longitude, algorithm.getElevation(latitude, longitude));
        return inverseLocation(sensorName, groundPoint, minLine, maxLine);
    }

    /** Inverse location of a point.
     * @param sensorName name of the line  sensor
     * @param point point to localize
     * @param minLine minimum line number
     * @param maxLine maximum line number
     * @return sensor pixel seeing point, or null if point cannot be seen between the
     * prescribed line numbers
     * @exception RuggedException if line cannot be localized, or sensor is unknown
     * @see #dateLocation(String, GeodeticPoint, int, int)
     */
    public SensorPixel inverseLocation(final String sensorName, final GeodeticPoint point,
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
                                                        MAX_EVAL, COARSE_INVERSE_LOCATION_ACCURACY);

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
                new SensorPixelCrossing(sensor, planeCrossing.getMeanPlaneNormal(),
                                        crossingResult.getTargetDirection().toVector3D(),
                                        MAX_EVAL, COARSE_INVERSE_LOCATION_ACCURACY);
        final double coarsePixel = pixelCrossing.locatePixel(crossingResult.getDate());
        if (Double.isNaN(coarsePixel)) {
            // target is out of search interval
            return null;
        }

        // fix line by considering the closest pixel exact position and line-of-sight
        // (this pixel might point towards a direction slightly above or below the mean sensor plane)
        final int      lowIndex        = FastMath.max(0, FastMath.min(sensor.getNbPixels() - 2, (int) FastMath.floor(coarsePixel)));
        final Vector3D lowLOS          = sensor.getLos(crossingResult.getDate(), lowIndex);
        final Vector3D highLOS         = sensor.getLos(crossingResult.getDate(), lowIndex + 1);
        final Vector3D localZ          = Vector3D.crossProduct(lowLOS, highLOS);
        final DerivativeStructure beta = FieldVector3D.angle(crossingResult.getTargetDirection(), localZ);
        final double   deltaL          = (0.5 * FastMath.PI - beta.getValue()) / beta.getPartialDerivative(1);
        final double   fixedLine       = crossingResult.getLine() + deltaL;
        final Vector3D fixedDirection  = new Vector3D(crossingResult.getTargetDirection().getX().taylor(deltaL),
                                                      crossingResult.getTargetDirection().getY().taylor(deltaL),
                                                      crossingResult.getTargetDirection().getZ().taylor(deltaL)).normalize();

        // fix neighbouring pixels
        final AbsoluteDate fixedDate   = sensor.getDate(fixedLine);
        final Vector3D fixedX          = sensor.getLos(fixedDate, lowIndex);
        final Vector3D fixedZ          = Vector3D.crossProduct(fixedX, sensor.getLos(fixedDate, lowIndex + 1));
        final Vector3D fixedY          = Vector3D.crossProduct(fixedZ, fixedX);

        // fix pixel
        final double pixelWidth = FastMath.atan2(Vector3D.dotProduct(highLOS,        fixedY),
                                                 Vector3D.dotProduct(highLOS,        fixedX));
        final double alpha      = FastMath.atan2(Vector3D.dotProduct(fixedDirection, fixedY),
                                                 Vector3D.dotProduct(fixedDirection, fixedX));
        final double fixedPixel = lowIndex + alpha / pixelWidth;

        return new SensorPixel(fixedLine, fixedPixel);

    }

    /** Get transform from spacecraft to inertial frame.
     * @param date date of the transform
     * @return transform from spacecraft to inertial frame
     * @exception RuggedException if spacecraft position or attitude cannot be computed at date
     */
    public Transform getScToInertial(final AbsoluteDate date)
        throws RuggedException {
        return scToBody.getScToInertial(date);
    }

    /** Get transform from inertial frame to observed body frame.
     * @param date date of the transform
     * @return transform from inertial frame to observed body frame
     * @exception RuggedException if frames cannot be computed at date
     */
    public Transform getInertialToBody(final AbsoluteDate date)
        throws RuggedException {
        return scToBody.getInertialToBody(date);
    }

    /** Get transform from observed body frame to inertial frame.
     * @param date date of the transform
     * @return transform from observed body frame to inertial frame
     * @exception RuggedException if frames cannot be computed at date
     */
    public Transform getBodyToInertial(final AbsoluteDate date)
        throws RuggedException {
        return scToBody.getBodyToInertial(date);
    }

    /** Get a sensor.
     * @param sensorName sensor name
     * @return selected sensor
     * @exception RuggedException if sensor is not known
     */
    public LineSensor getLineSensor(final String sensorName) throws RuggedException {
        final LineSensor sensor = sensors.get(sensorName);
        if (sensor == null) {
            throw new RuggedException(RuggedMessages.UNKNOWN_SENSOR, sensorName);
        }
        return sensor;
    }

}
