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
package org.orekit.rugged.api;

import java.io.IOException;
import java.io.InputStream;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.OutputStream;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import org.hipparchus.exception.LocalizedCoreFormats;
import org.hipparchus.geometry.euclidean.threed.Rotation;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.orekit.bodies.OneAxisEllipsoid;
import org.orekit.frames.Frame;
import org.orekit.frames.FramesFactory;
import org.orekit.propagation.Propagator;
import org.orekit.rugged.errors.RuggedException;
import org.orekit.rugged.errors.RuggedInternalError;
import org.orekit.rugged.errors.RuggedMessages;
import org.orekit.rugged.intersection.BasicScanAlgorithm;
import org.orekit.rugged.intersection.ConstantElevationAlgorithm;
import org.orekit.rugged.intersection.IgnoreDEMAlgorithm;
import org.orekit.rugged.intersection.IntersectionAlgorithm;
import org.orekit.rugged.intersection.duvenhage.DuvenhageAlgorithm;
import org.orekit.rugged.linesensor.LineSensor;
import org.orekit.rugged.raster.TileUpdater;
import org.orekit.rugged.refraction.AtmosphericRefraction;
import org.orekit.rugged.utils.ExtendedEllipsoid;
import org.orekit.rugged.utils.SpacecraftToObservedBody;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.AngularDerivativesFilter;
import org.orekit.utils.CartesianDerivativesFilter;
import org.orekit.utils.Constants;
import org.orekit.utils.IERSConventions;
import org.orekit.utils.PVCoordinates;
import org.orekit.utils.TimeStampedAngularCoordinates;
import org.orekit.utils.TimeStampedPVCoordinates;

/** Builder for {@link Rugged} instances.
 * <p>
 * This class implements the <em>builder pattern</em> to create {@link Rugged} instances.
 * It does so by using a <em>fluent API</em> in order to clarify reading and allow
 * later extensions with new configuration parameters.
 * </p>
 * <p>
 * A typical use would be:
 * </p>
 * <pre>
 *   Rugged rugged = new RuggedBuilder().
 *                   setEllipsoid(EllipsoidId.WGS84, BodyRotatingFrameId.ITRF).
 *                   setAlgorithmID(AlgorithmId.Duvenhage).
 *                   setDigitalElevationModel(tileUpdater, maxCachedTiles).
 *                   setTimeSpan(minDate, maxDate, tStep, overshootTolerance).
 *                   setTrajectory(positionsVelocities, pvInterpolationNumber, pvFilter,
 *                                 quaternions, aInterpolationNumber, aFilter).
 *                   addLineSensor(sensor1).
 *                   addLineSensor(sensor2).
 *                   addLineSensor(sensor3).
 *                   build();
 * </pre>
 * <p>
 * If a configuration parameter has not been set prior to the call to {]link #build()}, then
 * an exception will be triggered with an explicit error message.
 * </p>
 * @see <a href="https://en.wikipedia.org/wiki/Builder_pattern">Builder pattern (wikipedia)</a>
 * @see <a href="https://en.wikipedia.org/wiki/Fluent_interface">Fluent interface (wikipedia)</a>
 * @author Luc Maisonobe
 * @author Guylaine Prat
 */
public class RuggedBuilder {

    /** Reference ellipsoid. */
    private ExtendedEllipsoid ellipsoid;

    /** DEM intersection algorithm. */
    private AlgorithmId algorithmID;

    /** Updater used to load Digital Elevation Model tiles. */
    private TileUpdater tileUpdater;
    
    /** Flag to tell if the Digital Elevation Model tiles are overlapping. 
     * @since X.x */
    private boolean isOvelappingTiles = true;

    /** Constant elevation over ellipsoid (m).
     * used only with {@link AlgorithmId#CONSTANT_ELEVATION_OVER_ELLIPSOID. */
    private double constantElevation;

    /** Maximum number of tiles stored in the cache. */
    private int maxCachedTiles;

    /** Start of search time span. */
    private AbsoluteDate minDate;

    /** End of search time span. */
    private AbsoluteDate maxDate;

    /** Step to use for inertial frame to body frame transforms cache computations (s). */
    private double tStep;

    /** OvershootTolerance tolerance in seconds allowed for {@code minDate} and {@code maxDate} overshooting (s). */
    private double overshootTolerance;

    /** Inertial frame for position/velocity/attitude. */
    private Frame inertial;

    /** Satellite position and velocity (m and m/s in inertial frame). */
    private List<TimeStampedPVCoordinates> pvSample;

    /** Number of points to use for position/velocity interpolation. */
    private int pvNeighborsSize;

    /** Filter for derivatives from the sample to use in position/velocity interpolation. */
    private CartesianDerivativesFilter pvDerivatives;

    /** Satellite quaternions with respect to inertial frame. */
    private List<TimeStampedAngularCoordinates> aSample;

    /** Number of points to use for attitude interpolation. */
    private int aNeighborsSize;

    /** Filter for derivatives from the sample to use in attitude interpolation. */
    private AngularDerivativesFilter aDerivatives;

    /** Propagator for position/velocity/attitude. */
    private Propagator pvaPropagator;

    /** Step to use for inertial/Earth/spacecraft transforms interpolations (s). */
    private double iStep;

    /** Number of points to use for inertial/Earth/spacecraft transforms interpolations. */
    private int iN;

    /** Converter between spacecraft and body. */
    private SpacecraftToObservedBody scToBody;

    /** Flag for light time correction. */
    private boolean lightTimeCorrection;

    /** Flag for aberration of light correction. */
    private boolean aberrationOfLightCorrection;

    /** Atmospheric refraction to use for line of sight correction. */
    private AtmosphericRefraction atmosphericRefraction;

    /** Sensors. */
    private final List<LineSensor> sensors;

    /** Rugged name. */
    private String name;

    /** Create a non-configured builder.
     * <p>
     * The builder <em>must</em> be configured before calling the
     * {@link #build} method, otherwise an exception will be triggered
     * at build time.
     * </p>
     */
    public RuggedBuilder() {
        sensors                     = new ArrayList<>();
        constantElevation           = Double.NaN;
        lightTimeCorrection         = true;
        aberrationOfLightCorrection = true;
        name                        = "Rugged";
    }

    /** Set the reference ellipsoid.
     * @param ellipsoidID reference ellipsoid
     * @param bodyRotatingFrameID body rotating frame identifier
     * from an earlier run and frames mismatch
     * @return the builder instance
     * @see #setEllipsoid(OneAxisEllipsoid)
     * @see #getEllipsoid()
     */
    public RuggedBuilder setEllipsoid(final EllipsoidId ellipsoidID, final BodyRotatingFrameId bodyRotatingFrameID) {
        return setEllipsoid(selectEllipsoid(ellipsoidID, selectBodyRotatingFrame(bodyRotatingFrameID)));
    }

    /** Set the reference ellipsoid.
     * @param newEllipsoid reference ellipsoid
     * @return the builder instance
     * @see #setEllipsoid(EllipsoidId, BodyRotatingFrameId)
     * @see #getEllipsoid()
     */
    public RuggedBuilder setEllipsoid(final OneAxisEllipsoid newEllipsoid) {
        this.ellipsoid = new ExtendedEllipsoid(newEllipsoid.getEquatorialRadius(),
                                               newEllipsoid.getFlattening(),
                                               newEllipsoid.getBodyFrame());
        checkFramesConsistency();
        return this;
    }

    /** Get the ellipsoid.
     * @return the ellipsoid
     * @see #setEllipsoid(EllipsoidId, BodyRotatingFrameId)
     * @see #setEllipsoid(OneAxisEllipsoid)
     */
    public ExtendedEllipsoid getEllipsoid() {
        return ellipsoid;
    }

    /** Get the Rugged name.
     * @return the Rugged name
     * @since 2.0
     */
    public String getName() {
        return name;
    }

    /** Set the Rugged name.
     * @param name the Rugged name
     * @since 2.0
     */
    public void setName(final String name) {
        this.name = name;
    }

    /** Set the algorithm to use for Digital Elevation Model intersection.
     * <p>
     * Note that some algorithms require specific other methods to be called too:
     * <ul>
     *   <li>{@link AlgorithmId#DUVENHAGE DUVENHAGE},
     *   {@link AlgorithmId#DUVENHAGE_FLAT_BODY DUVENHAGE_FLAT_BODY}
     *   and {@link AlgorithmId#BASIC_SLOW_EXHAUSTIVE_SCAN_FOR_TESTS_ONLY
     *   BASIC_SLOW_EXHAUSTIVE_SCAN_FOR_TESTS_ONLY} all
     *   require {@link #setDigitalElevationModel(TileUpdater, int) setDigitalElevationModel}
     *   to be called,</li>
     *   <li>{@link AlgorithmId#CONSTANT_ELEVATION_OVER_ELLIPSOID
     *   CONSTANT_ELEVATION_OVER_ELLIPSOID} requires
     *   {@link #setConstantElevation(double) setConstantElevation} to be called,</li>
     *   <li>{@link AlgorithmId#IGNORE_DEM_USE_ELLIPSOID
     *   IGNORE_DEM_USE_ELLIPSOID} does not require
     *   any methods tobe called.</li>
     * </ul>
     *
     * @param newAlgorithmId identifier of algorithm to use for Digital Elevation Model intersection
     * @return the builder instance
     * @see #setDigitalElevationModel(TileUpdater, int)
     * @see #getAlgorithm()
     */
    public RuggedBuilder setAlgorithm(final AlgorithmId newAlgorithmId) {
        this.algorithmID = newAlgorithmId;
        return this;
    }

    /** Get the algorithm to use for Digital Elevation Model intersection.
     * @return algorithm to use for Digital Elevation Model intersection
     * @see #setAlgorithm(AlgorithmId)
     */
    public AlgorithmId getAlgorithm() {
        return algorithmID;
    }

    /** Set the user-provided {@link TileUpdater tile updater}.
     * <p>
     * The DEM tiles must be overlapping, otherwise use {@link #setDigitalElevationModel(TileUpdater, int, boolean)}
     * with flag set to false.
     * </p>
     * <p>
     * Note that when the algorithm specified in {@link #setAlgorithm(AlgorithmId)}
     * is either {@link AlgorithmId#CONSTANT_ELEVATION_OVER_ELLIPSOID
     * CONSTANT_ELEVATION_OVER_ELLIPSOID} or {@link
     * AlgorithmId#IGNORE_DEM_USE_ELLIPSOID IGNORE_DEM_USE_ELLIPSOID},
     * then this method becomes irrelevant and can either be not called at all,
     * or it can be called with an updater set to {@code null}. For all other
     * algorithms, the updater must be properly configured.
     * </p>
     * @param newTileUpdater updater used to load Digital Elevation Model tiles
     * @param newMaxCachedTiles maximum number of tiles stored in the cache
     * @return the builder instance
     * @see #setAlgorithm(AlgorithmId)
     * @see #getTileUpdater()
     * @see #getMaxCachedTiles()
     */
    public RuggedBuilder setDigitalElevationModel(final TileUpdater newTileUpdater, final int newMaxCachedTiles) {
        return setDigitalElevationModel(newTileUpdater, newMaxCachedTiles, true);
    }

    /** Set the user-provided {@link TileUpdater tile updater}.
     * <p>
     * Note that when the algorithm specified in {@link #setAlgorithm(AlgorithmId)}
     * is either {@link AlgorithmId#CONSTANT_ELEVATION_OVER_ELLIPSOID
     * CONSTANT_ELEVATION_OVER_ELLIPSOID} or {@link
     * AlgorithmId#IGNORE_DEM_USE_ELLIPSOID IGNORE_DEM_USE_ELLIPSOID},
     * then this method becomes irrelevant and can either be not called at all,
     * or it can be called with an updater set to {@code null}. For all other
     * algorithms, the updater must be properly configured.
     * </p>
     * @param newTileUpdater updater used to load Digital Elevation Model tiles
     * @param newMaxCachedTiles maximum number of tiles stored in the cache
     * @param newIsOverlappingTiles flag to tell if the DEM tiles are overlapping: 
     *                              true if overlapping; false otherwise.
     * @return the builder instance
     * @see #setAlgorithm(AlgorithmId)
     * @see #getTileUpdater()
     * @see #getMaxCachedTiles()
     * @see #isOvelappingTiles()
     * @since X.x
     */
    public RuggedBuilder setDigitalElevationModel(final TileUpdater newTileUpdater, final int newMaxCachedTiles, 
                                                  final boolean newIsOverlappingTiles) {
        this.tileUpdater    = newTileUpdater;
        this.maxCachedTiles = newMaxCachedTiles;
        this.isOvelappingTiles = newIsOverlappingTiles;
        return this;
    }

    /** Get the updater used to load Digital Elevation Model tiles.
     * @return updater used to load Digital Elevation Model tiles
     * @see #setDigitalElevationModel(TileUpdater, int)
     * @see #getMaxCachedTiles()
     */
    public TileUpdater getTileUpdater() {
        return tileUpdater;
    }
    
    /**
     * Get the flag telling if the DEM tiles are overlapping. 
     * @return true if the Digital Elevation Model tiles are overlapping; 
     *         false otherwise. Default = true.
     * @since X.x
     */
    public boolean isOvelappingTiles() {
        return isOvelappingTiles;
    }

    /**
     * Set the DEM overlapping tiles flag
     * @param isOvelappingTiles flag to tell if the Digital Elevation Model tiles are overlapping: 
     *        true if overlapping; false otherwise
     * @since X.x
     */
    public void setOvelappingTiles(boolean isOvelappingTiles) {
        this.isOvelappingTiles = isOvelappingTiles;
    }

    /** Set the user-provided constant elevation model.
     * <p>
     * Note that this method is relevant <em>only</em> if the algorithm specified
     * in {@link #setAlgorithm(AlgorithmId)} is {@link
     * AlgorithmId#CONSTANT_ELEVATION_OVER_ELLIPSOID CONSTANT_ELEVATION_OVER_ELLIPSOID}.
     * If it is called for another algorithm, the elevation set here will be ignored.
     * </p>
     * @param newConstantElevation constant elevation to use (m)
     * @return the builder instance
     * @see #setAlgorithm(AlgorithmId)
     * @see #getConstantElevation()
     */
    public RuggedBuilder setConstantElevation(final double newConstantElevation) {
        this.constantElevation = newConstantElevation;
        return this;
    }

    /** Get the constant elevation over ellipsoid to use with {@link AlgorithmId#CONSTANT_ELEVATION_OVER_ELLIPSOID}.
     * @return updater used to load Digital Elevation Model tiles
     * @see #setConstantElevation(double)
     */
    public double getConstantElevation() {
        return constantElevation;
    }

    /** Get the maximum number of tiles stored in the cache.
     * @return maximum number of tiles stored in the cache
     * @see #setDigitalElevationModel(TileUpdater, int)
     * @see #getTileUpdater()
     */
    public int getMaxCachedTiles() {
        return maxCachedTiles;
    }

    /** Set the time span to be covered for direct and inverse location calls.
     * <p>
     * This method set only the time span and not the trajectory, therefore it
     * <em>must</em> be used together with either
     * {@link #setTrajectory(InertialFrameId, List, int, CartesianDerivativesFilter, List, int, AngularDerivativesFilter)},
     * {@link #setTrajectory(Frame, List, int, CartesianDerivativesFilter, List, int, AngularDerivativesFilter)},
     * or {@link #setTrajectory(double, int, CartesianDerivativesFilter, AngularDerivativesFilter, Propagator)}
     * but should <em>not</em> be mixed with {@link #setTrajectoryAndTimeSpan(InputStream)}.
     * </p>
     * @param newMinDate start of search time span
     * @param newMaxDate end of search time span
     * @param newTstep step to use for inertial frame to body frame transforms cache computations (s)
     * @param newOvershootTolerance tolerance in seconds allowed for {@code minDate} and {@code maxDate} overshooting (s)
     * @return the builder instance
     * @see #setTrajectoryAndTimeSpan(InputStream)
     * @see #getMinDate()
     * @see #getMaxDate()
     * @see #getTStep()
     * @see #getOvershootTolerance()
     */
    public RuggedBuilder setTimeSpan(final AbsoluteDate newMinDate, final AbsoluteDate newMaxDate,
                                     final double newTstep, final double newOvershootTolerance) {
        this.minDate            = newMinDate;
        this.maxDate            = newMaxDate;
        this.tStep              = newTstep;
        this.overshootTolerance = newOvershootTolerance;
        this.scToBody           = null;
        return this;
    }

    /** Get the start of search time span.
     * @return start of search time span
     * @see #setTimeSpan(AbsoluteDate, AbsoluteDate, double, double)
     */
    public AbsoluteDate getMinDate() {
        return minDate;
    }

    /** Get the end of search time span.
     * @return end of search time span
     * @see #setTimeSpan(AbsoluteDate, AbsoluteDate, double, double)
     */
    public AbsoluteDate getMaxDate() {
        return maxDate;
    }

    /** Get the step to use for inertial frame to body frame transforms cache computations.
     * @return step to use for inertial frame to body frame transforms cache computations
     * @see #setTimeSpan(AbsoluteDate, AbsoluteDate, double, double)
     */
    public double getTStep() {
        return tStep;
    }

    /** Get the tolerance in seconds allowed for {@link #getMinDate()} and {@link #getMaxDate()} overshooting.
     * @return tolerance in seconds allowed for {@link #getMinDate()} and {@link #getMaxDate()} overshooting
     * @see #setTimeSpan(AbsoluteDate, AbsoluteDate, double, double)
     */
    public double getOvershootTolerance() {
        return overshootTolerance;
    }

    /** Set the spacecraft trajectory.
     * <p>
     * This method set only the trajectory and not the time span, therefore it
     * <em>must</em> be used together with the {@link #setTimeSpan(AbsoluteDate, AbsoluteDate, double, double)}
     * but should <em>not</em> be mixed with {@link #setTrajectoryAndTimeSpan(InputStream)}.
     * </p>
     * @param inertialFrameId inertial frame Id used for spacecraft positions/velocities/quaternions
     * @param positionsVelocities satellite position and velocity (m and m/s in inertial frame)
     * @param pvInterpolationNumber number of points to use for position/velocity interpolation
     * @param pvFilter filter for derivatives from the sample to use in position/velocity interpolation
     * @param quaternions satellite quaternions with respect to inertial frame
     * @param aInterpolationNumber number of points to use for attitude interpolation
     * @param aFilter filter for derivatives from the sample to use in attitude interpolation
     * @return the builder instance
     * @see #setTrajectory(Frame, List, int, CartesianDerivativesFilter, List, int, AngularDerivativesFilter)
     * @see #setTrajectory(double, int, CartesianDerivativesFilter, AngularDerivativesFilter, Propagator)
     * @see #setTrajectoryAndTimeSpan(InputStream)
     * @see #getInertialFrame()
     * @see #getPositionsVelocities()
     * @see #getPVInterpolationNumber()
     * @see #getPVInterpolationNumber()
     * @see #getQuaternions()
     * @see #getAInterpolationNumber()
     * @see #getAFilter()
     */
    public RuggedBuilder setTrajectory(final InertialFrameId inertialFrameId,
                                       final List<TimeStampedPVCoordinates> positionsVelocities, final int pvInterpolationNumber,
                                       final CartesianDerivativesFilter pvFilter,
                                       final List<TimeStampedAngularCoordinates> quaternions, final int aInterpolationNumber,
                                       final AngularDerivativesFilter aFilter) {

        return setTrajectory(selectInertialFrame(inertialFrameId),
                             positionsVelocities, pvInterpolationNumber, pvFilter,
                             quaternions, aInterpolationNumber, aFilter);
    }

    /** Set the spacecraft trajectory.
     * <p>
     * This method set only the trajectory and not the time span, therefore it
     * <em>must</em> be used together with the {@link #setTimeSpan(AbsoluteDate, AbsoluteDate, double, double)}
     * but should <em>not</em> be mixed with {@link #setTrajectoryAndTimeSpan(InputStream)}.
     * </p>
     * @param inertialFrame inertial frame used for spacecraft positions/velocities/quaternions
     * @param positionsVelocities satellite position and velocity (m and m/s in inertial frame)
     * @param pvInterpolationNumber number of points to use for position/velocity interpolation
     * @param pvFilter filter for derivatives from the sample to use in position/velocity interpolation
     * @param quaternions satellite quaternions with respect to inertial frame
     * @param aInterpolationNumber number of points to use for attitude interpolation
     * @param aFilter filter for derivatives from the sample to use in attitude interpolation
     * @return the builder instance
     * @see #setTrajectory(InertialFrameId, List, int, CartesianDerivativesFilter, List, int, AngularDerivativesFilter)
     * @see #setTrajectory(double, int, CartesianDerivativesFilter, AngularDerivativesFilter, Propagator)
     * @see #setTrajectoryAndTimeSpan(InputStream)
     * @see #getPositionsVelocities()
     * @see #getPVInterpolationNumber()
     * @see #getPVInterpolationNumber()
     * @see #getQuaternions()
     * @see #getAInterpolationNumber()
     * @see #getAFilter()
     */
    public RuggedBuilder setTrajectory(final Frame inertialFrame,
                                       final List<TimeStampedPVCoordinates> positionsVelocities, final int pvInterpolationNumber,
                                       final CartesianDerivativesFilter pvFilter,
                                       final List<TimeStampedAngularCoordinates> quaternions, final int aInterpolationNumber,
                                       final AngularDerivativesFilter aFilter) {

        this.inertial        = inertialFrame;
        this.pvSample        = positionsVelocities;
        this.pvNeighborsSize = pvInterpolationNumber;
        this.pvDerivatives   = pvFilter;
        this.aSample         = quaternions;
        this.aNeighborsSize  = aInterpolationNumber;
        this.aDerivatives    = aFilter;
        this.pvaPropagator   = null;
        this.iStep           = Double.NaN;
        this.iN              = -1;
        this.scToBody        = null;
        return this;
    }

    /** Set the spacecraft trajectory.
     * <p>
     * This method set only the trajectory and not the time span, therefore it
     * <em>must</em> be used together with the {@link #setTimeSpan(AbsoluteDate, AbsoluteDate, double, double)}
     * but should <em>not</em> be mixed with {@link #setTrajectoryAndTimeSpan(InputStream)}.
     * </p>
     * @param interpolationStep step to use for inertial/Earth/spacecraft transforms interpolations (s)
     * @param interpolationNumber number of points to use for inertial/Earth/spacecraft transforms interpolations
     * @param pvFilter filter for derivatives from the sample to use in position/velocity interpolation
     * @param aFilter filter for derivatives from the sample to use in attitude interpolation
     * @param propagator global propagator
     * @return the builder instance
     * @see #setTrajectory(InertialFrameId, List, int, CartesianDerivativesFilter, List, int, AngularDerivativesFilter)
     * @see #setTrajectory(Frame, List, int, CartesianDerivativesFilter, List, int, AngularDerivativesFilter)
     * @see #setTrajectoryAndTimeSpan(InputStream)
     */
    public RuggedBuilder setTrajectory(final double interpolationStep, final int interpolationNumber,
                                       final CartesianDerivativesFilter pvFilter, final AngularDerivativesFilter aFilter,
                                       final Propagator propagator) {
        this.inertial        = propagator.getFrame();
        this.pvSample        = null;
        this.pvNeighborsSize = -1;
        this.pvDerivatives   = pvFilter;
        this.aSample         = null;
        this.aNeighborsSize  = -1;
        this.aDerivatives    = aFilter;
        this.pvaPropagator   = propagator;
        this.iStep           = interpolationStep;
        this.iN              = interpolationNumber;
        this.scToBody        = null;
        return this;
    }

    /** Get the inertial frame.
     * @return inertial frame
     */
    public Frame getInertialFrame() {
        return inertial;
    }

    /** Get the satellite position and velocity (m and m/s in inertial frame).
     * @return satellite position and velocity (m and m/s in inertial frame)
     * @see #setTrajectory(double, int, CartesianDerivativesFilter, AngularDerivativesFilter, Propagator)
     */
    public List<TimeStampedPVCoordinates> getPositionsVelocities() {
        return pvSample;
    }

    /** Get the number of points to use for position/velocity interpolation.
     * @return number of points to use for position/velocity interpolation
     * @see #setTrajectory(double, int, CartesianDerivativesFilter, AngularDerivativesFilter, Propagator)
     */
    public int getPVInterpolationNumber() {
        return pvNeighborsSize;
    }

    /** Get the filter for derivatives from the sample to use in position/velocity interpolation.
     * @return filter for derivatives from the sample to use in position/velocity interpolation
     * @see #setTrajectory(double, int, CartesianDerivativesFilter, AngularDerivativesFilter, Propagator)
     */
    public CartesianDerivativesFilter getPVFilter() {
        return pvDerivatives;
    }

    /** Get the satellite quaternions with respect to inertial frame.
     * @return satellite quaternions with respect to inertial frame
     * @see #setTrajectory(double, int, CartesianDerivativesFilter, AngularDerivativesFilter, Propagator)
     */
    public List<TimeStampedAngularCoordinates> getQuaternions() {
        return aSample;
    }

    /** Get the number of points to use for attitude interpolation.
     * @return number of points to use for attitude interpolation
     * @see #setTrajectory(double, int, CartesianDerivativesFilter, AngularDerivativesFilter, Propagator)
     */
    public int getAInterpolationNumber() {
        return aNeighborsSize;
    }

    /** Get the filter for derivatives from the sample to use in attitude interpolation.
     * @return filter for derivatives from the sample to use in attitude interpolation
     * @see #setTrajectory(double, int, CartesianDerivativesFilter, AngularDerivativesFilter, Propagator)
     */
    public AngularDerivativesFilter getAFilter() {
        return aDerivatives;
    }

    /** Set both the spacecraft trajectory and the time span.
     * <p>
     * This method set both the trajectory and the time span in a tightly coupled
     * way, therefore it should <em>not</em> be mixed with the individual methods
     * {@link #setTrajectory(double, int, CartesianDerivativesFilter, AngularDerivativesFilter, Propagator)},
     * {@link #setTrajectory(Frame, List, int, CartesianDerivativesFilter, List, int, AngularDerivativesFilter)},
     * {@link #setTrajectory(InertialFrameId, List, int, CartesianDerivativesFilter, List, int, AngularDerivativesFilter)},
     * or {@link #setTimeSpan(AbsoluteDate, AbsoluteDate, double, double)}.
     * </p>
     * @param storageStream stream from where to read previous instance {@link #storeInterpolator(OutputStream)
     * stored interpolator} (caller opened it and remains responsible for closing it)
     * @return the builder instance
     * @see #setTrajectory(InertialFrameId, List, int, CartesianDerivativesFilter, List, int, AngularDerivativesFilter)
     * @see #setTrajectory(Frame, List, int, CartesianDerivativesFilter, List, int, AngularDerivativesFilter)
     * @see #setTrajectory(double, int, CartesianDerivativesFilter, AngularDerivativesFilter, Propagator)
     * @see #storeInterpolator(OutputStream)
     */
    public RuggedBuilder setTrajectoryAndTimeSpan(final InputStream storageStream) {

        try {
            this.inertial           = null;
            this.pvSample           = null;
            this.pvNeighborsSize    = -1;
            this.pvDerivatives      = null;
            this.aSample            = null;
            this.aNeighborsSize     = -1;
            this.aDerivatives       = null;
            this.pvaPropagator      = null;
            this.iStep              = Double.NaN;
            this.iN                 = -1;
            this.scToBody           = (SpacecraftToObservedBody) new ObjectInputStream(storageStream).readObject();
            this.minDate            = scToBody.getMinDate();
            this.maxDate            = scToBody.getMaxDate();
            this.tStep              = scToBody.getTStep();
            this.overshootTolerance = scToBody.getOvershootTolerance();
            checkFramesConsistency();
            return this;

        } catch (ClassNotFoundException cnfe) {
            throw new RuggedException(cnfe, RuggedMessages.NOT_INTERPOLATOR_DUMP_DATA);
        } catch (ClassCastException cce) {
            throw new RuggedException(cce, RuggedMessages.NOT_INTERPOLATOR_DUMP_DATA);
        } catch (IOException ioe) {
            throw new RuggedException(ioe, RuggedMessages.NOT_INTERPOLATOR_DUMP_DATA);
        }
    }

    /** Store frames transform interpolator.
     * <p>
     * This method allows to reuse the interpolator built in one instance, to build
     * another instance by calling {@link #setTrajectoryAndTimeSpan(InputStream)}.
     * This reduces the builder initialization time as setting up the interpolator can be long, it is
     * mainly intended to be used when several runs are done (for example in an image processing chain)
     * with the same configuration.
     * </p>
     * <p>
     * This method must be called <em>after</em> both the ellipsoid and trajectory have been set.
     * </p>
     * @param storageStream stream where to store the interpolator
     * (caller opened it and remains responsible for closing it)
     * @see #setEllipsoid(EllipsoidId, BodyRotatingFrameId)
     * @see #setEllipsoid(OneAxisEllipsoid)
     * @see #setTrajectory(InertialFrameId, List, int, CartesianDerivativesFilter, List, int, AngularDerivativesFilter)
     * @see #setTrajectory(Frame, List, int, CartesianDerivativesFilter, List, int, AngularDerivativesFilter)
     * @see #setTrajectory(double, int, CartesianDerivativesFilter, AngularDerivativesFilter, Propagator)
     * @see #setTrajectoryAndTimeSpan(InputStream)
     */
    public void storeInterpolator(final OutputStream storageStream) {
        try {
            createInterpolatorIfNeeded();
            new ObjectOutputStream(storageStream).writeObject(scToBody);
        } catch (IOException ioe) {
            throw new RuggedException(ioe, LocalizedCoreFormats.SIMPLE_MESSAGE, ioe.getMessage());
        }
    }

    /** Check frames consistency.
     */
    private void checkFramesConsistency() {
        if (ellipsoid != null && scToBody != null &&
            !ellipsoid.getBodyFrame().getName().equals(scToBody.getBodyFrame().getName())) {
            // if frames have been set both by direct calls and by deserializing an interpolator dump and a mismatch occurs
            throw new RuggedException(RuggedMessages.FRAMES_MISMATCH_WITH_INTERPOLATOR_DUMP,
                                      ellipsoid.getBodyFrame().getName(), scToBody.getBodyFrame().getName());
        }
    }

    /** Create a transform interpolator if needed.
     */
    private void createInterpolatorIfNeeded() {

        if (ellipsoid == null) {
            throw new RuggedException(RuggedMessages.UNINITIALIZED_CONTEXT, "RuggedBuilder.setEllipsoid()");
        }

        if (scToBody == null) {
            if (pvSample != null) {
                scToBody = createInterpolator(inertial, ellipsoid.getBodyFrame(),
                                              minDate, maxDate, tStep, overshootTolerance,
                                              pvSample, pvNeighborsSize, pvDerivatives,
                                              aSample, aNeighborsSize, aDerivatives);
            } else if (pvaPropagator != null) {
                scToBody = createInterpolator(inertial, ellipsoid.getBodyFrame(),
                                              minDate, maxDate, tStep, overshootTolerance,
                                              iStep, iN, pvDerivatives, aDerivatives, pvaPropagator);
            } else {
                throw new RuggedException(RuggedMessages.UNINITIALIZED_CONTEXT, "RuggedBuilder.setTrajectory()");
            }
        }
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
     */
    private static SpacecraftToObservedBody createInterpolator(final Frame inertialFrame, final Frame bodyFrame,
                                                               final AbsoluteDate minDate, final AbsoluteDate maxDate,
                                                               final double tStep, final double overshootTolerance,
                                                               final List<TimeStampedPVCoordinates> positionsVelocities,
                                                               final int pvInterpolationNumber,
                                                               final CartesianDerivativesFilter pvFilter,
                                                               final List<TimeStampedAngularCoordinates> quaternions,
                                                               final int aInterpolationNumber,
                                                               final AngularDerivativesFilter aFilter) {

        return new SpacecraftToObservedBody(inertialFrame, bodyFrame,
                                            minDate, maxDate, tStep, overshootTolerance,
                                            positionsVelocities, pvInterpolationNumber,
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
     * @param interpolationStep step to use for inertial/Earth/spacecraft transforms interpolations
     * @param interpolationNumber number of points of to use for inertial/Earth/spacecraft transforms interpolations
     * @param pvFilter filter for derivatives from the sample to use in position/velocity interpolation
     * @param aFilter filter for derivatives from the sample to use in attitude interpolation
     * @param propagator global propagator
     * @return transforms interpolator
     */
    private static SpacecraftToObservedBody createInterpolator(final Frame inertialFrame, final Frame bodyFrame,
                                                               final AbsoluteDate minDate, final AbsoluteDate maxDate,
                                                               final double tStep, final double overshootTolerance,
                                                               final double interpolationStep, final int interpolationNumber,
                                                               final CartesianDerivativesFilter pvFilter,
                                                               final AngularDerivativesFilter aFilter,
                                                               final Propagator propagator) {

        // extract position/attitude samples from propagator
        final List<TimeStampedPVCoordinates> positionsVelocities =
                new ArrayList<>();
        final List<TimeStampedAngularCoordinates> quaternions =
                new ArrayList<>();
        propagator.getMultiplexer().add(interpolationStep,
            currentState -> {
                final AbsoluteDate  date = currentState.getDate();
                final PVCoordinates pv   = currentState.getPVCoordinates(inertialFrame);
                final Rotation      q    = currentState.getAttitude().getRotation();
                positionsVelocities.add(new TimeStampedPVCoordinates(date, pv.getPosition(), pv.getVelocity(), Vector3D.ZERO));
                quaternions.add(new TimeStampedAngularCoordinates(date, q, Vector3D.ZERO, Vector3D.ZERO));
            });
        propagator.propagate(minDate.shiftedBy(-interpolationStep), maxDate.shiftedBy(interpolationStep));

        // orbit/attitude to body converter
        return createInterpolator(inertialFrame, bodyFrame,
                minDate, maxDate, tStep, overshootTolerance,
                positionsVelocities, interpolationNumber,
                pvFilter, quaternions, interpolationNumber,
                aFilter);
    }

    /** Set flag for light time correction.
     * <p>
     * This methods set the flag for compensating or not light time between
     * ground and spacecraft. Compensating this delay improves location
     * accuracy and is <em>enabled</em> by default (i.e. not calling this
     * method before building is therefore equivalent to calling it with
     * a parameter set to {@code true}). Not compensating it is mainly useful
     * for validation purposes against system that do not compensate it.
     * </p>
     * @param newLightTimeCorrection if true, the light travel time between ground
     * and spacecraft is compensated for more accurate location
     * @return the builder instance
     * @see #setAberrationOfLightCorrection(boolean)
     * @see #getLightTimeCorrection()
     */
    public RuggedBuilder setLightTimeCorrection(final boolean newLightTimeCorrection) {
        this.lightTimeCorrection = newLightTimeCorrection;
        return this;
    }

    /** Get the light time correction flag.
     * @return light time correction flag
     * @see #setLightTimeCorrection(boolean)
     */
    public boolean getLightTimeCorrection() {
        return lightTimeCorrection;
    }

    /** Set flag for aberration of light correction.
     * <p>
     * This methods set the flag for compensating or not aberration of light,
     * which is velocity composition between light and spacecraft when the
     * light from ground points reaches the sensor.
     * Compensating this velocity composition improves location
     * accuracy and is <em>enabled</em> by default (i.e. not calling this
     * method before building is therefore equivalent to calling it with
     * a parameter set to {@code true}). Not compensating it is useful
     * in two cases: for validation purposes against system that do not
     * compensate it or when the pixels line of sight already include the
     * correction.
     * </p>
     * @param newAberrationOfLightCorrection if true, the aberration of light
     * is corrected for more accurate location
     * @return the builder instance
     * @see #setLightTimeCorrection(boolean)
     * @see #getAberrationOfLightCorrection()
     */
    public RuggedBuilder setAberrationOfLightCorrection(final boolean newAberrationOfLightCorrection) {
        this.aberrationOfLightCorrection = newAberrationOfLightCorrection;
        return this;
    }

    /** Get the aberration of light correction flag.
     * @return aberration of light correction flag
     * @see #setAberrationOfLightCorrection(boolean)
     */
    public boolean getAberrationOfLightCorrection() {
        return aberrationOfLightCorrection;
    }

    /** Set atmospheric refraction for line of sight correction.
     * <p>
     * This method sets an atmospheric refraction model to be used between
     * spacecraft and ground for the correction of intersected points on ground.
     * Compensating for the effect of atmospheric refraction improves location
     * accuracy.
     * </p>
     * @param newAtmosphericRefraction the atmospheric refraction model to be used for more accurate location
     * @return the builder instance
     * @see #getRefractionCorrection()
     */
    public RuggedBuilder setRefractionCorrection(final AtmosphericRefraction newAtmosphericRefraction) {
        this.atmosphericRefraction = newAtmosphericRefraction;
        return this;
    }

    /** Get the atmospheric refraction model.
     * @return atmospheric refraction model
     * @see #setRefractionCorrection(AtmosphericRefraction)
     */
    public AtmosphericRefraction getRefractionCorrection() {
        return atmosphericRefraction;
    }

    /** Set up line sensor model.
     * @param lineSensor line sensor model
     * @return the builder instance
     */
    public RuggedBuilder addLineSensor(final LineSensor lineSensor) {
        sensors.add(lineSensor);
        return this;
    }

    /** Remove all line sensors.
     * @return the builder instance
     */
    public RuggedBuilder clearLineSensors() {
        sensors.clear();
        return this;
    }

    /** Get all line sensors.
     * @return all line sensors (in an unmodifiable list)
     */
    public List<LineSensor> getLineSensors() {
        return Collections.unmodifiableList(sensors);
    }

    /** Select inertial frame.
     * @param inertialFrameId inertial frame identifier
     * @return inertial frame
     */
    private static Frame selectInertialFrame(final InertialFrameId inertialFrameId) {

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
                throw new RuggedInternalError(null);
        }
    }

    /** Select body rotating frame.
     * @param bodyRotatingFrame body rotating frame identifier
     * @return body rotating frame
     */
    private static Frame selectBodyRotatingFrame(final BodyRotatingFrameId bodyRotatingFrame) {

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
                throw new RuggedInternalError(null);
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
                return new OneAxisEllipsoid(Constants.GRS80_EARTH_EQUATORIAL_RADIUS,
                                            Constants.GRS80_EARTH_FLATTENING,
                                            bodyFrame);
            case WGS84 :
                return new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                            Constants.WGS84_EARTH_FLATTENING,
                                            bodyFrame);
            case IERS96 :
                return new OneAxisEllipsoid(Constants.IERS96_EARTH_EQUATORIAL_RADIUS,
                                            Constants.IERS96_EARTH_FLATTENING,
                                            bodyFrame);
            case IERS2003 :
                return new OneAxisEllipsoid(Constants.IERS2003_EARTH_EQUATORIAL_RADIUS,
                                            Constants.IERS2003_EARTH_FLATTENING,
                                            bodyFrame);
            default :
                // this should never happen
                throw new RuggedInternalError(null);
        }

    }

    /** Create DEM intersection algorithm.
     * @param algorithmID intersection algorithm identifier
     * @param updater updater used to load Digital Elevation Model tiles
     * @param maxCachedTiles maximum number of tiles stored in the cache
     * @param constantElevation constant elevation over ellipsoid
     * @return selected algorithm
     */
    private static IntersectionAlgorithm createAlgorithm(final AlgorithmId algorithmID,
                                                         final TileUpdater updater, final int maxCachedTiles,
                                                         final double constantElevation, final boolean isOverlappingTiles) {
        // set up the algorithm
        switch (algorithmID) {
            case DUVENHAGE :
                return new DuvenhageAlgorithm(updater, maxCachedTiles, false, isOverlappingTiles);
            case DUVENHAGE_FLAT_BODY :
                return new DuvenhageAlgorithm(updater, maxCachedTiles, true, isOverlappingTiles);
            case BASIC_SLOW_EXHAUSTIVE_SCAN_FOR_TESTS_ONLY :
                return new BasicScanAlgorithm(updater, maxCachedTiles, isOverlappingTiles);
            case CONSTANT_ELEVATION_OVER_ELLIPSOID :
                return new ConstantElevationAlgorithm(constantElevation);
            case IGNORE_DEM_USE_ELLIPSOID :
                return new IgnoreDEMAlgorithm();
            default :
                // this should never happen
                throw new RuggedInternalError(null);
        }
    }

    /** Build a {@link Rugged} instance.
     * @return built instance
     */
    public Rugged build() {

        if (algorithmID == null) {
            throw new RuggedException(RuggedMessages.UNINITIALIZED_CONTEXT, "RuggedBuilder.setAlgorithmID()");
        }
        if (algorithmID == AlgorithmId.CONSTANT_ELEVATION_OVER_ELLIPSOID) {
            if (Double.isNaN(constantElevation)) {
                throw new RuggedException(RuggedMessages.UNINITIALIZED_CONTEXT, "RuggedBuilder.setConstantElevation()");
            }
        } else if (algorithmID != AlgorithmId.IGNORE_DEM_USE_ELLIPSOID) {
            if (tileUpdater == null) {
                throw new RuggedException(RuggedMessages.UNINITIALIZED_CONTEXT, "RuggedBuilder.setDigitalElevationModel()");
            }
        }
        createInterpolatorIfNeeded();
        return new Rugged(createAlgorithm(algorithmID, tileUpdater, maxCachedTiles, constantElevation, isOvelappingTiles), ellipsoid,
                          lightTimeCorrection, aberrationOfLightCorrection, atmosphericRefraction, scToBody, sensors, name);
    }
}
