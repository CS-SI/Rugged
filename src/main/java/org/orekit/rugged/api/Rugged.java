/* Copyright 2013-2016 CS Systèmes d'Information
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
import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.hipparchus.analysis.differentiation.DerivativeStructure;
import org.hipparchus.geometry.euclidean.threed.FieldVector3D;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.linear.Array2DRowRealMatrix;
import org.hipparchus.linear.ArrayRealVector;
import org.hipparchus.linear.RealMatrix;
import org.hipparchus.linear.RealVector;
import org.hipparchus.optim.ConvergenceChecker;
import org.hipparchus.optim.nonlinear.vector.leastsquares.LeastSquaresBuilder;
import org.hipparchus.optim.nonlinear.vector.leastsquares.LeastSquaresOptimizer;
import org.hipparchus.optim.nonlinear.vector.leastsquares.LeastSquaresOptimizer.Optimum;
import org.hipparchus.optim.nonlinear.vector.leastsquares.LeastSquaresProblem;
import org.hipparchus.optim.nonlinear.vector.leastsquares.LevenbergMarquardtOptimizer;
import org.hipparchus.optim.nonlinear.vector.leastsquares.GaussNewtonOptimizer;
import org.hipparchus.optim.nonlinear.vector.leastsquares.MultivariateJacobianFunction;
import org.hipparchus.optim.nonlinear.vector.leastsquares.ParameterValidator;
import org.hipparchus.util.FastMath;
import org.hipparchus.util.Pair;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.errors.OrekitException;
import org.orekit.errors.OrekitExceptionWrapper;
import org.orekit.frames.Transform;
import org.orekit.rugged.errors.DumpManager;
import org.orekit.rugged.errors.RuggedException;
import org.orekit.rugged.errors.RuggedExceptionWrapper;
import org.orekit.rugged.errors.RuggedMessages;
import org.orekit.rugged.intersection.IntersectionAlgorithm;
import org.orekit.rugged.linesensor.LineSensor;
import org.orekit.rugged.linesensor.SensorMeanPlaneCrossing;
import org.orekit.rugged.linesensor.SensorPixel;
import org.orekit.rugged.linesensor.SensorPixelCrossing;
import org.orekit.rugged.utils.DSGenerator;
import org.orekit.rugged.utils.ExtendedEllipsoid;
import org.orekit.rugged.utils.NormalizedGeodeticPoint;
import org.orekit.rugged.utils.SpacecraftToObservedBody;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.Constants;
import org.orekit.utils.PVCoordinates;
import org.orekit.utils.ParameterDriver;

/** Main class of Rugged library API.
 * @see RuggedBuilder
 * @author Luc Maisonobe
 * @author Lucie LabatAllee
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

    /** Margin used in parameters estimation for the inverse location lines range. */
    private static final int ESTIMATION_LINE_RANGE_MARGIN = 100;

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
     * @param algorithm algorithm to use for Digital Elevation Model intersection
     * @param ellipsoid f reference ellipsoid
     * @param lightTimeCorrection if true, the light travel time between ground
     * @param aberrationOfLightCorrection if true, the aberration of light
     * is corrected for more accurate location
     * and spacecraft is compensated for more accurate location
     * @param scToBody transforms interpolator
     * @param sensors sensors
     */
    Rugged(final IntersectionAlgorithm algorithm, final ExtendedEllipsoid ellipsoid,
           final boolean lightTimeCorrection, final boolean aberrationOfLightCorrection,
           final SpacecraftToObservedBody scToBody, final Collection<LineSensor> sensors) {

        // space reference
        this.ellipsoid = ellipsoid;

        // orbit/attitude to body converter
        this.scToBody = scToBody;

        // intersection algorithm
        this.algorithm = algorithm;

        this.sensors = new HashMap<String, LineSensor>();
        for (final LineSensor s : sensors) {
            this.sensors.put(s.getName(), s);
        }
        this.finders = new HashMap<String, SensorMeanPlaneCrossing>();

        this.lightTimeCorrection         = lightTimeCorrection;
        this.aberrationOfLightCorrection = aberrationOfLightCorrection;

    }

    /** Get the DEM intersection algorithm.
     * @return DEM intersection algorithm
     */
    public IntersectionAlgorithm getAlgorithm() {
        return algorithm;
    }

    /** Get flag for light time correction.
     * @return true if the light time between ground and spacecraft is
     * compensated for more accurate location
     */
    public boolean isLightTimeCorrected() {
        return lightTimeCorrection;
    }

    /** Get flag for aberration of light correction.
     * @return true if the aberration of light time is corrected
     * for more accurate location
     */
    public boolean isAberrationOfLightCorrected() {
        return aberrationOfLightCorrection;
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

    /** Get the observed body ellipsoid.
     * @return observed body ellipsoid
     */
    public ExtendedEllipsoid getEllipsoid() {
        return ellipsoid;
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

            DumpManager.dumpDirectLocation(date, sensor.getPosition(), sensor.getLOS(date, i),
                                           lightTimeCorrection, aberrationOfLightCorrection);

            final Vector3D obsLInert = scToInert.transformVector(sensor.getLOS(date, i));
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
                final Vector3D  sL       = approximate.transformVector(sensor.getLOS(date, i));
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

            DumpManager.dumpDirectLocationResult(gp[i]);

        }

        return gp;

    }

    /** Direct location of a single line-of-sight.
     * @param date date of the location
     * @param position pixel position in spacecraft frame
     * @param los normalized line-of-sight in spacecraft frame
     * @return ground position of intersection point between specified los and ground
     * @exception RuggedException if line cannot be localized, or sensor is unknown
     */
    public GeodeticPoint directLocation(final AbsoluteDate date, final Vector3D position, final Vector3D los)
        throws RuggedException {

        DumpManager.dumpDirectLocation(date, position, los, lightTimeCorrection, aberrationOfLightCorrection);

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

        final NormalizedGeodeticPoint result;
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
            result = algorithm.refineIntersection(ellipsoid,
                                                  shifted2.transformPosition(pInert),
                                                  shifted2.transformVector(lInert),
                                                  gp1);

        } else {
            // compute DEM intersection without light time correction
            final Vector3D pBody = inertToBody.transformPosition(pInert);
            final Vector3D lBody = inertToBody.transformVector(lInert);
            result = algorithm.refineIntersection(ellipsoid, pBody, lBody,
                                                  algorithm.intersection(ellipsoid, pBody, lBody));
        }

        DumpManager.dumpDirectLocationResult(result);
        return result;

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
     * <p>
     * Note that for each sensor name, the {@code minLine} and {@code maxLine} settings
     * are cached, because they induce costly frames computation. So these settings
     * should not be tuned very finely and changed at each call, but should rather be
     * a few thousand lines wide and refreshed only when needed. If for example an
     * inverse location is roughly estimated to occur near line 53764 (for example
     * using {@link org.orekit.rugged.utils.RoughVisibilityEstimator}), {@code minLine}
     * and {@code maxLine} could be set for example to 50000 and 60000, which would
     * be OK also if next line inverse location is expected to occur near line 53780,
     * and next one ... The setting could be changed for example to 55000 and 65000 when
     * an inverse location is expected to occur after 55750. Of course, these values
     * are only an example and should be adjusted depending on mission needs.
     * </p>
     * @param sensorName name of the line  sensor
     * @param latitude ground point latitude
     * @param longitude ground point longitude
     * @param minLine minimum line number
     * @param maxLine maximum line number
     * @return date at which ground point is seen by line sensor
     * @exception RuggedException if line cannot be localized, or sensor is unknown
     * @see #inverseLocation(String, double, double, int, int)
     * @see org.orekit.rugged.utils.RoughVisibilityEstimator
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
     * <p>
     * Note that for each sensor name, the {@code minLine} and {@code maxLine} settings
     * are cached, because they induce costly frames computation. So these settings
     * should not be tuned very finely and changed at each call, but should rather be
     * a few thousand lines wide and refreshed only when needed. If for example an
     * inverse location is roughly estimated to occur near line 53764 (for example
     * using {@link org.orekit.rugged.utils.RoughVisibilityEstimator}), {@code minLine}
     * and {@code maxLine} could be set for example to 50000 and 60000, which would
     * be OK also if next line inverse location is expected to occur near line 53780,
     * and next one ... The setting could be changed for example to 55000 and 65000 when
     * an inverse location is expected to occur after 55750. Of course, these values
     * are only an example and should be adjusted depending on mission needs.
     * </p>
     * @param sensorName name of the line  sensor
     * @param point point to localize
     * @param minLine minimum line number
     * @param maxLine maximum line number
     * @return date at which ground point is seen by line sensor
     * @exception RuggedException if line cannot be localized, or sensor is unknown
     * @see #inverseLocation(String, GeodeticPoint, int, int)
     * @see org.orekit.rugged.utils.RoughVisibilityEstimator
     */
    public AbsoluteDate dateLocation(final String sensorName, final GeodeticPoint point,
                                     final int minLine, final int maxLine)
        throws RuggedException {

        final LineSensor sensor = getLineSensor(sensorName);
        final SensorMeanPlaneCrossing planeCrossing = getPlaneCrossing(sensorName, minLine, maxLine);

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
     * <p>
     * Note that for each sensor name, the {@code minLine} and {@code maxLine} settings
     * are cached, because they induce costly frames computation. So these settings
     * should not be tuned very finely and changed at each call, but should rather be
     * a few thousand lines wide and refreshed only when needed. If for example an
     * inverse location is roughly estimated to occur near line 53764 (for example
     * using {@link org.orekit.rugged.utils.RoughVisibilityEstimator}), {@code minLine}
     * and {@code maxLine} could be set for example to 50000 and 60000, which would
     * be OK also if next line inverse location is expected to occur near line 53780,
     * and next one ... The setting could be changed for example to 55000 and 65000 when
     * an inverse location is expected to occur after 55750. Of course, these values
     * are only an example and should be adjusted depending on mission needs.
     * </p>
     * @param sensorName name of the line  sensor
     * @param latitude ground point latitude
     * @param longitude ground point longitude
     * @param minLine minimum line number
     * @param maxLine maximum line number
     * @return sensor pixel seeing ground point, or null if ground point cannot
     * be seen between the prescribed line numbers
     * @exception RuggedException if line cannot be localized, or sensor is unknown
     * @see org.orekit.rugged.utils.RoughVisibilityEstimator
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
     * <p>
     * Note that for each sensor name, the {@code minLine} and {@code maxLine} settings
     * are cached, because they induce costly frames computation. So these settings
     * should not be tuned very finely and changed at each call, but should rather be
     * a few thousand lines wide and refreshed only when needed. If for example an
     * inverse location is roughly estimated to occur near line 53764 (for example
     * using {@link org.orekit.rugged.utils.RoughVisibilityEstimator}), {@code minLine}
     * and {@code maxLine} could be set for example to 50000 and 60000, which would
     * be OK also if next line inverse location is expected to occur near line 53780,
     * and next one ... The setting could be changed for example to 55000 and 65000 when
     * an inverse location is expected to occur after 55750. Of course, these values
     * are only an example and should be adjusted depending on mission needs.
     * </p>
     * @param sensorName name of the line  sensor
     * @param point point to localize
     * @param minLine minimum line number
     * @param maxLine maximum line number
     * @return sensor pixel seeing point, or null if point cannot be seen between the
     * prescribed line numbers
     * @exception RuggedException if line cannot be localized, or sensor is unknown
     * @see #dateLocation(String, GeodeticPoint, int, int)
     * @see org.orekit.rugged.utils.RoughVisibilityEstimator
     */
    public SensorPixel inverseLocation(final String sensorName, final GeodeticPoint point,
                                       final int minLine, final int maxLine)
        throws RuggedException {

        final LineSensor sensor = getLineSensor(sensorName);
        DumpManager.dumpInverseLocation(sensor, point, minLine, maxLine,
                                        lightTimeCorrection, aberrationOfLightCorrection);

        final SensorMeanPlaneCrossing planeCrossing = getPlaneCrossing(sensorName, minLine, maxLine);

        DumpManager.dumpSensorMeanPlane(planeCrossing);

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
                                        crossingResult.getTargetDirection(),
                                        MAX_EVAL, COARSE_INVERSE_LOCATION_ACCURACY);
        final double coarsePixel = pixelCrossing.locatePixel(crossingResult.getDate());
        if (Double.isNaN(coarsePixel)) {
            // target is out of search interval
            return null;
        }

        // fix line by considering the closest pixel exact position and line-of-sight
        // (this pixel might point towards a direction slightly above or below the mean sensor plane)
        final int      lowIndex       = FastMath.max(0, FastMath.min(sensor.getNbPixels() - 2, (int) FastMath.floor(coarsePixel)));
        final Vector3D lowLOS         = sensor.getLOS(crossingResult.getDate(), lowIndex);
        final Vector3D highLOS        = sensor.getLOS(crossingResult.getDate(), lowIndex + 1);
        final Vector3D localZ         = Vector3D.crossProduct(lowLOS, highLOS).normalize();
        final double   beta           = FastMath.acos(Vector3D.dotProduct(crossingResult.getTargetDirection(),
                                                                          localZ));
        final double   s              = Vector3D.dotProduct(crossingResult.getTargetDirectionDerivative(),
                                                            localZ);
        final double   betaDer        = -s / FastMath.sqrt(1 - s * s);
        final double   deltaL         = (0.5 * FastMath.PI - beta) / betaDer;
        final double   fixedLine      = crossingResult.getLine() + deltaL;
        final Vector3D fixedDirection = new Vector3D(1, crossingResult.getTargetDirection(),
                                                     deltaL, crossingResult.getTargetDirectionDerivative()).normalize();

        // fix neighbouring pixels
        final AbsoluteDate fixedDate  = sensor.getDate(fixedLine);
        final Vector3D fixedX         = sensor.getLOS(fixedDate, lowIndex);
        final Vector3D fixedZ         = Vector3D.crossProduct(fixedX, sensor.getLOS(fixedDate, lowIndex + 1));
        final Vector3D fixedY         = Vector3D.crossProduct(fixedZ, fixedX);

        // fix pixel
        final double pixelWidth = FastMath.atan2(Vector3D.dotProduct(highLOS,        fixedY),
                                                 Vector3D.dotProduct(highLOS,        fixedX));
        final double alpha      = FastMath.atan2(Vector3D.dotProduct(fixedDirection, fixedY),
                                                 Vector3D.dotProduct(fixedDirection, fixedX));
        final double fixedPixel = lowIndex + alpha / pixelWidth;

        final SensorPixel result = new SensorPixel(fixedLine, fixedPixel);
        DumpManager.dumpInverseLocationResult(result);
        return result;

    }
    /** Compute distance between two line sensors.
     * @param sensorA line sensor A
     * @param dateA current date for sensor A
     * @param pixelA pixel index for sensor A
     * @param sensorB line sensor B
     * @param dateB current date for sensor B
     * @param pixelB pixel index for sensor B
     * @return distance computing
     * @exception RuggedException if sensor is unknown
     */
    public double distanceBetweenLOS(final LineSensor sensorA,  
                           final AbsoluteDate dateA, final int pixelA,
                           final LineSensor sensorB,
                           final AbsoluteDate dateB, final int pixelB)
        throws RuggedException {

        //final LineSensor sensorA = getLineSensor(sensorNameA);
        //final LineSensor sensorB = getLineSensor(sensorNameB);

        // Get sensors's position and LOS
        final Vector3D vA     = sensorA.getLOS(dateA, pixelA); // V_a : line of sight's vectorA
        final Vector3D vB     = sensorB.getLOS(dateB, pixelB); // V_b 
        final Vector3D sA     = sensorA.getPosition(); // S_a : sensorA 's position
        final Vector3D sB     = sensorB.getPosition(); // S_b 
        
        final Vector3D vBase= sB.subtract(sA); // S_b - S_a
        final double svA        = Vector3D.dotProduct(vBase, vA); // SV_a = (S_b - S_a).V_a
        final double svB        = Vector3D.dotProduct(vBase, vB); // SV_b = (S_b - S_a).V_b
        
        final double vAvB   = Vector3D.dotProduct(vA, vB); // V_a.V_b
        
        // Compute lambda_b = (SV_a * V_a.V_b - SV_b) / (1 + (V_a.V_b)²)
        final double lambdaB = (svA * vAvB - svB) / (1 + FastMath.pow(vAvB,2));
        
        // Compute lambda_a = SV_a + lambdaB * V_a.V_b
        final double lambdaA = svA + lambdaB * vAvB; 
        
        final Vector3D mA   = sA.add(vA.scalarMultiply(lambdaA)); // M_a = S_a + lambda_a * V_a
        final Vector3D mB   = sB.add(vB.scalarMultiply(lambdaB)); // M_b = S_b + lambda_b * V_b
  
        final Vector3D vDistance = mB.subtract(mA); // M_b - M_a
        
        // Get square of the euclidean norm
        final double d = vDistance.getNormSq();
        return d;
    }
    /** Compute distance between two line sensors with derivatives.
     * @param sensorA  line sensor A
     * @param dateA current date for sensor A
     * @param pixelA pixel index for sensor A
     * @param sensorB  line sensor B
     * @param dateB current date for sensor B
     * @param pixelB pixel index for sensor B
     * @param generator generator to use for building {@link DerivativeStructure} instances
     * @return distance computing with derivatives
     * @exception RuggedException if sensor is unknown
     * @see #distanceBetweenLOS(String, AbsoluteDate, int, String, AbsoluteDate, int)
     */
    private DerivativeStructure[]  distanceBetweenLOSDerivatives(final LineSensor sensorA,  
                           final AbsoluteDate dateA, final int pixelA,
                           final LineSensor sensorB,
                           final AbsoluteDate dateB, final int pixelB,
                           final DSGenerator generator)
        throws RuggedException {

        //final LineSensor sensorA = getLineSensor(sensorNameA);
        //final LineSensor sensorB = getLineSensor(sensorNameB);

        // Get sensors's LOS
        final FieldVector3D<DerivativeStructure> vA     = sensorA.getLOSDerivatives(dateA, pixelA, generator); // V_a : line of sight's vectorA
        final FieldVector3D<DerivativeStructure> vB     = sensorB.getLOSDerivatives(dateB, pixelB, generator); // V_b 

     // Get sensors's position. TODO: check if we have to implement getPositionDerivatives() method & CO
        final Vector3D sAtmp     = sensorA.getPosition(); // S_a : sensorA 's position
        final Vector3D sBtmp     = sensorB.getPosition(); // S_b 
        final DerivativeStructure scaleFactor = FieldVector3D.dotProduct(vA.normalize(), vA.normalize()); // V_a.V_a=1
        // Build a vector from position vector and a scale factor (equals to 1). 
        // The vector built will be scaleFactor * sA for example.
        final FieldVector3D<DerivativeStructure> sA     = new FieldVector3D<DerivativeStructure>(scaleFactor, sAtmp);
        final FieldVector3D<DerivativeStructure> sB     = new FieldVector3D<DerivativeStructure>(scaleFactor, sBtmp);
        // final FieldVector3D<DerivativeStructure> sA     = sensorA.getPositionDerivatives(); // S_a : sensorA 's position
        // final FieldVector3D<DerivativeStructure> sB     = sensorB.getPositionDerivatives(); // S_b
        
        final FieldVector3D<DerivativeStructure> vBase= sB.subtract(sA); // S_b - S_a
        final DerivativeStructure svA        = FieldVector3D.dotProduct(vBase, vA); // SV_a = (S_b - S_a).V_a
        final DerivativeStructure svB        = FieldVector3D.dotProduct(vBase, vB); // SV_b = (S_b - S_a).V_b
        
        final DerivativeStructure vAvB   = FieldVector3D.dotProduct(vA, vB); // V_a.V_b
        
        // Compute lambda_b = (SV_a * V_a.V_b - SV_b) / (1 + (V_a.V_b)²)
        final DerivativeStructure lambdaB = (svA.multiply(vAvB).subtract(svB)).divide(vAvB.pow(2).add(1));
        
        // Compute lambda_a = SV_a + lambdaB * V_a.V_b
        final DerivativeStructure lambdaA = vAvB.multiply(lambdaB).add(svA); 
        
        final FieldVector3D<DerivativeStructure> mA   = sA.add(vA.scalarMultiply(lambdaA)); // M_a = S_a + lambda_a * V_a
        final FieldVector3D<DerivativeStructure> mB   = sB.add(vB.scalarMultiply(lambdaB)); // M_b = S_b + lambda_b * V_b
  
        final FieldVector3D<DerivativeStructure> vDistance = mB.subtract(mA); // M_b - M_a
        
        // Get square of the euclidean norm
        final DerivativeStructure d = vDistance.getNormSq();
        return new DerivativeStructure[] {d};
    }
    /** Estimate the free parameters from two viewing models (A and B) 
     * @param references reference mappings between two sensors pixels from two models 
     * and the corresponding computed distance between LOS that
     * should ultimately be reached by adjusting selected viewing models parameters
     * @param maxEvaluations maximum number of evaluations
     * @param parametersConvergenceThreshold convergence threshold on
     * normalized parameters (dimensionless, related to parameters scales)
     * @param ruggedA rugged instance from viewing model A
     * @return optimum of the least squares problem
     * @exception RuggedException if several parameters with the same name exist,
     * if no parameters have been selected for estimation, or if parameters cannot be
     * estimated (too few measurements, ill-conditioned problem ...)
     */
    public Optimum estimateFreeParams2Models(final Collection<SensorToSensorMapping> references,
                                          final int maxEvaluations,
                                          final double parametersConvergenceThreshold,
                                          Rugged ruggedA)
        throws RuggedException {
        try {

            // TODO BEGIN-----------------------
            // Verify that createGenerator's construction is ok with the use of two Rugged instance 
            final List<LineSensor> selectedSensors = new ArrayList<>();
            for (final SensorToSensorMapping reference : references) {
                selectedSensors.add(getLineSensor(reference.getSensorNameA())); // from ruggedA instance
                selectedSensors.add(getLineSensor(reference.getSensorNameB())); // from current ruggedB instance
                
            }
            final DSGenerator generator = createGenerator(selectedSensors);
            final List<ParameterDriver> selected = generator.getSelected();
            if (selected.isEmpty()) {
                throw new RuggedException(RuggedMessages.NO_PARAMETERS_SELECTED);
            }
            // TODO END--------------------------

            // get start point (as a normalized value)
            final double[] start = new double[selected.size()];
            for (int i = 0; i < start.length; ++i) {
                start[i] = selected.get(i).getNormalizedValue();
            }

            // set up target : distance between two LOS from both viewing models (A and B)
            int n = 0;
            for (final SensorToSensorMapping reference : references) {
                n += reference.getMappings().size(); 
            }
            if (n == 0) {
                throw new RuggedException(RuggedMessages.NO_REFERENCE_MAPPINGS);
            }
            final double[] target = new double[n];
            int k = 0;
            for (final SensorToSensorMapping reference : references) {
                for (final Double distMeas : reference.getMapDistance()) {
                    target[k++] = distMeas.doubleValue();   // distances measurements  
                }
            }

            // prevent parameters to exceed their prescribed bounds
            final ParameterValidator validator = params -> {
                try {
                    int i = 0;
                    for (final ParameterDriver driver : selected) {
                        // let the parameter handle min/max clipping
                        driver.setNormalizedValue(params.getEntry(i));
                        params.setEntry(i++, driver.getNormalizedValue());
                    }
                    return params;
                } catch (OrekitException oe) {
                    throw new OrekitExceptionWrapper(oe);
                }
            };

            // convergence checker
            final ConvergenceChecker<LeastSquaresProblem.Evaluation> checker =
                (iteration, previous, current) ->
                current.getPoint().getLInfDistance(previous.getPoint()) <= parametersConvergenceThreshold;

            // model function
            final MultivariateJacobianFunction model = point -> {
                try {

                    // set the current parameters values
                    int i = 0;
                    for (final ParameterDriver driver : selected) {
                        driver.setNormalizedValue(point.getEntry(i++));
                        // TODO: to be confirmed with the remark done above. 
                    }

                    // compute distance and its partial derivatives
                    final RealVector value    = new ArrayRealVector(target.length);
                    final RealMatrix jacobian = new Array2DRowRealMatrix(target.length, selected.size());
                    int l = 0;
                    for (final SensorToSensorMapping reference : references) {
                        for (final Map.Entry<SensorPixel, SensorPixel> mapping : reference.getMappings()) {
                            final SensorPixel spA = mapping.getKey();
                            final SensorPixel spB = mapping.getValue();
                            LineSensor lineSensorB = this.getLineSensor(reference.getSensorNameB());
                            LineSensor lineSensorA = ruggedA.getLineSensor(reference.getSensorNameA());
                            final AbsoluteDate dateA = lineSensorA.getDate(spA.getLineNumber());
                            final AbsoluteDate dateB = lineSensorB.getDate(spB.getLineNumber());
                            final int pixelA = (int)spA.getPixelNumber(); // Note: Rugged don't deal with half-pixel
                            final int pixelB = (int)spB.getPixelNumber();
                            
                            final DerivativeStructure[] ilResult =
                                            distanceBetweenLOSDerivatives(reference.getSensorNameA(),
                                                                          dateA, 
                                                                          pixelA,
                                                                          reference.getSensorNameB(),
                                                                          dateB, 
                                                                          pixelB,
                                                                          generator);
                                            
                            if (ilResult == null) {
                                    // TODO
                            } else {
                                // extract the value
                                value.setEntry(l, ilResult[0].getValue());

                                // extract the Jacobian
                                final int[] orders = new int[selected.size()];
                                for (int m = 0; m < selected.size(); ++m) {
                                    final double scale = selected.get(m).getScale();
                                    orders[m] = 1;
                                    jacobian.setEntry(l, m, ilResult[0].getPartialDerivative(orders) * scale);
                                    orders[m] = 0;
                                }
                            }

                            l += 1; // pass to the next evaluation

                        }
                    }

                    // distance result with Jacobian for all reference points
                    return new Pair<RealVector, RealMatrix>(value, jacobian);

                } catch (RuggedException re) {
                    throw new RuggedExceptionWrapper(re);
                } catch (OrekitException oe) {
                    throw new OrekitExceptionWrapper(oe);
                }
            };

            // set up the least squares problem
            final LeastSquaresProblem problem = new LeastSquaresBuilder().
                            lazyEvaluation(false).
                            maxIterations(maxEvaluations).
                            maxEvaluations(maxEvaluations).
                            weight(null).
                            start(start).
                            target(target).
                            parameterValidator(validator).
                            checker(checker).
                            model(model).
                            build();

            // set up the optimizer
            final LeastSquaresOptimizer optimizer = new LevenbergMarquardtOptimizer();

            // solve the least squares problem
            return optimizer.optimize(problem);

        } catch (RuggedExceptionWrapper rew) {
            throw rew.getException();
        } catch (OrekitExceptionWrapper oew) {
            final OrekitException oe = oew.getException();
            throw new RuggedException(oe,  oe.getSpecifier(), oe.getParts());
        }
    }
    /** Get the mean plane crossing finder for a sensor.
     * @param sensorName name of the line  sensor
     * @param minLine minimum line number
     * @param maxLine maximum line number
     * @return mean plane crossing finder
     * @exception RuggedException if sensor is unknown
     */
    private SensorMeanPlaneCrossing getPlaneCrossing(final String sensorName,
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
            setPlaneCrossing(planeCrossing);

        }

        return planeCrossing;

    }

    /** Set the mean plane crossing finder for a sensor.
     * @param planeCrossing plane crossing finder
     * @exception RuggedException if sensor is unknown
     */
    private void setPlaneCrossing(final SensorMeanPlaneCrossing planeCrossing)
        throws RuggedException {
        finders.put(planeCrossing.getSensor().getName(), planeCrossing);
    }

    /** Inverse location of a point with derivatives.
     * @param sensorName name of the line  sensor
     * @param point point to localize
     * @param minLine minimum line number
     * @param maxLine maximum line number
     * @param generator generator to use for building {@link DerivativeStructure} instances
     * @return sensor pixel seeing point with derivatives, or null if point cannot be seen between the
     * prescribed line numbers
     * @exception RuggedException if line cannot be localized, or sensor is unknown
     * @see #inverseLocation(String, GeodeticPoint, int, int)
     */
    private DerivativeStructure[] inverseLocationDerivatives(final String sensorName,
                                                             final GeodeticPoint point,
                                                             final int minLine,
                                                             final int maxLine,
                                                             final DSGenerator generator)
        throws RuggedException {

        final LineSensor sensor = getLineSensor(sensorName);

        final SensorMeanPlaneCrossing planeCrossing = getPlaneCrossing(sensorName, minLine, maxLine);

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
                                        crossingResult.getTargetDirection(),
                                        MAX_EVAL, COARSE_INVERSE_LOCATION_ACCURACY);
        final double coarsePixel = pixelCrossing.locatePixel(crossingResult.getDate());
        if (Double.isNaN(coarsePixel)) {
            // target is out of search interval
            return null;
        }

        // fix line by considering the closest pixel exact position and line-of-sight
        // (this pixel might point towards a direction slightly above or below the mean sensor plane)
        final int lowIndex = FastMath.max(0, FastMath.min(sensor.getNbPixels() - 2, (int) FastMath.floor(coarsePixel)));
        final FieldVector3D<DerivativeStructure> lowLOS =
                        sensor.getLOSDerivatives(crossingResult.getDate(), lowIndex, generator);
        final FieldVector3D<DerivativeStructure> highLOS = sensor.getLOSDerivatives(crossingResult.getDate(), lowIndex + 1, generator);
        final FieldVector3D<DerivativeStructure> localZ = FieldVector3D.crossProduct(lowLOS, highLOS).normalize();
        final DerivativeStructure beta         = FieldVector3D.dotProduct(crossingResult.getTargetDirection(), localZ).acos();
        final DerivativeStructure s            = FieldVector3D.dotProduct(crossingResult.getTargetDirectionDerivative(), localZ);
        final DerivativeStructure minusBetaDer = s.divide(s.multiply(s).subtract(1).negate().sqrt());
        final DerivativeStructure deltaL       = beta.subtract(0.5 * FastMath.PI) .divide(minusBetaDer);
        final DerivativeStructure fixedLine    = deltaL.add(crossingResult.getLine());
        final FieldVector3D<DerivativeStructure> fixedDirection =
                        new FieldVector3D<DerivativeStructure>(deltaL.getField().getOne(), crossingResult.getTargetDirection(),
                                                               deltaL, crossingResult.getTargetDirectionDerivative()).normalize();

        // fix neighbouring pixels
        final AbsoluteDate fixedDate  = sensor.getDate(fixedLine.getValue());
        final FieldVector3D<DerivativeStructure> fixedX = sensor.getLOSDerivatives(fixedDate, lowIndex, generator);
        final FieldVector3D<DerivativeStructure> fixedZ = FieldVector3D.crossProduct(fixedX, sensor.getLOSDerivatives(fixedDate, lowIndex + 1, generator));
        final FieldVector3D<DerivativeStructure> fixedY = FieldVector3D.crossProduct(fixedZ, fixedX);

        // fix pixel
        final DerivativeStructure hY         = FieldVector3D.dotProduct(highLOS, fixedY);
        final DerivativeStructure hX         = FieldVector3D.dotProduct(highLOS, fixedX);
        final DerivativeStructure pixelWidth = hY.atan2(hX);
        final DerivativeStructure fY         = FieldVector3D.dotProduct(fixedDirection, fixedY);
        final DerivativeStructure fX         = FieldVector3D.dotProduct(fixedDirection, fixedX);
        final DerivativeStructure alpha      = fY.atan2(fX);
        final DerivativeStructure fixedPixel = alpha.divide(pixelWidth).add(lowIndex);

        return new DerivativeStructure[] {
            fixedLine, fixedPixel
        };

    }

    /** Estimate the free parameters in viewing model to match specified sensor
     * to ground mappings.
     * <p>
     * This method is typically used for calibration of on-board sensor parameters,
     * like rotation angles polynomial coefficients.
     * </p>
     * <p>
     * Before using this method, the {@link ParameterDriver viewing model
     * parameters} retrieved by calling the {@link
     * LineSensor#getParametersDrivers() getParametersDrivers()}
     * method on the desired sensors must be configured. The parameters that should
     * be estimated must have their {@link ParameterDriver#setSelected(boolean)
     * selection status} set to {@link true} whereas the parameters that should retain
     * their current value must have their {@link ParameterDriver#setSelected(boolean)
     * selection status} set to {@link false}. If needed, the {@link
     * ParameterDriver#setValue(double) value} of the estimated/selected parameters
     * can also be changed before calling the method, as this value will serve as the
     * initial value in the estimation process.
     * </p>
     * <p>
     * The method solves a least-squares problem to minimize the residuals between test
     * locations and the reference mappings by adjusting the selected viewing models
     * parameters.
     * </p>
     * <p>
     * The estimated parameters can be retrieved after the method completes by calling
     * again the {@link LineSensor#getParametersDrivers() getParametersDrivers()}
     * method on the desired sensors and checking the updated values of the parameters.
     * In fact, as the values of the parameters are already updated by this method, if
     * users want to use the updated values immediately to perform new direct/inverse
     * locations, they can do so without looking at the parameters: the viewing models
     * are already aware of the updated parameters.
     * </p>
     * @param references reference mappings between sensors pixels and ground point that
     * should ultimately be reached by adjusting selected viewing models parameters
     * @param maxEvaluations maximum number of evaluations
     * @param parametersConvergenceThreshold convergence threshold on
     * normalized parameters (dimensionless, related to parameters scales)
     * @return optimum of the least squares problem
     * @exception RuggedException if several parameters with the same name exist,
     * if no parameters have been selected for estimation, or if parameters cannot be
     * estimated (too few measurements, ill-conditioned problem ...)
     */
    public Optimum estimateFreeParameters(final Collection<SensorToGroundMapping> references,
                                          final int maxEvaluations,
                                          final double parametersConvergenceThreshold)
        throws RuggedException {
        try {

            final List<LineSensor> selectedSensors = new ArrayList<>();
            for (final SensorToGroundMapping reference : references) {
                selectedSensors.add(getLineSensor(reference.getSensorName()));
            }
            final DSGenerator generator = createGenerator(selectedSensors);
            final List<ParameterDriver> selected = generator.getSelected();
            if (selected.isEmpty()) {
                throw new RuggedException(RuggedMessages.NO_PARAMETERS_SELECTED);
            }

            // get start point (as a normalized value)
            final double[] start = new double[selected.size()];
            for (int i = 0; i < start.length; ++i) {
                start[i] = selected.get(i).getNormalizedValue();
            }

            // set up target in sensor domain
            int n = 0;
            for (final SensorToGroundMapping reference : references) {
                n += reference.getMappings().size();
            }
            if (n == 0) {
                throw new RuggedException(RuggedMessages.NO_REFERENCE_MAPPINGS);
            }
            final double[] target = new double[2 * n];
            double min = Double.POSITIVE_INFINITY;
            double max = Double.NEGATIVE_INFINITY;
            int k = 0;
            for (final SensorToGroundMapping reference : references) {
                for (final Map.Entry<SensorPixel, GeodeticPoint> mapping : reference.getMappings()) {
                    final SensorPixel sp = mapping.getKey();
                    target[k++] = sp.getLineNumber();
                    target[k++] = sp.getPixelNumber();
                    min = FastMath.min(min, sp.getLineNumber());
                    max = FastMath.max(max, sp.getLineNumber());
                }
            }
            final int minLine = (int) FastMath.floor(min - ESTIMATION_LINE_RANGE_MARGIN);
            final int maxLine = (int) FastMath.ceil(max - ESTIMATION_LINE_RANGE_MARGIN);

            // prevent parameters to exceed their prescribed bounds
            final ParameterValidator validator = params -> {
                try {
                    int i = 0;
                    for (final ParameterDriver driver : selected) {
                        // let the parameter handle min/max clipping
                        driver.setNormalizedValue(params.getEntry(i));
                        params.setEntry(i++, driver.getNormalizedValue());
                    }
                    return params;
                } catch (OrekitException oe) {
                    throw new OrekitExceptionWrapper(oe);
                }
            };

            // convergence checker
            final ConvergenceChecker<LeastSquaresProblem.Evaluation> checker =
                (iteration, previous, current) ->
                current.getPoint().getLInfDistance(previous.getPoint()) <= parametersConvergenceThreshold;

            // model function
            final MultivariateJacobianFunction model = point -> {
                try {

                    // set the current parameters values
                    int i = 0;
                    for (final ParameterDriver driver : selected) {
                        driver.setNormalizedValue(point.getEntry(i++));
                    }

                    // compute inverse loc and its partial derivatives
                    final RealVector value    = new ArrayRealVector(target.length);
                    final RealMatrix jacobian = new Array2DRowRealMatrix(target.length, selected.size());
                    int l = 0;
                    for (final SensorToGroundMapping reference : references) {
                        for (final Map.Entry<SensorPixel, GeodeticPoint> mapping : reference.getMappings()) {
                            final GeodeticPoint gp = mapping.getValue();
                            final DerivativeStructure[] ilResult =
                                            inverseLocationDerivatives(reference.getSensorName(),
                                                                       gp, minLine, maxLine, generator);

                            if (ilResult == null) {
                                value.setEntry(l,     minLine - 100.0); // arbitrary line far away
                                value.setEntry(l + 1, -100.0);          // arbitrary pixel far away
                            } else {
                                // extract the value
                                value.setEntry(l,     ilResult[0].getValue());
                                value.setEntry(l + 1, ilResult[1].getValue());

                                // extract the Jacobian
                                final int[] orders = new int[selected.size()];
                                for (int m = 0; m < selected.size(); ++m) {
                                    final double scale = selected.get(m).getScale();
                                    orders[m] = 1;
                                    jacobian.setEntry(l,     m, ilResult[0].getPartialDerivative(orders) * scale);
                                    jacobian.setEntry(l + 1, m, ilResult[1].getPartialDerivative(orders) * scale);
                                    orders[m] = 0;
                                }
                            }

                            l += 2;

                        }
                    }

                    // inverse loc result with Jacobian for all reference points
                    return new Pair<RealVector, RealMatrix>(value, jacobian);

                } catch (RuggedException re) {
                    throw new RuggedExceptionWrapper(re);
                } catch (OrekitException oe) {
                    throw new OrekitExceptionWrapper(oe);
                }
            };

            // set up the least squares problem
            final LeastSquaresProblem problem = new LeastSquaresBuilder().
                            lazyEvaluation(false).
                            maxIterations(maxEvaluations).
                            maxEvaluations(maxEvaluations).
                            weight(null).
                            start(start).
                            target(target).
                            parameterValidator(validator).
                            checker(checker).
                            model(model).
                            build();

            // set up the optimizer
            //final LeastSquaresOptimizer optimizer = new LevenbergMarquardtOptimizer();
            final LeastSquaresOptimizer optimizer = new GaussNewtonOptimizer().withDecomposition(GaussNewtonOptimizer.Decomposition.QR);
            // solve the least squares problem
            return optimizer.optimize(problem);

        } catch (RuggedExceptionWrapper rew) {
            throw rew.getException();
        } catch (OrekitExceptionWrapper oew) {
            final OrekitException oe = oew.getException();
            throw new RuggedException(oe,  oe.getSpecifier(), oe.getParts());
        }
    }

    /** Create the generator for {@link DerivativeStructure} instances.
     * @param selectedSensors sensors referencing the parameters drivers
     * @return a new generator
     * @exception RuggedException if several parameters with the same name exist
     */
    private DSGenerator createGenerator(final List<LineSensor> selectedSensors)
        throws RuggedException {

        // we are more stringent than Orekit orbit determination:
        // we do not allow different parameters with the same name
        final Set<String> names = new HashSet<>();
        for (final LineSensor sensor : selectedSensors) {
            sensor.getParametersDrivers().forEach(driver -> {
                if (names.contains(driver.getName())) {
                    throw new RuggedExceptionWrapper(new RuggedException(RuggedMessages.DUPLICATED_PARAMETER_NAME,
                                                                         driver.getName()));
                }
                names.add(driver.getName());
            });
        }

        // set up generator list and map
        final List<ParameterDriver> selected = new ArrayList<>();
        final Map<String, Integer> map = new HashMap<>();
        for (final LineSensor sensor : selectedSensors) {
            sensor.
                getParametersDrivers().
                filter(driver -> driver.isSelected()).
                forEach(driver -> {
                        selected.add(driver);
                        map.put(driver.getName(), map.size());
                    });
        }

        return new DSGenerator() {

            /** {@inheritDoc} */
            @Override
            public List<ParameterDriver> getSelected() {
                return selected;
            }

            /** {@inheritDoc} */
            @Override
            public DerivativeStructure constant(final double value) {
                return new DerivativeStructure(map.size(), 1, value);
            }

            /** {@inheritDoc} */
            @Override
            public DerivativeStructure variable(final ParameterDriver driver) {
                final Integer index = map.get(driver.getName());
                if (index == null) {
                    return constant(driver.getValue());
                } else {
                    return new DerivativeStructure(map.size(), 1, index.intValue(), driver.getValue());
                }
            }

        };
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
