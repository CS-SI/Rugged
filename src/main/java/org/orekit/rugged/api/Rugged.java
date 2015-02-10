/* Copyright 2013-2015 CS Systèmes d'Information
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

import java.util.Collection;
import java.util.HashMap;
import java.util.Map;

import org.apache.commons.math3.analysis.differentiation.DerivativeStructure;
import org.apache.commons.math3.geometry.euclidean.threed.FieldVector3D;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.util.FastMath;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.frames.Transform;
import org.orekit.rugged.errors.DumpManager;
import org.orekit.rugged.errors.RuggedException;
import org.orekit.rugged.errors.RuggedMessages;
import org.orekit.rugged.intersection.IntersectionAlgorithm;
import org.orekit.rugged.linesensor.LineSensor;
import org.orekit.rugged.linesensor.SensorMeanPlaneCrossing;
import org.orekit.rugged.linesensor.SensorPixel;
import org.orekit.rugged.linesensor.SensorPixelCrossing;
import org.orekit.rugged.utils.ExtendedEllipsoid;
import org.orekit.rugged.utils.NormalizedGeodeticPoint;
import org.orekit.rugged.utils.SpacecraftToObservedBody;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.Constants;
import org.orekit.utils.PVCoordinates;

/** Main class of Rugged library API.
 * @see RuggedBuilder
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

            DumpManager.dumpDirectLocation(date, sensor.getPosition(), sensor.getLos(date, i),
                                           lightTimeCorrection, aberrationOfLightCorrection);

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
