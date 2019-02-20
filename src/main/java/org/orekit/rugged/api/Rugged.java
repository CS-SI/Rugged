/* Copyright 2013-2019 CS Systèmes d'Information
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

import org.hipparchus.analysis.differentiation.DerivativeStructure;
import org.hipparchus.geometry.euclidean.threed.FieldVector3D;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.util.FastMath;
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
import org.orekit.rugged.refraction.AtmosphericRefraction;
import org.orekit.rugged.utils.DSGenerator;
import org.orekit.rugged.utils.ExtendedEllipsoid;
import org.orekit.rugged.utils.NormalizedGeodeticPoint;
import org.orekit.rugged.utils.SpacecraftToObservedBody;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.Constants;
import org.orekit.utils.PVCoordinates;

/** Main class of Rugged library API.
 * @see RuggedBuilder
 * @author Luc Maisonobe
 * @author Guylaine Prat
 * @author Jonathan Guinet
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

    /** Maximum number of evaluations for crossing algorithms. */
    private static final int MAX_EVAL = 50;

    /** Margin for computation of inverse location with atmospheric refraction correction. */
    private static final double INVLOC_MARGIN = 0.8;

    /** Threshold for pixel convergence in fixed point method
     * (for inverse location with atmospheric refraction correction). */
    private static final double PIXEL_CV_THRESHOLD = 1.e-4;

    /** Threshold for line convergence in fixed point method
     * (for inverse location with atmospheric refraction correction). */
    private static final double LINE_CV_THRESHOLD = 1.e-4;

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

    /** Rugged name. */
    private final String name;

    /** Atmospheric refraction for line of sight correction. */
    private AtmosphericRefraction atmosphericRefraction;

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
     * @param atmosphericRefraction the atmospheric refraction model to be used for more accurate location
     * @param scToBody transforms interpolator
     * @param sensors sensors
     * @param name Rugged name
     */
    Rugged(final IntersectionAlgorithm algorithm, final ExtendedEllipsoid ellipsoid, final boolean lightTimeCorrection,
           final boolean aberrationOfLightCorrection, final AtmosphericRefraction atmosphericRefraction,
           final SpacecraftToObservedBody scToBody, final Collection<LineSensor> sensors, final String name) {


        // space reference
        this.ellipsoid = ellipsoid;

        // orbit/attitude to body converter
        this.scToBody = scToBody;

        // intersection algorithm
        this.algorithm = algorithm;

        // Rugged name
        // @since 2.0
        this.name = name;

        this.sensors = new HashMap<String, LineSensor>();
        for (final LineSensor s : sensors) {
            this.sensors.put(s.getName(), s);
        }
        this.finders = new HashMap<String, SensorMeanPlaneCrossing>();

        this.lightTimeCorrection         = lightTimeCorrection;
        this.aberrationOfLightCorrection = aberrationOfLightCorrection;
        this.atmosphericRefraction       = atmosphericRefraction;
    }

    /** Get the Rugged name.
     * @return Rugged name
     * @since 2.0
     */
    public String getName() {
        return name;
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

    /** Get the atmospheric refraction model.
     * @return atmospheric refraction model
     * @since 2.0
     */
    public AtmosphericRefraction getRefractionCorrection() {
        return atmosphericRefraction;
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
     * The support range is given by the {@code minDate} and
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
     */
    public GeodeticPoint[] directLocation(final String sensorName, final double lineNumber) {

        final LineSensor   sensor = getLineSensor(sensorName);
        final Vector3D sensorPosition   = sensor.getPosition();
        final AbsoluteDate date   = sensor.getDate(lineNumber);

        // Compute the transform for the date
        // from spacecraft to inertial
        final Transform    scToInert   = scToBody.getScToInertial(date);
        // from inertial to body
        final Transform    inertToBody = scToBody.getInertialToBody(date);

        // Compute spacecraft velocity in inertial frame
        final Vector3D spacecraftVelocity = scToInert.transformPVCoordinates(PVCoordinates.ZERO).getVelocity();
        // Compute sensor position in inertial frame
        // TBN: for simplicity, due to the size of sensor, we consider each pixel to be at sensor position
        final Vector3D pInert = scToInert.transformPosition(sensorPosition);

        // Compute location of each pixel
        final GeodeticPoint[] gp = new GeodeticPoint[sensor.getNbPixels()];
        for (int i = 0; i < sensor.getNbPixels(); ++i) {

            final Vector3D los = sensor.getLOS(date, i);
            DumpManager.dumpDirectLocation(date, sensorPosition, los, lightTimeCorrection,
                    aberrationOfLightCorrection, atmosphericRefraction != null);

            // compute the line of sight in inertial frame (without correction)
            final Vector3D obsLInert = scToInert.transformVector(los);
            final Vector3D lInert;

            if (aberrationOfLightCorrection) {
                // apply aberration of light correction on LOS
                lInert = applyAberrationOfLightCorrection(obsLInert, spacecraftVelocity);
            } else {
                // don't apply aberration of light correction on LOS
                lInert = obsLInert;
            }

            if (lightTimeCorrection) {
                // compute DEM intersection with light time correction
                // TBN: for simplicity, due to the size of sensor, we consider each pixel to be at sensor position
                gp[i] = computeWithLightTimeCorrection(date, sensorPosition, los, scToInert, inertToBody, pInert, lInert);

            } else {
                // compute DEM intersection without light time correction
                final Vector3D pBody = inertToBody.transformPosition(pInert);
                final Vector3D lBody = inertToBody.transformVector(lInert);
                gp[i] = algorithm.refineIntersection(ellipsoid, pBody, lBody,
                                                     algorithm.intersection(ellipsoid, pBody, lBody));
            }

            // compute with atmospheric refraction correction if necessary
            if (atmosphericRefraction != null && atmosphericRefraction.mustBeComputed()) {

                // apply atmospheric refraction correction
                final Vector3D pBody = inertToBody.transformPosition(pInert);
                final Vector3D lBody = inertToBody.transformVector(lInert);
                gp[i] = atmosphericRefraction.applyCorrection(pBody, lBody, (NormalizedGeodeticPoint) gp[i], algorithm);
            }
            DumpManager.dumpDirectLocationResult(gp[i]);
        }
        return gp;
    }

    /** Direct location of a single line-of-sight.
     * @param date date of the location
     * @param sensorPosition sensor position in spacecraft frame. For simplicity, due to the size of sensor,
     * we consider each pixel to be at sensor position
     * @param los normalized line-of-sight in spacecraft frame
     * @return ground position of intersection point between specified los and ground
     */
    public GeodeticPoint directLocation(final AbsoluteDate date, final Vector3D sensorPosition, final Vector3D los) {

        DumpManager.dumpDirectLocation(date, sensorPosition, los, lightTimeCorrection, aberrationOfLightCorrection,
                                       atmosphericRefraction != null);

        // Compute the transforms for the date
        // from spacecraft to inertial
        final Transform    scToInert   = scToBody.getScToInertial(date);
        // from inertial to body
        final Transform    inertToBody = scToBody.getInertialToBody(date);

        // Compute spacecraft velocity in inertial frame
        final Vector3D spacecraftVelocity = scToInert.transformPVCoordinates(PVCoordinates.ZERO).getVelocity();
        // Compute sensor position in inertial frame
        // TBN: for simplicity, due to the size of sensor, we consider each pixel to be at sensor position
        final Vector3D pInert    = scToInert.transformPosition(sensorPosition);

        // Compute the line of sight in inertial frame (without correction)
        final Vector3D obsLInert = scToInert.transformVector(los);

        final Vector3D lInert;
        if (aberrationOfLightCorrection) {
            // apply aberration of light correction on LOS
            lInert = applyAberrationOfLightCorrection(obsLInert, spacecraftVelocity);
        } else {
            // don't apply aberration of light correction on LOS
            lInert = obsLInert;
        }

        // Compute ground location of specified pixel
        final NormalizedGeodeticPoint gp;

        if (lightTimeCorrection) {
            // compute DEM intersection with light time correction
            // TBN: for simplicity, due to the size of sensor, we consider each pixel to be at sensor position
            gp = computeWithLightTimeCorrection(date, sensorPosition, los, scToInert, inertToBody, pInert, lInert);

        } else {
            // compute DEM intersection without light time correction
            final Vector3D pBody = inertToBody.transformPosition(pInert);
            final Vector3D lBody = inertToBody.transformVector(lInert);
            gp = algorithm.refineIntersection(ellipsoid, pBody, lBody,
                                              algorithm.intersection(ellipsoid, pBody, lBody));
        }

        NormalizedGeodeticPoint result = gp;

        // compute the ground location with atmospheric correction if asked for
        if (atmosphericRefraction != null && atmosphericRefraction.mustBeComputed()) {

            // apply atmospheric refraction correction
            final Vector3D pBody = inertToBody.transformPosition(pInert);
            final Vector3D lBody = inertToBody.transformVector(lInert);
            result = atmosphericRefraction.applyCorrection(pBody, lBody, gp, algorithm);

        } // end test on atmosphericRefraction != null

        DumpManager.dumpDirectLocationResult(result);
        return result;
    }

    /** Find the date at which sensor sees a ground point.
     * <p>
     * This method is a partial {@link #inverseLocation(String, GeodeticPoint, int, int) inverse location} focusing only on date.
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
     * @param latitude ground point latitude (rad)
     * @param longitude ground point longitude (rad)
     * @param minLine minimum line number
     * @param maxLine maximum line number
     * @return date at which ground point is seen by line sensor
     * @see #inverseLocation(String, double, double, int, int)
     * @see org.orekit.rugged.utils.RoughVisibilityEstimator
     */
    public AbsoluteDate dateLocation(final String sensorName,
                                     final double latitude, final double longitude,
                                     final int minLine, final int maxLine) {

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
     * @param sensorName name of the line sensor
     * @param point point to localize
     * @param minLine minimum line number
     * @param maxLine maximum line number
     * @return date at which ground point is seen by line sensor
     * @see #inverseLocation(String, GeodeticPoint, int, int)
     * @see org.orekit.rugged.utils.RoughVisibilityEstimator
     */
    public AbsoluteDate dateLocation(final String sensorName, final GeodeticPoint point,
                                     final int minLine, final int maxLine) {

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
     * @param latitude ground point latitude (rad)
     * @param longitude ground point longitude (rad)
     * @param minLine minimum line number
     * @param maxLine maximum line number
     * @return sensor pixel seeing ground point, or null if ground point cannot
     * be seen between the prescribed line numbers
     * @see org.orekit.rugged.utils.RoughVisibilityEstimator
     */
    public SensorPixel inverseLocation(final String sensorName,
                                       final double latitude, final double longitude,
                                       final int minLine,  final int maxLine) {

        final GeodeticPoint groundPoint = new GeodeticPoint(latitude, longitude, algorithm.getElevation(latitude, longitude));
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
     * @param sensorName name of the line sensor
     * @param point geodetic point to localize
     * @param minLine minimum line number where the search will be performed
     * @param maxLine maximum line number where the search will be performed
     * @return sensor pixel seeing point, or null if point cannot be seen between the
     * prescribed line numbers
     * @see #dateLocation(String, GeodeticPoint, int, int)
     * @see org.orekit.rugged.utils.RoughVisibilityEstimator
     */
    public SensorPixel inverseLocation(final String sensorName, final GeodeticPoint point,
                                       final int minLine, final int maxLine) {

        final LineSensor sensor = getLineSensor(sensorName);
        DumpManager.dumpInverseLocation(sensor, point, ellipsoid, minLine, maxLine, lightTimeCorrection,
                                        aberrationOfLightCorrection, atmosphericRefraction != null);

        final SensorMeanPlaneCrossing planeCrossing = getPlaneCrossing(sensorName, minLine, maxLine);
        DumpManager.dumpSensorMeanPlane(planeCrossing);

        if (atmosphericRefraction == null || !atmosphericRefraction.mustBeComputed()) {
            // Compute inverse location WITHOUT atmospheric refraction
            return findSensorPixelWithoutAtmosphere(point, sensor, planeCrossing);
        } else {
            // Compute inverse location WITH atmospheric refraction
            return findSensorPixelWithAtmosphere(point, sensor, minLine, maxLine);
        }
    }

    /** Apply aberration of light correction (for direct location).
     * @param spacecraftVelocity spacecraft velocity in inertial frame
     * @param obsLInert line of sight in inertial frame
     * @return line of sight with aberration of light correction
     */
    private Vector3D applyAberrationOfLightCorrection(final Vector3D obsLInert, final Vector3D spacecraftVelocity) {

        // As the spacecraft velocity is small with respect to speed of light,
        // we use classical velocity addition and not relativistic velocity addition
        // we look for a positive k such that: c * lInert + vsat = k * obsLInert
        // with lInert normalized
        final double a = obsLInert.getNormSq();
        final double b = -Vector3D.dotProduct(obsLInert, spacecraftVelocity);
        final double c = spacecraftVelocity.getNormSq() - Constants.SPEED_OF_LIGHT * Constants.SPEED_OF_LIGHT;

        // a > 0 and c < 0
        final double s = FastMath.sqrt(b * b - a * c);

        // Only the k > 0 are kept as solutions (the solutions: -(s+b)/a and c/(s-b) are useless)
        final double k = (b > 0) ? -c / (s + b) : (s - b) / a;

        final Vector3D lInert = new Vector3D( k / Constants.SPEED_OF_LIGHT, obsLInert, -1.0 / Constants.SPEED_OF_LIGHT, spacecraftVelocity);
        return lInert;
    }

    /** Compute the DEM intersection with light time correction.
     * @param date date of the los
     * @param sensorPosition sensor position in spacecraft frame
     * @param los los in spacecraft frame
     * @param scToInert transform for the date from spacecraft to inertial
     * @param inertToBody transform for the date from inertial to body
     * @param pInert sensor position in inertial frame
     * @param lInert line of sight in inertial frame
     * @return geodetic point with light time correction
     */
    private NormalizedGeodeticPoint computeWithLightTimeCorrection(final AbsoluteDate date,
                                                                   final Vector3D sensorPosition, final Vector3D los,
                                                                   final Transform scToInert, final Transform inertToBody,
                                                                   final Vector3D pInert, final Vector3D lInert) {

        // compute the approximate transform between spacecraft and observed body
        final Transform approximate = new Transform(date, scToInert, inertToBody);

        final Vector3D  sL       = approximate.transformVector(los);
        final Vector3D  sP       = approximate.transformPosition(sensorPosition);

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
    }

    /**
     * Find the sensor pixel WITHOUT atmospheric refraction correction.
     * @param point geodetic point to localize
     * @param sensor the line sensor
     * @param planeCrossing the sensor mean plane crossing
     * @return the sensor pixel crossing or null if cannot be found
     * @since 2.1
     */
    private SensorPixel findSensorPixelWithoutAtmosphere(final GeodeticPoint point,
                                                         final LineSensor sensor, final SensorMeanPlaneCrossing planeCrossing) {

        // find approximately the sensor line at which ground point crosses sensor mean plane
        final Vector3D target = ellipsoid.transform(point);
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
        final double   beta           = FastMath.acos(Vector3D.dotProduct(crossingResult.getTargetDirection(), localZ));
        final double   s              = Vector3D.dotProduct(crossingResult.getTargetDirectionDerivative(), localZ);
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

    /**
     * Find the sensor pixel WITH atmospheric refraction correction.
     * @param point geodetic point to localize
     * @param sensor the line sensor
     * @param minLine minimum line number where the search will be performed
     * @param maxLine maximum line number where the search will be performed
     * @return the sensor pixel crossing or null if cannot be found
     * @since 2.1
     */
    private SensorPixel findSensorPixelWithAtmosphere(final GeodeticPoint point,
                                                      final LineSensor sensor, final int minLine, final int maxLine) {

        // TBN: there is no direct way to compute the inverse location.
        // The method is based on an interpolation grid associated with the fixed point method

        final String sensorName = sensor.getName();

        // Compute a correction grid (at sensor level)
        // ===========================================
        // Need to be computed only once for a given sensor (with the same minLine and maxLine)
        if (atmosphericRefraction.getBifPixel() == null || atmosphericRefraction.getBifLine() == null || // lazy evaluation
            (!atmosphericRefraction.isSameContext(sensorName, minLine, maxLine))) { // Must be recomputed if the context changed

            // Definition of a regular grid (at sensor level)
            atmosphericRefraction.configureCorrectionGrid(sensor, minLine, maxLine);

            // Get the grid nodes
            final int nbPixelGrid = atmosphericRefraction.getComputationParameters().getNbPixelGrid();
            final int nbLineGrid = atmosphericRefraction.getComputationParameters().getNbLineGrid();
            final double[] pixelGrid = atmosphericRefraction.getComputationParameters().getUgrid();
            final double[] lineGrid = atmosphericRefraction.getComputationParameters().getVgrid();

            // Computation, for the sensor grid, of the direct location WITH atmospheric refraction
            // (full computation)
            atmosphericRefraction.reactivateComputation();
            final GeodeticPoint[][] geodeticGridWithAtmosphere = computeDirectLocOnGridWithAtmosphere(pixelGrid, lineGrid, sensor);
            // pixelGrid and lineGrid are the nodes where the direct loc is computed WITH atmosphere

            // Computation of the inverse location WITHOUT atmospheric refraction for the grid nodes
            atmosphericRefraction.deactivateComputation();
            final SensorPixel[][] sensorPixelGridInverseWithout = computeInverseLocOnGridWithoutAtmosphere(geodeticGridWithAtmosphere,
                                                            nbPixelGrid, nbLineGrid, sensor, minLine, maxLine);
            atmosphericRefraction.reactivateComputation();

            // Compute the grid correction functions (for pixel and line)
            atmosphericRefraction.computeGridCorrectionFunctions(sensorPixelGridInverseWithout);
        }

        // Fixed point method
        // ==================
        // Initialization
        // --------------
        // Deactivate the dump because no need to keep intermediate computations of inverse loc (can be regenerate)
        final Boolean wasSuspended = DumpManager.suspend();

        // compute the sensor pixel on the desired ground point WITHOUT atmosphere
        atmosphericRefraction.deactivateComputation();
        final SensorPixel sp0 = inverseLocation(sensorName, point, minLine, maxLine);
        atmosphericRefraction.reactivateComputation();
        // Reactivate the dump
        DumpManager.resume(wasSuspended);

        if (sp0 == null) {
            // In order for the dump to end nicely
            DumpManager.endNicely();
            // Impossible to find the point in the given min line and max line (without atmosphere)
            throw new RuggedException(RuggedMessages.INVALID_RANGE_FOR_LINES, minLine, maxLine, "");
        }

        // set up the starting point of the fixed point method
        final double pixel0 = sp0.getPixelNumber();
        final double line0 = sp0.getLineNumber();
        // Needed data for the dump
        sensor.dumpRate(line0);

        // Apply fixed point method until convergence in pixel and line
        // ------------------------------------------------------------
        // compute the first (pixel, line) value:
        // initial sensor pixel value + correction due to atmosphere at this same sensor pixel
        double corrPixelPrevious =  pixel0 + atmosphericRefraction.getBifPixel().value(pixel0, line0);
        double corrLinePrevious = line0 + atmosphericRefraction.getBifLine().value(pixel0, line0);

        double deltaCorrPixel = Double.POSITIVE_INFINITY;
        double deltaCorrLine = Double.POSITIVE_INFINITY;

        while (deltaCorrPixel > PIXEL_CV_THRESHOLD && deltaCorrLine > LINE_CV_THRESHOLD) {
            // Compute the current (pixel, line) value =
            // initial sensor pixel value + correction due to atmosphere on the previous sensor pixel
            final double corrPixelCurrent = pixel0 + atmosphericRefraction.getBifPixel().value(corrPixelPrevious, corrLinePrevious);
            final double corrLineCurrent = line0 + atmosphericRefraction.getBifLine().value(corrPixelPrevious, corrLinePrevious);

            // Compute the delta in pixel and line to check the convergence
            deltaCorrPixel = FastMath.abs(corrPixelCurrent - corrPixelPrevious);
            deltaCorrLine = FastMath.abs(corrLineCurrent - corrLinePrevious);

            // Store the (pixel, line) for next loop
            corrPixelPrevious = corrPixelCurrent;
            corrLinePrevious = corrLineCurrent;
        }
        // The sensor pixel is found !
        final SensorPixel sensorPixelWithAtmosphere = new SensorPixel(corrLinePrevious, corrPixelPrevious);

        // Dump the found sensorPixel
        DumpManager.dumpInverseLocationResult(sensorPixelWithAtmosphere);

        return sensorPixelWithAtmosphere;
    }

    /** Compute the inverse location WITHOUT atmospheric refraction for the geodetic points
     * associated to the sensor grid nodes.
     * @param groundGridWithAtmosphere ground grid found for sensor grid nodes with atmosphere
     * @param nbPixelGrid size of the pixel grid
     * @param nbLineGrid size of the line grid
     * @param sensor the line sensor
     * @param minLine minimum line number where the search will be performed
     * @param maxLine maximum line number where the search will be performed
     * @return the sensor pixel grid computed without atmosphere
     * @since 2.1
     */
    private SensorPixel[][] computeInverseLocOnGridWithoutAtmosphere(final GeodeticPoint[][] groundGridWithAtmosphere,
                                                                     final int nbPixelGrid, final int nbLineGrid,
                                                                     final LineSensor sensor, final int minLine, final int maxLine) {

        // Deactivate the dump because no need to keep intermediate computations of inverse loc (can be regenerate)
        final Boolean wasSuspended = DumpManager.suspend();

        final SensorPixel[][] sensorPixelGrid = new SensorPixel[nbPixelGrid][nbLineGrid];
        final String sensorName = sensor.getName();

        for (int uIndex = 0; uIndex < nbPixelGrid; uIndex++) {
            for (int vIndex = 0; vIndex < nbLineGrid; vIndex++) {

                // Check if the geodetic point exists
                if (groundGridWithAtmosphere[uIndex][vIndex] != null) {
                    final GeodeticPoint groundPoint = groundGridWithAtmosphere[uIndex][vIndex];
                    final double currentLat = groundPoint.getLatitude();
                    final double currentLon = groundPoint.getLongitude();

                    try {
                        // Compute the inverse location for the current node
                        sensorPixelGrid[uIndex][vIndex] = inverseLocation(sensorName, currentLat, currentLon, minLine, maxLine);

                    } catch (RuggedException re) { // This should never happen
                        // In order for the dump to end nicely
                        DumpManager.endNicely();
                        throw RuggedException.createInternalError(re);
                    }

                    // Check if the pixel is inside the sensor (with a margin) OR if the inverse location was impossible (null result)
                    if ((sensorPixelGrid[uIndex][vIndex] != null &&
                           (sensorPixelGrid[uIndex][vIndex].getPixelNumber() < (-INVLOC_MARGIN) ||
                            sensorPixelGrid[uIndex][vIndex].getPixelNumber() > (INVLOC_MARGIN + sensor.getNbPixels() - 1))) ||
                        (sensorPixelGrid[uIndex][vIndex] == null) ) {
                        // In order for the dump to end nicely
                        DumpManager.endNicely();
                        // Impossible to find the point in the given min line
                        throw new RuggedException(RuggedMessages.INVALID_RANGE_FOR_LINES, minLine, maxLine, "");
                    }

                } else { // groundGrid[uIndex][vIndex] == null: impossible to compute inverse loc because ground point not defined

                    sensorPixelGrid[uIndex][vIndex] = null;

                } // groundGrid[uIndex][vIndex] != null
            } // end loop vIndex
        } // end loop uIndex

        // Reactivate the dump
        DumpManager.resume(wasSuspended);

        // The sensor grid computed WITHOUT atmospheric refraction correction
        return sensorPixelGrid;
    }

    /** Computation, for the sensor pixels grid, of the direct location WITH atmospheric refraction.
     * (full computation)
     * @param pixelGrid the pixel grid
     * @param lineGrid the line grid
     * @param sensor the line sensor
     * @return the ground grid computed with atmosphere
     * @since 2.1
     */
    private GeodeticPoint[][] computeDirectLocOnGridWithAtmosphere(final double[] pixelGrid, final double[] lineGrid,
                                                                   final LineSensor sensor) {

        // Deactivate the dump because no need to keep intermediate computations of direct loc (can be regenerate)
        final Boolean wasSuspended = DumpManager.suspend();

        final int nbPixelGrid = pixelGrid.length;
        final int nbLineGrid = lineGrid.length;
        final GeodeticPoint[][] groundGridWithAtmosphere = new GeodeticPoint[nbPixelGrid][nbLineGrid];
        final Vector3D sensorPosition = sensor.getPosition();

        for (int uIndex = 0; uIndex < nbPixelGrid; uIndex++) {
            final double pixelNumber = pixelGrid[uIndex];
            for (int vIndex = 0; vIndex < nbLineGrid; vIndex++) {
                final double lineNumber = lineGrid[vIndex];
                final AbsoluteDate date = sensor.getDate(lineNumber);
                final Vector3D los = sensor.getLOS(date, pixelNumber);
                try {
                    // Compute the direct location for the current node
                    groundGridWithAtmosphere[uIndex][vIndex] = directLocation(date, sensorPosition, los);

                } catch (RuggedException re) { // This should never happen
                    // In order for the dump to end nicely
                    DumpManager.endNicely();
                    throw RuggedException.createInternalError(re);
                }
            } // end loop vIndex
        } // end loop uIndex

        // Reactivate the dump
        DumpManager.resume(wasSuspended);

        // The ground grid computed WITH atmospheric refraction correction
        return groundGridWithAtmosphere;
    }

    /** Compute distances between two line sensors.
     * @param sensorA line sensor A
     * @param dateA current date for sensor A
     * @param pixelA pixel index for sensor A
     * @param scToBodyA spacecraft to body transform for sensor A
     * @param sensorB line sensor B
     * @param dateB current date for sensor B
     * @param pixelB pixel index for sensor B
     * @return distances computed between LOS and to the ground
     * @since 2.0
     */
    public double[] distanceBetweenLOS(final LineSensor sensorA, final AbsoluteDate dateA, final double pixelA,
                                       final SpacecraftToObservedBody scToBodyA,
                                       final LineSensor sensorB, final AbsoluteDate dateB, final double pixelB) {

        // Compute the approximate transform between spacecraft and observed body
        // from Rugged instance A
        final Transform scToInertA = scToBodyA.getScToInertial(dateA);
        final Transform inertToBodyA = scToBodyA.getInertialToBody(dateA);
        final Transform transformScToBodyA = new Transform(dateA, scToInertA, inertToBodyA);

        // from (current) Rugged instance B
        final Transform scToInertB = scToBody.getScToInertial(dateB);
        final Transform inertToBodyB = scToBody.getInertialToBody(dateB);
        final Transform transformScToBodyB = new Transform(dateB, scToInertB, inertToBodyB);

        // Get sensors LOS into local frame
        final Vector3D vALocal = sensorA.getLOS(dateA, pixelA);
        final Vector3D vBLocal = sensorB.getLOS(dateB, pixelB);

        // Position of sensors into local frame
        final Vector3D sALocal = sensorA.getPosition(); // S_a : sensorA 's position
        final Vector3D sBLocal = sensorB.getPosition(); // S_b : sensorB 's position

        // Get sensors position and LOS into body frame
        final Vector3D sA = transformScToBodyA.transformPosition(sALocal); // S_a : sensorA's position
        final Vector3D vA = transformScToBodyA.transformVector(vALocal);   // V_a : line of sight's vectorA
        final Vector3D sB = transformScToBodyB.transformPosition(sBLocal); // S_b : sensorB's position
        final Vector3D vB = transformScToBodyB.transformVector(vBLocal);   // V_b : line of sight's vectorB

        // Compute distance
        final Vector3D vBase = sB.subtract(sA);            // S_b - S_a
        final double svA = Vector3D.dotProduct(vBase, vA); // SV_a = (S_b - S_a).V_a
        final double svB = Vector3D.dotProduct(vBase, vB); // SV_b = (S_b - S_a).V_b

        final double vAvB = Vector3D.dotProduct(vA, vB); // V_a.V_b

        // Compute lambda_b = (SV_a * V_a.V_b - SV_b) / (1 - (V_a.V_b)²)
        final double lambdaB = (svA * vAvB - svB) / (1 - vAvB * vAvB);

        // Compute lambda_a = SV_a + lambdaB * V_a.V_b
        final double lambdaA = svA + lambdaB * vAvB;

        // Compute vector M_a = S_a + lambda_a * V_a
        final Vector3D mA = sA.add(vA.scalarMultiply(lambdaA));
        // Compute vector M_b = S_b + lambda_b * V_b
        final Vector3D mB = sB.add(vB.scalarMultiply(lambdaB));

        // Compute vector M_a -> M_B for which distance between LOS is minimum
        final Vector3D vDistanceMin = mB.subtract(mA); // M_b - M_a

        // Compute vector from mid point of vector M_a -> M_B to the ground (corresponds to minimum elevation)
        final Vector3D midPoint = (mB.add(mA)).scalarMultiply(0.5);

        // Get the euclidean norms to compute the minimum distances: between LOS and to the ground
        final double[] distances = {vDistanceMin.getNorm(), midPoint.getNorm()};

        return distances;
    }

    /** Compute distances between two line sensors with derivatives.
     * @param sensorA line sensor A
     * @param dateA current date for sensor A
     * @param pixelA pixel index for sensor A
     * @param scToBodyA spacecraftToBody transform for sensor A
     * @param sensorB line sensor B
     * @param dateB current date for sensor B
     * @param pixelB pixel index for sensor B
     * @param generator generator to use for building {@link DerivativeStructure} instances
     * @return distances computed, with derivatives, between LOS and to the ground
     * @see #distanceBetweenLOS(LineSensor, AbsoluteDate, double, SpacecraftToObservedBody, LineSensor, AbsoluteDate, double)
     */
    public DerivativeStructure[] distanceBetweenLOSderivatives(
                                 final LineSensor sensorA, final AbsoluteDate dateA, final double pixelA,
                                 final SpacecraftToObservedBody scToBodyA,
                                 final LineSensor sensorB, final AbsoluteDate dateB, final double pixelB,
                                 final DSGenerator generator) {

        // Compute the approximate transforms between spacecraft and observed body
        // from Rugged instance A
        final Transform scToInertA = scToBodyA.getScToInertial(dateA);
        final Transform inertToBodyA = scToBodyA.getInertialToBody(dateA);
        final Transform transformScToBodyA = new Transform(dateA, scToInertA, inertToBodyA);

        // from (current) Rugged instance B
        final Transform scToInertB = scToBody.getScToInertial(dateB);
        final Transform inertToBodyB = scToBody.getInertialToBody(dateB);
        final Transform transformScToBodyB = new Transform(dateB, scToInertB, inertToBodyB);

        // Get sensors LOS into local frame
        final FieldVector3D<DerivativeStructure> vALocal = sensorA.getLOSDerivatives(dateA, pixelA, generator);
        final FieldVector3D<DerivativeStructure> vBLocal = sensorB.getLOSDerivatives(dateB, pixelB, generator);

        // Get sensors LOS into body frame
        final FieldVector3D<DerivativeStructure> vA = transformScToBodyA.transformVector(vALocal); // V_a : line of sight's vectorA
        final FieldVector3D<DerivativeStructure> vB = transformScToBodyB.transformVector(vBLocal); // V_b : line of sight's vectorB

        // Position of sensors into local frame
        final Vector3D sAtmp = sensorA.getPosition();
        final Vector3D sBtmp = sensorB.getPosition();

        final DerivativeStructure scaleFactor = FieldVector3D.dotProduct(vA.normalize(), vA.normalize()); // V_a.V_a=1

        // Build a vector from the position and a scale factor (equals to 1).
        // The vector built will be scaleFactor * sAtmp for example.
        final FieldVector3D<DerivativeStructure> sALocal = new FieldVector3D<DerivativeStructure>(scaleFactor, sAtmp);
        final FieldVector3D<DerivativeStructure> sBLocal = new FieldVector3D<DerivativeStructure>(scaleFactor, sBtmp);

        // Get sensors position into body frame
        final FieldVector3D<DerivativeStructure> sA = transformScToBodyA.transformPosition(sALocal); // S_a : sensorA 's position
        final FieldVector3D<DerivativeStructure> sB = transformScToBodyB.transformPosition(sBLocal); // S_b : sensorB 's position

        // Compute distance
        final FieldVector3D<DerivativeStructure> vBase = sB.subtract(sA);    // S_b - S_a
        final DerivativeStructure svA = FieldVector3D.dotProduct(vBase, vA); // SV_a = (S_b - S_a).V_a
        final DerivativeStructure svB = FieldVector3D.dotProduct(vBase, vB); // SV_b = (S_b - S_a).V_b

        final DerivativeStructure vAvB = FieldVector3D.dotProduct(vA, vB); // V_a.V_b

        // Compute lambda_b = (SV_a * V_a.V_b - SV_b) / (1 - (V_a.V_b)²)
        final DerivativeStructure lambdaB = (svA.multiply(vAvB).subtract(svB)).divide(vAvB.multiply(vAvB).subtract(1).negate());

        // Compute lambda_a = SV_a + lambdaB * V_a.V_b
        final DerivativeStructure lambdaA = vAvB.multiply(lambdaB).add(svA);

        // Compute vector M_a:
        final FieldVector3D<DerivativeStructure> mA = sA.add(vA.scalarMultiply(lambdaA)); // M_a = S_a + lambda_a * V_a
        // Compute vector M_b
        final FieldVector3D<DerivativeStructure> mB = sB.add(vB.scalarMultiply(lambdaB)); // M_b = S_b + lambda_b * V_b

        // Compute vector M_a -> M_B for which distance between LOS is minimum
        final FieldVector3D<DerivativeStructure> vDistanceMin = mB.subtract(mA); // M_b - M_a

        // Compute vector from mid point of vector M_a -> M_B to the ground (corresponds to minimum elevation)
        final FieldVector3D<DerivativeStructure> midPoint = (mB.add(mA)).scalarMultiply(0.5);

        // Get the euclidean norms to compute the minimum distances:
        // between LOS
        final DerivativeStructure dMin = vDistanceMin.getNorm();
        // to the ground
        final DerivativeStructure dCentralBody = midPoint.getNorm();

        return new DerivativeStructure[] {dMin, dCentralBody};
    }


    /** Get the mean plane crossing finder for a sensor.
     * @param sensorName name of the line sensor
     * @param minLine minimum line number
     * @param maxLine maximum line number
     * @return mean plane crossing finder
     */
    private SensorMeanPlaneCrossing getPlaneCrossing(final String sensorName,
                                                     final int minLine, final int maxLine) {

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
     */
    private void setPlaneCrossing(final SensorMeanPlaneCrossing planeCrossing) {
        finders.put(planeCrossing.getSensor().getName(), planeCrossing);
    }

    /** Inverse location of a point with derivatives.
     * @param sensorName name of the line sensor
     * @param point point to localize
     * @param minLine minimum line number
     * @param maxLine maximum line number
     * @param generator generator to use for building {@link DerivativeStructure} instances
     * @return sensor pixel seeing point with derivatives, or null if point cannot be seen between the
     * prescribed line numbers
     * @see #inverseLocation(String, GeodeticPoint, int, int)
     * @since 2.0
     */

    public DerivativeStructure[] inverseLocationDerivatives(final String sensorName,
                                                            final GeodeticPoint point,
                                                            final int minLine,
                                                            final int maxLine,
                                                            final DSGenerator generator) {

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

    /** Get transform from spacecraft to inertial frame.
     * @param date date of the transform
     * @return transform from spacecraft to inertial frame
     */
    public Transform getScToInertial(final AbsoluteDate date) {
        return scToBody.getScToInertial(date);
    }

    /** Get transform from inertial frame to observed body frame.
     * @param date date of the transform
     * @return transform from inertial frame to observed body frame
     */
    public Transform getInertialToBody(final AbsoluteDate date) {
        return scToBody.getInertialToBody(date);
    }

    /** Get transform from observed body frame to inertial frame.
     * @param date date of the transform
     * @return transform from observed body frame to inertial frame
     */
    public Transform getBodyToInertial(final AbsoluteDate date) {
        return scToBody.getBodyToInertial(date);
    }

    /** Get a sensor.
     * @param sensorName sensor name
     * @return selected sensor
     */
    public LineSensor getLineSensor(final String sensorName) {

        final LineSensor sensor = sensors.get(sensorName);
        if (sensor == null) {
            throw new RuggedException(RuggedMessages.UNKNOWN_SENSOR, sensorName);
        }
        return sensor;
    }

    /** Get converter between spacecraft and body.
     * @return the scToBody
     * @since 2.0
     */
    public SpacecraftToObservedBody getScToBody() {
        return scToBody;
    }
}
