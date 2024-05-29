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
package org.orekit.rugged.utils;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import org.hipparchus.util.FastMath;
import org.orekit.frames.Frame;
import org.orekit.frames.Transform;
import org.orekit.rugged.errors.DumpManager;
import org.orekit.rugged.errors.RuggedException;
import org.orekit.rugged.errors.RuggedMessages;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeInterpolator;
import org.orekit.utils.AngularDerivativesFilter;
import org.orekit.utils.CartesianDerivativesFilter;
import org.orekit.utils.ImmutableTimeStampedCache;
import org.orekit.utils.TimeStampedAngularCoordinates;
import org.orekit.utils.TimeStampedAngularCoordinatesHermiteInterpolator;
import org.orekit.utils.TimeStampedCache;
import org.orekit.utils.TimeStampedPVCoordinates;
import org.orekit.utils.TimeStampedPVCoordinatesHermiteInterpolator;

/** Provider for observation transforms.
 * @author Luc Maisonobe
 * @author Guylaine Prat
 */
public class SpacecraftToObservedBody implements Serializable {

    /** Serializable UID. */
    private static final long serialVersionUID = 20140909L;

    /** Inertial frame. */
    private final Frame inertialFrame;

    /** Body frame. */
    private final Frame bodyFrame;

    /** Start of search time span. */
    private final AbsoluteDate minDate;

    /** End of search time span. */
    private final AbsoluteDate maxDate;

    /** Step to use for inertial frame to body frame transforms cache computations. */
    private final double tStep;

    /** Tolerance in seconds allowed for {@code minDate} and {@code maxDate} overshooting. */
    private final double overshootTolerance;

    /** Transforms sample from observed body frame to inertial frame. */
    private final List<Transform> bodyToInertial;

    /** Transforms sample from inertial frame to observed body frame. */
    private final List<Transform> inertialToBody;

    /** Transforms sample from spacecraft frame to inertial frame. */
    private final List<Transform> scToInertial;

    /** Simple constructor.
     * @param inertialFrame inertial frame
     * @param bodyFrame observed body frame
     * @param minDate start of search time span
     * @param maxDate end of search time span
     * @param tStep step to use for inertial frame to body frame transforms cache computations
     * @param overshootTolerance tolerance in seconds allowed for {@code minDate} and {@code maxDate} overshooting
     * slightly the position, velocity and quaternions ephemerides
     * @param positionsVelocities satellite position and velocity
     * @param pvInterpolationNumber number of points to use for position/velocity interpolation
     * @param pvFilter filter for derivatives from the sample to use in position/velocity interpolation
     * @param quaternions satellite quaternions
     * @param aInterpolationNumber number of points to use for attitude interpolation
     * @param aFilter filter for derivatives from the sample to use in attitude interpolation
     */
    public SpacecraftToObservedBody(final Frame inertialFrame, final Frame bodyFrame,
                                    final AbsoluteDate minDate, final AbsoluteDate maxDate, final double tStep,
                                    final double overshootTolerance,
                                    final List<TimeStampedPVCoordinates> positionsVelocities, final int pvInterpolationNumber,
                                    final CartesianDerivativesFilter pvFilter,
                                    final List<TimeStampedAngularCoordinates> quaternions, final int aInterpolationNumber,
                                    final AngularDerivativesFilter aFilter) {

        this.inertialFrame      = inertialFrame;
        this.bodyFrame          = bodyFrame;
        this.minDate            = minDate;
        this.maxDate            = maxDate;
        this.overshootTolerance = overshootTolerance;

        // safety checks
        final AbsoluteDate minPVDate = positionsVelocities.get(0).getDate();
        final AbsoluteDate maxPVDate = positionsVelocities.get(positionsVelocities.size() - 1).getDate();
        if (minPVDate.durationFrom(minDate) > overshootTolerance) {
            throw new RuggedException(RuggedMessages.OUT_OF_TIME_RANGE, minDate, minPVDate, maxPVDate);
        }
        if (maxDate.durationFrom(maxPVDate) > overshootTolerance) {
            throw new RuggedException(RuggedMessages.OUT_OF_TIME_RANGE, maxDate, minPVDate, maxPVDate);
        }

        final AbsoluteDate minQDate  = quaternions.get(0).getDate();
        final AbsoluteDate maxQDate  = quaternions.get(quaternions.size() - 1).getDate();
        if (minQDate.durationFrom(minDate) > overshootTolerance) {
            throw new RuggedException(RuggedMessages.OUT_OF_TIME_RANGE, minDate, minQDate, maxQDate);
        }
        if (maxDate.durationFrom(maxQDate) > overshootTolerance) {
            throw new RuggedException(RuggedMessages.OUT_OF_TIME_RANGE, maxDate, minQDate, maxQDate);
        }

        // set up the cache for position-velocities
        final TimeStampedCache<TimeStampedPVCoordinates> pvCache =
                new ImmutableTimeStampedCache<>(pvInterpolationNumber, positionsVelocities);

        // set up the TimeStampedPVCoordinates interpolator
        final TimeInterpolator<TimeStampedPVCoordinates> pvInterpolator =
                new TimeStampedPVCoordinatesHermiteInterpolator(pvInterpolationNumber, pvFilter);

        // set up the cache for attitudes
        final TimeStampedCache<TimeStampedAngularCoordinates> aCache =
                new ImmutableTimeStampedCache<>(aInterpolationNumber, quaternions);

        // set up the TimeStampedAngularCoordinates Hermite interpolator
        final TimeInterpolator<TimeStampedAngularCoordinates> angularInterpolator =
                new TimeStampedAngularCoordinatesHermiteInterpolator(aInterpolationNumber, aFilter);

        final int n = (int) FastMath.ceil(maxDate.durationFrom(minDate) / tStep);
        this.tStep          = tStep;
        this.bodyToInertial = new ArrayList<>(n);
        this.inertialToBody = new ArrayList<>(n);
        this.scToInertial   = new ArrayList<>(n);
        for (AbsoluteDate date = minDate; bodyToInertial.size() < n; date = date.shiftedBy(tStep)) {

            // interpolate position-velocity, allowing slight extrapolation near the boundaries
            final AbsoluteDate pvInterpolationDate;
            if (date.compareTo(pvCache.getEarliest().getDate()) < 0) {
                pvInterpolationDate = pvCache.getEarliest().getDate();
            } else if (date.compareTo(pvCache.getLatest().getDate()) > 0) {
                pvInterpolationDate = pvCache.getLatest().getDate();
            } else {
                pvInterpolationDate = date;
            }
            final TimeStampedPVCoordinates interpolatedPV =
                    pvInterpolator.interpolate(pvInterpolationDate,
                            pvCache.getNeighbors(pvInterpolationDate));
            final TimeStampedPVCoordinates pv = interpolatedPV.shiftedBy(date.durationFrom(pvInterpolationDate));

            // interpolate attitude, allowing slight extrapolation near the boundaries
            final AbsoluteDate aInterpolationDate;
            if (date.compareTo(aCache.getEarliest().getDate()) < 0) {
                aInterpolationDate = aCache.getEarliest().getDate();
            } else if (date.compareTo(aCache.getLatest().getDate()) > 0) {
                aInterpolationDate = aCache.getLatest().getDate();
            } else {
                aInterpolationDate = date;
            }
            final TimeStampedAngularCoordinates interpolatedQuaternion =
                    angularInterpolator.interpolate(aInterpolationDate,
                            aCache.getNeighbors(aInterpolationDate).collect(Collectors.toList()));
            final TimeStampedAngularCoordinates quaternion = interpolatedQuaternion.shiftedBy(date.durationFrom(aInterpolationDate));

            // store transform from spacecraft frame to inertial frame
            scToInertial.add(new Transform(date,
                    new Transform(date, quaternion.revert()),
                    new Transform(date, pv)));

            // store transform from body frame to inertial frame
            final Transform b2i = bodyFrame.getTransformTo(inertialFrame, date);
            bodyToInertial.add(b2i);
            inertialToBody.add(b2i.getInverse());

        }
    }

    /** Simple constructor.
     * @param inertialFrame inertial frame
     * @param bodyFrame observed body frame
     * @param minDate start of search time span
     * @param maxDate end of search time span
     * @param tStep step to use for inertial frame to body frame transforms cache computations
     * @param overshootTolerance tolerance in seconds allowed for {@code minDate} and {@code maxDate} overshooting
     * slightly the position, velocity and quaternions ephemerides
     * @param bodyToInertial transforms sample from observed body frame to inertial frame
     * @param scToInertial transforms sample from spacecraft frame to inertial frame
     */
    public SpacecraftToObservedBody(final Frame inertialFrame, final Frame bodyFrame,
                                    final AbsoluteDate minDate, final AbsoluteDate maxDate, final double tStep,
                                    final double overshootTolerance,
                                    final List<Transform> bodyToInertial, final List<Transform> scToInertial) {

        this.inertialFrame      = inertialFrame;
        this.bodyFrame          = bodyFrame;
        this.minDate            = minDate;
        this.maxDate            = maxDate;
        this.tStep              = tStep;
        this.overshootTolerance = overshootTolerance;
        this.bodyToInertial     = bodyToInertial;
        this.scToInertial       = scToInertial;

        this.inertialToBody = new ArrayList<>(bodyToInertial.size());
        for (final Transform b2i : bodyToInertial) {
            inertialToBody.add(b2i.getInverse());
        }

    }

    /** Get the inertial frame.
     * @return inertial frame
     */
    public Frame getInertialFrame() {
        return inertialFrame;
    }

    /** Get the body frame.
     * @return body frame
     */
    public Frame getBodyFrame() {
        return bodyFrame;
    }

    /** Get the start of search time span.
     * @return start of search time span
     */
    public AbsoluteDate getMinDate() {
        return minDate;
    }

    /** Get the end of search time span.
     * @return end of search time span
     */
    public AbsoluteDate getMaxDate() {
        return maxDate;
    }

    /** Get the step to use for inertial frame to body frame transforms cache computations.
     * @return step to use for inertial frame to body frame transforms cache computations
     */
    public double getTStep() {
        return tStep;
    }

    /** Get the tolerance in seconds allowed for {@link #getMinDate()} and {@link #getMaxDate()} overshooting.
     * @return tolerance in seconds allowed for {@link #getMinDate()} and {@link #getMaxDate()} overshooting
     */
    public double getOvershootTolerance() {
        return overshootTolerance;
    }

    /** Get transform from spacecraft to inertial frame.
     * @param date date of the transform
     * @return transform from spacecraft to inertial frame
     */
    public Transform getScToInertial(final AbsoluteDate date) {
        return interpolate(date, scToInertial);
    }

    /** Get transform from inertial frame to observed body frame.
     * @param date date of the transform
     * @return transform from inertial frame to observed body frame
     */
    public Transform getInertialToBody(final AbsoluteDate date) {
        return interpolate(date, inertialToBody);
    }

    /** Get transform from observed body frame to inertial frame.
     * @param date date of the transform
     * @return transform from observed body frame to inertial frame
     */
    public Transform getBodyToInertial(final AbsoluteDate date) {
        return interpolate(date, bodyToInertial);
    }

    /** Interpolate transform.
     * @param date date of the transform
     * @param list transforms list to interpolate from
     * @return interpolated transform
     */
    private Transform interpolate(final AbsoluteDate date, final List<Transform> list) {

        // check date range
        if (!isInRange(date)) {
            throw new RuggedException(RuggedMessages.OUT_OF_TIME_RANGE, date, minDate, maxDate);
        }

        final double    s     = date.durationFrom(list.get(0).getDate()) / tStep;
        final int       index = FastMath.max(0, FastMath.min(list.size() - 1, (int) FastMath.rint(s)));

        DumpManager.dumpTransform(this, index, bodyToInertial.get(index), scToInertial.get(index));

        final Transform close = list.get(index);
        return close.shiftedBy(date.durationFrom(close.getDate()));

    }

    /** Check if a date is in the supported range.
     * @param date date to check
     * @return true if date is in the supported range
     */
    public boolean isInRange(final AbsoluteDate date) {
        return minDate.durationFrom(date) <= overshootTolerance &&
               date.durationFrom(maxDate) <= overshootTolerance;
    }

}
