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
package org.orekit.rugged.utils;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.math3.util.FastMath;
import org.orekit.errors.OrekitException;
import org.orekit.frames.Frame;
import org.orekit.frames.Transform;
import org.orekit.rugged.api.RuggedException;
import org.orekit.rugged.api.RuggedMessages;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.ImmutableTimeStampedCache;
import org.orekit.utils.TimeStampedAngularCoordinates;
import org.orekit.utils.TimeStampedCache;
import org.orekit.utils.TimeStampedPVCoordinates;

/** Provider for observation transforms.
 * @author Luc Maisonobe
 */
public class SpacecraftToObservedBody {

    /** Step to use for inertial frame to body frame transforms cache computations. */
    private final double tStep;

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
     * @param positionsVelocities satellite position and velocity
     * @param pvInterpolationOrder order to use for position/velocity interpolation
     * @param quaternions satellite quaternions
     * @param aInterpolationOrder order to use for attitude interpolation
     * @param tStep step to use for inertial frame to body frame transforms cache computations
     * @exception RuggedException if position or attitude samples do not fully cover the
     * [{@code minDate}, {@code maxDate}] search time span
     */
    public SpacecraftToObservedBody(final Frame inertialFrame, final Frame bodyFrame,
                                    final AbsoluteDate minDate, final AbsoluteDate maxDate,
                                    final List<TimeStampedPVCoordinates> positionsVelocities, final int pvInterpolationOrder,
                                    final List<TimeStampedAngularCoordinates> quaternions, final int aInterpolationOrder,
                                    final double tStep)
        throws RuggedException {
        try {
            // safety checks
            final AbsoluteDate minPVDate = positionsVelocities.get(0).getDate();
            final AbsoluteDate maxPVDate = positionsVelocities.get(positionsVelocities.size() - 1).getDate();
            if (minDate.compareTo(minPVDate) < 0) {
                throw new RuggedException(RuggedMessages.OUT_OF_TIME_RANGE, minDate, minPVDate, maxPVDate);
            }
            if (maxDate.compareTo(maxPVDate) > 0) {
                throw new RuggedException(RuggedMessages.OUT_OF_TIME_RANGE, maxDate, minPVDate, maxPVDate);
            }

            final AbsoluteDate minQDate  = quaternions.get(0).getDate();
            final AbsoluteDate maxQDate  = quaternions.get(quaternions.size() - 1).getDate();
            if (minDate.compareTo(minQDate) < 0) {
                throw new RuggedException(RuggedMessages.OUT_OF_TIME_RANGE, minDate, minQDate, maxQDate);
            }
            if (maxDate.compareTo(maxQDate) > 0) {
                throw new RuggedException(RuggedMessages.OUT_OF_TIME_RANGE, maxDate, minQDate, maxQDate);
            }

            // set up the cache for position-velocities
            final TimeStampedCache<TimeStampedPVCoordinates> pvCache =
                    new ImmutableTimeStampedCache<TimeStampedPVCoordinates>(pvInterpolationOrder, positionsVelocities);

            // set up the cache for attitudes
            final TimeStampedCache<TimeStampedAngularCoordinates> aCache =
                    new ImmutableTimeStampedCache<TimeStampedAngularCoordinates>(aInterpolationOrder, quaternions);

            final int n = (int) FastMath.ceil(maxDate.durationFrom(minDate) / tStep);
            this.tStep          = tStep;
            this.bodyToInertial = new ArrayList<Transform>(n);
            this.inertialToBody = new ArrayList<Transform>(n);
            this.scToInertial   = new ArrayList<Transform>(n);
            for (AbsoluteDate date = minDate; bodyToInertial.size() < n; date = date.shiftedBy(tStep)) {

                // interpolate position-velocity
                final TimeStampedPVCoordinates pv =
                        TimeStampedPVCoordinates.interpolate(date, true, pvCache.getNeighbors(date));

                // interpolate attitude
                final TimeStampedAngularCoordinates quaternion =
                        TimeStampedAngularCoordinates.interpolate(date, false, aCache.getNeighbors(date));

                // store transform from spacecraft frame to inertial frame
                scToInertial.add(new Transform(date,
                                               new Transform(date, quaternion.revert()),
                                               new Transform(date, pv)));

                // store transform from body frame to inertial frame
                final Transform b2i = bodyFrame.getTransformTo(inertialFrame, date);
                bodyToInertial.add(b2i);
                inertialToBody.add(b2i.getInverse());

            }

        } catch (OrekitException oe) {
            throw new RuggedException(oe, oe.getSpecifier(), oe.getParts());
        }
    }

    /** Get transform from spacecraft to inertial frame.
     * @param date date of the transform
     * @return transform from spacecraft to inertial frame
     * @exception OrekitException if spacecraft position or attitude cannot be computed at date
     */
    public Transform getScToInertial(final AbsoluteDate date)
        throws OrekitException {
        return interpolate(date, scToInertial);
    }

    /** Get transform from inertial frame to observed body frame.
     * @param date date of the transform
     * @return transform from inertial frame to observed body frame
     * @exception OrekitException if frames cannot be computed at date
     */
    public Transform getInertialToBody(final AbsoluteDate date)
        throws OrekitException {
        return interpolate(date, inertialToBody);
    }

    /** Get transform from observed body frame to inertial frame.
     * @param date date of the transform
     * @return transform from observed body frame to inertial frame
     * @exception OrekitException if frames cannot be computed at date
     */
    public Transform getBodyToInertial(final AbsoluteDate date)
        throws OrekitException {
        return interpolate(date, bodyToInertial);
    }

    /** Interpolate transform.
     * @param date date of the transform
     * @param list transforms list to interpolate from
     * @return interpolated transform
     * @exception OrekitException if frames cannot be computed at date
     */
    private Transform interpolate(final AbsoluteDate date, final List<Transform> list)
        throws OrekitException {
        final double    s     = date.durationFrom(list.get(0).getDate()) / tStep;
        final int       index = FastMath.max(0, FastMath.min(list.size() - 1, (int) FastMath.rint(s)));
        final Transform close = list.get(index);
        return close.shiftedBy(date.durationFrom(close.getDate()));
    }

}
