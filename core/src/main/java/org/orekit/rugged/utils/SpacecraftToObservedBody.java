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

import java.util.List;

import org.orekit.errors.OrekitException;
import org.orekit.frames.Frame;
import org.orekit.frames.Transform;
import org.orekit.rugged.api.RuggedException;
import org.orekit.rugged.api.RuggedMessages;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.ImmutableTimeStampedCache;
import org.orekit.utils.TimeStampedAngularCoordinates;
import org.orekit.utils.TimeStampedPVCoordinates;

/** Provider for observation transforms.
 * @author Luc Maisonobe
 */
public class SpacecraftToObservedBody {

    /** Inertial frame. */
    private final Frame inertialFrame;

    /** Observed body frame. */
    private final Frame bodyFrame;

    /** Satellite orbits. */
    private final ImmutableTimeStampedCache<TimeStampedPVCoordinates> positionsVelocities;

    /** Satellite quaternions. */
    private final ImmutableTimeStampedCache<TimeStampedAngularCoordinates> quaternions;

    /** Simple constructor.
     * @param inertialFrame inertial frame
     * @param bodyFrame observed body frame
     * @param minDate start of search time span
     * @param maxDate end of search time span
     * @param positionsVelocities satellite position and velocity
     * @param pvInterpolationOrder order to use for position/velocity interpolation
     * @param quaternions satellite quaternions
     * @param aInterpolationOrder order to use for attitude interpolation
     * @exception RuggedException if position or attitude samples do not fully cover the
     * [{@code minDate}, {@code maxDate}] search time span
     */
    public SpacecraftToObservedBody(final Frame inertialFrame, final Frame bodyFrame,
                                    final AbsoluteDate minDate, final AbsoluteDate maxDate,
                                    final List<TimeStampedPVCoordinates> positionsVelocities, final int pvInterpolationOrder,
                                    final List<TimeStampedAngularCoordinates> quaternions, final int aInterpolationOrder)
        throws RuggedException {

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

        this.inertialFrame = inertialFrame;
        this.bodyFrame     = bodyFrame;

        // set up the cache for position-velocities
        this.positionsVelocities = new ImmutableTimeStampedCache<TimeStampedPVCoordinates>(pvInterpolationOrder, positionsVelocities);

        // set up the cache for attitudes
        this.quaternions = new ImmutableTimeStampedCache<TimeStampedAngularCoordinates>(aInterpolationOrder, quaternions);

    }

    /** Get transform from spacecraft to inertial frame.
     * @param date date of the transform
     * @return transform from spacecraft to inertial frame
     * @exception OrekitException if spacecraft position or attitude cannot be computed at date
     */
    public Transform getScToInertial(final AbsoluteDate date)
        throws OrekitException {

        // interpolate position-velocity
        final List<TimeStampedPVCoordinates> sample = positionsVelocities.getNeighbors(date);
        final TimeStampedPVCoordinates pv = TimeStampedPVCoordinates.interpolate(date, true, sample);

        // interpolate attitude
        final List<TimeStampedAngularCoordinates> sampleAC = quaternions.getNeighbors(date);
        final TimeStampedAngularCoordinates quaternion = TimeStampedAngularCoordinates.interpolate(date, false, sampleAC);

        // compute transform from spacecraft frame to inertial frame
        return new Transform(date,
                             new Transform(date, quaternion.revert()),
                             new Transform(date, pv));

    }

    /** Get transform from inertial frame to body frame.
     * @param date date of the transform
     * @return transform from inertial frame to body frame
     * @exception OrekitException if frames cannot be computed at date
     */
    public Transform getInertialToBody(final AbsoluteDate date)
        throws OrekitException {

        // compute transform from inertial frame to body frame
        return inertialFrame.getTransformTo(bodyFrame, date);

    }

}
