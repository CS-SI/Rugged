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

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.util.Pair;
import org.orekit.attitudes.Attitude;
import org.orekit.attitudes.TabulatedProvider;
import org.orekit.errors.OrekitException;
import org.orekit.frames.Frame;
import org.orekit.frames.Transform;
import org.orekit.orbits.CartesianOrbit;
import org.orekit.orbits.Orbit;
import org.orekit.rugged.api.RuggedException;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.Constants;
import org.orekit.utils.ImmutableTimeStampedCache;
import org.orekit.utils.PVCoordinates;

/** Provider for observation transforms.
 * @author Luc Maisonobe
 */
public class SpacecraftToObservedBody {

    /** Inertial frame. */
    private final Frame inertialFrame;

    /** Observed body frame. */
    private final Frame bodyFrame;

    /** Satellite orbits. */
    private final ImmutableTimeStampedCache<Orbit> orbits;

    /** Satellite quaternions. */
    private final TabulatedProvider attitudes;

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
                                    final List<Pair<AbsoluteDate, PVCoordinates>> positionsVelocities, final int pvInterpolationOrder,
                                    final List<Pair<AbsoluteDate, Rotation>> quaternions, final int aInterpolationOrder)
        throws RuggedException {

        this.inertialFrame        = inertialFrame;
        this.bodyFrame            = bodyFrame;

        // set up the orbit provider
        final List<Orbit> orbits = new ArrayList<Orbit>(positionsVelocities.size());
        for (final Pair<AbsoluteDate, PVCoordinates> pv : positionsVelocities) {
            final CartesianOrbit orbit = new CartesianOrbit(pv.getSecond(), inertialFrame,
                                                            pv.getFirst(), Constants.EIGEN5C_EARTH_MU);
            orbits.add(orbit);
        }
        this.orbits = new ImmutableTimeStampedCache<Orbit>(pvInterpolationOrder, orbits);

        // set up the attitude provider
        final List<Attitude> attitudes = new ArrayList<Attitude>(quaternions.size());
        for (final Pair<AbsoluteDate, Rotation> q : quaternions) {
            attitudes.add(new Attitude(q.getFirst(), inertialFrame, q.getSecond(), Vector3D.ZERO));
        }
        this.attitudes = new TabulatedProvider(attitudes, aInterpolationOrder, false);

    }

    /** Get transform from spacecraft to inertial frame.
     * @param date date of the transform
     * @return transform from spacecraft to inertial frame
     * @exception OrekitException if spacecraft position or attitude cannot be computed at date
     */
    public Transform getScToInertial(final AbsoluteDate date)
        throws OrekitException {

        //interpolate orbit and attitude
        final List<Orbit>   sample = orbits.getNeighbors(date);
        final Orbit         orbit  = sample.get(0).interpolate(date, sample);
        final PVCoordinates pv     = orbit.getPVCoordinates(date, inertialFrame);

        //interpolate attitude
        final Attitude attitude = attitudes.getAttitude(orbit, date, inertialFrame);

        // compute transform from spacecraft frame to inertial frame
        return new Transform(date,
                             new Transform(date, attitude.getOrientation().revert()),
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
