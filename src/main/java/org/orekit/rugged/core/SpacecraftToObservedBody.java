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
package org.orekit.rugged.core;

import org.orekit.attitudes.Attitude;
import org.orekit.attitudes.AttitudeProvider;
import org.orekit.errors.OrekitException;
import org.orekit.frames.Frame;
import org.orekit.frames.Transform;
import org.orekit.frames.TransformProvider;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.PVCoordinates;
import org.orekit.utils.PVCoordinatesProvider;

/** Transform provider from Spacecraft frame to observed body frame.
 * @author Luc Maisonobe
 */
class SpacecraftToObservedBody implements TransformProvider {

    /** Serializable UID. */
    private static final long serialVersionUID = 20140311L;

    /** Inertial frame. */
    private Frame inertialFrame;

    /** Observed body frame. */
    private Frame bodyFrame;

    /** Orbit propagator/interpolator. */
    private PVCoordinatesProvider pvProvider;

    /** Attitude propagator/interpolator. */
    private AttitudeProvider aProvider;

    /** Simple constructor.
     * @param inertialFrame inertial frame
     * @param bodyFrame observed body frame
     * @param pvProvider orbit propagator/interpolator
     * @param aProvider attitude propagator/interpolator
     */
    protected SpacecraftToObservedBody(final Frame inertialFrame, final Frame bodyFrame,
                                       final PVCoordinatesProvider pvProvider,
                                       final AttitudeProvider aProvider) {
        this.inertialFrame = inertialFrame;
        this.bodyFrame     = bodyFrame;
        this.pvProvider    = pvProvider;
        this.aProvider     = aProvider;
    }

    /** {@inheritDoc} */
    @Override
    public Transform getTransform(final AbsoluteDate date)
        throws OrekitException {

        // propagate/interpolate orbit and attitude
        final PVCoordinates pv    = pvProvider.getPVCoordinates(date, inertialFrame);
        final Attitude attitude   = aProvider.getAttitude(pvProvider, date, inertialFrame);

        // compute transform from spacecraft frame to inertial frame
        final Transform scToInert = new Transform(date,
                                                  new Transform(date, attitude.getOrientation().revert()),
                                                  new Transform(date, pv));

        // compute transform from inertial frame to body frame
        final Transform inertToBody = inertialFrame.getTransformTo(bodyFrame, date);

        // combine transforms
        return new Transform(date, scToInert, inertToBody);

    }

}
