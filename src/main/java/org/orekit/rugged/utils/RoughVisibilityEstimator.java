/* Copyright 2013-2018 CS Systèmes d'Information
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

import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.util.FastMath;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.bodies.OneAxisEllipsoid;
import org.orekit.frames.Frame;
import org.orekit.frames.Transform;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.TimeStampedPVCoordinates;

/** Class estimating very roughly when a point may be visible from spacecraft.
 * <p>
 * The class only uses spacecraft position to compute a very rough sub-satellite
 * point. It assumes the position-velocities are regular enough and without holes.
 * It is intended only only has a quick estimation in order to set up search
 * boundaries in inverse location.
 * </p>
 * @see org.orekit.rugged.api.Rugged#dateLocation(String, org.orekit.bodies.GeodeticPoint, int, int)
 * @see org.orekit.rugged.api.Rugged#dateLocation(String, double, double, int, int)
 * @see org.orekit.rugged.api.Rugged#inverseLocation(String, org.orekit.bodies.GeodeticPoint, int, int)
 * @see org.orekit.rugged.api.Rugged#inverseLocation(String, double, double, int, int)
 * @author Luc Maisonobe
 */
public class RoughVisibilityEstimator {

    /** Ground ellipsoid. */
    private final OneAxisEllipsoid ellipsoid;

    /** Sub-satellite point. */
    private final List<TimeStampedPVCoordinates> pvGround;

    /** Mean angular rate with respect to position-velocity indices. */
    private final double rateVSIndices;

    /** Mean angular rate with respect to time. */
    private final double rateVSTime;

    /** Last found index. */
    private int last;

    /**
     * Simple constructor.
     * @param ellipsoid ground ellipsoid
     * @param frame frame in which position and velocity are defined (may be inertial or body frame)
     * @param positionsVelocities satellite position and velocity (m and m/s in specified frame)
     */
    public RoughVisibilityEstimator(final OneAxisEllipsoid ellipsoid, final Frame frame,
                                    final List<TimeStampedPVCoordinates> positionsVelocities) {

        this.ellipsoid = ellipsoid;

        // project spacecraft position-velocity to ground
        final Frame bodyFrame = ellipsoid.getBodyFrame();
        final int n = positionsVelocities.size();
        this.pvGround = new ArrayList<TimeStampedPVCoordinates>(n);
        for (final TimeStampedPVCoordinates pv : positionsVelocities) {
            final Transform t = frame.getTransformTo(bodyFrame, pv.getDate());
            pvGround.add(ellipsoid.projectToGround(t.transformPVCoordinates(pv), bodyFrame));
        }

        // initialize first search at mid point
        this.last = n / 2;

        // estimate mean angular rate with respect to indices
        double alpha = 0;
        for (int i = 0; i < n - 1; ++i) {
            // angular motion between points i and i+1
            alpha += Vector3D.angle(pvGround.get(i).getPosition(),
                                    pvGround.get(i + 1).getPosition());
        }
        this.rateVSIndices = alpha / n;

        // estimate mean angular rate with respect to time
        final AbsoluteDate firstDate = pvGround.get(0).getDate();
        final AbsoluteDate lastDate  = pvGround.get(pvGround.size() - 1).getDate();
        this.rateVSTime              = alpha / lastDate.durationFrom(firstDate);

    }

    /** Estimate <em>very roughly</em> when spacecraft comes close to a ground point.
     * @param groundPoint ground point to check
     * @return rough date at which spacecraft comes close to ground point (never null,
     * but may be really far from reality if ground point is away from trajectory)
     */
    public AbsoluteDate estimateVisibility(final GeodeticPoint groundPoint) {

        final Vector3D point = ellipsoid.transform(groundPoint);
        int closeIndex = findClose(last, point);

        // check if there are closer points in previous periods
        final int repeat = (int) FastMath.rint(2.0 * FastMath.PI / rateVSIndices);
        for (int index = closeIndex - repeat; index > 0; index -= repeat) {
            final int otherIndex = findClose(index, point);
            if (otherIndex != closeIndex &&
                Vector3D.distance(pvGround.get(otherIndex).getPosition(), point) <
                Vector3D.distance(pvGround.get(closeIndex).getPosition(), point)) {
                closeIndex = otherIndex;
            }
        }

        // check if there are closer points in next periods
        for (int index = closeIndex + repeat; index < pvGround.size(); index += repeat) {
            final int otherIndex = findClose(index, point);
            if (otherIndex != closeIndex &&
                Vector3D.distance(pvGround.get(otherIndex).getPosition(), point) <
                Vector3D.distance(pvGround.get(closeIndex).getPosition(), point)) {
                closeIndex = otherIndex;
            }
        }

        // we have found the closest sub-satellite point index
        last = closeIndex;

        // final adjustment
        final TimeStampedPVCoordinates closest = pvGround.get(closeIndex);
        final double alpha = neededMotion(closest, point);
        return closest.getDate().shiftedBy(alpha / rateVSTime);

    }

    /** Find the index of a close sub-satellite point.
     * @param start start index for the search
     * @param point test point
     * @return index of a sub-satellite point close to the test point
     */
    private int findClose(final int start, final Vector3D point) {
        int current  = start;
        int previous = Integer.MIN_VALUE;
        int maxLoop  = 1000;
        while (maxLoop-- > 0 && FastMath.abs(current - previous) > 1) {
            previous = current;
            final double alpha = neededMotion(pvGround.get(current), point);
            final int    shift = (int) FastMath.rint(alpha / rateVSIndices);
            current = FastMath.max(0, FastMath.min(pvGround.size() - 1, current + shift));
        }
        return current;
    }

    /** Estimate angular motion needed to go past test point.
     * <p>
     * This estimation is quite crude. The sub-satellite point is properly on the
     * ellipsoid surface, but we compute the angle assuming a spherical shape.
     * </p>
     * @param subSatellite current sub-satellite position-velocity
     * @param point test point
     * @return angular motion to go past test point (positive is
     * test point is ahead, negative if it is behind)
     */
    private double neededMotion(final TimeStampedPVCoordinates subSatellite,
                                final Vector3D point) {

        final Vector3D ssP      = subSatellite.getPosition();
        final Vector3D momentum = subSatellite.getMomentum();
        final double   y        = Vector3D.dotProduct(point, Vector3D.crossProduct(momentum, ssP).normalize());
        final double   x        = Vector3D.dotProduct(point, ssP.normalize());

        return FastMath.atan2(y, x);

    }

}
