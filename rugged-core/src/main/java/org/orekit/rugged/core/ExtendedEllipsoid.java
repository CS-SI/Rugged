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

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.util.FastMath;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.bodies.OneAxisEllipsoid;
import org.orekit.errors.OrekitException;
import org.orekit.frames.Frame;
import org.orekit.rugged.api.RuggedException;
import org.orekit.rugged.api.RuggedMessages;

/** Transform provider from Spacecraft frame to observed body frame.
 * @author Luc Maisonobe
 */
public class ExtendedEllipsoid extends OneAxisEllipsoid {

    /** Serializable UID. */
    private static final long serialVersionUID = 20140312L;

    /** Convergence threshold for {@link #pointAtAltitude(Vector3D, Vector3D, double)}. */
    private static final double ALTITUDE_CONVERGENCE = 1.0e-3;

    /** Simple constructor.
     * @param ae equatorial radius
     * @param f the flattening (f = (a-b)/a)
     * @param bodyFrame body frame related to body shape
     * @see org.orekit.frames.FramesFactory#getITRF(org.orekit.utils.IERSConventions, boolean)
     */
    public ExtendedEllipsoid(final double ae, final double f, final Frame bodyFrame) {
        super(ae, f, bodyFrame);
    }

    /** Get point at some latitude along a pixel line of sight.
     * @param position pixel position (in body frame)
     * @param los pixel line-of-sight, not necessarily normalized (in body frame)
     * @param latitude latitude with respect to ellipsoid
     * @return point at altitude
     * @exception RuggedException if no such point exists
     */
    public Vector3D pointAtLatitude(final Vector3D position, final Vector3D los, final double latitude)
        throws RuggedException {

        // find apex of iso-latitude cone, somewhere along polar axis
        final GeodeticPoint groundPoint = new GeodeticPoint(latitude, 0, 0);
        final Vector3D gpCartesian = transform(groundPoint);
        final double r             = FastMath.sqrt(gpCartesian.getX() * gpCartesian.getX() +
                                                   gpCartesian.getY() * gpCartesian.getY());
        final Vector3D zenith      = groundPoint.getZenith();
        final Vector3D apex        = new Vector3D(1, gpCartesian, -r / FastMath.cos(latitude), zenith);

        // quadratic equation representing line intersection with iso-latitude cone
        // a k² + 2 b k + c = 0
        final Vector3D delta = position.subtract(apex);
        final double zz2     = zenith.getZ() * zenith.getZ();
        final double   a     = zz2 * los.getNormSq()                 - los.getZ()   * los.getZ();
        final double   b     = zz2 * Vector3D.dotProduct(delta, los) - delta.getZ() * los.getZ();
        final double   c     = zz2 * delta.getNormSq()               - delta.getZ() * delta.getZ();

        // find the two intersections along the line
        final double   b2    = b * b;
        final double   ac    = a * c;
        if (b2 < ac) {
            throw new RuggedException(RuggedMessages.LINE_OF_SIGHT_NEVER_CROSSES_LATITUDE,
                                      FastMath.toDegrees(latitude));
        }
        final double s  = FastMath.sqrt(b2 - ac);
        final double k1 = (b > 0) ? -(s + b) / a : c / (s - b);
        final double k2 = c / (a * k1);

        // the quadratic equation may find spurious solutions in the wrong cone nappe
        final boolean  k1IsOK = (delta.getZ() + k1 * los.getZ()) * latitude >= 0;
        final boolean  k2IsOK = (delta.getZ() + k2 * los.getZ()) * latitude >= 0;
        final double selectedK;
        if (k1IsOK) {
            if (k2IsOK) {
                // both solutions are in the good nappe,
                // we arbitrarily select the one closest to the initial position
                selectedK = FastMath.abs(k1) <= FastMath.abs(k2) ? k1 : k2;
            } else {
                // only k1 is in the good nappe
                selectedK = k1;
            }
        } else {  
            if (k2IsOK) {
                // only k2 is in the good nappe
                selectedK = k2;
            } else {
                // both solutions are in the wrong nappe,
                // there are no solutions
                throw new RuggedException(RuggedMessages.LINE_OF_SIGHT_NEVER_CROSSES_LATITUDE,
                                          FastMath.toDegrees(latitude));
            }
        }

        // compute point
        return new Vector3D(1, position, selectedK, los);

    }

    /** Get point at some longitude along a pixel line of sight.
     * @param position pixel position (in body frame)
     * @param los pixel line-of-sight, not necessarily normalized (in body frame)
     * @param longitude longitude with respect to ellipsoid
     * @return point at altitude
     * @exception RuggedException if no such point exists
     */
    public Vector3D pointAtLongitude(final Vector3D position, final Vector3D los, final double longitude)
        throws RuggedException {

        // normal to meridian
        final Vector3D normal = new Vector3D(-FastMath.sin(longitude), FastMath.cos(longitude), 0);
        final double d = Vector3D.dotProduct(los, normal);
        if (FastMath.abs(d) < 1.0e-12) {
            throw new RuggedException(RuggedMessages.LINE_OF_SIGHT_NEVER_CROSSES_LONGITUDE,
                                      FastMath.toDegrees(longitude));
        }

        // compute point
       return new Vector3D(1, position, -Vector3D.dotProduct(position, normal) / d, los);

    }

    /** Get point at some altitude along a pixel line of sight.
     * @param position pixel position (in body frame)
     * @param los pixel line-of-sight, not necessarily normalized (in body frame)
     * @param altitude altitude with respect to ellipsoid
     * @return point at altitude
     * @exception RuggedException if no such point exists (typically too negative altitude)
     */
    public Vector3D pointAtAltitude(final Vector3D position, final Vector3D los, final double altitude)
        throws RuggedException {
        try {

            // point on line closest to origin
            final double   los2   = los.getNormSq();
            final double   dot    = Vector3D.dotProduct(position, los);
            final double   k0     = -dot / los2;
            final Vector3D close0 = new Vector3D(1, position, k0, los);

            // very rough guess: if body is spherical, the desired point on line
            // is at distance ae + altitude from origin
            final double r        = getEquatorialRadius() + altitude;
            final double delta2   = r * r - close0.getNormSq();
            if (delta2 < 0) {
                throw new RuggedException(RuggedMessages.LINE_OF_SIGHT_NEVER_CROSSES_ALTITUDE, altitude);
            }
            final double deltaK   = FastMath.sqrt(delta2 / los2);
            final double k1       = k0 + deltaK;
            final double k2       = k0 - deltaK;
            double k              = (FastMath.abs(k1) <= FastMath.abs(k2)) ? k1 : k2;

            // this loop generally converges in 3 iterations
            for (int i = 0; i < 100; ++i) {

                final Vector3D      point   = new Vector3D(1, position, k, los);
                final GeodeticPoint gpK     = transform(point, getBodyFrame(), null);
                final double        deltaH  = altitude - gpK.getAltitude();
                if (FastMath.abs(deltaH) <= ALTITUDE_CONVERGENCE) {
                    return point;
                }

                // improve the offset using linear ratio between
                // altitude variation and displacement along line-of-sight
                k += deltaH / Vector3D.dotProduct(gpK.getZenith(), los);

            }

            // this should never happen
            throw new RuggedException(RuggedMessages.LINE_OF_SIGHT_NEVER_CROSSES_ALTITUDE, altitude);

        } catch (OrekitException oe) {
            // this should never happen
            throw new RuggedException(oe, oe.getSpecifier(), oe.getParts());
        }
    }

}
