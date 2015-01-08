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
package org.orekit.rugged.utils;

import org.apache.commons.math3.geometry.euclidean.threed.Line;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.util.FastMath;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.bodies.OneAxisEllipsoid;
import org.orekit.errors.OrekitException;
import org.orekit.frames.Frame;
import org.orekit.rugged.errors.RuggedException;
import org.orekit.rugged.errors.RuggedMessages;
import org.orekit.time.AbsoluteDate;

/** Transform provider from Spacecraft frame to observed body frame.
 * @author Luc Maisonobe
 */
public class ExtendedEllipsoid extends OneAxisEllipsoid {

    /** Serializable UID. */
    private static final long serialVersionUID = 20140312L;

    /** Convergence threshold for {@link #pointAtAltitude(Vector3D, Vector3D, double)}. */
    private static final double ALTITUDE_CONVERGENCE = 1.0e-3;

    /** Equatorial radius power 2. */
    private final double a2;

    /** Polar radius power 2. */
    private final double b2;

    /** Simple constructor.
     * @param ae equatorial radius
     * @param f the flattening (f = (a-b)/a)
     * @param bodyFrame body frame related to body shape
     * @see org.orekit.frames.FramesFactory#getITRF(org.orekit.utils.IERSConventions, boolean)
     */
    public ExtendedEllipsoid(final double ae, final double f, final Frame bodyFrame) {
        super(ae, f, bodyFrame);
        a2 = ae * ae;
        final double b = ae * (1.0 - f);
        b2 = b * b;
    }

    /** Get point at some latitude along a pixel line of sight.
     * @param position cell position (in body frame)
     * @param los pixel line-of-sight, not necessarily normalized (in body frame)
     * @param latitude latitude with respect to ellipsoid
     * @param closeReference reference point used to select the closest solution
     * when there are two points at the desired latitude along the line
     * @return point at latitude
     * @exception RuggedException if no such point exists
     */
    public Vector3D pointAtLatitude(final Vector3D position, final Vector3D los,
                                    final double latitude, final Vector3D closeReference)
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
        final double   bb    = b * b;
        final double   ac    = a * c;
        if (bb < ac) {
            throw new RuggedException(RuggedMessages.LINE_OF_SIGHT_NEVER_CROSSES_LATITUDE,
                                      FastMath.toDegrees(latitude));
        }
        final double s  = FastMath.sqrt(bb - ac);
        final double k1 = (b > 0) ? -(s + b) / a : c / (s - b);
        final double k2 = c / (a * k1);

        // the quadratic equation has two solutions
        final boolean  k1IsOK = (delta.getZ() + k1 * los.getZ()) * latitude >= 0;
        final boolean  k2IsOK = (delta.getZ() + k2 * los.getZ()) * latitude >= 0;
        final double selectedK;
        if (k1IsOK) {
            if (k2IsOK) {
                // both solutions are in the good nappe,
                // select the one closest to the specified reference
                final double kRef = Vector3D.dotProduct(los, closeReference.subtract(position)) /
                                    los.getNormSq();
                selectedK = FastMath.abs(k1 - kRef) <= FastMath.abs(k2 - kRef) ? k1 : k2;
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
     * @param position cell position (in body frame)
     * @param los pixel line-of-sight, not necessarily normalized (in body frame)
     * @param longitude longitude with respect to ellipsoid
     * @return point at longitude
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

    /** Get point on ground along a pixel line of sight.
     * @param position cell position (in body frame)
     * @param los pixel line-of-sight, not necessarily normalized (in body frame)
     * @param centralLongitude reference longitude lc such that the point longitude will
     * be normalized between lc-π and lc+π
     * @return point on ground
     * @exception RuggedException if no such point exists (typically line-of-sight missing body)
     */
    public NormalizedGeodeticPoint pointOnGround(final Vector3D position, final Vector3D los,
                                                 final double centralLongitude)
        throws RuggedException {
        try {
            final GeodeticPoint gp =
                    getIntersectionPoint(new Line(position, new Vector3D(1, position, 1e6, los), 1.0e-12),
                                         position, getBodyFrame(), null);
            if (gp == null) {
                throw new RuggedException(RuggedMessages.LINE_OF_SIGHT_DOES_NOT_REACH_GROUND);
            }
            return new NormalizedGeodeticPoint(gp.getLatitude(), gp.getLongitude(), gp.getAltitude(),
                                               centralLongitude);
        } catch (OrekitException oe) {
            throw new RuggedException(oe, oe.getSpecifier(), oe.getParts());
        }
    }

    /** Get point at some altitude along a pixel line of sight.
     * @param position cell position (in body frame)
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

    /** Convert a line-of-sight from Cartesian to topocentric.
     * @param point geodetic point on the line-of-sight
     * @param los line-of-sight, not necessarily normalized (in body frame and Cartesian coordinates)
     * @return line-of-sight in topocentric frame (East, North, Zenith) of the point,
     * scaled to match radians in the horizontal plane and meters along the vertical axis
     */
    public Vector3D convertLos(final GeodeticPoint point, final Vector3D los) {

        // Cartesian coordinates of the topocentric frame origin
        final Vector3D p3D = transform(point);

        // local radius of curvature in the East-West direction (parallel)
        final double r     = FastMath.hypot(p3D.getX(), p3D.getY());

        // local radius of curvature in the North-South direction (meridian)
        final double b2r   = b2 * r;
        final double b4r2  = b2r * b2r;
        final double a2z   = a2 * p3D.getZ();
        final double a4z2  = a2z * a2z;
        final double q     = a4z2 + b4r2;
        final double rho   = q * FastMath.sqrt(q) / (b2 * a4z2 + a2 * b4r2);

        final double norm = los.getNorm();
        return new Vector3D(Vector3D.dotProduct(los, point.getEast())   / (norm * r),
                            Vector3D.dotProduct(los, point.getNorth())  / (norm * rho),
                            Vector3D.dotProduct(los, point.getZenith()) / norm);

    }

    /** Convert a line-of-sight from Cartesian to topocentric.
     * @param primary reference point on the line-of-sight (in body frame and Cartesian coordinates)
     * @param secondary secondary point on the line-of-sight, only used to define a direction
     * with respect to the primary point (in body frame and Cartesian coordinates)
     * @return line-of-sight in topocentric frame (East, North, Zenith) of the point,
     * scaled to match radians in the horizontal plane and meters along the vertical axis
     * @exception RuggedException if points cannot be converted to geodetic coordinates
     */
    public Vector3D convertLos(final Vector3D primary, final Vector3D secondary)
        throws RuggedException {
        try {

            // switch to geodetic coordinates using primary point as reference
            final GeodeticPoint point = transform(primary, getBodyFrame(), null);
            final Vector3D      los   = secondary.subtract(primary);

            // convert line of sight
            return convertLos(point, los);

        } catch (OrekitException oe) {
            throw new RuggedException(oe, oe.getSpecifier(), oe.getParts());
        }
    }

    /** Transform a cartesian point to a surface-relative point.
     * @param point cartesian point
     * @param frame frame in which cartesian point is expressed
     * @param date date of the computation (used for frames conversions)
     * @param centralLongitude reference longitude lc such that the point longitude will
     * be normalized between lc-π and lc+π
     * @return point at the same location but as a surface-relative point
     * @exception OrekitException if point cannot be converted to body frame
     */
    public NormalizedGeodeticPoint transform(final Vector3D point, final Frame frame, final AbsoluteDate date,
                                             final double centralLongitude)
        throws OrekitException {
        final GeodeticPoint gp = transform(point, frame, date);
        return new NormalizedGeodeticPoint(gp.getLatitude(), gp.getLongitude(), gp.getAltitude(),
                                           centralLongitude);
    }

}
