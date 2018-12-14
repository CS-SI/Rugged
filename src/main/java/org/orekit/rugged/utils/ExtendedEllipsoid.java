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

import org.hipparchus.geometry.euclidean.threed.Line;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.util.FastMath;
import org.hipparchus.util.MathArrays;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.bodies.OneAxisEllipsoid;
import org.orekit.frames.Frame;
import org.orekit.rugged.errors.DumpManager;
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
     * @param ae equatorial radius (m)
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

    /** {@inheritDoc} */
    @Override
    public Vector3D transform(final GeodeticPoint point) {
        DumpManager.dumpEllipsoid(this);
        return super.transform(point);
    }

    /** {@inheritDoc} */
    @Override
    public GeodeticPoint transform(final Vector3D point, final Frame frame, final AbsoluteDate date) {
        DumpManager.dumpEllipsoid(this);
        return super.transform(point, frame, date);
    }

    /** Get point at some latitude along a pixel line of sight.
     * @param position cell position (in body frame) (m)
     * @param los pixel line-of-sight, not necessarily normalized (in body frame)
     * @param latitude latitude with respect to ellipsoid (rad)
     * @param closeReference reference point used to select the closest solution
     * when there are two points at the desired latitude along the line, it should
     * be close to los surface intersection (m)
     * @return point at latitude (m)
     */
    public Vector3D pointAtLatitude(final Vector3D position, final Vector3D los,
                                    final double latitude, final Vector3D closeReference) {

        DumpManager.dumpEllipsoid(this);

        // find apex of iso-latitude cone, somewhere along polar axis
        final double sinPhi  = FastMath.sin(latitude);
        final double sinPhi2 = sinPhi * sinPhi;
        final double e2      = getFlattening() * (2 - getFlattening());
        final double apexZ   = -getA() * e2 * sinPhi / FastMath.sqrt(1 - e2 * sinPhi2);

        // quadratic equation representing line intersection with iso-latitude cone
        // a k² + 2 b k + c = 0
        // when line of sight is almost along an iso-latitude generatrix, the quadratic
        // equation above may become unsolvable due to numerical noise (we get catastrophic
        // cancellation when computing b * b - a * c). So we set up the model in two steps,
        // first searching k₀ such that position + k₀ los is close to closeReference, and
        // then using position + k₀ los as the new initial position, which should be in
        // the neighborhood of the solution
        final double cosPhi  = FastMath.cos(latitude);
        final double cosPhi2 = cosPhi * cosPhi;
        final double k0 = Vector3D.dotProduct(closeReference.subtract(position), los) / los.getNormSq();

        final Vector3D delta  = new Vector3D(MathArrays.linearCombination(1, position.getX(), k0, los.getX()),
                                             MathArrays.linearCombination(1, position.getY(), k0, los.getY()),
                                             MathArrays.linearCombination(1, position.getZ(), k0, los.getZ(), -1.0, apexZ));
        final double   a     = MathArrays.linearCombination(+sinPhi2, los.getX() * los.getX() + los.getY() * los.getY(),
                                                            -cosPhi2, los.getZ() * los.getZ());
        final double   b      = MathArrays.linearCombination(+sinPhi2, MathArrays.linearCombination(delta.getX(), los.getX(),
                                                                                                    delta.getY(), los.getY()),
                                                             -cosPhi2, delta.getZ() * los.getZ());
        final double   c      = MathArrays.linearCombination(+sinPhi2, delta.getX() * delta.getX() + delta.getY() * delta.getY(),
                                                             -cosPhi2, delta.getZ() * delta.getZ());

        // find the two intersections along the line
        if (b * b < a * c) {
            throw new RuggedException(RuggedMessages.LINE_OF_SIGHT_NEVER_CROSSES_LATITUDE,
                                      FastMath.toDegrees(latitude));
        }
        final double s  = FastMath.sqrt(MathArrays.linearCombination(b, b, -a, c));
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
                                    los.getNormSq() - k0;
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
        return new Vector3D(1, position, k0 + selectedK, los);

    }

    /** Get point at some longitude along a pixel line of sight.
     * @param position cell position (in body frame) (m)
     * @param los pixel line-of-sight, not necessarily normalized (in body frame)
     * @param longitude longitude with respect to ellipsoid (rad)
     * @return point at longitude (m)
     */
    public Vector3D pointAtLongitude(final Vector3D position, final Vector3D los, final double longitude) {

        DumpManager.dumpEllipsoid(this);

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
     * @param position cell position (in body frame) (m)
     * @param los pixel line-of-sight, not necessarily normalized (in body frame)
     * @param centralLongitude reference longitude lc such that the point longitude will
     * be normalized between lc-π and lc+π (rad)
     * @return point on ground
     */
    public NormalizedGeodeticPoint pointOnGround(final Vector3D position, final Vector3D los,
                                                 final double centralLongitude) {
        
        DumpManager.dumpEllipsoid(this);
        final GeodeticPoint gp =
                getIntersectionPoint(new Line(position, new Vector3D(1, position, 1e6, los), 1.0e-12),
                        position, getBodyFrame(), null);
        if (gp == null) {
            throw new RuggedException(RuggedMessages.LINE_OF_SIGHT_DOES_NOT_REACH_GROUND);
        }
        return new NormalizedGeodeticPoint(gp.getLatitude(), gp.getLongitude(), gp.getAltitude(),
                centralLongitude);
    }

    /** Get point at some altitude along a pixel line of sight.
     * @param position cell position (in body frame) (m)
     * @param los pixel line-of-sight, not necessarily normalized (in body frame)
     * @param altitude altitude with respect to ellipsoid (m)
     * @return point at altitude (m)
     */
    public Vector3D pointAtAltitude(final Vector3D position, final Vector3D los, final double altitude) {

        DumpManager.dumpEllipsoid(this);

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
     */
    public Vector3D convertLos(final Vector3D primary, final Vector3D secondary) {

        // switch to geodetic coordinates using primary point as reference
        final GeodeticPoint point = transform(primary, getBodyFrame(), null);
        final Vector3D      los   = secondary.subtract(primary);

        // convert line of sight
        return convertLos(point, los);
    }

    /** Transform a cartesian point to a surface-relative point.
     * @param point cartesian point (m)
     * @param frame frame in which cartesian point is expressed
     * @param date date of the computation (used for frames conversions)
     * @param centralLongitude reference longitude lc such that the point longitude will
     * be normalized between lc-π and lc+π (rad)
     * @return point at the same location but as a surface-relative point
     */
    public NormalizedGeodeticPoint transform(final Vector3D point, final Frame frame, final AbsoluteDate date,
                                             final double centralLongitude) {
        final GeodeticPoint gp = transform(point, frame, date);
        return new NormalizedGeodeticPoint(gp.getLatitude(), gp.getLongitude(), gp.getAltitude(),
                                           centralLongitude);
    }

}
