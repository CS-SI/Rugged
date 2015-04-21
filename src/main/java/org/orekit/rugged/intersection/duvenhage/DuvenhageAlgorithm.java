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
package org.orekit.rugged.intersection.duvenhage;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.util.FastMath;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.errors.OrekitException;
import org.orekit.rugged.api.AlgorithmId;
import org.orekit.rugged.errors.DumpManager;
import org.orekit.rugged.errors.RuggedException;
import org.orekit.rugged.errors.RuggedMessages;
import org.orekit.rugged.intersection.IntersectionAlgorithm;
import org.orekit.rugged.raster.Tile;
import org.orekit.rugged.raster.TileUpdater;
import org.orekit.rugged.raster.TilesCache;
import org.orekit.rugged.utils.ExtendedEllipsoid;
import org.orekit.rugged.utils.NormalizedGeodeticPoint;

/** Digital Elevation Model intersection using Bernardt Duvenhage's algorithm.
 * <p>
 * The algorithm is described in the 2009 paper:
 * <a href="http://researchspace.csir.co.za/dspace/bitstream/10204/3041/1/Duvenhage_2009.pdf">Using
 * An Implicit Min/Max KD-Tree for Doing Efficient Terrain Line of Sight Calculations</a>.
 * </p>
 * @author Luc Maisonobe
 */
public class DuvenhageAlgorithm implements IntersectionAlgorithm {

    /** Step size when skipping from one tile to a neighbor one, in meters. */
    private static final double STEP = 0.01;

    /** Cache for DEM tiles. */
    private final TilesCache<MinMaxTreeTile> cache;

    /** Flag for flat-body hypothesis. */
    private final boolean flatBody;

    /** Simple constructor.
     * @param updater updater used to load Digital Elevation Model tiles
     * @param maxCachedTiles maximum number of tiles stored in the cache
     * @param flatBody if true, the body is considered flat, i.e. lines computed
     * from entry/exit points in the DEM are considered to be straight lines also
     * in geodetic coordinates. The sagitta resulting from real ellipsoid curvature
     * is therefore <em>not</em> corrected in this case. As this computation is not
     * costly (a few percents overhead), it is highly recommended to set this parameter
     * to {@code false}. This flag is mainly intended for comparison purposes with other systems.
     */
    public DuvenhageAlgorithm(final TileUpdater updater, final int maxCachedTiles,
                              final boolean flatBody) {
        this.cache = new TilesCache<MinMaxTreeTile>(new MinMaxTreeTileFactory(), updater, maxCachedTiles);
        this.flatBody = flatBody;
    }

    /** {@inheritDoc} */
    @Override
    public NormalizedGeodeticPoint intersection(final ExtendedEllipsoid ellipsoid,
                                                final Vector3D position, final Vector3D los)
        throws RuggedException {
        try {

            DumpManager.dumpAlgorithm(flatBody ? AlgorithmId.DUVENHAGE_FLAT_BODY : AlgorithmId.DUVENHAGE);

            // compute intersection with ellipsoid
            final NormalizedGeodeticPoint gp0 = ellipsoid.pointOnGround(position, los, 0.0);

            // locate the entry tile along the line-of-sight
            MinMaxTreeTile tile = cache.getTile(gp0.getLatitude(), gp0.getLongitude());

            NormalizedGeodeticPoint current = null;
            double hMax = tile.getMaxElevation();
            while (current == null) {

                // find where line-of-sight crosses tile max altitude
                final Vector3D entryP = ellipsoid.pointAtAltitude(position, los, hMax + STEP);
                if (Vector3D.dotProduct(entryP.subtract(position), los) < 0) {
                    // the entry point is behind spacecraft!
                    throw new RuggedException(RuggedMessages.DEM_ENTRY_POINT_IS_BEHIND_SPACECRAFT);
                }
                current = ellipsoid.transform(entryP, ellipsoid.getBodyFrame(), null, tile.getMinimumLongitude());

                if (tile.getLocation(current.getLatitude(), current.getLongitude()) != Tile.Location.HAS_INTERPOLATION_NEIGHBORS) {
                    // the entry point is in another tile
                    tile    = cache.getTile(current.getLatitude(), current.getLongitude());
                    hMax    = FastMath.max(hMax, tile.getMaxElevation());
                    current = null;
                }

            }

            // loop along the path
            while (true) {

                // find where line-of-sight exit tile
                final LimitPoint exit = findExit(tile, ellipsoid, position, los);

                // compute intersection with Digital Elevation Model
                final int entryLat = FastMath.max(0,
                                                  FastMath.min(tile.getLatitudeRows() - 1,
                                                               tile.getFloorLatitudeIndex(current.getLatitude())));
                final int entryLon = FastMath.max(0,
                                                  FastMath.min(tile.getLongitudeColumns() - 1,
                                                               tile.getFloorLongitudeIndex(current.getLongitude())));
                final int exitLat  = FastMath.max(0,
                                                  FastMath.min(tile.getLatitudeRows() - 1,
                                                               tile.getFloorLatitudeIndex(exit.getPoint().getLatitude())));
                final int exitLon  = FastMath.max(0,
                                                  FastMath.min(tile.getLongitudeColumns() - 1,
                                                               tile.getFloorLongitudeIndex(exit.getPoint().getLongitude())));
                final NormalizedGeodeticPoint intersection = recurseIntersection(0, ellipsoid, position, los, tile,
                                                                       current, entryLat, entryLon,
                                                                       exit.getPoint(), exitLat, exitLon);

                if (intersection != null) {
                    // we have found the intersection
                    return intersection;
                } else if (exit.atSide()) {
                    // no intersection on this tile, we can proceed to next part of the line-of-sight

                    // select next tile after current point
                    final Vector3D forward = new Vector3D(1.0, ellipsoid.transform(exit.getPoint()), STEP, los);
                    current = ellipsoid.transform(forward, ellipsoid.getBodyFrame(), null, tile.getMinimumLongitude());
                    tile = cache.getTile(current.getLatitude(), current.getLongitude());

                    if (tile.interpolateElevation(current.getLatitude(), current.getLongitude()) >= current.getAltitude()) {
                        // extremely rare case! The line-of-sight traversed the Digital Elevation Model
                        // during the very short forward step we used to move to next tile
                        // we consider this point to be OK
                        return current;
                    }

                } else {
                    // this should never happen
                    // we should have left the loop with an intersection point
                    throw RuggedException.createInternalError(null);
                }


            }


        } catch (OrekitException oe) {
            throw new RuggedException(oe, oe.getSpecifier(), oe.getParts());
        }
    }

    /** {@inheritDoc} */
    @Override
    public NormalizedGeodeticPoint refineIntersection(final ExtendedEllipsoid ellipsoid,
                                                      final Vector3D position, final Vector3D los,
                                                      final NormalizedGeodeticPoint closeGuess)
        throws RuggedException {
        try {

            DumpManager.dumpAlgorithm(flatBody ? AlgorithmId.DUVENHAGE_FLAT_BODY : AlgorithmId.DUVENHAGE);

            if (flatBody) {
                // under the (bad) flat-body assumption, the reference point must remain
                // at DEM entry and exit, even if we already have a much better close guess :-(
                // this is in order to remain consistent with other systems
                final Tile tile = cache.getTile(closeGuess.getLatitude(), closeGuess.getLongitude());
                final Vector3D      exitP  = ellipsoid.pointAtAltitude(position, los, tile.getMinElevation());
                final Vector3D      entryP = ellipsoid.pointAtAltitude(position, los, tile.getMaxElevation());
                final NormalizedGeodeticPoint entry  = ellipsoid.transform(entryP, ellipsoid.getBodyFrame(), null,
                                                                           tile.getMinimumLongitude());
                return tile.cellIntersection(entry, ellipsoid.convertLos(entryP, exitP),
                                              tile.getFloorLatitudeIndex(closeGuess.getLatitude()),
                                              tile.getFloorLongitudeIndex(closeGuess.getLongitude()));
            } else {
                final Vector3D      delta     = ellipsoid.transform(closeGuess).subtract(position);
                final double        s         = Vector3D.dotProduct(delta, los) / los.getNormSq();
                final GeodeticPoint projected = ellipsoid.transform(new Vector3D(1, position, s, los),
                                                                    ellipsoid.getBodyFrame(), null);
                final Tile          tile      = cache.getTile(projected.getLatitude(), projected.getLongitude());
                return tile.cellIntersection(projected, ellipsoid.convertLos(projected, los),
                                              tile.getFloorLatitudeIndex(projected.getLatitude()),
                                              tile.getFloorLongitudeIndex(projected.getLongitude()));
            }
        } catch (OrekitException oe) {
            throw new RuggedException(oe, oe.getSpecifier(), oe.getParts());
        }
    }

    /** {@inheritDoc} */
    @Override
    public double getElevation(final double latitude, final double longitude)
        throws RuggedException {
        DumpManager.dumpAlgorithm(flatBody ? AlgorithmId.DUVENHAGE_FLAT_BODY : AlgorithmId.DUVENHAGE);
        final Tile tile = cache.getTile(latitude, longitude);
        return tile.interpolateElevation(latitude, longitude);
    }

    /** Compute intersection of line with Digital Elevation Model in a sub-tile.
     * @param depth recursion depth
     * @param ellipsoid reference ellipsoid
     * @param position pixel position in ellipsoid frame
     * @param los pixel line-of-sight in ellipsoid frame
     * @param tile Digital Elevation Model tile
     * @param entry line-of-sight entry point in the sub-tile
     * @param entryLat index to use for interpolating entry point elevation
     * @param entryLon index to use for interpolating entry point elevation
     * @param exit line-of-sight exit point from the sub-tile
     * @param exitLat index to use for interpolating exit point elevation
     * @param exitLon index to use for interpolating exit point elevation
     * @return point at which the line first enters ground, or null if does not enter
     * ground in the search sub-tile
     * @exception RuggedException if intersection cannot be found
     * @exception OrekitException if points cannot be converted to geodetic coordinates
     */
    private NormalizedGeodeticPoint recurseIntersection(final int depth, final ExtendedEllipsoid ellipsoid,
                                                        final Vector3D position, final Vector3D los,
                                                        final MinMaxTreeTile tile,
                                                        final NormalizedGeodeticPoint entry, final int entryLat, final int entryLon,
                                                        final NormalizedGeodeticPoint exit, final int exitLat, final int exitLon)
        throws RuggedException, OrekitException {

        if (depth > 30) {
            // this should never happen
            throw RuggedException.createInternalError(null);
        }

        if (entryLat == exitLat && entryLon == exitLon) {
            // we have narrowed the search down to a single Digital Elevation Model cell
            return tile.cellIntersection(entry, ellipsoid.convertLos(entry, los), exitLat, exitLon);
        }

        // find the deepest level in the min/max kd-tree at which entry and exit share a sub-tile
        final int level = tile.getMergeLevel(entryLat, entryLon, exitLat, exitLon);
        if (level >= 0  && exit.getAltitude() >= tile.getMaxElevation(exitLat, exitLon, level)) {
            // the line-of-sight segment is fully above Digital Elevation Model
            // we can safely reject it and proceed to next part of the line-of-sight
            return null;
        }

        NormalizedGeodeticPoint previousGP    = entry;
        int                     previousLat   = entryLat;
        int                     previousLon   = entryLon;
        final double            angularMargin = STEP / ellipsoid.getEquatorialRadius();

        // introduce all intermediate points corresponding to the line-of-sight
        // intersecting the boundary between level 0 sub-tiles
        if (tile.isColumnMerging(level + 1)) {
            // recurse through longitude crossings

            final int[] crossings = tile.getCrossedBoundaryColumns(previousLon, exitLon, level + 1);
            for (final int crossingLon : crossings) {

                // compute segment endpoints
                final double longitude = tile.getLongitudeAtIndex(crossingLon);
                if (longitude >= FastMath.min(entry.getLongitude(), exit.getLongitude()) - angularMargin &&
                    longitude <= FastMath.max(entry.getLongitude(), exit.getLongitude()) + angularMargin) {

                    NormalizedGeodeticPoint crossingGP = null;
                    if (!flatBody) {
                        try {
                            // full computation of crossing point
                            final Vector3D crossingP = ellipsoid.pointAtLongitude(position, los, longitude);
                            crossingGP = ellipsoid.transform(crossingP, ellipsoid.getBodyFrame(), null,
                                                             tile.getMinimumLongitude());
                        } catch  (RuggedException re) {
                            // in some very rare cases of numerical noise, we miss the crossing point
                            crossingGP = null;
                        }
                    }
                    if (crossingGP == null) {
                        // linear approximation of crossing point
                        final double d  = exit.getLongitude() - entry.getLongitude();
                        final double cN = (exit.getLongitude() - longitude) / d;
                        final double cX = (longitude - entry.getLongitude()) / d;
                        crossingGP = new NormalizedGeodeticPoint(cN * entry.getLatitude() + cX * exit.getLatitude(),
                                                                 longitude,
                                                                 cN * entry.getAltitude() + cX * exit.getAltitude(),
                                                                 tile.getMinimumLongitude());
                    }
                    final int crossingLat = tile.getFloorLatitudeIndex(crossingGP.getLatitude());

                    // adjust indices as the crossing point is by definition between the sub-tiles
                    final int crossingLonBefore = crossingLon - (entryLon <= exitLon ? 1 : 0);
                    final int crossingLonAfter  = crossingLon - (entryLon <= exitLon ? 0 : 1);

                    if (inRange(crossingLonBefore, entryLon, exitLon)) {
                        // look for intersection
                        final NormalizedGeodeticPoint intersection =
                                recurseIntersection(depth + 1, ellipsoid, position, los, tile,
                                                    previousGP, previousLat, previousLon,
                                                    crossingGP, crossingLat, crossingLonBefore);
                        if (intersection != null) {
                            return intersection;
                        }
                    }

                    // prepare next segment
                    previousGP  = crossingGP;
                    previousLat = crossingLat;
                    previousLon = crossingLonAfter;

                }

            }
        } else {
            // recurse through latitude crossings
            final int[] crossings = tile.getCrossedBoundaryRows(previousLat, exitLat, level + 1);
            for (final int crossingLat : crossings) {

                // compute segment endpoints
                final double latitude = tile.getLatitudeAtIndex(crossingLat);
                if (latitude >= FastMath.min(entry.getLatitude(), exit.getLatitude()) - angularMargin &&
                    latitude <= FastMath.max(entry.getLatitude(), exit.getLatitude()) + angularMargin) {

                    NormalizedGeodeticPoint crossingGP = null;
                    if (!flatBody) {
                        // full computation of crossing point
                        try {
                            final Vector3D crossingP = ellipsoid.pointAtLatitude(position, los,
                                                                                 tile.getLatitudeAtIndex(crossingLat),
                                                                                 ellipsoid.transform(entry));
                            crossingGP = ellipsoid.transform(crossingP, ellipsoid.getBodyFrame(), null,
                                                             tile.getMinimumLongitude());
                        } catch  (RuggedException re) {
                            // in some very rare cases of numerical noise, we miss the crossing point
                            crossingGP = null;
                        }
                    }
                    if (crossingGP == null) {
                        // linear approximation of crossing point
                        final double d  = exit.getLatitude() - entry.getLatitude();
                        final double cN = (exit.getLatitude() - latitude) / d;
                        final double cX = (latitude - entry.getLatitude()) / d;
                        crossingGP = new NormalizedGeodeticPoint(latitude,
                                                                 cN * entry.getLongitude() + cX * exit.getLongitude(),
                                                                 cN * entry.getAltitude()  + cX * exit.getAltitude(),
                                                                 tile.getMinimumLongitude());
                    }
                    final int crossingLon = tile.getFloorLongitudeIndex(crossingGP.getLongitude());

                    // adjust indices as the crossing point is by definition between the sub-tiles
                    final int crossingLatBefore = crossingLat - (entryLat <= exitLat ? 1 : 0);
                    final int crossingLatAfter  = crossingLat - (entryLat <= exitLat ? 0 : 1);

                    if (inRange(crossingLatBefore, entryLat, exitLat)) {
                        // look for intersection
                        final NormalizedGeodeticPoint intersection = recurseIntersection(depth + 1, ellipsoid, position, los, tile,
                                                                               previousGP, previousLat, previousLon,
                                                                               crossingGP, crossingLatBefore, crossingLon);
                        if (intersection != null) {
                            return intersection;
                        }
                    }

                    // prepare next segment
                    previousGP  = crossingGP;
                    previousLat = crossingLatAfter;
                    previousLon = crossingLon;

                }

            }
        }

        if (inRange(previousLat, entryLat, exitLat) && inRange(previousLon, entryLon, exitLon)) {
            // last part of the segment, up to exit point
            return recurseIntersection(depth + 1, ellipsoid, position, los, tile,
                                       previousGP, previousLat, previousLon,
                                       exit, exitLat, exitLon);
        } else {
            return null;
        }

    }

    /** Check if an index is inside a range.
     * @param i index to check
     * @param a first bound of the range (may be either below or above b)
     * @param b second bound of the range (may be either below or above a)
     * @return true if i is between a and b (inclusive)
     */
    private boolean inRange(final int i, final int a, final int b) {
        return i >= FastMath.min(a, b) && i <= FastMath.max(a, b);
    }

    /** Compute a line-of-sight exit point from a tile.
     * @param tile tile to consider
     * @param ellipsoid reference ellipsoid
     * @param position pixel position in ellipsoid frame
     * @param los pixel line-of-sight in ellipsoid frame
     * @return exit point
     * @exception RuggedException if exit point cannot be found
     * @exception OrekitException if geodetic coordinates cannot be computed
     */
    private LimitPoint findExit(final Tile tile, final ExtendedEllipsoid ellipsoid,
                                final Vector3D position, final Vector3D los)
        throws RuggedException, OrekitException {

        // look for an exit at bottom
        final double                  reference = tile.getMinimumLongitude();
        final Vector3D                exitP     = ellipsoid.pointAtAltitude(position, los, tile.getMinElevation() - STEP);
        final NormalizedGeodeticPoint exitGP    = ellipsoid.transform(exitP, ellipsoid.getBodyFrame(), null, reference);

        switch (tile.getLocation(exitGP.getLatitude(), exitGP.getLongitude())) {
        case SOUTH_WEST :
            return new LimitPoint(ellipsoid, reference,
                                  selectClosest(latitudeCrossing(ellipsoid, position,  los, tile.getMinimumLatitude(),  exitP),
                                                longitudeCrossing(ellipsoid, position, los, tile.getMinimumLongitude(), exitP),
                                                position),
                                  true);
        case WEST :
            return new LimitPoint(ellipsoid, reference,
                                  longitudeCrossing(ellipsoid, position, los, tile.getMinimumLongitude(), exitP),
                                  true);
        case NORTH_WEST:
            return new LimitPoint(ellipsoid, reference,
                                  selectClosest(latitudeCrossing(ellipsoid, position,  los, tile.getMaximumLatitude(),  exitP),
                                                longitudeCrossing(ellipsoid, position, los, tile.getMinimumLongitude(), exitP),
                                                position),
                                  true);
        case NORTH :
            return new LimitPoint(ellipsoid, reference,
                                  latitudeCrossing(ellipsoid, position, los, tile.getMaximumLatitude(), exitP),
                                  true);
        case NORTH_EAST :
            return new LimitPoint(ellipsoid, reference,
                                  selectClosest(latitudeCrossing(ellipsoid, position,  los, tile.getMaximumLatitude(),  exitP),
                                                longitudeCrossing(ellipsoid, position, los, tile.getMaximumLongitude(), exitP),
                                                position),
                                  true);
        case EAST :
            return new LimitPoint(ellipsoid, reference,
                                  longitudeCrossing(ellipsoid, position, los, tile.getMaximumLongitude(), exitP),
                                  true);
        case SOUTH_EAST :
            return new LimitPoint(ellipsoid, reference,
                                  selectClosest(latitudeCrossing(ellipsoid, position,  los, tile.getMinimumLatitude(),  exitP),
                                                longitudeCrossing(ellipsoid, position, los, tile.getMaximumLongitude(), exitP),
                                                position),
                                  true);
        case SOUTH :
            return new LimitPoint(ellipsoid, reference,
                                  latitudeCrossing(ellipsoid, position, los, tile.getMinimumLatitude(), exitP),
                                  true);
        case HAS_INTERPOLATION_NEIGHBORS :
            return new LimitPoint(exitGP, false);

        default :
            // this should never happen
            throw RuggedException.createInternalError(null);

        }

    }

    /** Select point closest to line-of-sight start.
     * @param p1 first point to consider
     * @param p2 second point to consider
     * @param position pixel position in ellipsoid frame
     * @return either p1 or p2, depending on which is closest to position
     */
    private Vector3D selectClosest(final Vector3D p1, final Vector3D p2, final Vector3D position) {
        return Vector3D.distance(p1, position) <= Vector3D.distance(p2, position) ? p1 : p2;
    }

    /** Get point at some latitude along a pixel line of sight.
     * @param ellipsoid reference ellipsoid
     * @param position pixel position (in body frame)
     * @param los pixel line-of-sight, not necessarily normalized (in body frame)
     * @param latitude latitude with respect to ellipsoid
     * @param closeReference reference point used to select the closest solution
     * when there are two points at the desired latitude along the line
     * @return point at altitude, or closeReference if no such point can be found
     */
    private Vector3D latitudeCrossing(final ExtendedEllipsoid ellipsoid,
                                      final Vector3D position, final Vector3D los,
                                      final double latitude, final Vector3D closeReference) {
        try {
            return ellipsoid.pointAtLatitude(position, los, latitude, closeReference);
        } catch (RuggedException re) {
            return closeReference;
        }
    }

    /** Get point at some latitude along a pixel line of sight.
     * @param ellipsoid reference ellipsoid
     * @param position pixel position (in body frame)
     * @param los pixel line-of-sight, not necessarily normalized (in body frame)
     * @param longitude longitude with respect to ellipsoid
     * @param closeReference reference point used to select the closest solution
     * when there are two points at the desired latitude along the line
     * @return point at altitude, or closeReference if no such point can be found
     */
    private Vector3D longitudeCrossing(final ExtendedEllipsoid ellipsoid,
                                       final Vector3D position, final Vector3D los,
                                       final double longitude, final Vector3D closeReference) {
        try {
            return ellipsoid.pointAtLongitude(position, los, longitude);
        } catch (RuggedException re) {
            return closeReference;
        }
    }

    /** Point at tile boundary. */
    private static class LimitPoint {

        /** Coordinates. */
        private final NormalizedGeodeticPoint point;

        /** Limit status. */
        private final boolean side;

        /** Simple constructor.
         * @param ellipsoid reference ellipsoid
         * @param referenceLongitude reference longitude lc such that the point longitude will
         * be normalized between lc-π and lc+π
         * @param cartesian Cartesian point
         * @param side if true, the point is on a side limit, otherwise
         * it is on a top/bottom limit
         * @exception OrekitException if geodetic coordinates cannot be computed
         */
        public LimitPoint(final ExtendedEllipsoid ellipsoid, final double referenceLongitude,
                          final Vector3D cartesian, final boolean side)
            throws OrekitException {
            this(ellipsoid.transform(cartesian, ellipsoid.getBodyFrame(), null, referenceLongitude), side);
        }

        /** Simple constructor.
         * @param point coordinates
         * @param side if true, the point is on a side limit, otherwise
         * it is on a top/bottom limit
         */
        public LimitPoint(final NormalizedGeodeticPoint point, final boolean side) {
            this.point = point;
            this.side  = side;
        }

        /** Get the point coordinates.
         * @return point coordinates
         */
        public NormalizedGeodeticPoint getPoint() {
            return point;
        }

        /** Check if point is on the side of a tile.
         * @return true if the point is on a side limit, otherwise
         * it is on a top/bottom limit
         */
        public boolean atSide() {
            return side;
        }

    }

}
