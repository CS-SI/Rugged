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
package org.orekit.rugged.core.duvenhage;

import java.util.ArrayDeque;
import java.util.Deque;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.errors.OrekitException;
import org.orekit.rugged.api.RuggedException;
import org.orekit.rugged.api.RuggedMessages;
import org.orekit.rugged.api.TileUpdater;
import org.orekit.rugged.core.ExtendedEllipsoid;
import org.orekit.rugged.core.dem.IntersectionAlgorithm;
import org.orekit.rugged.core.dem.Tile;
import org.orekit.rugged.core.dem.TilesCache;

/** Digital Elevation Model intersection using Duvenhage's algorithm.
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
    private TilesCache<MinMaxTreeTile> cache;

    /** Simple constructor.
     */
    public DuvenhageAlgorithm() {
    }

    /** {@inheritDoc} */
    @Override
    public void setUpTilesManagement(final TileUpdater updater, final int maxCachedTiles) {
        cache = new TilesCache<MinMaxTreeTile>(new MinMaxTreeTileFactory(),
                                               updater, maxCachedTiles);
    }

    /** {@inheritDoc} */
    @Override
    public GeodeticPoint intersection(final ExtendedEllipsoid ellipsoid,
                                      final Vector3D position, final Vector3D los)
        throws RuggedException {
        try {

            // compute intersection with ellipsoid
            final Vector3D      p0  = ellipsoid.pointAtAltitude(position, los, 0.0);
            final GeodeticPoint gp0 = ellipsoid.transform(p0, ellipsoid.getBodyFrame(), null);

            // locate the entry tile along the line-of-sight
            MinMaxTreeTile tile = cache.getTile(gp0.getLatitude(), gp0.getLongitude());

            GeodeticPoint current = null;
            while (current == null) {

                // find where line-of-sight crosses tile max altitude
                final Vector3D entryP = ellipsoid.pointAtAltitude(position, los, tile.getMaxElevation());
                if (Vector3D.dotProduct(entryP.subtract(position), los) < 0) {
                    // the entry point is behind spacecraft!
                    throw new RuggedException(RuggedMessages.DEM_ENTRY_POINT_IS_BEHIND_SPACECRAFT);
                }
                current = ellipsoid.transform(entryP, ellipsoid.getBodyFrame(), null);

                if (tile.getLocation(current.getLatitude(), current.getLongitude()) != Tile.Location.IN_TILE) {
                    // the entry point is in another tile
                    tile    = cache.getTile(current.getLatitude(), current.getLongitude());
                    current = null;
                } 

            }

            // loop along the path
            while (true) {

                int currentLatIndex = tile.getLatitudeIndex(current.getLatitude());
                int currentLonIndex = tile.getLontitudeIndex(current.getLongitude());

                // find where line-of-sight exit tile
                final LimitPoint exit = findExit(tile, ellipsoid, position, los);
                final Deque<GeodeticPoint> splitPointsQueue = new ArrayDeque<GeodeticPoint>();
                splitPointsQueue.addFirst(exit.getPoint());

                while (!splitPointsQueue.isEmpty()) {

                    final GeodeticPoint next = splitPointsQueue.removeFirst();
                    final int nextLatIndex   = tile.getLatitudeIndex(next.getLatitude());
                    final int nextLonIndex   = tile.getLontitudeIndex(next.getLongitude());
                    if (currentLatIndex == nextLatIndex && currentLonIndex == nextLonIndex) {

                        // we have narrowed the search down to a single Digital Elevation Model pixel
                        if (next.getAltitude() <= tile.getElevationAtIndices(nextLatIndex,     nextLonIndex)     ||
                            next.getAltitude() <= tile.getElevationAtIndices(nextLatIndex,     nextLonIndex + 1) ||
                            next.getAltitude() <= tile.getElevationAtIndices(nextLatIndex + 1, nextLonIndex)     ||
                            next.getAltitude() <= tile.getElevationAtIndices(nextLatIndex + 1, nextLonIndex + 1)) {
                            // TODO: compute intersection
                            throw RuggedException.createInternalError(null);
                        } else {
                            // no intersection on this pixel, we can proceed to next part of the line-of-sight
                            current = next;
                        }

                    } else {

                        // find the largest level in the min/max kd-tree at which entry and exit share a sub-tile
                        int level = tile.getMergeLevel(currentLatIndex, currentLonIndex, nextLatIndex, nextLonIndex);
                        if (level < 0) {
                            // TODO: push intermediate points at sub-tiles boundaries on the queue
                            throw RuggedException.createInternalError(null);
                        } else {
                            if (next.getAltitude() >= tile.getMaxElevation(nextLatIndex, nextLonIndex, level)) {
                                // the line-of-sight segment is fully above Digital Elevation Model
                                // we can safely reject it and proceed to next part of the line-of-sight
                                current = next;
                            } else {
                                // the line-of-sight segment has at least some undecided parts which may
                                // intersect the Digital Elevation Model, we need to refine the
                                // search by using a finer-grained level in the min/max kd-tree
                                // TODO: split line-of-sight
                            }
                        }

                    }

                }

                if (!exit.atSide()) {
                    // this should never happen
                    // we should have left the loop with an intersection point
                    throw RuggedException.createInternalError(null);                    
                }

                // select next tile after current point
                final Vector3D forward = new Vector3D(1.0, ellipsoid.transform(current), STEP, los);
                current = ellipsoid.transform(forward, ellipsoid.getBodyFrame(), null);
                tile = cache.getTile(current.getLatitude(), current.getLongitude());
                if (tile.interpolateElevation(current.getLatitude(), current.getLongitude()) <= current.getAltitude()) {
                    // extremely rare case! The line-of-sight traversed the Digital Elevation Model
                    // during the very short forward step we used to move to next tile
                    // we consider this point to be OK
                    return current;
                }

            }


        } catch (OrekitException oe) {
            throw new RuggedException(oe, oe.getSpecifier(), oe.getParts());
        }
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
        Vector3D exitP = ellipsoid.pointAtAltitude(position, los, tile.getMinElevation());
        GeodeticPoint exitGP = ellipsoid.transform(exitP, ellipsoid.getBodyFrame(), null);

        switch (tile.getLocation(exitGP.getLatitude(), exitGP.getLongitude())) {
            case SOUTH_WEST :
                return new LimitPoint(ellipsoid,
                                      selectClosest(ellipsoid.pointAtLatitude(position,  los, tile.getMinimumLatitude()),
                                                    ellipsoid.pointAtLongitude(position, los, tile.getMinimumLongitude()),
                                                    position),
                                      true);
            case WEST :
                return new LimitPoint(ellipsoid,
                                      ellipsoid.pointAtLongitude(position, los, tile.getMinimumLongitude()),
                                      true);
            case NORTH_WEST:
                return new LimitPoint(ellipsoid,
                                      selectClosest(ellipsoid.pointAtLatitude(position,  los, tile.getMaximumLatitude()),
                                                    ellipsoid.pointAtLongitude(position, los, tile.getMinimumLongitude()),
                                                    position),
                                      true);
            case NORTH :
                return new LimitPoint(ellipsoid,
                                      ellipsoid.pointAtLatitude(position, los, tile.getMaximumLatitude()),
                                      true);
            case NORTH_EAST :
                return new LimitPoint(ellipsoid,
                                      selectClosest(ellipsoid.pointAtLatitude(position,  los, tile.getMaximumLatitude()),
                                                    ellipsoid.pointAtLongitude(position, los, tile.getMaximumLongitude()),
                                                    position),
                                      true);
            case EAST :
                return new LimitPoint(ellipsoid,
                                      ellipsoid.pointAtLongitude(position, los, tile.getMaximumLongitude()),
                                      true);
            case SOUTH_EAST :
                return new LimitPoint(ellipsoid,
                                      selectClosest(ellipsoid.pointAtLatitude(position,  los, tile.getMinimumLatitude()),
                                                    ellipsoid.pointAtLongitude(position, los, tile.getMaximumLongitude()),
                                                    position),
                                      true);
            case SOUTH :
                return new LimitPoint(ellipsoid,
                                      ellipsoid.pointAtLatitude(position, los, tile.getMinimumLatitude()),
                                      true);
            case IN_TILE :
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
   private Vector3D selectClosest(Vector3D p1, Vector3D p2, Vector3D position) {
       return Vector3D.distance(p1, position) <= Vector3D.distance(p2, position) ? p1 : p2;
   }

    /** Point at tile boundary. */
    private static class LimitPoint {

        /** Coordinates. */
        private final GeodeticPoint point;

        /** Limit status. */
        private final boolean side;

        /** Simple constructor.
         * @param cartesian point cartesian
         * @param ellipsoid reference ellipsoid
         * @param side if true, the point is on a side limit, otherwise
         * it is on a top/bottom limit
         * @exception OrekitException if geodetic coordinates cannot be computed
         */
        public LimitPoint(final ExtendedEllipsoid ellipsoid, final Vector3D cartesian, final boolean side)
            throws OrekitException {
            this(ellipsoid.transform(cartesian, ellipsoid.getBodyFrame(), null), side);
        }

        /** Simple constructor.
         * @param point coordinates
         * @param side if true, the point is on a side limit, otherwise
         * it is on a top/bottom limit
         */
        public LimitPoint(final GeodeticPoint point, final boolean side) {
            this.point = point;
            this.side  = side;
        }

        /** Get the point coordinates.
         * @return point coordinates
         */
        public GeodeticPoint getPoint() {
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
