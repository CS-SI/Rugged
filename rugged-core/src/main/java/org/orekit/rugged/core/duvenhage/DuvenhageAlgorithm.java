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

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.util.FastMath;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.errors.OrekitException;
import org.orekit.rugged.api.RuggedException;
import org.orekit.rugged.api.RuggedMessages;
import org.orekit.rugged.api.TileUpdater;
import org.orekit.rugged.core.ExtendedEllipsoid;
import org.orekit.rugged.core.raster.IntersectionAlgorithm;
import org.orekit.rugged.core.raster.Tile;
import org.orekit.rugged.core.raster.TilesCache;

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
    private TilesCache<MinMaxTreeTile> cache;

    /** Simple constructor.
     */
    public DuvenhageAlgorithm() {
    }

    /** {@inheritDoc} */
    @Override
    public void setUpTilesManagement(final TileUpdater updater, final int maxCachedTiles) {
        cache = new TilesCache<MinMaxTreeTile>(new MinMaxTreeTileFactory(), updater, maxCachedTiles);
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
                int currentLonIndex = tile.getLongitudeIndex(current.getLongitude());

                // find where line-of-sight exit tile
                final LimitPoint exit = findExit(tile, ellipsoid, position, los);
                final List<GeodeticPoint> lineOfSightQueue = new ArrayList<GeodeticPoint>();
                lineOfSightQueue.add(exit.getPoint());

                while (!lineOfSightQueue.isEmpty()) {

                    final GeodeticPoint next = lineOfSightQueue.remove(lineOfSightQueue.size() - 1);
                    final int nextLatIndex   = tile.getLatitudeIndex(next.getLatitude());
                    final int nextLonIndex   = tile.getLongitudeIndex(next.getLongitude());
                    if (FastMath.abs(currentLatIndex - nextLatIndex) <= 1 &&
                        FastMath.abs(currentLonIndex - nextLonIndex) <= 1) {

                        // we have narrowed the search down to a single Digital Elevation Model pixel
                        final GeodeticPoint intersection =
                                tile.pixelIntersection(ellipsoid, current, next, nextLatIndex, nextLonIndex);
                        if (intersection != null) {
                            return intersection;
                        } else {
                            // no intersection on this pixel, we can proceed to next part of the line-of-sight
                            current = next;
                        }

                    } else {

                        // find the deepest level in the min/max kd-tree at which entry and exit share a sub-tile
                        final int level = tile.getMergeLevel(currentLatIndex, currentLonIndex, nextLatIndex, nextLonIndex);
                        if (level < 0) {
                            // introduce all intermediate points corresponding to the line-of-sight
                            // intersecting the boundary between level 0 sub-tiles
                            lineOfSightQueue.addAll(crossingPoints(ellipsoid, position, los,
                                                                   tile, 0,
                                                                   nextLatIndex, nextLonIndex,
                                                                   currentLatIndex, currentLonIndex));
                        } else {
                            if (next.getAltitude() >= tile.getMaxElevation(nextLatIndex, nextLonIndex, level)) {
                                // the line-of-sight segment is fully above Digital Elevation Model
                                // we can safely reject it and proceed to next part of the line-of-sight
                                current = next;
                            } else {
                                // the line-of-sight segment has at least some undecided parts which may
                                // intersect the Digital Elevation Model, we need to refine the
                                // search by using a finer-grained level in the min/max kd-tree

                                // push the point back into the queue
                                lineOfSightQueue.add(next);

                                // introduce all intermediate points corresponding to the line-of-sight
                                // intersecting the boundary between finer sub-tiles as we go deeper
                                // in the tree
                                lineOfSightQueue.addAll(crossingPoints(ellipsoid, position, los,
                                                                       tile, level,
                                                                       nextLatIndex, nextLonIndex,
                                                                       currentLatIndex, currentLonIndex));

                                // the current point remains the same

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

    /** Find the crossing points between sub-tiles.
     * <p>
     * When we go deeper in the min/max kd-tree, we get closer to individual pixels,
     * or un-merge the merged sub-tiles. This un-merging implies finding the boundary
     * between sub-tiles that are merged at level l and not merged at level l+1.
     * This boundary is an iso-latitude if the merge is a row merging and is an
     * iso-longitude if the merge is a column merging.
     * </p>
     * @param ellipsoid reference ellipsoid
     * @param position pixel position in ellipsoid frame
     * @param los pixel line-of-sight in ellipsoid frame
     * @param tile Digital Elevation Model tile
     * @param level merged level
     * @param nextLatitude latitude index of next point (closer to Earth)
     * @param nextLongitude longitude index of next point (closer to Earth)
     * @param currentLatitude latitude index of current point (closer to satellite)
     * @param currentLongitude longitude index of current point (closer to satellite)
     * @return point corresponding to line-of-sight crossing the longitude/latitude
     * limit between the un-merged sub-tiles at level-1
     * @exception RuggedException if intersection point cannot be computed
     * @exception OrekitException if intersection point cannot be converted to geodetic coordinates
     */
    private List<GeodeticPoint> crossingPoints(final ExtendedEllipsoid ellipsoid, final Vector3D position, final Vector3D los,
                                               final MinMaxTreeTile tile, final int level,
                                               final int nextLatitude, final int nextLongitude,
                                               final int currentLatitude, final int currentLongitude)
        throws RuggedException, OrekitException {

        final List<GeodeticPoint> crossings = new ArrayList<GeodeticPoint>();

        if (tile.isColumnMerging(level + 1)) {
            // sub-tiles at current level come from column merging at deeper level
            for (final int longitudeIndex : tile.getCrossedBoundaryColumns(nextLongitude, currentLongitude, level)) {
                final double crossingLongitude = tile.getLongitudeAtIndex(longitudeIndex);
                final Vector3D crossingPoint   = ellipsoid.pointAtLongitude(position, los, crossingLongitude);
                crossings.add(ellipsoid.transform(crossingPoint, ellipsoid.getBodyFrame(), null));
            }
        } else {
            // sub-tiles at current level come from row merging at deeper level
            for (final int latitudeIndex : tile.getCrossedBoundaryRows(nextLatitude, currentLatitude, level)) {
                final double crossingLatitude = tile.getLatitudeAtIndex(latitudeIndex);
                final Vector3D crossingPoint   = ellipsoid.pointAtLatitude(position, los, crossingLatitude);
                crossings.add(ellipsoid.transform(crossingPoint, ellipsoid.getBodyFrame(), null));
            }
        }

        return crossings;

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
