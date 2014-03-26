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

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
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
                final Vector3D entryP = ellipsoid.pointAtAltitude(position, los, tile.getMaxElevation() + STEP);
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

                // find where line-of-sight exit tile
                final LimitPoint exit = findExit(tile, ellipsoid, position, los);

                final GeodeticPoint intersection =
                        recurseIntersection(ellipsoid, position, los, tile,
                                            current,
                                            tile.getLatitudeIndex(current.getLatitude()),
                                            tile.getLongitudeIndex(current.getLongitude()),
                                            exit.getPoint(),
                                            tile.getLatitudeIndex(exit.getPoint().getLatitude()),
                                            tile.getLongitudeIndex(exit.getPoint().getLongitude()));

                if (intersection != null) {
                    // we have found the intersection
                    return intersection;
                } else if (exit.atSide()) {
                    // no intersection on this tile, we can proceed to next part of the line-of-sight

                    // select next tile after current point
                    final Vector3D forward = new Vector3D(1.0, ellipsoid.transform(exit.getPoint()), STEP, los);
                    current = ellipsoid.transform(forward, ellipsoid.getBodyFrame(), null);
                    tile = cache.getTile(current.getLatitude(), current.getLongitude());

                    if (tile.interpolateElevation(current.getLatitude(), current.getLongitude()) <= current.getAltitude()) {
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

    /** Compute intersection of line with Digital Elevation Model in a sub-tile.
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
    private GeodeticPoint recurseIntersection(final ExtendedEllipsoid ellipsoid, final Vector3D position,
                                              final Vector3D los, final MinMaxTreeTile tile,
                                              final GeodeticPoint entry, final int entryLat, final int entryLon,
                                              final GeodeticPoint exit, final int exitLat, final int exitLon)
        throws RuggedException, OrekitException {

        if (entryLat == exitLat && entryLon == exitLon) {
            // we have narrowed the search down to a single Digital Elevation Model pixel
            GeodeticPoint intersection = tile.pixelIntersection(entry, ellipsoid.convertLos(entry, los), exitLat, exitLon);
            if (intersection != null) {
                // improve the point, by projecting it back on the 3D line, fixing the small body curvature at pixel level
                final Vector3D      delta     = ellipsoid.transform(intersection).subtract(position);
                final double        s         = Vector3D.dotProduct(delta, los) / los.getNormSq();
                final GeodeticPoint projected = ellipsoid.transform(new Vector3D(1, position, s, los),
                                                                    ellipsoid.getBodyFrame(), null);
                intersection = tile.pixelIntersection(projected, ellipsoid.convertLos(projected, los), exitLat, exitLon);
            }
            return intersection;
        }

        // find the deepest level in the min/max kd-tree at which entry and exit share a sub-tile
        final int level = tile.getMergeLevel(entryLat, entryLon, exitLat, exitLon);
        if (level >= 0  && exit.getAltitude() >= tile.getMaxElevation(exitLat, exitLon, level)) {
            // the line-of-sight segment is fully above Digital Elevation Model
            // we can safely reject it and proceed to next part of the line-of-sight
            return null;
        }

        GeodeticPoint previousGP  = entry;
        int           previousLat = entryLat;
        int           previousLon = entryLon;

        // introduce all intermediate points corresponding to the line-of-sight
        // intersecting the boundary between level 0 sub-tiles
        if (tile.isColumnMerging(level + 1)) {
            // recurse through longitude crossings

            int[] crossings = tile.getCrossedBoundaryColumns(previousLon, exitLon, level + 1);
            for (final int crossingLon : crossings) {

                // compute segment endpoints
                final Vector3D      crossingP    = ellipsoid.pointAtLongitude(position, los,
                                                                              tile.getLongitudeAtIndex(crossingLon));
                final GeodeticPoint crossingGP   = ellipsoid.transform(crossingP, ellipsoid.getBodyFrame(), null);
                final int           crossingLat  = tile.getLatitudeIndex(crossingGP.getLatitude());

                // adjust indices as the crossing point is by definition between the sub-tiles
                final int crossingLonBefore = crossingLon - (entryLon <= exitLon ? 1 : 0);
                final int crossingLonAfter  = crossingLon - (entryLon <= exitLon ? 0 : 1);

                // look for intersection
                final GeodeticPoint intersection = recurseIntersection(ellipsoid, position, los, tile,
                                                                       previousGP, previousLat, previousLon,
                                                                       crossingGP, crossingLat, crossingLonBefore);
                if (intersection != null) {
                    return intersection;
                }

                // prepare next segment
                previousGP  = crossingGP;
                previousLat = crossingLat;
                previousLon = crossingLonAfter;

            }
        } else {
            // recurse through latitude crossings
            int[] crossings = tile.getCrossedBoundaryRows(previousLat, exitLat, level + 1);
            for (final int crossingLat : crossings) {

                // compute segment endpoints
                final Vector3D      crossingP    = ellipsoid.pointAtLatitude(position, los,
                                                                             tile.getLatitudeAtIndex(crossingLat));
                final GeodeticPoint crossingGP   = ellipsoid.transform(crossingP, ellipsoid.getBodyFrame(), null);
                final int           crossingLon  = tile.getLongitudeIndex(crossingGP.getLongitude());

                // adjust indices as the crossing point is by definition between the sub-tiles
                final int crossingLatBefore = crossingLat - (entryLat <= exitLat ? 1 : 0);
                final int crossingLatAfter  = crossingLat - (entryLat <= exitLat ? 0 : 1);

                // look for intersection
                final GeodeticPoint intersection = recurseIntersection(ellipsoid, position, los, tile,
                                                                       previousGP, previousLat, previousLon,
                                                                       crossingGP, crossingLatBefore, crossingLon);
                if (intersection != null) {
                    return intersection;
                }

                // prepare next segment
                previousGP  = crossingGP;
                previousLat = crossingLatAfter;
                previousLon = crossingLon;

            }
        }

        // last part of the segment, up to exit point
        return recurseIntersection(ellipsoid, position, los, tile,
                                   previousGP, previousLat, previousLon,
                                   exit, exitLat, exitLon);

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
        Vector3D exitP = ellipsoid.pointAtAltitude(position, los, tile.getMinElevation() - STEP);
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
