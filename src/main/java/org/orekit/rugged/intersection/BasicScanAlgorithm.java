/* Copyright 2013-2017 CS Systèmes d'Information
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
package org.orekit.rugged.intersection;

import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.util.FastMath;
import java.util.ArrayList;
import java.util.List;

import org.orekit.bodies.GeodeticPoint;
import org.orekit.errors.OrekitException;
import org.orekit.rugged.api.AlgorithmId;
import org.orekit.rugged.errors.DumpManager;
import org.orekit.rugged.errors.RuggedException;
import org.orekit.rugged.raster.SimpleTile;
import org.orekit.rugged.raster.SimpleTileFactory;
import org.orekit.rugged.raster.Tile;
import org.orekit.rugged.raster.TileUpdater;
import org.orekit.rugged.raster.TilesCache;
import org.orekit.rugged.utils.ExtendedEllipsoid;
import org.orekit.rugged.utils.NormalizedGeodeticPoint;

/** Intersection computation using a basic algorithm based on exhaustive scan.
 * <p>
 * The algorithm simply computes entry and exit points at high and low altitudes,
 * and scans all Digital Elevation Models in the sub-tiles defined by these two
 * corner points. It is not designed for operational use.
 * </p>
 * @author Luc Maisonobe
 */
public class BasicScanAlgorithm implements IntersectionAlgorithm {

    /** Cache for DEM tiles. */
    private final TilesCache<SimpleTile> cache;

    /** Minimum altitude encountered. */
    private double hMin;

    /** Maximum altitude encountered. */
    private double hMax;

    /** Simple constructor.
     * @param updater updater used to load Digital Elevation Model tiles
     * @param maxCachedTiles maximum number of tiles stored in the cache
     */
    public BasicScanAlgorithm(final TileUpdater updater, final int maxCachedTiles) {
        cache = new TilesCache<SimpleTile>(new SimpleTileFactory(), updater, maxCachedTiles);
        hMin  = Double.POSITIVE_INFINITY;
        hMax  = Double.NEGATIVE_INFINITY;
    }

    /** {@inheritDoc} */
    @Override
    public NormalizedGeodeticPoint intersection(final ExtendedEllipsoid ellipsoid,
                                                final Vector3D position, final Vector3D los)
        throws RuggedException {
        try {

            DumpManager.dumpAlgorithm(AlgorithmId.BASIC_SLOW_EXHAUSTIVE_SCAN_FOR_TESTS_ONLY);

            // find the tiles between the entry and exit point in the Digital Elevation Model
            NormalizedGeodeticPoint entryPoint = null;
            NormalizedGeodeticPoint exitPoint  = null;
            double minLatitude  = Double.NaN;
            double maxLatitude  = Double.NaN;
            double minLongitude = Double.NaN;
            double maxLongitude = Double.NaN;
            final List<SimpleTile> scannedTiles = new ArrayList<SimpleTile>();
            double centralLongitude = Double.NaN;
            for (boolean changedMinMax = true; changedMinMax; changedMinMax = checkMinMax(scannedTiles)) {

                scannedTiles.clear();
                // compute entry and exit points
                entryPoint = ellipsoid.transform(ellipsoid.pointAtAltitude(position, los, Double.isInfinite(hMax) ? 0.0 : hMax),
                                                 ellipsoid.getBodyFrame(), null,
                                                 Double.isNaN(centralLongitude) ? 0.0 : centralLongitude);
                final SimpleTile entryTile = cache.getTile(entryPoint.getLatitude(), entryPoint.getLongitude());
                if (Double.isNaN(centralLongitude)) {
                    centralLongitude = entryTile.getMinimumLongitude();
                    entryPoint = new NormalizedGeodeticPoint(entryPoint.getLatitude(), entryPoint.getLongitude(),
                                                             entryPoint.getAltitude(), centralLongitude);
                }
                addIfNotPresent(scannedTiles, entryTile);

                exitPoint = ellipsoid.transform(ellipsoid.pointAtAltitude(position, los, Double.isInfinite(hMin) ? 0.0 : hMin),
                                                ellipsoid.getBodyFrame(), null, centralLongitude);
                final SimpleTile exitTile = cache.getTile(exitPoint.getLatitude(), exitPoint.getLongitude());
                addIfNotPresent(scannedTiles, exitTile);

                minLatitude  = FastMath.min(entryPoint.getLatitude(),  exitPoint.getLatitude());
                maxLatitude  = FastMath.max(entryPoint.getLatitude(),  exitPoint.getLatitude());
                minLongitude = FastMath.min(entryPoint.getLongitude(), exitPoint.getLongitude());
                maxLongitude = FastMath.max(entryPoint.getLongitude(), exitPoint.getLongitude());

                if (scannedTiles.size() > 1) {
                    // the entry and exit tiles are different, maybe other tiles should be added on the way
                    // in the spirit of simple and exhaustive, we add all tiles in a rectangular area
                    final double latStep = 0.5 * FastMath.min(entryTile.getLatitudeStep()  * entryTile.getLatitudeRows(),
                                                              exitTile.getLatitudeStep()   * exitTile.getLatitudeRows());
                    final double lonStep = 0.5 * FastMath.min(entryTile.getLongitudeStep() * entryTile.getLongitudeColumns(),
                                                              exitTile.getLongitudeStep()  * exitTile.getLongitudeColumns());
                    for (double latitude = minLatitude; latitude <= maxLatitude; latitude += latStep) {
                        for (double longitude = minLongitude; longitude < maxLongitude; longitude += lonStep) {
                            addIfNotPresent(scannedTiles, cache.getTile(latitude, longitude));
                        }
                    }
                }

            }

            // scan the tiles
            NormalizedGeodeticPoint intersectionGP = null;
            double intersectionDot = Double.POSITIVE_INFINITY;
            for (final SimpleTile tile : scannedTiles) {
                for (int i = latitudeIndex(tile, minLatitude); i <= latitudeIndex(tile, maxLatitude); ++i) {
                    for (int j = longitudeIndex(tile, minLongitude); j <= longitudeIndex(tile, maxLongitude); ++j) {
                        final NormalizedGeodeticPoint gp = tile.cellIntersection(entryPoint, ellipsoid.convertLos(entryPoint, los), i, j);
                        if (gp != null) {

                            // improve the point, by projecting it back on the 3D line, fixing the small body curvature at cell level
                            final Vector3D      delta     = ellipsoid.transform(gp).subtract(position);
                            final double        s         = Vector3D.dotProduct(delta, los) / los.getNormSq();
                            final GeodeticPoint projected = ellipsoid.transform(new Vector3D(1, position, s, los),
                                                                                ellipsoid.getBodyFrame(), null);
                            final NormalizedGeodeticPoint normalizedProjected = new NormalizedGeodeticPoint(projected.getLatitude(),
                                                                                                            projected.getLongitude(),
                                                                                                            projected.getAltitude(),
                                                                                                            gp.getLongitude());
                            final NormalizedGeodeticPoint gpImproved = tile.cellIntersection(normalizedProjected,
                                                                                             ellipsoid.convertLos(normalizedProjected, los),
                                                                                             i, j);

                            if (gpImproved != null) {
                                final Vector3D point = ellipsoid.transform(gpImproved);
                                final double dot = Vector3D.dotProduct(point.subtract(position), los);
                                if (dot < intersectionDot) {
                                    intersectionGP  = gpImproved;
                                    intersectionDot = dot;
                                }
                            }

                        }
                    }
                }
            }

            return intersectionGP;

        } catch (OrekitException oe) {
            // this should never happen
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
            DumpManager.dumpAlgorithm(AlgorithmId.BASIC_SLOW_EXHAUSTIVE_SCAN_FOR_TESTS_ONLY);
            final Vector3D      delta     = ellipsoid.transform(closeGuess).subtract(position);
            final double        s         = Vector3D.dotProduct(delta, los) / los.getNormSq();
            final GeodeticPoint projected = ellipsoid.transform(new Vector3D(1, position, s, los),
                                                                ellipsoid.getBodyFrame(), null);
            final NormalizedGeodeticPoint normalizedProjected = new NormalizedGeodeticPoint(projected.getLatitude(),
                                                                                            projected.getLongitude(),
                                                                                            projected.getAltitude(),
                                                                                            closeGuess.getLongitude());
            final Tile          tile      = cache.getTile(normalizedProjected.getLatitude(),
                                                          normalizedProjected.getLongitude());
            return tile.cellIntersection(normalizedProjected,
                                         ellipsoid.convertLos(normalizedProjected, los),
                                         tile.getFloorLatitudeIndex(normalizedProjected.getLatitude()),
                                         tile.getFloorLongitudeIndex(normalizedProjected.getLongitude()));
        } catch (OrekitException oe) {
            throw new RuggedException(oe, oe.getSpecifier(), oe.getParts());
        }
    }

    /** {@inheritDoc} */
    @Override
    public double getElevation(final double latitude, final double longitude)
        throws RuggedException {
        DumpManager.dumpAlgorithm(AlgorithmId.BASIC_SLOW_EXHAUSTIVE_SCAN_FOR_TESTS_ONLY);
        final Tile tile = cache.getTile(latitude, longitude);
        return tile.interpolateElevation(latitude, longitude);
    }

    /** Check the overall min and max altitudes.
     * @param tiles tiles to check
     * @return true if the tile changed either min or max altitude
     */
    private boolean checkMinMax(final List<SimpleTile> tiles) {

        boolean changedMinMax = false;

        for (final SimpleTile tile : tiles) {

            // check minimum altitude
            if (tile.getMinElevation() < hMin) {
                hMin          = tile.getMinElevation();
                changedMinMax = true;
            }

            // check maximum altitude
            if (tile.getMaxElevation() > hMax) {
                hMax          = tile.getMaxElevation();
                changedMinMax = true;
            }

        }

        return changedMinMax;

    }

    /** Add a tile to a list if not already present.
     * @param list tiles list
     * @param tile new tile to consider
     */
    private void addIfNotPresent(final List<SimpleTile> list, final SimpleTile tile) {

        // look for existing tiles in the list
        for (final SimpleTile existing : list) {
            if (existing == tile) {
                return;
            }
        }

        // the tile was not there, add it
        list.add(tile);

    }

    /** Get latitude index.
     * @param tile current tile
     * @param latitude current latitude
     * @return index of latitude, truncated at tiles limits
     */
    private int latitudeIndex(final SimpleTile tile, final double latitude) {
        final int rawIndex = tile.getFloorLatitudeIndex(latitude);
        return FastMath.min(FastMath.max(0, rawIndex), tile.getLatitudeRows());
    }

    /** Get longitude index.
     * @param tile current tile
     * @param longitude current longitude
     * @return index of longitude, truncated at tiles limits
     */
    private int longitudeIndex(final SimpleTile tile, final double longitude) {
        final int rawIndex = tile.getFloorLongitudeIndex(longitude);
        return FastMath.min(FastMath.max(0, rawIndex), tile.getLongitudeColumns());
    }

}
