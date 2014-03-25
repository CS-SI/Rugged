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

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.util.FastMath;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.errors.OrekitException;
import org.orekit.rugged.api.RuggedException;
import org.orekit.rugged.api.TileUpdater;
import org.orekit.rugged.core.raster.IntersectionAlgorithm;
import org.orekit.rugged.core.raster.SimpleTile;
import org.orekit.rugged.core.raster.SimpleTileFactory;
import org.orekit.rugged.core.raster.TilesCache;

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
    private TilesCache<SimpleTile> cache;

    /** Minimum altitude encountered. */
    private double hMin;

    /** Maximum altitude encountered. */
    private double hMax;

    /** Simple constructor.
     */
    public BasicScanAlgorithm() {
    }

    /** {@inheritDoc} */
    @Override
    public void setUpTilesManagement(final TileUpdater updater, final int maxCachedTiles) {
        cache = new TilesCache<SimpleTile>(new SimpleTileFactory(), updater, maxCachedTiles);
        hMin  = Double.POSITIVE_INFINITY;
        hMax  = Double.NEGATIVE_INFINITY;
    }

    /** {@inheritDoc} */
    @Override
    public GeodeticPoint intersection(final ExtendedEllipsoid ellipsoid,
                                      final Vector3D position, final Vector3D los)
        throws RuggedException {
        try {

            // find the tiles between the entry and exit point in the Digital Elevation Model
            GeodeticPoint entryPoint = null;
            GeodeticPoint exitPoint  = null;
            double minLatitude  = Double.NaN;
            double maxLatitude  = Double.NaN;
            double minLongitude = Double.NaN;
            double maxLongitude = Double.NaN;
            List<SimpleTile> scannedTiles = new ArrayList<SimpleTile>();
            for (boolean changedMinMax = true; changedMinMax; changedMinMax = checkMinMax(scannedTiles)) {

                scannedTiles.clear();
                // compute entry and exit points
                entryPoint = ellipsoid.transform(ellipsoid.pointAtAltitude(position, los, Double.isInfinite(hMax) ? 0.0 : hMax),
                                                 ellipsoid.getBodyFrame(), null);
                final SimpleTile entryTile = cache.getTile(entryPoint.getLatitude(), entryPoint.getLongitude());
                addIfNotPresent(scannedTiles, entryTile);

                exitPoint = ellipsoid.transform(ellipsoid.pointAtAltitude(position, los, Double.isInfinite(hMin) ? 0.0 : hMin),
                                                ellipsoid.getBodyFrame(), null);
                final SimpleTile exitTile = cache.getTile(exitPoint.getLatitude(), exitPoint.getLongitude());
                addIfNotPresent(scannedTiles, entryTile);

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
            GeodeticPoint intersectionGP = null;
            double intersectionDot = Double.POSITIVE_INFINITY;
            for (final SimpleTile tile : scannedTiles) {
                for (int i = latitudeIndex(tile, minLatitude); i <= latitudeIndex(tile, maxLatitude); ++i) {
                    for (int j = longitudeIndex(tile, minLongitude); j <= longitudeIndex(tile, maxLongitude); ++j) {
                        GeodeticPoint gp = tile.pixelIntersection(entryPoint, ellipsoid.convertLos(entryPoint, los), i, j);
                        if (gp != null) {
                            final Vector3D point = ellipsoid.transform(gp);
                            final double dot = Vector3D.dotProduct(point.subtract(position), los);
                            if (dot < intersectionDot) {
                                intersectionGP  = gp;
                                intersectionDot = dot;
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
        final int rawIndex = tile.getLatitudeIndex(latitude);
        return FastMath.min(FastMath.max(0, rawIndex), tile.getLatitudeRows());
    }

    /** Get longitude index.
     * @param tile current tile
     * @param longitude current longitude
     * @return index of longitude, truncated at tiles limits
     */
    private int longitudeIndex(final SimpleTile tile, final double longitude) {
        final int rawIndex = tile.getLongitudeIndex(longitude);
        return FastMath.min(FastMath.max(0, rawIndex), tile.getLongitudeColumns());
    }

}
