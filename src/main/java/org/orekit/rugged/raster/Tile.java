/* Copyright 2013-2016 CS Systèmes d'Information
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
package org.orekit.rugged.raster;

import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.rugged.errors.RuggedException;
import org.orekit.rugged.utils.NormalizedGeodeticPoint;

/** Interface representing a raster tile.
 * <p>
 * The elevations are considered to be at the <em>center</em> of each cells.
 * The minimum latitude and longitude hence correspond to the <em>center</em>
 * of the most South-West cell, and the maximum latitude and longitude
 * correspond to the <em>center</em> of the most North-East cell.
 * </p>
 * @author Luc Maisonobe
 */
public interface Tile extends UpdatableTile {

    /** Enumerate for point location with respect to the interpolation grid of a tile.
     * <p>
     * Elevations in a tile are interpolated using the four neighboring points
     * in a grid: (i, j), (i+1, j), (i, j+1), (i+1), (j+1). This implies that a point
     * can be interpolated only if the elevation for these four points is available
     * in the tile. A consequence is that a point in the northernmost row (resp.
     * easternmost column) miss neighboring points at row j+1 (resp. neighboring points
     * at column i+1) and therefore cannot be interpolated.
     * </p>
     * <p>
     * This enumerate represent the position of a point taking this off-by-one property
     * into account, the value {@link #HAS_INTERPOLATION_NEIGHBORS} correspond to points that
     * do have the necessary four neightbors, whereas the other values correspond to points
     * that are either completely outside of the tile or within the tile but in either the
     * northernmost row or easternmost column.
     * </p>
     */
    enum Location {

        /** Location for points out of tile interpolation grid, in the South-West corner direction. */
        SOUTH_WEST,

        /** Location for points out of tile interpolation grid, in the West edge direction. */
        WEST,

        /** Location for points out of tile interpolation grid, in the North-West corner direction.
         * <p>
         * The point may still be in the tile, but in the northernmost row thus missing required
         * interpolation points.
         * </p>
         */
        NORTH_WEST,

        /** Location for points out of tile interpolation grid, in the North edge direction.
         * <p>
         * The point may still be in the tile, but in the northernmost row thus missing required
         * interpolation points.
         * </p>
         */
        NORTH,

        /** Location for points out of tile interpolation grid, in the North-East corner direction.
         * <p>
         * The point may still be in the tile, but either in the northernmost row or in the
         * easternmost column thus missing required interpolation points.
         * </p>
         */
        NORTH_EAST,

        /** Location for points out of tile interpolation grid, in the East edge direction.
         * <p>
         * The point may still be in the tile, but in the easternmost column thus missing required
         * interpolation points.
         * </p>
         */
        EAST,

        /** Location for points out of tile interpolation grid, in the South-East corner direction.
         * <p>
         * The point may still be in the tile, but in the easternmost column thus missing required
         * interpolation points.
         * </p>
         */
        SOUTH_EAST,

        /** Location for points out of tile interpolation grid, in the South edge direction. */
        SOUTH,

        /** Location for points that do have interpolation neighbors.
         * <p>
         * The value corresponds to points that can be interpolated using their four
         * neighboring points in the grid at indices (i, j), (i+1, j), (i, j+1), (i+1),
         * (j+1). This implies that these points are neither in the northernmost latitude
         * row nor in the easternmost longitude column.
         * </p>
         */
        HAS_INTERPOLATION_NEIGHBORS

    }

    /** Hook called at the end of tile update completion.
     * @exception RuggedException if something wrong occurs
     * (missing data ...)
     */
    void tileUpdateCompleted() throws RuggedException;

    /** Get minimum latitude of grid interpolation points.
     * @return minimum latitude of grid interpolation points
     * (latitude of the center of the cells of South row)
     */
    double getMinimumLatitude();

    /** Get the latitude at some index.
     * @param latitudeIndex latitude index
     * @return latitude at the specified index
     * (latitude of the center of the cells of specified row)
     */
    double getLatitudeAtIndex(int latitudeIndex);

    /** Get maximum latitude.
     * <p>
     * Beware that as a point at maximum latitude is the northernmost
     * one of the grid, it doesn't have a northwards neighbor and
     * therefore calling {@link #getLocation(double, double) getLocation}
     * on such a latitude will return either {@link Location#NORTH_WEST},
     * {@link Location#NORTH} or {@link Location#NORTH_EAST}, but can
     * <em>never</em> return {@link Location#HAS_INTERPOLATION_NEIGHBORS}!
     * </p>
     * @return maximum latitude
     * (latitude of the center of the cells of North row)
     */
    double getMaximumLatitude();

    /** Get minimum longitude.
     * @return minimum longitude
     * (longitude of the center of the cells of West column)
     */
    double getMinimumLongitude();

    /** Get the longitude at some index.
     * @param longitudeIndex longitude index
     * @return longitude at the specified index
     * (longitude of the center of the cells of specified column)
     */
    double getLongitudeAtIndex(int longitudeIndex);

    /** Get maximum longitude.
     * <p>
     * Beware that as a point at maximum longitude is the easternmost
     * one of the grid, it doesn't have an eastwards neighbor and
     * therefore calling {@link #getLocation(double, double) getLocation}
     * on such a longitude will return either {@link Location#SOUTH_EAST},
     * {@link Location#EAST} or {@link Location#NORTH_EAST}, but can
     * <em>never</em> return {@link Location#HAS_INTERPOLATION_NEIGHBORS}!
     * </p>
     * @return maximum longitude
     * (longitude of the center of the cells of East column)
     */
    double getMaximumLongitude();

    /** Get step in latitude (size of one raster element).
     * @return step in latitude
     */
    double getLatitudeStep();

    /** Get step in longitude (size of one raster element).
     * @return step in longitude
     */
    double getLongitudeStep();

    /** Get number of latitude rows.
     * @return number of latitude rows
     */
    int getLatitudeRows();

    /** Get number of longitude columns.
     * @return number of longitude columns
     */
    int getLongitudeColumns();

    /** Get the floor latitude index of a point.
     * <p>
     * The specified latitude is always between index and index+1.
     * </p>
     * @param latitude geodetic latitude
     * @return floor latitude index (it may lie outside of the tile!)
     */
    int getFloorLatitudeIndex(double latitude);

    /** Get the floor longitude index of a point.
     * <p>
     * The specified longitude is always between index and index+1.
     * </p>
     * @param longitude geodetic longitude
     * @return floor longitude index (it may lie outside of the tile!)
     */
    int getFloorLongitudeIndex(double longitude);

    /** Get the minimum elevation in the tile.
     * @return minimum elevation in the tile
     */
    double getMinElevation();

    /** Get the latitude index of min elevation.
     * @return latitude index of min elevation*/
    int getMinElevationLatitudeIndex();

    /** Get the longitude index of min elevation.
     * @return longitude index of min elevation*/
    int getMinElevationLongitudeIndex();

   /** Get the maximum elevation in the tile.
     * @return maximum elevation in the tile
     */
    double getMaxElevation();

    /** Get the latitude index of max elevation.
     * @return latitude index of max elevation*/
    int getMaxElevationLatitudeIndex();

    /** Get the longitude index of max elevation.
     * @return longitude index of max elevation*/
    int getMaxElevationLongitudeIndex();

    /** Get the elevation of an exact grid point.
     * @param latitudeIndex grid point index along latitude
     * @param longitudeIndex grid point index along longitude
     * @return elevation at grid point
     * @exception RuggedException if indices are out of bound
     */
    double getElevationAtIndices(int latitudeIndex, int longitudeIndex)
        throws RuggedException;

    /** Interpolate elevation.
     * <p>
     * In order to cope with numerical accuracy issues when computing
     * points at tile boundary, a slight tolerance (typically 1/8 cell)
     * around the tile is allowed. Elevation can therefore be interpolated
     * (really extrapolated in this case) even for points slightly overshooting
     * tile boundaries, using the closest tile cell. Attempting to interpolate
     * too far from the tile will trigger an exception.
     * </p>
     * @param latitude ground point latitude
     * @param longitude ground point longitude
     * @return interpolated elevation
     * @exception RuggedException if point is farthest from the tile than the tolerance
     */
    double interpolateElevation(double latitude, double longitude)
        throws RuggedException;

    /** Find the intersection of a line-of-sight and a Digital Elevation Model cell.
     * @param p point on the line
     * @param los line-of-sight, in the topocentric frame (East, North, Zenith) of the point,
     * scaled to match radians in the horizontal plane and meters along the vertical axis
     * @param latitudeIndex latitude index of the Digital Elevation Model cell
     * @param longitudeIndex longitude index of the Digital Elevation Model cell
     * @return point corresponding to line-of-sight crossing the Digital Elevation Model surface
     * if it lies within the cell, null otherwise
     * @exception RuggedException if intersection point cannot be computed
     */
    NormalizedGeodeticPoint cellIntersection(GeodeticPoint p, Vector3D los,
                                              int latitudeIndex, int longitudeIndex)
        throws RuggedException;

    /** Check if a tile covers a ground point.
     * @param latitude ground point latitude
     * @param longitude ground point longitude
     * @return location of the ground point with respect to tile
     */
    Location getLocation(double latitude, double longitude);

}
