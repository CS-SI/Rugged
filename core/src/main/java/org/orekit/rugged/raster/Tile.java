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
package org.orekit.rugged.raster;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.rugged.api.RuggedException;

/** Interface representing a raster tile.
 * @author Luc Maisonobe
 */
public interface Tile extends UpdatableTile {

    /** Enumerate for point location with respect to tile. */
    enum Location {

        /** Location for points out of tile, past the South-West corner. */
        SOUTH_WEST,

        /** Location for points out of tile, past the West edge. */
        WEST,

        /** Location for points out of tile, past the North-West corner. */
        NORTH_WEST,

        /** Location for points out of tile, past the North edge. */
        NORTH,

        /** Location for points out of tile, past the North-East corner. */
        NORTH_EAST,

        /** Location for points out of tile, past the East edge. */
        EAST,

        /** Location for points out of tile, past the South-East corner. */
        SOUTH_EAST,

        /** Location for points out of tile, past the South edge. */
        SOUTH,

        /** Location for points within tile. */
        IN_TILE

    }

    /** Hook called at the end of tile update completion.
     * @exception RuggedException if something wrong occurs
     * (missing data ...)
     */
    void tileUpdateCompleted() throws RuggedException;

    /** Get minimum latitude.
     * @return minimum latitude
     */
    double getMinimumLatitude();

    /** Get the latitude at some index.
     * @param latitudeIndex latitude index
     * @return latitude at the specified index
     */
    double getLatitudeAtIndex(int latitudeIndex);

    /** Get maximum latitude.
     * @return maximum latitude
     */
    double getMaximumLatitude();

    /** Get minimum longitude.
     * @return minimum longitude
     */
    double getMinimumLongitude();

    /** Get the longitude at some index.
     * @param longitudeIndex longitude index
     * @return longitude at the specified index
     */
    double getLongitudeAtIndex(int longitudeIndex);

    /** Get maximum longitude.
     * @return maximum longitude
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

    /** Get the latitude index of a point.
     * @param latitude geodetic latitude
     * @return latirute index (it may lie outside of the tile!)
     */
    int getLatitudeIndex(double latitude);

    /** Get the longitude index of a point.
     * @param longitude geodetic latitude
     * @return longitude index (it may lie outside of the tile!)
     */
    int getLongitudeIndex(double longitude);

    /** Get the minimum elevation in the tile.
     * @return minimum elevation in the tile
     */
    double getMinElevation();

    /** Get the maximum elevation in the tile.
     * @return maximum elevation in the tile
     */
    double getMaxElevation();

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
     * points at tile boundary, a slight tolerance (typically 1/8 pixel)
     * around the tile is allowed. Elevation can therefore be interpolated
     * (really extrapolated in this case) even for points slightly overshooting
     * tile boundaries, using the closest tile pixel. Attempting to interpolate
     * too far from the tile will trigger an exception.
     * </p>
     * @param latitude ground point latitude
     * @param longitude ground point longitude
     * @return interpolated elevation
     * @exception RuggedException if point is farthest from the tile than the tolerance
     */
    double interpolateElevation(double latitude, double longitude)
        throws RuggedException;

    /** Find the intersection of a line-of-sight and a Digital Elevation Model pixel.
     * @param p point on the line
     * @param los line-of-sight, in the topocentric frame (East, North, Zenith) of the point,
     * scaled to match radians in the horizontal plane and meters along the vertical axis
     * @param latitudeIndex latitude index of the Digital Elevation Model pixel
     * @param longitudeIndex longitude index of the Digital Elevation Model pixel
     * @return point corresponding to line-of-sight crossing the Digital Elevation Model surface
     * if it lies within the pixel, null otherwise
     * @exception RuggedException if intersection point cannot be computed
     */
    GeodeticPoint pixelIntersection(GeodeticPoint p, Vector3D los,
                                    int latitudeIndex, int longitudeIndex)
        throws RuggedException;

    /** Check if a tile covers a ground point.
     * @param latitude ground point latitude
     * @param longitude ground point longitude
     * @return location of the ground point with respect to tile
     */
    Location getLocation(double latitude, double longitude);

}
