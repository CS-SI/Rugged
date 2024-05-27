/* Copyright 2013-2022 CS GROUP
 * Licensed to CS GROUP (CS) under one or more
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

/** Interface used to update Digital Elevation Model tiles.
 * <p>
 * Implementations of this interface must be provided by
 * the image processing mission-specific layer, thus allowing
 * the Rugged library to access the Digital Elevation Model data.
 * </p>
 * @author Luc Maisonobe
 * @author Guylaine Prat
 */
public interface TileUpdater {

    /** Update the tile according to the Digital Elevation Model.
     * <p>
     * This method is the hook used by the Rugged library to delegate
     * Digital Elevation Model loading to user-provided mission-specific
     * code. When this method is called, the specified {@link UpdatableTile
     * tile} is empty and must be updated by calling {@link
     * UpdatableTile#setGeometry(double, double, double, double, int, int)
     * tile.setGeometry} once at the start of the method to set up the tile
     * geometry, and then calling {@link UpdatableTile#setElevation(int, int,
     * double) tile.setElevation} once for each cell in the tile to set the
     * cell elevation.
     * </p>
     * <p>
     * The implementation must fulfill the requirements:
     * </p>
     * <ul>
     *   <li>
     *     The tiles must overlap each other by one cell (i.e. cells
     *     that belong to the northernmost row of one tile must also belong
     *     to the sourthernmost row of another tile and cells that
     *     belong to the easternmost column of one tile must also belong
     *     to the westernmost column of another tile).
     *   </li>
     *   <li>
     *     As elevations are interpolated within Digital Elevation Model
     *     cells using four cells at indices (kLat, kLon), (kLat+1, kLon),
     *     (kLat, kLon+1), (kLat+1, kLon+1). A point in the northernmost row
     *     (resp. easternmost column) miss neighboring points at row kLat+1
     *     (resp. neighboring points at column kLon+1) and therefore cannot
     *     be interpolated. The method should therefore select the northernmost
     *     tile if the specified latitude is in the overlapping row between two
     *     tiles, and it should select the easternmost tile if the specified
     *     longitude is in the overlapping column between two tiles. Failing
     *     to do so will trigger an error at caller level mentioning the missing
     *     required neighbors.
     *   </li>
     *   <li>
     *     The elevation at cells as set when calling {@link
     *     UpdatableTile#setElevation(int, int, double) tile.setElevation(kLat, kLon,
     *     elevation)} must be the elevation corresponding to the latitude
     *     {@code minLatitude + kLat * latitudeStep} and longitude {@code
     *     minLongitude + kLon * longitudeStep}, where {@code minLatitude},
     *     {@code latitudeStep}, {@code minLongitude} and {@code longitudeStep}
     *     correspond to the parameter of the {@link UpdatableTile#setGeometry(double,
     *     double, double, double, int, int) tile.setGeometry(minLatitude, minLongitude,
           latitudeStep, longitudeStep, latitudeRows, longitudeColumns)} call.
     *   </li>
     * </ul>
     * @param latitude latitude that must be covered by the tile (rad)
     * @param longitude longitude that must be covered by the tile (rad)
     * @param tile to update
     */
    void updateTile(double latitude, double longitude, UpdatableTile tile);

}
