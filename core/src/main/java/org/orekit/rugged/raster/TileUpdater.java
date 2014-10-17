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

import org.orekit.rugged.api.RuggedException;

/** Interface used to update Digital Elevation Model tiles.
 * <p>
 * Implementations of this interface are must be provided by
 * the image processing mission-specific layer, thus allowing
 * the Rugged library to access the Digital Elevation Model data.
 * </p>
 * @author Luc Maisonobe
 */
public interface TileUpdater {

    /** Set the tile global geometry.
     * <p>
     * As elevations are interpolated within Digital Elevation Model
     * pixels using four pixels at indices (i, j), (i+1, j), (i, j+1)
     * (i+1, j+1), the last row and last column of each tile is considered
     * to be <em>not</em> covered by this tile (as checked using {@link
     * Tile#getLocation(double, double)}) and should therefore be
     * in the next tile (Eastwards or Northwards), which should have
     * on row/column of <em>overlap</em>. This implies that this method
     * must ensure that the latitude and longitude specified here do
     * not lie in the last row/column of the returned tile, otherwise
     * an error will be triggered by caller.
     * <p>
     * @param latitude latitude that must be covered by the tile
     * @param longitude longitude that must be covered by the tile
     * @param tile to update
     * @exception RuggedException if tile cannot be updated
     */
    void updateTile(double latitude, double longitude, UpdatableTile tile)
        throws RuggedException;

}
