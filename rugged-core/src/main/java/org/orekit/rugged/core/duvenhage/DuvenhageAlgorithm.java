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
import org.orekit.rugged.api.GroundPoint;
import org.orekit.rugged.api.TileUpdater;
import org.orekit.rugged.core.dem.IntersectionAlgorithm;
import org.orekit.rugged.core.dem.TilesCache;

/** Direct and inverse localization using Duvenhage's algorithm.
 * <p>
 * The algorithm is described in the 2009 paper:
 * <a href="http://researchspace.csir.co.za/dspace/bitstream/10204/3041/1/Duvenhage_2009.pdf">Using
 * An Implicit Min/Max KD-Tree for Doing Efficient Terrain Line of Sight Calculations</a>.
 * </p>
 * @author Luc Maisonobe
 */
public class DuvenhageAlgorithm implements IntersectionAlgorithm {

    /** Cache for DEM tiles. */
    private TilesCache<MinMaxTreeTile> cache;

    /** Simple constructor.
     */
    public DuvenhageAlgorithm() {
    }

    /** {@inheritDoc} */
    @Override
    public void setUpTilesManagement(TileUpdater updater, int maxCachedTiles) {
        cache = new TilesCache<MinMaxTreeTile>(new MinMaxTreeTileFactory(),
                                               updater, maxCachedTiles);
    }

    /** {@inheritDoc} */
    @Override
    public GroundPoint intersection(double latitude0, double longitude0,
                                    Vector3D direction) {
        // TODO: compute intersection
        return null;
    }

}
