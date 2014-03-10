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
package org.orekit.rugged.core.dem;


/** Simple implementation of a {@link Tile}.
 * @see SimpleTileFactory
 * @author Luc Maisonobe
 */
public class SimpleTile extends AbstractTile {

    /** Elevation array. */
    private double[] elevations;

    /** Simple constructor.
     * <p>
     * Creates an empty tile.
     * </p>
     */
    SimpleTile() {
    }

    /** {@inheritDoc} */
    @Override
    protected void doSetGeometry(final double minLatitude, final double minLongitude,
                                 final double latitudeStep, final double longitudeStep,
                                 final int latitudeRows, final int longitudeColumns) {
        this.elevations = new double[latitudeRows * longitudeColumns];
    }

    /** {@inheritDoc} */
    @Override
    protected void doSetElevation(final int latitudeIndex, final int longitudeIndex,
                                  final double elevation) {
        elevations[latitudeIndex * getLongitudeColumns() + longitudeIndex] = elevation;
    }

    /** {@inheritDoc} */
    @Override
    protected double doGetElevationAtIndices(int latitudeIndex, int longitudeIndex) {
        return elevations[latitudeIndex * getLongitudeColumns() + longitudeIndex];
    }

}
