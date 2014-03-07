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

import org.orekit.rugged.api.UpdatableTile;

/** Simple implementation of a {@link UpdatableTile}.
 * @author Luc Maisonobe
 */
public class SimpleTile implements Tile {

    /** Reference latitude. */
    private double referenceLatitude;

    /** Reference longitude. */
    private double referenceLongitude;

    /** Step in latitude (size of one raster element). */
    private double latitudeStep;

    /** Step in longitude (size of one raster element). */
    private double longitudeStep;

    /** Number of latitude rows. */
    private int latitudeRows;

    /** Number of longitude columns. */
    private int longitudeColumns;

    /** Elevation array. */
    private double[] elevations;

    /** Simple constructor.
     * <p>
     * Creates an empty tile.
     * </p>
     */
    public SimpleTile() {
    }

    /** {@inheritDoc} */
    @Override
    public void setGeometry(final double referenceLatitude, final double referenceLongitude,
                     final double latitudeStep, final double longitudeStep,
                     final int latitudeRows, final int longitudeColumns) {
        this.referenceLatitude  = referenceLatitude;
        this.referenceLongitude = referenceLongitude;
        this.latitudeStep       = latitudeStep;
        this.longitudeStep      = longitudeStep;
        this.latitudeRows       = latitudeRows;
        this.longitudeColumns   = longitudeColumns;
        this.elevations         = new double[latitudeRows * longitudeColumns];
    }

    /** {@inheritDoc} */
    @Override
    public void setElevation(final int latitudeIndex, final int longitudeIndex,
                             final double elevation) throws IllegalArgumentException {
        checkIndices(latitudeIndex, longitudeIndex);
        elevations[latitudeIndex * longitudeColumns + longitudeIndex] = elevation;
    }

    /** {@inheritDoc} */
    @Override
    public double getReferenceLatitude() {
        return referenceLatitude;
    }

    /** {@inheritDoc} */
    @Override
    public double getReferenceLongitude() {
        return referenceLongitude;
    }

    /** {@inheritDoc} */
    @Override
    public double getLatitudeStep() {
        return latitudeStep;
    }

    /** {@inheritDoc} */
    @Override
    public double getLongitudeStep() {
        return longitudeStep;
    }

    /** {@inheritDoc} */
    @Override
    public int getLatitudeRows() {
        return latitudeRows;
    }

    /** {@inheritDoc} */
    @Override
    public int getLongitudeColumns() {
        return longitudeColumns;
    }

    /** {@inheritDoc} */
    @Override
    public double getElevationAtIndices(int latitudeIndex, int longitudeIndex)
        throws IllegalArgumentException {
        checkIndices(latitudeIndex, longitudeIndex);
        return elevations[latitudeIndex * longitudeColumns + longitudeIndex];
    }

    /** Check indices.
     * @param latitudeIndex
     * @param longitudeIndex
     * @exception IllegalArgumentException if indices are out of bound
     */
    private void checkIndices(int latitudeIndex, int longitudeIndex)
        throws IllegalArgumentException {
        if (latitudeIndex < 0 || latitudeIndex >= latitudeRows ||
            longitudeIndex < 0 || longitudeIndex >= longitudeColumns) {
            throw new IllegalArgumentException();
        }        
    }

}
