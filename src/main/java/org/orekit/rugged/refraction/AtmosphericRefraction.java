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
package org.orekit.rugged.refraction;


import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.orekit.rugged.errors.RuggedException;
import org.orekit.rugged.intersection.IntersectionAlgorithm;
import org.orekit.rugged.linesensor.SensorPixel;
import org.orekit.rugged.utils.NormalizedGeodeticPoint;

/**
 * Interface for atmospheric refraction model.
 * @author Sergio Esteves
 * @author Guylaine Prat
 * @since 2.0
 */
public abstract class AtmosphericRefraction {

    /**
     * Flag to tell if we must compute the correction with an optimization grid
     */
    private boolean isOptimized = false;

    /**
     * The current optimization grid
     */
    private AtmosphericOptimizationGrid optimizationGrid;

    /** Apply correction to the intersected point with an atmospheric refraction model.
     * @param satPos satellite position, in <em>body frame</em>
     * @param satLos satellite line of sight, in <em>body frame</em>
     * @param rawIntersection intersection point before refraction correction
     * @param algorithm intersection algorithm
     * @return corrected point with the effect of atmospheric refraction
     * @throws RuggedException if there is no refraction data at altitude of rawIntersection or see
     * {@link org.orekit.rugged.utils.ExtendedEllipsoid#pointAtAltitude(Vector3D, Vector3D, double)} or see
     * {@link org.orekit.rugged.intersection.IntersectionAlgorithm#refineIntersection(org.orekit.rugged.utils.ExtendedEllipsoid, Vector3D, Vector3D, NormalizedGeodeticPoint)}
     */
    public abstract NormalizedGeodeticPoint applyCorrection(Vector3D satPos, Vector3D satLos, NormalizedGeodeticPoint rawIntersection,
                                            IntersectionAlgorithm algorithm)
        throws RuggedException;


    // TODO to be implemented
    public void setOptimizationGrid() throws RuggedException {

        this.isOptimized = true;

        // TODO default value to compute/initialize here ...
        final SensorPixel sensorPixelStart = new SensorPixel(Double.NaN, Double.NaN);
        final int pixelColumns = Integer.MAX_VALUE;
        final int lineRows = Integer.MAX_VALUE;


        // Set the grid for optimization
        this.optimizationGrid = new AtmosphericOptimizationGrid(sensorPixelStart, pixelColumns, lineRows);

        //TODO add check of input : see SimpleTile

    };

    public void setOptimizationGrid(final SensorPixel sensorPixelStart,
                                    final int pixelColumns, final int lineRows) throws RuggedException {

        this.isOptimized = true;

        // Set the grid for optimization
        this.optimizationGrid = new AtmosphericOptimizationGrid(sensorPixelStart, pixelColumns, lineRows);

        //TODO add check of input : see SimpleTile

    };

    /** Tell if the computation must be optimized
     * @return the isOptimized
     */
    public boolean isOptimized() {
        return isOptimized;
    }

    /**
     * @return the optimizationGrid
     */
    public AtmosphericOptimizationGrid getOptimizationGrid() {
        return optimizationGrid;
    }

    /**
     * @param optimizationGrid the optimizationGrid to set
     */
    public void setOptimizationGrid(final AtmosphericOptimizationGrid optimizationGrid) {
        this.optimizationGrid = optimizationGrid;
    }
}
