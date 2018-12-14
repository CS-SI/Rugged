/* Copyright 2013-2018 CS Systèmes d'Information
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

import org.hipparchus.analysis.interpolation.BilinearInterpolatingFunction;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.orekit.rugged.errors.RuggedException;
import org.orekit.rugged.errors.RuggedMessages;
import org.orekit.rugged.intersection.IntersectionAlgorithm;
import org.orekit.rugged.linesensor.LineSensor;
import org.orekit.rugged.linesensor.SensorPixel;
import org.orekit.rugged.utils.NormalizedGeodeticPoint;

/**
 * Interface for atmospheric refraction model.
 * @author Sergio Esteves
 * @author Guylaine Prat
 * @since 2.0
 */
public abstract class AtmosphericRefraction {

    /** Flag to tell if we must compute the correction.
     * By default: computation is set up.
     * @since 3.0
     */
    private boolean mustBeComputed = true;

    /** Flag to tell if we must compute the correction (for direct location) with an optimization grid.
     * By default: optimization is not set up.
     * @since 3.0
     */
    private boolean isOptimized = false;

    /** The current atmospheric parameters.
     * @since 3.0
     */
    private AtmosphericComputationParameters atmosphericParams;

    /** Bilinear interpolating function for pixel (used by inverse location).
     * @since 3.0
    */
    private BilinearInterpolatingFunction bifPixel = null;

    /** Bilinear interpolating function of line (used by inverse location).
     * @since 3.0
    */
    private BilinearInterpolatingFunction bifLine = null;

    /**
     * Default constructor.
     */
    protected AtmosphericRefraction() {
        // Set up the atmospheric parameters ... with lazy evaluation of the grid (done only if necessary)
        this.atmosphericParams = new AtmosphericComputationParameters();
    }

    /** Apply correction to the intersected point with an atmospheric refraction model.
     * @param satPos satellite position, in <em>body frame</em>
     * @param satLos satellite line of sight, in <em>body frame</em>
     * @param rawIntersection intersection point before refraction correction
     * @param algorithm intersection algorithm
     * @return corrected point with the effect of atmospheric refraction
     * {@link org.orekit.rugged.utils.ExtendedEllipsoid#pointAtAltitude(Vector3D, Vector3D, double)} or see
     * {@link org.orekit.rugged.intersection.IntersectionAlgorithm#refineIntersection(org.orekit.rugged.utils.ExtendedEllipsoid, Vector3D, Vector3D, NormalizedGeodeticPoint)}
     */
    public abstract NormalizedGeodeticPoint applyCorrection(Vector3D satPos, Vector3D satLos, NormalizedGeodeticPoint rawIntersection,
                                            IntersectionAlgorithm algorithm);

    /** Apply correction to the intersected point with an atmospheric refraction model,
     * using a time optimized algorithm.
     * @param lineSensor the line sensor
     * @param sensorPixel the sensor pixel (must be defined)
     * @param satPos satellite position, in <em>body frame</em>
     * @param satLos sensor pixel line of sight, in <em>body frame</em>
     * @param rawIntersection intersection point before refraction correction
     * @param algorithm intersection algorithm
     * @return corrected point with the effect of atmospheric refraction
     * {@link org.orekit.rugged.utils.ExtendedEllipsoid#pointAtAltitude(Vector3D, Vector3D, double)} or see
     * {@link org.orekit.rugged.intersection.IntersectionAlgorithm#refineIntersection(org.orekit.rugged.utils.ExtendedEllipsoid, Vector3D, Vector3D, NormalizedGeodeticPoint)}
     * @since 3.0
     */
    public abstract NormalizedGeodeticPoint applyCorrection(LineSensor lineSensor, SensorPixel sensorPixel,
                                            Vector3D satPos, Vector3D satLos, NormalizedGeodeticPoint rawIntersection,
                                            IntersectionAlgorithm algorithm);

    /** Deactivate computation (needed for the inverse location computation).
     * @since 3.0
     */
    public void deactivateComputation() {
        this.mustBeComputed = false;
    }

    /** Reactivate computation (needed for the inverse location computation).
     * @since 3.0
     */
    public void reactivateComputation() {
        this.mustBeComputed = true;
    }

    /** Tell if the computation must be performed.
     * @return true if computation must be performed; false otherwise
     * @since 3.0
     */
    public boolean mustBeComputed() {
        return mustBeComputed;
    }

    /** Tell if the computation (for direct location) must be optimized.
     * @return true if computation must be optimized; false otherwise
     * @since 3.0
     */
    public boolean isOptimized() {
        return isOptimized;
    }

    /** Configuration of the interpolation grid. This grid is associated to the given sensor,
     * with the given min and max lines.
     * @param sensor line sensor
     * @param minLine min line defined for the inverse location
     * @param maxLine max line defined for the inverse location
     */
    public void configureCorrectionGrid(final LineSensor sensor, final int minLine, final int maxLine) {

        atmosphericParams.configureCorrectionGrid(sensor, minLine, maxLine);
    }

   /** Check if the current atmospheric parameters are the same as the asked ones.
    * @param sensorName the asked sensor name
    * @param minLine the asked min line
    * @param maxLine the asked max line
    * @return true if same context; false otherwise
    */
    public Boolean isSameContext(final String sensorName, final int minLine, final int maxLine) {

        return (Double.compare(atmosphericParams.getMinLineSensor(), minLine) == 0) &&
               (Double.compare(atmosphericParams.getMaxLineSensor(), maxLine) == 0) &&
               (atmosphericParams.getSensorName().compareTo(sensorName) == 0);
    }

    /** Get the computation parameters.
     * @return the AtmosphericComputationParameters
     */
    public AtmosphericComputationParameters getComputationParameters() {
        return atmosphericParams;
    }

    /** Set the grid steps in pixel and line (used to compute inverse location).
     * Overwrite the default values, for time optimization for instance.
     * @param pixelStep pixel step for the inverse location computation
     * @param lineStep line step for the inverse location computation
     */
    public void setGridSteps(final int pixelStep, final int lineStep) {
        atmosphericParams.setGridSteps(pixelStep, lineStep);
    }

    /** Compute the correction functions for pixel and lines.
     * The corrections are computed for pixels and lines, on a regular grid at sensor level.
     * The corrections are based on the difference on grid knots (where direct loc is known with atmosphere refraction)
     * and the sensor pixel found by inverse loc without atmosphere refraction.
     * The bilinear interpolating functions are then computed for pixel and for line.
     * Need to be computed only once for a given sensor with the same minLine and maxLine.
     * @param sensorPixelGridInverseWithout inverse location grid WITHOUT atmospheric refraction
     */
    public void computeGridCorrectionFunctions(final SensorPixel[][] sensorPixelGridInverseWithout) {

        // Compute for a sensor grid, the associated ground grid, WITH atmospheric effect
        // (for interpolations we need a regular grid)
        // ==================================================================================
        final int nbPixelGrid = atmosphericParams.getNbPixelGrid();
        final int nbLineGrid = atmosphericParams.getNbLineGrid();
        final double[] pixelGrid = atmosphericParams.getUgrid();
        final double[] lineGrid = atmosphericParams.getVgrid();
        final double[][] gridDiffPixel = new double[nbPixelGrid][nbLineGrid];
        final double[][] gridDiffLine = new double[nbPixelGrid][nbLineGrid];

        // Compute the difference between grids knots WITH - without atmosphere
        for (int lineIndex = 0; lineIndex < nbLineGrid; lineIndex++) {
            for (int pixelIndex = 0; pixelIndex < nbPixelGrid; pixelIndex++) {

                if (sensorPixelGridInverseWithout[pixelIndex][lineIndex] != null) {
                    final double diffLine = lineGrid[lineIndex] - sensorPixelGridInverseWithout[pixelIndex][lineIndex].getLineNumber();
                    final double diffPixel = pixelGrid[pixelIndex] - sensorPixelGridInverseWithout[pixelIndex][lineIndex].getPixelNumber();
                    gridDiffPixel[pixelIndex][lineIndex] = diffPixel;
                    gridDiffLine[pixelIndex][lineIndex] = diffLine;

                } else {
                    // Impossible to find the point in the given min line and max line
                    throw new RuggedException(RuggedMessages.INVALID_RANGE_FOR_LINES,
                                              atmosphericParams.getMinLineSensor(), atmosphericParams.getMaxLineSensor(), "");
                }
            }
        }
        // Definition of the interpolating function for pixel and for line
        this.bifPixel = new BilinearInterpolatingFunction(pixelGrid, lineGrid, gridDiffPixel);
        this.bifLine = new BilinearInterpolatingFunction(pixelGrid, lineGrid, gridDiffLine);
    }

    public BilinearInterpolatingFunction getBifPixel() {
        return bifPixel;
    }

    public BilinearInterpolatingFunction getBifLine() {
        return bifLine;
    }
}
