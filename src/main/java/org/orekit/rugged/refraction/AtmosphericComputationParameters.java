/* Copyright 2013-2019 CS Systèmes d'Information
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

import org.orekit.rugged.errors.RuggedException;
import org.orekit.rugged.errors.RuggedMessages;
import org.orekit.rugged.linesensor.LineSensor;
import org.orekit.rugged.utils.GridCreation;

/**
 * Atmospheric refraction computation parameters.
 * Defines for inverse location a set of parameters in order to be able to perform the computation.
 * @author Guylaine Prat
 * @since 2.1
 */
public class AtmosphericComputationParameters {

    /** Margin for definition of the interpolation grid.
     * To be inside the min line and max line range to avoid problem with inverse location grid computation. */
    private static final int MARGIN_LINE = 10;

    /** Default value for pixel step. */
    private static final int DEFAULT_STEP_PIXEL = 100;
    /** Default value for line step. */
    private static final int DEFAULT_STEP_LINE = 100;

    /** Actual values for pixel step in case default are overwritten. */
    private int pixelStep;
    /** Actual values for line step in case default are overwritten. */
    private int lineStep;

    // Definition of grids for sensor (u = along pixel; v = along line)
    /** Linear grid in pixel. */
    private double[] uGrid;
    /** Linear grid in line. */
    private double[] vGrid;
    /** Size of uGrid = nbPixelGrid. */
    private int nbPixelGrid;
    /** Size of vGrid = nbLineGrid. */
    private int nbLineGrid;

    // Definition of the associated sensor
    /** Current min line. */
    private double minLineSensor = Double.NaN;
    /** Current max line. */
    private double maxLineSensor = Double.NaN;
    /** Current sensor name. */
    private String sensorName = null;

    /**
     * Default constructor.
     */
    public AtmosphericComputationParameters() {
        this.pixelStep = DEFAULT_STEP_PIXEL;
        this.lineStep = DEFAULT_STEP_LINE;
    }

    /** Configuration of the interpolation grid. This grid is associated to the given sensor,
     * with the given min and max lines.
     * @param sensor line sensor
     * @param minLine min line defined for the inverse location
     * @param maxLine max line defined for the inverse location
     */
    public void configureCorrectionGrid(final LineSensor sensor, final int minLine, final int maxLine) {

        // Keep information about the sensor and the required search lines.
        // Needed to test if the grid is initialized with this context.
        this.minLineSensor = minLine;
        this.maxLineSensor = maxLine;
        this.sensorName = sensor.getName();

        // Compute the number of pixels and lines for the grid (round value is sufficient)
        final int sensorNbPxs = sensor.getNbPixels();
        this.nbPixelGrid = sensorNbPxs / this.pixelStep;

        // check the validity of the min and max lines
        if ((maxLine - minLine + 1 - 2 * MARGIN_LINE) < 2 * this.lineStep) {
            final String info = ": (maxLine - minLine + 1 - 2*" + MARGIN_LINE + ") < 2*" + this.lineStep;
            throw new RuggedException(RuggedMessages.INVALID_RANGE_FOR_LINES, minLine, maxLine, info);
        }
        this.nbLineGrid = (maxLine - minLine + 1 - 2 * MARGIN_LINE) / this.lineStep;

        // CHECKSTYLE: stop UnnecessaryParentheses check

        // Compute the linear grids in pixel (u index) and line (v index)
        this.uGrid = GridCreation.createLinearGrid(0, (sensorNbPxs - 1), this.nbPixelGrid);
        this.vGrid = GridCreation.createLinearGrid((minLine + MARGIN_LINE), (maxLine - MARGIN_LINE), this.nbLineGrid);

        // CHECKSTYLE: resume UnnecessaryParentheses check

    }

    /**
     * Set the grid steps in pixel and line (used to compute inverse location).
     * Overwrite the default values, for time optimization if necessary.
     * @param gridPixelStep grid pixel step for the inverse location computation
     * @param gridLineStep grid line step for the inverse location computation
     */
    public void setGridSteps(final int gridPixelStep, final int gridLineStep) {

        if (gridPixelStep <= 0) {
            final String reason = " pixelStep <= 0";
            throw new RuggedException(RuggedMessages.INVALID_STEP, gridPixelStep, reason);
        }
        if (gridLineStep <= 0) {
            final String reason = " lineStep <= 0";
            throw new RuggedException(RuggedMessages.INVALID_STEP, gridLineStep, reason);
        }
        this.pixelStep = gridPixelStep;
        this.lineStep = gridLineStep;
    }

    /**
     * @return the size of pixel grid
     */
    public int getNbPixelGrid() {
        return nbPixelGrid;
    }

    /**
     * @return the size of line grid
     */
    public int getNbLineGrid() {
        return nbLineGrid;
    }

    /**
     * @return the pixel grid
     */
    public double[] getUgrid() {
        return uGrid.clone();
    }

    /**
     * @return the line grid
     */
    public double[] getVgrid() {
        return vGrid.clone();
    }

    /**
     * @return the min line used to compute the current grids
     */
    public double getMinLineSensor() {
        return minLineSensor;
    }

    /**
     * @return the max line used to compute the current grids
     */
    public double getMaxLineSensor() {
        return maxLineSensor;
    }

    /**
     * @return the sensor name used to compute the current grids
     */
    public String getSensorName() {
        return sensorName;
    }
}
