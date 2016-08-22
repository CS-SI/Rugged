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
package org.orekit.rugged.linesensor;

import org.hipparchus.analysis.UnivariateFunction;
import org.hipparchus.analysis.solvers.BracketingNthOrderBrentSolver;
import org.hipparchus.analysis.solvers.UnivariateSolver;
import org.hipparchus.exception.MathIllegalArgumentException;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.util.FastMath;
import org.orekit.rugged.errors.RuggedException;
import org.orekit.rugged.errors.RuggedExceptionWrapper;
import org.orekit.time.AbsoluteDate;

/** Class devoted to locate where ground point crosses a sensor line.
 * <p>
 * This class is used in the first stage of inverse location.
 * </p>
 * @author Luc Maisonobe
 */
public class SensorPixelCrossing {

    /** Margin before and after end pixels, in order to avoid search failures near boundaries. */
    private static final double MARGIN = 10.0;

    /** Line sensor. */
    private final LineSensor sensor;

    /** Cross direction in spacecraft frame. */
    private final Vector3D cross;

    /** Maximum number of evaluations. */
    private final int maxEval;

    /** Accuracy to use for finding crossing line number. */
    private final double accuracy;

    /** Simple constructor.
     * @param sensor sensor to consider
     * @param meanNormal mean plane normal of the line sensor
     * @param targetDirection target direction in spacecraft frame
     * @param maxEval maximum number of evaluations
     * @param accuracy accuracy to use for finding crossing line number
     */
    public SensorPixelCrossing(final LineSensor sensor, final Vector3D meanNormal,
                               final Vector3D targetDirection,
                               final int maxEval, final double accuracy) {
        this.sensor   = sensor;
        this.cross    = Vector3D.crossProduct(meanNormal, targetDirection).normalize();
        this.maxEval  = maxEval;
        this.accuracy = accuracy;
    }

    /** Locate pixel along sensor line.
     * @param date current date
     * @return pixel location ({@code Double.NaN} if the first and last
     * pixels of the line do not bracket a location)
     * @exception RuggedException if the maximum number of evaluations is exceeded
     */
    public double locatePixel(final AbsoluteDate date) throws RuggedException {
        try {

            // set up function evaluating to 0.0 where target matches pixel
            final UnivariateFunction f = new UnivariateFunction() {
                /** {@inheritDoc} */
                @Override
                public double value(final double x) throws RuggedExceptionWrapper {
                    try {
                        return Vector3D.angle(cross, getLOS(date, x)) - 0.5 * FastMath.PI;
                    } catch (RuggedException re) {
                        throw new RuggedExceptionWrapper(re);
                    }
                }
            };

            // find the root
            final UnivariateSolver solver =
                    new BracketingNthOrderBrentSolver(0.0, accuracy, 5);
            return solver.solve(maxEval, f, -MARGIN, sensor.getNbPixels() - 1 + MARGIN);

        } catch (MathIllegalArgumentException nbe) {
            // there are no solutions in the search interval
            return Double.NaN;
        } catch (RuggedExceptionWrapper rew) {
            throw rew.getException();
        }
    }

    /** Interpolate sensor pixels at some pixel index.
     * @param date current date
     * @param x pixel index
     * @return interpolated direction for specified index
     * @exception RuggedException if date cannot be handled
     */
    private Vector3D getLOS(final AbsoluteDate date, final double x)
        throws RuggedException {

        // find surrounding pixels
        final int iInf = FastMath.max(0, FastMath.min(sensor.getNbPixels() - 2, (int) FastMath.floor(x)));
        final int iSup = iInf + 1;

        // interpolate
        return new Vector3D(iSup - x, sensor.getLOS(date, iInf),
                            x - iInf, sensor.getLOS(date, iSup)).normalize();

    }

}
