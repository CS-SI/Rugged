/* Copyright 2013-2025 CS GROUP
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
package org.orekit.rugged.linesensor;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Stream;

import org.hipparchus.analysis.UnivariateFunction;
import org.hipparchus.analysis.solvers.BracketingNthOrderBrentSolver;
import org.hipparchus.analysis.solvers.UnivariateSolver;
import org.hipparchus.exception.MathIllegalArgumentException;
import org.hipparchus.exception.MathIllegalStateException;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.linear.Array2DRowRealMatrix;
import org.hipparchus.linear.ArrayRealVector;
import org.hipparchus.linear.DecompositionSolver;
import org.hipparchus.linear.MatrixUtils;
import org.hipparchus.linear.QRDecomposition;
import org.hipparchus.linear.RealMatrix;
import org.hipparchus.linear.RealVector;
import org.hipparchus.linear.SingularValueDecomposition;
import org.hipparchus.util.FastMath;
import org.hipparchus.util.Precision;
import org.orekit.frames.Transform;
import org.orekit.rugged.errors.RuggedException;
import org.orekit.rugged.errors.RuggedInternalError;
import org.orekit.rugged.utils.SpacecraftToObservedBody;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.Constants;
import org.orekit.utils.PVCoordinates;

/** Class dedicated to find when ground point crosses mean sensor plane.
 * <p>
 * This class is used in the first stage of inverse location.
 * </p>
 * @author Luc Maisonobe
 */
public class SensorMeanPlaneCrossing {

    /** Number of cached results for guessing the line number. */
    private static final int CACHED_RESULTS = 6;

    /** Converter between spacecraft and body. */
    private final SpacecraftToObservedBody scToBody;

    /** Middle line. */
    private final double midLine;

    /** Body to inertial transform for middle line. */
    private final Transform midBodyToInert;

    /** Spacecraft to inertial transform for middle line. */
    private final Transform midScToInert;

    /** Minimum line number in the search interval. */
    private final int minLine;

    /** Maximum line number in the search interval. */
    private final int maxLine;

    /** Flag for light time correction. */
    private final boolean lightTimeCorrection;

    /** Flag for aberration of light correction. */
    private final boolean aberrationOfLightCorrection;

    /** Line sensor. */
    private final LineSensor sensor;

    /** Sensor mean plane normal. */
    private final Vector3D meanPlaneNormal;

    /** Maximum number of evaluations. */
    private final int maxEval;

    /** Accuracy to use for finding crossing line number. */
    private final double accuracy;

    /** Cached results. */
    private final List<CrossingResult> cachedResults;

    /** Simple constructor.
     * @param sensor sensor to consider
     * @param scToBody converter between spacecraft and body
     * @param minLine minimum line number
     * @param maxLine maximum line number
     * @param lightTimeCorrection flag for light time correction
     * @param aberrationOfLightCorrection flag for aberration of light correction.
     * @param maxEval maximum number of evaluations
     * @param accuracy accuracy to use for finding crossing line number
     */
    public SensorMeanPlaneCrossing(final LineSensor sensor,
                                   final SpacecraftToObservedBody scToBody,
                                   final int minLine, final int maxLine,
                                   final boolean lightTimeCorrection,
                                   final boolean aberrationOfLightCorrection,
                                   final int maxEval, final double accuracy) {
        this(sensor, scToBody, minLine, maxLine, lightTimeCorrection, aberrationOfLightCorrection,
             maxEval, accuracy, computeMeanPlaneNormal(sensor, minLine, maxLine),
             Stream.<CrossingResult>empty());
    }

    /** Simple constructor.
     * @param sensor sensor to consider
     * @param scToBody converter between spacecraft and body
     * @param minLine minimum line number
     * @param maxLine maximum line number
     * @param lightTimeCorrection flag for light time correction
     * @param aberrationOfLightCorrection flag for aberration of light correction.
     * @param maxEval maximum number of evaluations
     * @param accuracy accuracy to use for finding crossing line number
     * @param meanPlaneNormal mean plane normal
     * @param cachedResults cached results
     */
    public SensorMeanPlaneCrossing(final LineSensor sensor,
                                   final SpacecraftToObservedBody scToBody,
                                   final int minLine, final int maxLine,
                                   final boolean lightTimeCorrection,
                                   final boolean aberrationOfLightCorrection,
                                   final int maxEval, final double accuracy,
                                   final Vector3D meanPlaneNormal,
                                   final Stream<CrossingResult> cachedResults) {

        this.sensor                      = sensor;
        this.minLine                     = minLine;
        this.maxLine                     = maxLine;
        this.lightTimeCorrection         = lightTimeCorrection;
        this.aberrationOfLightCorrection = aberrationOfLightCorrection;
        this.maxEval                     = maxEval;
        this.accuracy                    = accuracy;
        this.scToBody                    = scToBody;

        this.midLine                     = 0.5 * (minLine + maxLine);
        final AbsoluteDate midDate       = sensor.getDate(midLine);
        this.midBodyToInert              = scToBody.getBodyToInertial(midDate);
        this.midScToInert                = scToBody.getScToInertial(midDate);

        this.meanPlaneNormal             = meanPlaneNormal;

        this.cachedResults               = new ArrayList<>(CACHED_RESULTS);
        cachedResults.forEach(crossingResult -> {
            if (crossingResult != null && this.cachedResults.size() < CACHED_RESULTS) {
                this.cachedResults.add(crossingResult);
            }
        });

    }

    /** Compute the plane containing origin that best fits viewing directions point cloud.
     * @param sensor line sensor
     * @param minLine minimum line number
     * @param maxLine maximum line number
     * <p>
     * The normal is oriented such that traversing pixels in increasing indices
     * order corresponds to trigonometric order (i.e. counterclockwise).
     * </p>
     * @return normal of the mean plane
     */
    private static Vector3D computeMeanPlaneNormal(final LineSensor sensor, final int minLine, final int maxLine) {

        final AbsoluteDate midDate = sensor.getDate(0.5 * (minLine + maxLine));

        // build a centered data matrix
        // (for each viewing direction, we add both the direction and its
        //  opposite, thus ensuring the plane will contain origin)
        final RealMatrix matrix = MatrixUtils.createRealMatrix(3, 2 * sensor.getNbPixels());
        for (int i = 0; i < sensor.getNbPixels(); ++i) {
            final Vector3D l = sensor.getLOS(midDate, i);
            matrix.setEntry(0, 2 * i,      l.getX());
            matrix.setEntry(1, 2 * i,      l.getY());
            matrix.setEntry(2, 2 * i,      l.getZ());
            matrix.setEntry(0, 2 * i + 1, -l.getX());
            matrix.setEntry(1, 2 * i + 1, -l.getY());
            matrix.setEntry(2, 2 * i + 1, -l.getZ());
        }

        // compute Singular Value Decomposition
        final SingularValueDecomposition svd = new SingularValueDecomposition(matrix);

        // extract the left singular vector corresponding to least singular value
        // (i.e. last vector since Hipparchus returns the values
        //  in non-increasing order)
        final Vector3D singularVector = new Vector3D(svd.getU().getColumn(2)).normalize();

        // check rotation order
        final Vector3D first = sensor.getLOS(midDate, 0);
        final Vector3D last  = sensor.getLOS(midDate, sensor.getNbPixels() - 1);
        if (Vector3D.dotProduct(singularVector, Vector3D.crossProduct(first, last)) >= 0) {
            return singularVector;
        } else {
            return singularVector.negate();
        }

    }

    /** Get the underlying sensor.
     * @return underlying sensor
     */
    public LineSensor getSensor() {
        return sensor;
    }

    /** Get converter between spacecraft and body.
     * @return converter between spacecraft and body
     */
    public SpacecraftToObservedBody getScToBody() {
        return scToBody;
    }

    /** Get the minimum line number in the search interval.
     * @return minimum line number in the search interval
     */
    public int getMinLine() {
        return minLine;
    }

    /** Get the maximum line number in the search interval.
     * @return maximum line number in the search interval
     */
    public int getMaxLine() {
        return maxLine;
    }

    /** Get the maximum number of evaluations.
     * @return maximum number of evaluations
     */
    public int getMaxEval() {
        return maxEval;
    }

    /** Get the accuracy to use for finding crossing line number.
     * @return accuracy to use for finding crossing line number
     */
    public double getAccuracy() {
        return accuracy;
    }

    /** Get the mean plane normal.
     * <p>
     * The normal is oriented such traversing pixels in increasing indices
     * order corresponds is consistent with trigonometric order (i.e.
     * counterclockwise).
     * </p>
     * @return mean plane normal
     */
    public Vector3D getMeanPlaneNormal() {
        return meanPlaneNormal;
    }

    /** Get cached previous results.
     * @return cached previous results
     */
    public Stream<CrossingResult> getCachedResults() {
        return cachedResults.stream();
    }

    /** Container for mean plane crossing result. */
    public static class CrossingResult {

        /** Crossing date. */
        private final AbsoluteDate crossingDate;

        /** Crossing line. */
        private final double crossingLine;

        /** Target ground point. */
        private final Vector3D target;

        /** Target direction in spacecraft frame. */
        private final Vector3D targetDirection;

        /** Derivative of the target direction in spacecraft frame with respect to line number. */
        private final Vector3D targetDirectionDerivative;

        /** Simple constructor.
         * @param crossingDate crossing date
         * @param crossingLine crossing line
         * @param target target ground point
         * @param targetDirection target direction in spacecraft frame
         * @param targetDirectionDerivative derivative of the target direction in spacecraft frame with respect to line number
         */
        public CrossingResult(final AbsoluteDate crossingDate, final double crossingLine,
                              final Vector3D target,
                              final Vector3D targetDirection,
                              final Vector3D targetDirectionDerivative) {
            this.crossingDate              = crossingDate;
            this.crossingLine              = crossingLine;
            this.target                    = target;
            this.targetDirection           = targetDirection;
            this.targetDirectionDerivative = targetDirectionDerivative;
        }

        /** Get the crossing date.
         * @return crossing date
         */
        public AbsoluteDate getDate() {
            return crossingDate;
        }

        /** Get the crossing line.
         * @return crossing line
         */
        public double getLine() {
            return crossingLine;
        }

        /** Get the target ground point.
         * @return target ground point
         */
        public Vector3D getTarget() {
            return target;
        }

        /** Get the normalized target direction in spacecraft frame at crossing.
         * @return normalized target direction in spacecraft frame at crossing
         * with respect to line number
         */
        public Vector3D getTargetDirection() {
            return targetDirection;
        }

        /** Get the derivative of the normalized target direction in spacecraft frame at crossing.
         * @return derivative of the normalized target direction in spacecraft frame at crossing
         * with respect to line number
         * @since 2.0
         */
        public Vector3D getTargetDirectionDerivative() {
            return targetDirectionDerivative;
        }

    }

    /** Find mean plane crossing.
     * @param target target ground point
     * @return line number and target direction at mean plane crossing,
     * or null if search interval does not bracket a solution
     */
    public CrossingResult find(final Vector3D target) {

        double crossingLine     = midLine;
        Transform bodyToInert   = midBodyToInert;
        Transform scToInert     = midScToInert;

        // count the number of available results
        if (cachedResults.size() >= 4) {
            // we already have computed at lest 4 values, we attempt to build a linear
            // model to guess a better start line
            final double guessedCrossingLine = guessStartLine(target);
            if (guessedCrossingLine >= minLine && guessedCrossingLine <= maxLine) {
                crossingLine = guessedCrossingLine;
                final AbsoluteDate date = sensor.getDate(crossingLine);
                bodyToInert = scToBody.getBodyToInertial(date);
                scToInert   = scToBody.getScToInertial(date);
            }
        }

        final PVCoordinates targetPV = new PVCoordinates(target, Vector3D.ZERO);

        // we don't use an Hipparchus solver here because we are more
        // interested in reducing the number of evaluations than being accurate,
        // as we know the solution is improved in the second stage of inverse location.
        // We expect two or three evaluations only. Each new evaluation shows up quickly in
        // the performances as it involves frames conversions
        final double[]  crossingLineHistory = new double[maxEval];
        final double[]  betaHistory         = new double[maxEval];
        final double[]  betaDerHistory      = new double[maxEval];
        boolean         atMin               = false;
        boolean         atMax               = false;
        for (int i = 0; i < maxEval; ++i) {

            crossingLineHistory[i] = crossingLine;
            final Vector3D[] targetDirection =
                    evaluateLine(crossingLine, targetPV, bodyToInert, scToInert);
            betaHistory[i] = FastMath.acos(Vector3D.dotProduct(targetDirection[0], meanPlaneNormal));
            final double s = Vector3D.dotProduct(targetDirection[1], meanPlaneNormal);
            betaDerHistory[i] = -s / FastMath.sqrt(1 - s * s);

            final double deltaL;
            if (i == 0) {
                // simple Newton iteration
                deltaL        = (0.5 * FastMath.PI - betaHistory[i]) / betaDerHistory[i];
                crossingLine += deltaL;
            } else {
                // inverse cubic iteration
                final double a0    = betaHistory[i - 1] - 0.5 * FastMath.PI;
                final double l0    = crossingLineHistory[i - 1];
                final double d0    = betaDerHistory[i - 1];
                final double a1    = betaHistory[i]     - 0.5 * FastMath.PI;
                final double l1    = crossingLineHistory[i];
                final double d1    = betaDerHistory[i];
                final double a1Ma0 = a1 - a0;
                crossingLine = ((l0 * (a1 - 3 * a0) - a0 * a1Ma0 / d0) * a1 * a1 +
                                (l1 * (3 * a1 - a0) - a1 * a1Ma0 / d1) * a0 * a0
                               ) / (a1Ma0 * a1Ma0 * a1Ma0);
                deltaL = crossingLine - l1;
            }
            if (FastMath.abs(deltaL) <= accuracy) {
                // return immediately, without doing any additional evaluation!
                final CrossingResult crossingResult =
                    new CrossingResult(sensor.getDate(crossingLine), crossingLine, target,
                                       targetDirection[0], targetDirection[1]);
                boolean isNew = true;
                for (final CrossingResult existing : cachedResults) {
                    isNew = isNew && FastMath.abs(crossingLine - existing.crossingLine) > accuracy;
                }
                if (isNew) {
                    // this result is different from the existing ones,
                    // it brings new sampling data to the cache
                    if (cachedResults.size() >= CACHED_RESULTS) {
                        cachedResults.remove(cachedResults.size() - 1);
                    }
                    cachedResults.add(0, crossingResult);
                }
                return crossingResult;
            }
            for (int j = 0; j < i; ++j) {
                if (FastMath.abs(crossingLine - crossingLineHistory[j]) <= 1.0) {
                    // rare case: we are stuck in a loop!
                    // switch to a more robust (but slower) algorithm in this case
                    final CrossingResult slowResult = slowFind(targetPV, crossingLine);
                    if (slowResult == null) {
                        return null;
                    }
                    if (cachedResults.size() >= CACHED_RESULTS) {
                        cachedResults.remove(cachedResults.size() - 1);
                    }
                    cachedResults.add(0, slowResult);
                    return cachedResults.get(0);
                }
            }

            if (crossingLine < minLine) {
                if (atMin) {
                    // we were already trying at minLine and we need to go below that
                    // give up as the solution is out of search interval
                    return null;
                }
                atMin        = true;
                crossingLine = minLine;
            } else if (crossingLine > maxLine) {
                if (atMax) {
                    // we were already trying at maxLine and we need to go above that
                    // give up as the solution is out of search interval
                    return null;
                }
                atMax        = true;
                crossingLine = maxLine;
            } else {
                // the next evaluation will be a regular point
                atMin = false;
                atMax = false;
            }

            final AbsoluteDate date = sensor.getDate(crossingLine);
            bodyToInert = scToBody.getBodyToInertial(date);
            scToInert   = scToBody.getScToInertial(date);
        }

        return null;

    }

    /** Guess a start line using the last four results.
     * @param target target ground point
     * @return guessed start line
     */
    private double guessStartLine(final Vector3D target) {
        try {

            // assume a linear model of the form: l = ax + by + cz + d
            final int        n = cachedResults.size();
            final RealMatrix m = new Array2DRowRealMatrix(n, 4);
            final RealVector v = new ArrayRealVector(n);
            for (int i = 0; i < n; ++i) {
                final CrossingResult crossingResult = cachedResults.get(i);
                m.setEntry(i, 0, crossingResult.getTarget().getX());
                m.setEntry(i, 1, crossingResult.getTarget().getY());
                m.setEntry(i, 2, crossingResult.getTarget().getZ());
                m.setEntry(i, 3, 1.0);
                v.setEntry(i, crossingResult.getLine());
            }

            final DecompositionSolver solver = new QRDecomposition(m, Precision.SAFE_MIN).getSolver();
            final RealVector coefficients = solver.solve(v);

            // apply the linear model
            return target.getX() * coefficients.getEntry(0) +
                   target.getY() * coefficients.getEntry(1) +
                   target.getZ() * coefficients.getEntry(2) +
                   coefficients.getEntry(3);

        } catch (MathIllegalStateException sme) {
            // the points are not independent
            return Double.NaN;
        }
    }

    /** Find mean plane crossing using a slow but robust method.
     * @param targetPV target ground point
     * @param initialGuess initial guess for the crossing line
     * @return line number and target direction at mean plane crossing,
     * or null if search interval does not bracket a solution
     */
    private CrossingResult slowFind(final PVCoordinates targetPV, final double initialGuess) {

        try {
            // safety check
            final double startValue;
            if (initialGuess < minLine || initialGuess > maxLine) {
                startValue = midLine;
            } else {
                startValue = initialGuess;
            }

            final UnivariateSolver solver = new BracketingNthOrderBrentSolver(accuracy, 5);
            final double crossingLine = solver.solve(maxEval, new UnivariateFunction() {
                /** {@inheritDoc} */
                @Override
                public double value(final double x) {
                    try {
                        final AbsoluteDate date = sensor.getDate(x);
                        final Vector3D[] targetDirection =
                                evaluateLine(x, targetPV, scToBody.getBodyToInertial(date), scToBody.getScToInertial(date));
                        return 0.5 * FastMath.PI - FastMath.acos(Vector3D.dotProduct(targetDirection[0], meanPlaneNormal));
                    } catch (RuggedException re) {
                        throw new RuggedInternalError(re);
                    }
                }
            }, minLine, maxLine, startValue);

            final AbsoluteDate date = sensor.getDate(crossingLine);
            final Vector3D[] targetDirection =
                    evaluateLine(crossingLine, targetPV, scToBody.getBodyToInertial(date), scToBody.getScToInertial(date));
            return new CrossingResult(sensor.getDate(crossingLine), crossingLine, targetPV.getPosition(),
                                      targetDirection[0], targetDirection[1]);

        } catch (MathIllegalArgumentException nbe) {
            return null;
        }
    }

    /** Evaluate geometry for a given line number.
     * @param lineNumber current line number
     * @param targetPV target ground point
     * @param bodyToInert transform from observed body to inertial frame, for current line
     * @param scToInert transform from inertial frame to spacecraft frame, for current line
     * @return target direction in spacecraft frame, with its first derivative
     * with respect to line number
     */
    private Vector3D[] evaluateLine(final double lineNumber, final PVCoordinates targetPV,
                                    final Transform bodyToInert, final Transform scToInert) {

        // compute the transform between spacecraft and observed body
        final PVCoordinates refInert =
                scToInert.transformPVCoordinates(new PVCoordinates(sensor.getPosition(), Vector3D.ZERO));

        final PVCoordinates targetInert;
        if (lightTimeCorrection) {
            // apply light time correction
            final Vector3D iT     = bodyToInert.transformPosition(targetPV.getPosition());
            final double   deltaT = refInert.getPosition().distance(iT) / Constants.SPEED_OF_LIGHT;
            targetInert           = bodyToInert.shiftedBy(-deltaT).transformPVCoordinates(targetPV);
        } else {
            // don't apply light time correction
            targetInert = bodyToInert.transformPVCoordinates(targetPV);
        }

        final PVCoordinates lInert    = new PVCoordinates(refInert, targetInert);
        final Transform     inertToSc = scToInert.getInverse();
        final Vector3D obsLInert;
        final Vector3D obsLInertDot;
        if (aberrationOfLightCorrection) {

            // apply aberration of light correction
            // as the spacecraft velocity is small with respect to speed of light,
            // we use classical velocity addition and not relativistic velocity addition
            // we have: c * lInert + vsat = k * obsLInert
            final PVCoordinates spacecraftPV = scToInert.transformPVCoordinates(PVCoordinates.ZERO);
            final Vector3D  l        = lInert.getPosition().normalize();
            final Vector3D  lDot     = normalizedDot(lInert.getPosition(), lInert.getVelocity());
            final Vector3D  kObs     = new Vector3D(Constants.SPEED_OF_LIGHT, l, +1.0, spacecraftPV.getVelocity());
            obsLInert = kObs.normalize();

            // the following derivative is computed under the assumption the spacecraft velocity
            // is constant in inertial frame ... It is obviously not true, but as this velocity
            // is very small with respect to speed of light, the error is expected to remain small
            obsLInertDot = normalizedDot(kObs, new Vector3D(Constants.SPEED_OF_LIGHT, lDot));

        } else {

            // don't apply aberration of light correction
            obsLInert    = lInert.getPosition().normalize();
            obsLInertDot = normalizedDot(lInert.getPosition(), lInert.getVelocity());

        }
        final Vector3D dir    = inertToSc.transformVector(obsLInert);
        final Vector3D dirDot = new Vector3D(+1.0, inertToSc.transformVector(obsLInertDot),
                                             -1.0, Vector3D.crossProduct(inertToSc.getRotationRate(), dir));

        // combine vector value and derivative
        final double rate = sensor.getRate(lineNumber);
        return new Vector3D[] {
            dir, dirDot.scalarMultiply(1.0 / rate)
        };

    }

    /** Compute the derivative of normalized vector.
     * @param u base vector
     * @param uDot derivative of the base vector
     * @return vDot, where v = u / ||u||
     */
    private Vector3D normalizedDot(final Vector3D u, final Vector3D uDot) {
        final double n = u.getNorm();
        return new Vector3D(1.0 / n, uDot, -Vector3D.dotProduct(u, uDot) / (n * n * n), u);
    }

}
