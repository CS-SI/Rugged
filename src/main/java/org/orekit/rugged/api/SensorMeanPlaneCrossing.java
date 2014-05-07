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
package org.orekit.rugged.api;

import org.apache.commons.math3.analysis.UnivariateFunction;
import org.apache.commons.math3.analysis.solvers.BracketingNthOrderBrentSolver;
import org.apache.commons.math3.analysis.solvers.UnivariateSolver;
import org.apache.commons.math3.exception.NoBracketingException;
import org.apache.commons.math3.exception.TooManyEvaluationsException;
import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.util.FastMath;
import org.orekit.errors.OrekitException;
import org.orekit.errors.OrekitExceptionWrapper;
import org.orekit.frames.Transform;
import org.orekit.rugged.core.SpacecraftToObservedBody;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.Constants;
import org.orekit.utils.PVCoordinates;

/** Class dedicated to when ground point crosses mean sensor plane. */
class SensorMeanPlaneCrossing {

    /** Converter between spacecraft and body. */
    private final SpacecraftToObservedBody scToBody;

    /** Flag for light time correction. */
    private boolean lightTimeCorrection;

    /** Flag for aberration of light correction. */
    private boolean aberrationOfLightCorrection;

    /** Line sensor. */
    private final LineSensor sensor;

    /** Target ground point. */
    private final Vector3D target;

    /** Accuracy to use for finding crossing line number. */
    private final double accuracy;

    /** Evaluated lines. */
    private double[] lineNumbers;

    /** Observed body to inertial transforms, with selected fixes included. */
    private final Transform[] fixedBodyToInert;

    /** Spacecraft to inertial transforms,. */
    private final Transform[] scToInert;

    /** Inertial to spacecraft transforms, with selected fixes included. */
    private final Transform[] fixedInertToSc;

    /** Target direction in spacecraft frame. */
    private Vector3D[] targetDirection;

    /** Index of last evaluation. */
    private int last;

    /** Index of best evaluation. */
    private int best;

    /** Simple constructor.
     * @param sensor sensor to consider
     * @param target target ground point
     * @param scToBody converter between spacecraft and body
     * @param lightTimeCorrection flag for light time correction
     * @param aberrationOfLightCorrection flag for aberration of light correction.
     * @param maxEval maximum number of evaluations
     * @param accuracy accuracy to use for finding crossing line number
     */
    public SensorMeanPlaneCrossing(final LineSensor sensor, final Vector3D target,
                                   final SpacecraftToObservedBody scToBody,
                                   final boolean lightTimeCorrection,
                                   final boolean aberrationOfLightCorrection,
                                   final int maxEval, final double accuracy) {
        this.sensor                      = sensor;
        this.target                      = target;
        this.scToBody                    = scToBody;
        this.lightTimeCorrection         = lightTimeCorrection;
        this.aberrationOfLightCorrection = aberrationOfLightCorrection;
        this.accuracy                    = accuracy;
        this.lineNumbers                 = new double[maxEval];
        this.fixedBodyToInert            = new Transform[maxEval];
        this.scToInert                   = new Transform[maxEval];
        this.fixedInertToSc              = new Transform[maxEval];
        this.targetDirection             = new Vector3D[maxEval];
        this.last                        = -1;
    }

    /** Find mean plane crossing.
     * @param minLine minimum line number
     * @param maxLine maximum line number
     * @return true if a solution has been found in the search interval,
     * false if search interval does not bracket a solution
     * @exception RuggedException if geometry cannot be computed for some line or
     * if the maximum number of evaluations is exceeded
     */
    public boolean find(final double minLine, final double maxLine)
        throws RuggedException {
        try {

            // set up function evaluating to 0.0 at mean plane crossing
            final UnivariateFunction f = new UnivariateFunction() {
                /** {@inheritDoc} */
                @Override
                public double value(final double x) throws OrekitExceptionWrapper {
                    // the target crosses the mean plane if it orthogonal to the normal vector
                    evaluateLine(x);
                    return Vector3D.angle(targetDirection[last], sensor.getMeanPlaneNormal()) - 0.5 * FastMath.PI;
                }
            };

            // find the root
            final UnivariateSolver solver =
                    new BracketingNthOrderBrentSolver(0.0, accuracy, 5);
            final double crossingLine = solver.solve(lineNumbers.length, f, minLine, maxLine);

            // identify the selected solution
            best = -1;
            double min   = Double.POSITIVE_INFINITY;
            for (int i = 0; i <= last; ++i) {
                final double delta = FastMath.abs(lineNumbers[i] - crossingLine);
                if (delta < min) {
                    best = i;
                    min  = delta;
                }

            }

            return true;

        } catch (NoBracketingException nbe) {
            // there are no solutions in the search interval
            return false;
        } catch (TooManyEvaluationsException tmee) {
            throw new RuggedException(tmee);
        } catch (OrekitExceptionWrapper oew) {
            final OrekitException oe = oew.getException();
            throw new RuggedException(oe, oe.getSpecifier(), oe.getParts());
        }
    }

    /** Get the crossing line.
     * @return crossing line
     */
    public double getLine() {
        return lineNumbers[best];
    }

    /** Get the normalized target direction in spacecraft frame at crossing.
     * @return normalized target direction in spacecraft frame at crossing
     */
    public Vector3D getTargetDirection() {
        return targetDirection[best];
    }

    /** Get the derivative of normalized target direction in spacecraft frame at crossing.
     * @return derivative of normalized target direction in spacecraft frame at crossing
     */
    public Vector3D getTargetDirectionDot() {
        final PVCoordinates targetBody  = new PVCoordinates(target, Vector3D.ZERO);
        final PVCoordinates targetInert = fixedBodyToInert[best].transformPVCoordinates(targetBody);
        final PVCoordinates targetSc    = fixedInertToSc[best].transformPVCoordinates(targetInert);
        return new Vector3D(1.0 / targetSc.getPosition().subtract(sensor.getPosition()).getNorm(),
                            targetSc.getVelocity());
    }

    /** Evaluate geometry for a given line number.
     * @param lineNumber current line number
     * @exception OrekitExceptionWrapper if some frame conversion fails
     */
    private void evaluateLine(final double lineNumber) throws OrekitExceptionWrapper {
        try {

            lineNumbers[++last] = lineNumber;

            // compute the transform between spacecraft and observed body
            final AbsoluteDate date        = sensor.getDate(lineNumber);
            final Transform    bodyToInert = scToBody.getInertialToBody(date).getInverse();
            scToInert[last]                = scToBody.getScToInertial(date);
            final Vector3D     refInert    = scToInert[last].transformPosition(sensor.getPosition());

            if (lightTimeCorrection) {
                // apply light time correction
                final Vector3D iT     = bodyToInert.transformPosition(target);
                final double   deltaT = refInert.distance(iT) / Constants.SPEED_OF_LIGHT;
                fixedBodyToInert[last] = bodyToInert.shiftedBy(-deltaT);
            } else {
                // don't apply light time correction
                fixedBodyToInert[last] = bodyToInert;
            }
            final Vector3D targetInert = fixedBodyToInert[last].transformPosition(target);

            final Vector3D  lInert    = targetInert.subtract(refInert).normalize();
            final Transform inertToSc = scToInert[last].getInverse();
            if (aberrationOfLightCorrection) {
                // apply aberration of light correction
                // as the spacecraft velocity is small with respect to speed of light,
                // we use classical velocity addition and not relativistic velocity addition
                final Vector3D spacecraftVelocity =
                        scToInert[last].transformPVCoordinates(PVCoordinates.ZERO).getVelocity();
                final Vector3D  rawLInert = new Vector3D(Constants.SPEED_OF_LIGHT, lInert,
                                                         -1.0, spacecraftVelocity);
                fixedInertToSc[last] = new Transform(date,
                                                     inertToSc,
                                                     new Transform(date,
                                                                   new Rotation(inertToSc.transformVector(lInert),
                                                                                inertToSc.transformVector(rawLInert))));
            } else {
                // don't apply aberration of light correction
                fixedInertToSc[last] = inertToSc;
            }

            // direction of the target point in spacecraft frame
            targetDirection[last] = fixedInertToSc[last].transformVector(lInert);

        } catch (OrekitException oe) {
            throw new OrekitExceptionWrapper(oe);
        }
    }

}
