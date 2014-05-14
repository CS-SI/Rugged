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

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.math3.analysis.differentiation.DerivativeStructure;
import org.apache.commons.math3.geometry.euclidean.threed.FieldVector3D;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.util.FastMath;
import org.orekit.errors.OrekitException;
import org.orekit.frames.Transform;
import org.orekit.rugged.core.SpacecraftToObservedBody;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.Constants;
import org.orekit.utils.PVCoordinates;

/** Class dedicated to find when ground point crosses mean sensor plane.
 * <p>
 * This class is used in the first stage of inverse localization.
 * </p>
 * @author Luc Maisonobe
 */
class SensorMeanPlaneCrossing {

    /** Transforms sample from observed body frame to inertial frame. */
    private final List<Transform> bodyToInertial;

    /** Transforms sample from spacecraft frame to inertial frame. */
    private final List<Transform> scToInertial;

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

    /** Maximum number of evaluations. */
    private final int maxEval;

    /** Accuracy to use for finding crossing line number. */
    private final double accuracy;

    /** Simple constructor.
     * @param sensor sensor to consider
     * @param scToBody converter between spacecraft and body
     * @param minLine minimum line number
     * @param maxLine maximum line number
     * @param lightTimeCorrection flag for light time correction
     * @param aberrationOfLightCorrection flag for aberration of light correction.
     * @param maxEval maximum number of evaluations
     * @param accuracy accuracy to use for finding crossing line number
     * @exception RuggedException if some frame conversion fails
     */
    public SensorMeanPlaneCrossing(final LineSensor sensor,
                                   final SpacecraftToObservedBody scToBody,
                                   final int minLine, final int maxLine,
                                   final boolean lightTimeCorrection,
                                   final boolean aberrationOfLightCorrection,
                                   final int maxEval, final double accuracy)
        throws RuggedException {
        try {

            this.sensor                      = sensor;
            this.minLine                     = minLine;
            this.maxLine                     = maxLine;
            this.lightTimeCorrection         = lightTimeCorrection;
            this.aberrationOfLightCorrection = aberrationOfLightCorrection;
            this.maxEval                     = maxEval;
            this.accuracy                    = accuracy;

            bodyToInertial = new ArrayList<Transform>(maxLine - minLine + 1);
            scToInertial   = new ArrayList<Transform>(maxLine - minLine + 1);
            for (int line = minLine; line <= maxLine; ++line) {
                final AbsoluteDate date = sensor.getDate(line);
                bodyToInertial.add(scToBody.getInertialToBody(date).getInverse());
                scToInertial.add(scToBody.getScToInertial(date));
            }

        } catch (OrekitException oe) {
            throw new RuggedException(oe, oe.getSpecifier(), oe.getParts());
        }
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

    /** Container for mean plane crossing result. */
    public static class CrossingResult {

        /** Crossing line. */
        private final double crossingLine;

        /** Target direction in spacecraft frame. */
        private final FieldVector3D<DerivativeStructure> targetDirection;

        /** Simple constructor.
         * @param crossingLine crossing line
         * @param targetDirection target direction in spacecraft frame
         */
        private CrossingResult(final double crossingLine,
                               final FieldVector3D<DerivativeStructure> targetDirection) {
            this.crossingLine    = crossingLine;
            this.targetDirection = targetDirection;
        }

        /** Get the crossing line.
         * @return crossing line
         */
        public double getLine() {
            return crossingLine;
        }

        /** Get the normalized target direction in spacecraft frame at crossing.
         * @return normalized target direction in spacecraft frame at crossing, with its first derivative
         * with respect to line number
         */
        public FieldVector3D<DerivativeStructure> getTargetDirection() {
            return targetDirection;
        }

    }

    /** Find mean plane crossing.
     * @param target target ground point
     * @return line number and target direction at mean plane crossing,
     * or null if search interval does not bracket a solution
     * @exception RuggedException if geometry cannot be computed for some line or
     * if the maximum number of evaluations is exceeded
     */
    public CrossingResult find(final Vector3D target)
        throws RuggedException {
        try {

            final PVCoordinates targetPV = new PVCoordinates(target, Vector3D.ZERO);

            // we don't use an Apache Commons Math solver here because we are more
            // interested in reducing the number of evaluations than being accurate,
            // as we know the solution is improved in the second stage of inverse localization.
            // We expect two or three evaluations only. Each new evaluation shows up quickly in
            // the performances as it involves frames conversions
            final int midIndex    = bodyToInertial.size() / 2;
            double  crossingLine  = minLine + midIndex;
            Transform bodyToInert = bodyToInertial.get(midIndex);
            Transform scToInert   = scToInertial.get(midIndex);
            boolean atMin         = false;
            boolean atMax         = false;
            for (int i = 0; i < maxEval; ++i) {

                final FieldVector3D<DerivativeStructure> targetDirection =
                        evaluateLine(crossingLine, targetPV, bodyToInert, scToInert);
                final DerivativeStructure beta = FieldVector3D.angle(targetDirection, sensor.getMeanPlaneNormal());

                final double deltaL = (0.5 * FastMath.PI - beta.getValue()) / beta.getPartialDerivative(1);
                if (FastMath.abs(deltaL) <= accuracy) {
                    // return immediately, without doing any additional evaluation!
                    return new CrossingResult(crossingLine, targetDirection);
                }
                crossingLine += deltaL;

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

                final int inf = FastMath.max(0, FastMath.min(scToInertial.size() - 2, (int) FastMath.floor(crossingLine)));
                final AbsoluteDate date = sensor.getDate(crossingLine);
                    bodyToInert = Transform.interpolate(date, false, false, bodyToInertial.subList(inf, inf + 2));
                    scToInert   = Transform.interpolate(date, false, false, scToInertial.subList(inf, inf + 2));
            }

            return null;

        } catch (OrekitException oe) {
            throw new RuggedException(oe, oe.getSpecifier(), oe.getParts());
        }
    }

    /** Evaluate geometry for a given line number.
     * @param lineNumber current line number
     * @param targetPV target ground point
     * @param bodyToInert transform from observed body to inertial frame, for middle line
     * @param scToInert transform from inertial frame to spacecraft frame, for middle line
     * @return target direction in spacecraft frame, with its first derivative
     * with respect to line number
     */
    private FieldVector3D<DerivativeStructure> evaluateLine(final double lineNumber, final PVCoordinates targetPV,
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
                                             -1.0, Vector3D.crossProduct(inertToSc.getRotationRate(),
                                                                         dir));

        // combine vector value and derivative
        final double rate = sensor.getRate(lineNumber);
        return new FieldVector3D<DerivativeStructure>(new DerivativeStructure(1, 1, dir.getX(), dirDot.getX() / rate),
                new DerivativeStructure(1, 1, dir.getY(), dirDot.getY() / rate),
                new DerivativeStructure(1, 1, dir.getZ(), dirDot.getZ() / rate));

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
