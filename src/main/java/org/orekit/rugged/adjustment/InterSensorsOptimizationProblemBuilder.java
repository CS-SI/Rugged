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
package org.orekit.rugged.adjustment;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import org.hipparchus.analysis.differentiation.DerivativeStructure;
import org.hipparchus.linear.Array2DRowRealMatrix;
import org.hipparchus.linear.ArrayRealVector;
import org.hipparchus.linear.RealMatrix;
import org.hipparchus.linear.RealVector;
import org.hipparchus.optim.ConvergenceChecker;
import org.hipparchus.optim.nonlinear.vector.leastsquares.LeastSquaresBuilder;
import org.hipparchus.optim.nonlinear.vector.leastsquares.LeastSquaresProblem;
import org.hipparchus.optim.nonlinear.vector.leastsquares.MultivariateJacobianFunction;
import org.hipparchus.optim.nonlinear.vector.leastsquares.ParameterValidator;
import org.hipparchus.util.Pair;
import org.orekit.errors.OrekitException;
import org.orekit.errors.OrekitExceptionWrapper;
import org.orekit.rugged.api.Rugged;
import org.orekit.rugged.errors.RuggedException;
import org.orekit.rugged.errors.RuggedExceptionWrapper;
import org.orekit.rugged.errors.RuggedMessages;
import org.orekit.rugged.linesensor.LineSensor;
import org.orekit.rugged.linesensor.SensorPixel;
import org.orekit.rugged.adjustment.measurements.Observables;
import org.orekit.rugged.adjustment.measurements.SensorToSensorMapping;
import org.orekit.rugged.utils.SpacecraftToObservedBody;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.ParameterDriver;

/** Constructs the optimization problem for a list of tie points.
 * @author Guylaine Prat
 * @author Lucie Labat Allee
 * @author Jonathan Guinet
 * @author Luc Maisonobe
 * @since 2.0
 */
public class InterSensorsOptimizationProblemBuilder extends OptimizationProblemBuilder {

    /** Key for target. */
    private static final String TARGET = "Target";

    /** Key for weight. */
    private static final String WEIGHT = "Weight";

    /** List of rugged instance to refine.*/
    private Map<String, Rugged> ruggedMap;

    /** Sensor to ground mapping to generate target tab for optimization.*/
    private List<SensorToSensorMapping> sensorToSensorMappings;

    /** Targets and weights of optimization problem. */
    private HashMap<String, double[] > targetAndWeight;

    /** Constructor.
     * @param sensors list of sensors to refine
     * @param measurements set of observables
     * @param ruggedList names of rugged to refine
     * @throws RuggedException an exception is generated if no parameters has been selected for refining
     */
    public InterSensorsOptimizationProblemBuilder(final List<LineSensor> sensors,
                                                  final Observables measurements, final Collection<Rugged> ruggedList)
        throws RuggedException {

        super(sensors, measurements);
        this.ruggedMap = new LinkedHashMap<String, Rugged>();
        for (final Rugged rugged : ruggedList) {
            this.ruggedMap.put(rugged.getName(), rugged);
        }
        this.initMapping();
    }

    /* (non-Javadoc)
     * @see org.orekit.rugged.adjustment.OptimizationProblemBuilder#initMapping()
     */
    @Override
    protected void initMapping() {

        this.sensorToSensorMappings = new ArrayList<SensorToSensorMapping>();

        for (final String ruggedNameA : this.ruggedMap.keySet()) {
            for (final String ruggedNameB : this.ruggedMap.keySet()) {

                for (final LineSensor sensorA : this.getSensors()) {
                    for (final LineSensor sensorB : this.getSensors()) {

                        final String sensorNameA = sensorA.getName();
                        final String sensorNameB = sensorB.getName();
                        final SensorToSensorMapping mapping = this.getMeasurements().getInterMapping(ruggedNameA, sensorNameA, ruggedNameB, sensorNameB);

                        if (mapping != null) {
                            this.sensorToSensorMappings.add(mapping);
                        }
                    }
                }
            }
        }
    }

    /* (non-Javadoc)
     * @see org.orekit.rugged.adjustment.OptimizationProblemBuilder#createTargetAndWeight()
     */
    @Override
    protected void createTargetAndWeight() throws RuggedException {

        try {
            int n = 0;
            for (final SensorToSensorMapping reference : this.sensorToSensorMappings) {
                n += reference.getMapping().size();
            }
            if (n == 0) {
                throw new RuggedException(RuggedMessages.NO_REFERENCE_MAPPINGS);
            }

            n = 2 * n;

            final double[] target = new double[n];
            final double[] weight = new double[n];

            int k = 0;
            for (final SensorToSensorMapping reference : this.sensorToSensorMappings) {

                // Get central body constraint weight
                final double bodyConstraintWeight = reference.getBodyConstraintWeight();

                int i = 0;
                for (Iterator<Map.Entry<SensorPixel, SensorPixel>> gtIt = reference.getMapping().iterator(); gtIt.hasNext(); i++) {

                    if (i == reference.getMapping().size()) break;

                    // Get LOS distance
                    final Double losDistance  = reference.getLosDistance(i);

                    weight[k] = 1.0 - bodyConstraintWeight;
                    target[k++] = losDistance.doubleValue();

                    // Get central body distance (constraint)
                    final Double bodyDistance  = reference.getBodyDistance(i);
                    weight[k] = bodyConstraintWeight;
                    target[k++] = bodyDistance.doubleValue();
                }
            }

            this.targetAndWeight = new HashMap<String, double[]>();
            this.targetAndWeight.put(TARGET, target);
            this.targetAndWeight.put(WEIGHT, weight);

        } catch  (RuggedExceptionWrapper rew) {
            throw rew.getException();
        }
    }

    /* (non-Javadoc)
     * @see org.orekit.rugged.adjustment.OptimizationProblemBuilder#createFunction()
     */
    @Override
    protected MultivariateJacobianFunction createFunction() {

        // model function
        final MultivariateJacobianFunction model = point -> {

            try {
                // set the current parameters values
                int i = 0;
                for (final ParameterDriver driver : this.getDrivers()) {
                    driver.setNormalizedValue(point.getEntry(i++));
                }

                final double[] target = this.targetAndWeight.get(TARGET);

                // compute distance and its partial derivatives
                final RealVector value = new ArrayRealVector(target.length);
                final RealMatrix jacobian = new Array2DRowRealMatrix(target.length, this.getNbParams());

                int l = 0;
                for (final SensorToSensorMapping reference : this.sensorToSensorMappings) {

                    final String ruggedNameA = reference.getRuggedNameA();
                    final String ruggedNameB = reference.getRuggedNameB();
                    final Rugged ruggedA = this.ruggedMap.get(ruggedNameA);
                    if (ruggedA == null) {
                        throw new RuggedException(RuggedMessages.INVALID_RUGGED_NAME);
                    }

                    final Rugged ruggedB = this.ruggedMap.get(ruggedNameB);
                    if (ruggedB == null) {
                        throw new RuggedException(RuggedMessages.INVALID_RUGGED_NAME);
                    }

                    for (final Map.Entry<SensorPixel, SensorPixel> mapping : reference.getMapping()) {

                        final SensorPixel spA = mapping.getKey();
                        final SensorPixel spB = mapping.getValue();

                        final LineSensor lineSensorB = ruggedB.getLineSensor(reference.getSensorNameB());
                        final LineSensor lineSensorA = ruggedA.getLineSensor(reference.getSensorNameA());

                        final AbsoluteDate dateA = lineSensorA.getDate(spA.getLineNumber());
                        final AbsoluteDate dateB = lineSensorB.getDate(spB.getLineNumber());

                        final double pixelA = spA.getPixelNumber();
                        final double pixelB = spB.getPixelNumber();

                        final SpacecraftToObservedBody scToBodyA = ruggedA.getScToBody();

                        final DerivativeStructure[] ilResult =
                                ruggedB.distanceBetweenLOSDerivatives(lineSensorA, dateA, pixelA, scToBodyA,
                                                                      lineSensorB, dateB, pixelB, this.getGenerator());

                        if (ilResult == null) {
                            // TODO GP manque code
                        } else {
                            // extract the value
                            value.setEntry(l, ilResult[0].getValue());
                            value.setEntry(l + 1, ilResult[1].getValue());

                            // extract the Jacobian
                            final int[] orders = new int[this.getNbParams()];
                            int m = 0;

                            for (final ParameterDriver driver : this.getDrivers()) {
                                final double scale = driver.getScale();
                                orders[m] = 1;
                                jacobian.setEntry(l, m, ilResult[0].getPartialDerivative(orders) * scale);
                                jacobian.setEntry(l + 1, m, ilResult[1].getPartialDerivative(orders) * scale);
                                orders[m] = 0;
                                m++;
                            }
                        }

                        l += 2; // pass to the next evaluation
                    }
                }

                // distance result with Jacobian for all reference points
                return new Pair<RealVector, RealMatrix>(value, jacobian);

            } catch (RuggedException re) {
                throw new RuggedExceptionWrapper(re);
            } catch (OrekitException oe) {
                throw new OrekitExceptionWrapper(oe);
            }
        };

        return model;
    }


    /** Least square problem builder.
     * @param maxEvaluations maxIterations and evaluations
     * @param convergenceThreshold parameter convergence threshold
     * @throws RuggedException if sensor is not found
     * @return the least square problem
     */
    @Override
    public final LeastSquaresProblem build(final int maxEvaluations, final double convergenceThreshold) throws RuggedException {

        this.createTargetAndWeight();
        final double[] target = this.targetAndWeight.get(TARGET);
        final double[] start = this.createStartTab();
        final ParameterValidator validator = this.createParameterValidator();
        final ConvergenceChecker<LeastSquaresProblem.Evaluation> checker = this.createChecker(convergenceThreshold);
        final MultivariateJacobianFunction model = this.createFunction();
        return new LeastSquaresBuilder()
                        .lazyEvaluation(false).maxIterations(maxEvaluations)
                        .maxEvaluations(maxEvaluations).weight(null).start(start)
                        .target(target).parameterValidator(validator).checker(checker)
                        .model(model).build();
    }
}
