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
package org.orekit.rugged.adjustment;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.hipparchus.analysis.differentiation.Gradient;
import org.hipparchus.linear.Array2DRowRealMatrix;
import org.hipparchus.linear.ArrayRealVector;
import org.hipparchus.linear.RealMatrix;
import org.hipparchus.linear.RealVector;
import org.hipparchus.optim.ConvergenceChecker;
import org.hipparchus.optim.nonlinear.vector.leastsquares.LeastSquaresBuilder;
import org.hipparchus.optim.nonlinear.vector.leastsquares.LeastSquaresProblem;
import org.hipparchus.optim.nonlinear.vector.leastsquares.MultivariateJacobianFunction;
import org.hipparchus.optim.nonlinear.vector.leastsquares.ParameterValidator;
import org.hipparchus.util.FastMath;
import org.hipparchus.util.Pair;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.rugged.adjustment.measurements.Observables;
import org.orekit.rugged.adjustment.measurements.SensorToGroundMapping;
import org.orekit.rugged.api.Rugged;
import org.orekit.rugged.errors.RuggedException;
import org.orekit.rugged.errors.RuggedMessages;
import org.orekit.rugged.linesensor.LineSensor;
import org.orekit.rugged.linesensor.SensorPixel;
import org.orekit.utils.ParameterDriver;

/** Ground optimization problem builder.
 * builds the optimization problem relying on ground measurements.
 * @author Guylaine Prat
 * @author Lucie Labat Allee
 * @author Jonathan Guinet
 * @author Luc Maisonobe
 * @since 2.0
 */
public class GroundOptimizationProblemBuilder extends OptimizationProblemBuilder {

    /** Key for target. */
    private static final String TARGET = "Target";

    /** Key for weight. */
    private static final String WEIGHT = "Weight";

    /** Rugged instance to refine.*/
    private final Rugged rugged;

    /** Sensor to ground mapping to generate target tab for optimization.*/
    private List<SensorToGroundMapping> sensorToGroundMappings;

    /** Minimum line for inverse location estimation.*/
    private int minLine;

    /** Maximum line for inverse location estimation.*/
    private int maxLine;

    /** Target and weight (the solution of the optimization problem).*/
    private HashMap<String, double[] > targetAndWeight;


    /** Build a new instance of the optimization problem.
     * @param sensors list of sensors to refine
     * @param measurements set of observables
     * @param rugged name of rugged to refine
     */
    public GroundOptimizationProblemBuilder(final List<LineSensor> sensors,
                                            final Observables measurements, final Rugged rugged) {

        super(sensors, measurements);
        this.rugged = rugged;
        this.initMapping();
    }

    /** {@inheritDoc} */
    @Override
    protected void initMapping() {

        final String ruggedName = rugged.getName();
        this.sensorToGroundMappings = new ArrayList<>();
        for (final LineSensor lineSensor : this.getSensors()) {
            final SensorToGroundMapping mapping = this.getMeasurements().getGroundMapping(ruggedName, lineSensor.getName());
            if (mapping != null) {
                this.sensorToGroundMappings.add(mapping);
            }
        }
    }

    /** {@inheritDoc} */
    @Override
    protected void createTargetAndWeight() {

        int n = 0;
        for (final SensorToGroundMapping reference : this.sensorToGroundMappings) {
            n += reference.getMapping().size();
        }

        if (n == 0) {
            throw new RuggedException(RuggedMessages.NO_REFERENCE_MAPPINGS);
        }
        final double[] target = new double[2 * n];
        final double[] weight = new double[2 * n];

        double min = Double.POSITIVE_INFINITY;
        double max = Double.NEGATIVE_INFINITY;
        int k = 0;

        for (final SensorToGroundMapping reference : this.sensorToGroundMappings) {
            for (final Map.Entry<SensorPixel, GeodeticPoint> mapping : reference.getMapping()) {
                final SensorPixel sp = mapping.getKey();
                weight[k] = 1.0;
                target[k++] = sp.getLineNumber();
                weight[k] = 1.0;
                target[k++] = sp.getPixelNumber();
                min = FastMath.min(min, sp.getLineNumber());
                max = FastMath.max(max, sp.getLineNumber());
            }
        }

        this.minLine = (int) FastMath.floor(min - ESTIMATION_LINE_RANGE_MARGIN);
        this.maxLine = (int) FastMath.ceil(max - ESTIMATION_LINE_RANGE_MARGIN);
        this.targetAndWeight = new HashMap<String, double[]>();
        this.targetAndWeight.put(TARGET, target);
        this.targetAndWeight.put(WEIGHT, weight);
    }

    /** {@inheritDoc} */
    @Override
    protected MultivariateJacobianFunction createFunction() {

        // model function
        final MultivariateJacobianFunction model = point -> {

            // set the current parameters values
            int i = 0;
            for (final ParameterDriver driver : this.getDrivers()) {
                driver.setNormalizedValue(point.getEntry(i++));
            }

            final double[] target = this.targetAndWeight.get(TARGET);

            // compute inverse loc and its partial derivatives
            final RealVector value = new ArrayRealVector(target.length);
            final RealMatrix jacobian = new Array2DRowRealMatrix(target.length, this.getNbParams());
            int l = 0;
            for (final SensorToGroundMapping reference : this.sensorToGroundMappings) {
                for (final Map.Entry<SensorPixel, GeodeticPoint> mapping : reference.getMapping()) {
                    final GeodeticPoint gp = mapping.getValue();
                    final Gradient[] ilResult = this.rugged.inverseLocationDerivatives(reference.getSensorName(), gp, minLine, maxLine, this.getGenerator());

                    if (ilResult == null) {
                        value.setEntry(l, minLine - 100.0); // arbitrary
                        // line far
                        // away
                        value.setEntry(l + 1, -100.0); // arbitrary
                        // pixel far away
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
                            jacobian.setEntry(l, m,
                                    ilResult[0]
                                            .getPartialDerivative(orders) *
                                            scale);
                            jacobian.setEntry(l + 1, m,
                                    ilResult[1]
                                            .getPartialDerivative(orders) *
                                            scale);
                            orders[m] = 0;
                            m++;
                        }
                    }

                    l += 2;
                }
            }

            // inverse loc result with Jacobian for all reference points
            return new Pair<RealVector, RealMatrix>(value, jacobian);
        };

        return model;
    }


    /** Least square problem builder.
     * @param maxEvaluations maxIterations and evaluations
     * @param convergenceThreshold parameter convergence threshold
     * @return the least square problem
     */
    @Override
    public final LeastSquaresProblem build(final int maxEvaluations, final double convergenceThreshold) {

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
