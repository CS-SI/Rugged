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

package org.orekit.rugged.adjustment;

import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.hipparchus.analysis.differentiation.DerivativeStructure;
import org.hipparchus.optim.ConvergenceChecker;
import org.hipparchus.optim.nonlinear.vector.leastsquares.LeastSquaresProblem;
import org.hipparchus.optim.nonlinear.vector.leastsquares.MultivariateJacobianFunction;
import org.hipparchus.optim.nonlinear.vector.leastsquares.ParameterValidator;
import org.orekit.errors.OrekitException;
import org.orekit.errors.OrekitExceptionWrapper;
import org.orekit.rugged.errors.RuggedException;
import org.orekit.rugged.errors.RuggedExceptionWrapper;
import org.orekit.rugged.errors.RuggedMessages;
import org.orekit.rugged.linesensor.LineSensor;
import org.orekit.rugged.refining.measures.Observables;
import org.orekit.rugged.utils.DSGenerator;
import org.orekit.utils.ParameterDriver;
import org.orekit.utils.ParameterDriversList;

/**
 * Builder for optimization problem.
 * <p>
 * </p>
 *
 * @author Jonathan Guinet
 */
abstract class OptimizationProblemBuilder {

    /**
     * Margin used in parameters estimation for the inverse location lines
     * range.
     */
    protected static final int ESTIMATION_LINE_RANGE_MARGIN = 100;

    /** Derivative structure generator.*/
    final DSGenerator generator;

    /** parameterDriversList. */
    protected final ParameterDriversList drivers;

    /** number of params to refine. */
    protected final int nbParams;


    /** measures .*/
    protected Observables measures;

    protected final List<LineSensor> sensors;

    /**  OptimizationProblemBuilder constructor
     * @param sensors
     * @param measures
     * @throws RuggedException
     */
    OptimizationProblemBuilder(final List<LineSensor> sensors,
                               Observables measures)
                                               throws RuggedException {
        try {
            this.generator = this.createGenerator(sensors);
            this.drivers = this.generator.getSelected();
            this.nbParams = this.drivers.getNbParams();
            if (this.nbParams == 0) {
                throw new RuggedException(RuggedMessages.NO_PARAMETERS_SELECTED);
            }
        } catch (RuggedExceptionWrapper rew) {
            throw rew.getException();
        }

        this.measures = measures;
        this.sensors = sensors;
    }

    /**  nbParams getter.
     * @return returns the number of variable parameters
     */
    public final int getNbParams()
    {
        return this.getNbParams();
    }

    /**
     * parameters drivers list getter.
     *
     * @return selected parameters driver list
     */
    public final ParameterDriversList getSelectedParametersDriver() {
        return this.drivers;
    }



    /** leastsquare problem builder.
     * @param maxEvaluations maxIterations and evaluations
     * @param convergenceThreshold parameter convergence threshold
     * @throws RuggedException if sensor is not found
     * @return the problem
     */

    public abstract LeastSquaresProblem build(final int maxEvaluations, final double convergenceThreshold) throws RuggedException;


    /**
     * Create the convergence check.
     * <p>
     * check Linf distance of parameters variation between previous and current
     * iteration
     * </p>
     *
     * @param parametersConvergenceThreshold convergence threshold
     * @return the checker
     */
    final ConvergenceChecker<LeastSquaresProblem.Evaluation>
    createChecker(final double parametersConvergenceThreshold) {
        final ConvergenceChecker<LeastSquaresProblem.Evaluation> checker = (iteration,
                        previous,
                        current) -> current
                        .getPoint()
                        .getLInfDistance(previous
                                         .getPoint()) <= parametersConvergenceThreshold;
                        return checker;
    }

    /**
     * create start point for optimization algorithm.
     *
     * @return start parameters values
     */
    final double[] createStartTab() {
        int iStart = 0;
        // get start point (as a normalized value)
        final double[] start = new double[this.drivers.getNbParams()];
        for (final ParameterDriver driver : this.drivers.getDrivers()) {
            start[iStart++] = driver.getNormalizedValue();
        }
        return start;

    }

    protected abstract void createTargetAndWeight() throws RuggedException;


    protected abstract MultivariateJacobianFunction createFunction();

    /** parse the observables to select mapping .*/
    protected abstract void initMapping();

    /**
     * create parameter validator.
     *
     * @return parameter validator
     */
    final ParameterValidator createParameterValidator() {
        // prevent parameters to exceed their prescribed bounds
        final ParameterValidator validator = params -> {
            try {
                int i = 0;
                for (final ParameterDriver driver : this.drivers.getDrivers()) {
                    // let the parameter handle min/max clipping
                    driver.setNormalizedValue(params.getEntry(i));
                    params.setEntry(i++, driver.getNormalizedValue());
                }
                return params;
            } catch (OrekitException oe) {
                throw new OrekitExceptionWrapper(oe);
            }
        };
        return validator;
    }

    /**
     * Create the generator for {@link DerivativeStructure} instances.
     *
     * @param selectedSensors sensors referencing the parameters drivers
     * @return a new generator
     */
    private DSGenerator
    createGenerator(final List<LineSensor> selectedSensors) {

        final Set<String> names = new HashSet<>();
        for (final LineSensor sensor : selectedSensors) {
            sensor.getParametersDrivers().forEach(driver -> {
                if (names.contains(driver.getName()) == false) {
                    names.add(driver.getName());
                }
            });
        }

        // set up generator list and map
        final ParameterDriversList selected = new ParameterDriversList();
        final Map<String, Integer> map = new HashMap<>();
        for (final LineSensor sensor : selectedSensors) {
            sensor.getParametersDrivers().filter(driver -> driver.isSelected())
            .forEach(driver -> {
                if (map.get(driver.getName()) == null) {
                    map.put(driver.getName(), map.size());
                }

                try {
                    selected.add(driver);
                } catch (OrekitException e) {
                    e.printStackTrace();
                }

            });
        }

        return new DSGenerator() {

            /** {@inheritDoc} */
            @Override
            public ParameterDriversList getSelected() {
                return selected;
            }

            /** {@inheritDoc} */
            @Override
            public DerivativeStructure constant(final double value) {
                return new DerivativeStructure(map.size(), 1, value);
            }

            /** {@inheritDoc} */
            @Override
            public DerivativeStructure variable(final ParameterDriver driver) {
                final Integer index = map.get(driver.getName());
                if (index == null) {
                    return constant(driver.getValue());
                } else {
                    return new DerivativeStructure(map.size(), 1,
                                                   index.intValue(),
                                                   driver.getValue());
                }
            }

        };
    }

}
