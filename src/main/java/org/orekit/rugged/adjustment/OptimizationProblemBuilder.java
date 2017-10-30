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
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.hipparchus.analysis.differentiation.DSFactory;
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
import org.orekit.rugged.adjustment.measurements.Observables;
import org.orekit.rugged.utils.DSGenerator;
import org.orekit.utils.ParameterDriver;

/**
 * Builder for optimization problem.
 * <p>
 * TODO GP description a completer 
 * </p>
 * @author Jonathan Guinet
 * @author Guylaine Prat
 * @since 2.0
 */
abstract class OptimizationProblemBuilder {

    /** Margin used in parameters estimation for the inverse location lines range. */
    protected static final int ESTIMATION_LINE_RANGE_MARGIN = 100;

    /** Derivative structure generator.*/
    private final DSGenerator generator;

    /** Parameter drivers list. */
    private final List<ParameterDriver> drivers;

    /** Number of parameters to refine. */
    private final int nbParams;

    /** Measurements. */
    private Observables measurements;

    /** Sensors list. */
    private final List<LineSensor> sensors;

    /** Constructor.
     * @param sensors list of sensors to refine
     * @param measurements set of observables
     * @throws RuggedException an exception is generated if no parameters has been selected for refining
     */
    OptimizationProblemBuilder(final List<LineSensor> sensors, final Observables measurements) throws RuggedException {
        
        try {
            this.generator = this.createGenerator(sensors);
            this.drivers = this.generator.getSelected();
            this.nbParams = this.drivers.size();
            if (this.nbParams == 0) {
                throw new RuggedException(RuggedMessages.NO_PARAMETERS_SELECTED);
            }
        } catch (RuggedExceptionWrapper rew) {
            throw rew.getException();
        }

        this.measurements = measurements;
        this.sensors = sensors;
    }

    /** Least squares problem builder.
     * @param maxEvaluations maximum number of evaluations
     * @param convergenceThreshold convergence threshold
     * @throws RuggedException if sensor is not found
     * @return the least squares problem
     */

    public abstract LeastSquaresProblem build(int maxEvaluations, double convergenceThreshold) 
        throws RuggedException;

    /** Create the convergence check.
     * <p>
     * check LInf distance of parameters variation between previous and current iteration
     * </p>
     * @param parametersConvergenceThreshold convergence threshold
     * @return the checker
     */
    final ConvergenceChecker<LeastSquaresProblem.Evaluation> 
                            createChecker(final double parametersConvergenceThreshold) {

        // TODO GP description a completer 
        final ConvergenceChecker<LeastSquaresProblem.Evaluation> checker = 
                (iteration, previous, current) 
                -> current.getPoint().getLInfDistance(previous.getPoint()) 
                <= parametersConvergenceThreshold;
                
        return checker;
    }

    /** Create start points for optimization algorithm.
     * @return start parameters values (normalized)
     */
    final double[] createStartTab() {
        
        // Get start points (as a normalized value)
        final double[] start = new double[this.nbParams];
        int iStart = 0;
        for (final ParameterDriver driver : this.drivers) {
            start[iStart++] = driver.getNormalizedValue();
        }
        return start;
    }

    // TODO GP description a completer 
    protected abstract void createTargetAndWeight() throws RuggedException;

    // TODO GP description a completer 
    protected abstract MultivariateJacobianFunction createFunction();

    /** Parse the observables to select mapping .*/
    protected abstract void initMapping();
    
    /** Create parameter validator.
     * @return parameter validator
     */
    final ParameterValidator createParameterValidator() {
        
        // Prevent parameters to exceed their prescribed bounds
        // TODO GP description a completer 
        final ParameterValidator validator = params -> {
            try {
                int i = 0;
                for (final ParameterDriver driver : this.drivers) {
                    
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

    /** Create the generator for {@link DerivativeStructure} instances.
     * @param selectedSensors list of sensors referencing the parameters drivers
     * @return a new generator
     */
    private DSGenerator createGenerator(final List<LineSensor> selectedSensors) {

        // Initialize set of drivers name
        final Set<String> names = new HashSet<>();
        
        // Get the drivers name 
        for (final LineSensor sensor : selectedSensors) {
            
            // Get the drivers name for the sensor 
            sensor.getParametersDrivers().forEach(driver -> {
                
                // Add the name of the driver to the set of drivers name
                if (names.contains(driver.getName()) == false) {
                    names.add(driver.getName());
                }
            });
        }

        // Set up generator list and map
        final List<ParameterDriver> selected = new ArrayList<>();
        final Map<String, Integer> map = new HashMap<>();
        
        // Get the list of selected drivers
        for (final LineSensor sensor : selectedSensors) {
            
            sensor.getParametersDrivers().filter(driver -> driver.isSelected()).forEach(driver -> {
                if (map.get(driver.getName()) == null) {
                    map.put(driver.getName(), map.size());
                    selected.add(driver);
                }
            });
        }

        final DSFactory factory = new DSFactory(map.size(), 1);

        // TODO GP description a completer 
        return new DSGenerator() {

            /** {@inheritDoc} */
            @Override
            public List<ParameterDriver> getSelected() {
                return selected;
            }

            /** {@inheritDoc} */
            @Override
            public DerivativeStructure constant(final double value) {
                return factory.constant(value);
            }

            /** {@inheritDoc} */
            @Override
            public DerivativeStructure variable(final ParameterDriver driver) {
                
                final Integer index = map.get(driver.getName());
                if (index == null) {
                    return constant(driver.getValue());
                } else {
                    return factory.variable(index.intValue(), driver.getValue());
                }
            }
        };
    }

    /** Get the sensors list.
     * @return the sensors list
     */
    protected List<LineSensor> getSensors() {
        return sensors;
    }
    
    /** Get the number of parameters to refine.
     * @return the number of parameters to refine
     */
    protected final int getNbParams() {
        return this.nbParams;
    }

    /**
     * Get the parameters drivers list.
     * @return the selected list of parameters driver
     */
    protected final List<ParameterDriver> getDrivers() {
        return this.drivers;
    }

    /**
     * Get the derivative structure generator.
     * @return the derivative structure generator.
     */
    protected final DSGenerator getGenerator() {
        return this.generator;
    }
    
    /** Get the measurements.
     * @return the measurements
     */
    protected Observables getMeasurements() {
        return measurements;
    }
}
