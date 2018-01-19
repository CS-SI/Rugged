/* Copyright 2013-2017 Systèmes d'Information
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
package fr.cs.examples.refiningPleiades;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;

import org.hipparchus.optim.nonlinear.vector.leastsquares.LeastSquaresOptimizer.Optimum;
import org.orekit.errors.OrekitException;
import org.orekit.errors.OrekitExceptionWrapper;
import org.orekit.rugged.adjustment.AdjustmentContext;
import org.orekit.rugged.api.Rugged;
import org.orekit.rugged.errors.RuggedException;
import org.orekit.rugged.errors.RuggedMessages;
import org.orekit.rugged.adjustment.measurements.Observables;
import org.orekit.rugged.adjustment.measurements.SensorToGroundMapping;
import org.orekit.rugged.adjustment.measurements.SensorToSensorMapping;

import fr.cs.examples.refiningPleiades.generators.GroundMeasurementGenerator;
import fr.cs.examples.refiningPleiades.generators.InterMeasurementGenerator;
import fr.cs.examples.refiningPleiades.generators.Noise;
import fr.cs.examples.refiningPleiades.metrics.LocalisationMetrics;

/**
 * Class for refining problems common methods
 * @author Jonathan Guinet
 * @author Lucie Labat-Allee
 * @author Guylaine Prat
 * @see SensorToGroundMapping
 * @see SensorToSensorMapping
 * @see GroundMeasurementGenerator
 * @see InterMeasurementGenerator
 * @since 2.0
 */
public class Refining {

    /**
     * Constructor
     */
    public Refining() throws RuggedException {
    }

    /** Apply disruptions on acquisition for roll, pitch and scale factor
     * @param rugged Rugged instance
     * @param sensorName line sensor name
     * @param rollValue rotation on roll value
     * @param pitchValue rotation on pitch value
     * @param factorValue scale factor
     * @throws RuggedException
     */
    public void applyDisruptions(final Rugged rugged, final String sensorName,
    		                     final double rollValue, final double pitchValue, final double factorValue)
        throws OrekitException, RuggedException {

        rugged.
        getLineSensor(sensorName).
        getParametersDrivers().
        filter(driver -> driver.getName().equals(sensorName + rollSuffix)).
        findFirst().get().setValue(rollValue);

        rugged.
        getLineSensor(sensorName).
        getParametersDrivers().
        filter(driver -> driver.getName().equals(sensorName + pitchSuffix)).
        findFirst().get().setValue(pitchValue);

        rugged.
        getLineSensor(sensorName).
        getParametersDrivers().
        filter(driver -> driver.getName().equals(factorName)).
        findFirst().get().setValue(factorValue);
    }

    /** Generate measurements without noise (sensor to ground mapping)
     * @param lineSampling line sampling
     * @param pixelSampling pixel sampling
     * @param rugged Rugged instance
     * @param sensorName line sensor name
     * @param dimension number of line of the sensor
     * @return ground measurements generator (sensor to ground mapping)
     * @throws RuggedException
     */
    public GroundMeasurementGenerator generatePoints(final int lineSampling, final int pixelSampling,
    		                                     final Rugged rugged, final String sensorName,
                                                 final int dimension) throws RuggedException {

        GroundMeasurementGenerator measurements = new GroundMeasurementGenerator(rugged, sensorName, dimension);

        System.out.format("\n**** Generate measurements (without noise; sensor to ground mapping) **** %n");

        // Generation measurements without noise
        measurements.createMeasurement(lineSampling, pixelSampling);

        System.out.format("Number of tie points generated: %d %n", measurements.getMeasurementCount());

        return measurements;
    }

    /** Generate measurements without noise (sensor to sensor mapping)
     * @param lineSampling line sampling
     * @param pixelSampling pixel sampling
     * @param ruggedA Rugged instance of acquisition A
     * @param sensorNameA line sensor name A
     * @param dimensionA dimension for acquisition A
     * @param ruggedB Rugged instance of acquisition B
     * @param sensorNameB line sensor name B
     * @param dimensionB dimension for acquisition B
     * @return inter measurements generator (sensor to sensor mapping)
     * @throws RuggedException
     */
    public InterMeasurementGenerator generatePoints(final int lineSampling, final int pixelSampling,
    		                                    final Rugged ruggedA, final String sensorNameA, final int dimensionA,
    		                                    final Rugged ruggedB, final String sensorNameB, final int dimensionB)
        throws RuggedException {

    	// Outliers control
    	final double outlierValue = 1e+2;
    	// Earth constraint weight
    	final double earthConstraintWeight = 0.1;

        // Generate measurements with constraints on outliers control and Earth distance
        InterMeasurementGenerator measurements = new InterMeasurementGenerator(ruggedA, sensorNameA, dimensionA,
                                                                   ruggedB, sensorNameB, dimensionB,
                                                                   outlierValue,
                                                                   earthConstraintWeight);

        System.out.format("\n**** Generate measurements (without noise; sensor to sensor mapping) **** %n");

        // Generation measurements without noise
        measurements.createMeasurement(lineSampling, pixelSampling);

        System.out.format("Number of tie points generated: %d %n", measurements.getMeasurementCount());

        return measurements;
    }

    /** Generate noisy measurements (sensor to ground mapping)
     * @param lineSampling line sampling
     * @param pixelSampling pixel sampling
     * @param rugged Rugged instance
     * @param sensorName line sensor name
     * @param dimension dimension
     * @param noise Noise structure to generate noisy measurements
     * @return ground measurements generator (sensor to ground mapping)
     * @throws RuggedException
     */
    public GroundMeasurementGenerator generateNoisyPoints(final int lineSampling, final int pixelSampling,
    		                                          final Rugged rugged, final String sensorName, final int dimension,
    		                                          final Noise noise) throws RuggedException {

        // Generate ground measurements
    	GroundMeasurementGenerator measurements = new GroundMeasurementGenerator(rugged, sensorName, dimension);

        System.out.format("\n**** Generate noisy measurements (sensor to ground mapping) **** %n");

        // Generate noisy measurements
        measurements.createNoisyMeasurement(lineSampling, pixelSampling, noise);

        System.out.format("Number of tie points generated: %d %n", measurements.getMeasurementCount());

        return measurements;
    }

    /** Generate noisy measurements (sensor to sensor mapping)
     * @param lineSampling line sampling
     * @param pixelSampling pixel sampling
     * @param ruggedA Rugged instance of acquisition A
     * @param sensorNameA line sensor name A
     * @param dimensionA dimension for acquisition A
     * @param ruggedB Rugged instance of acquisition B
     * @param sensorNameB line sensor name B
     * @param dimensionB dimension for acquisition B
     * @param noise noise structure to generate noisy measurements
     * @return inter-measurements generator (sensor to sensor mapping)
     * @throws RuggedException
     */
    public InterMeasurementGenerator generateNoisyPoints(final int lineSampling, final int pixelSampling,
    		                                         final Rugged ruggedA, final String sensorNameA, final int dimensionA,
                                                     final Rugged ruggedB, final String sensorNameB, final int dimensionB,
                                                     final Noise noise) throws RuggedException {

    	// Outliers control
    	final double outlierValue = 1.e+2;
    	
        // Earth constraint weight
        final double earthConstraintWeight = 0.1;

        // Generate measurements with constraints on Earth distance and outliers control
        InterMeasurementGenerator measurements = new InterMeasurementGenerator(ruggedA, sensorNameA, dimensionA,
                                                                   ruggedB, sensorNameB, dimensionB,
                                                                   outlierValue,
                                                                   earthConstraintWeight);
        System.out.format("\n**** Generate noisy measurements (sensor to sensor mapping) **** %n");

        // Generation noisy measurements
        measurements.createNoisyMeasurement(lineSampling, pixelSampling, noise);

        System.out.format("Number of tie points generated: %d %n", measurements.getMeasurementCount());

        return measurements;
    }

    /** Compute metrics to evaluate geometric performances in location,
     * for Ground Control Points GCP study.
     * @param groundMapping sensor to ground mapping
     * @param rugged Rugged instance
     * @param unit flag to know if distance is computed in meters (false) or with angular (true)
     * @throws RuggedException
     */
    public void computeMetrics(final SensorToGroundMapping groundMapping,
    		                   final Rugged rugged, final boolean unit) throws RuggedException {

        String stUnit = null;
        if(unit) {
        	stUnit="degrees";
        } else {
        	stUnit="meters";
        }

        LocalisationMetrics residuals = new LocalisationMetrics(groundMapping, rugged, unit);
        System.out.format("Max: %3.4e Mean: %3.4e %s%n",residuals.getMaxResidual(),residuals.getMeanResidual(), stUnit);
    }

    /** Compute metrics to evaluate geometric performances in location,
     * for tie points study.
     * @param interMapping sensor to sensor mapping
     * @param ruggedA Rugged instance A
     * @param ruggedB Rugged instance B
     * @param unit flag to know if distance is computed in meters (false) or with angular (true)
     * @throws RuggedException
     */
    public void computeMetrics(final SensorToSensorMapping interMapping,
    		                   final Rugged ruggedA, final Rugged ruggedB,
                               final boolean unit) throws RuggedException {

        String stUnit = null;
        if(unit) stUnit="degrees";
        else stUnit="meters";

        LocalisationMetrics residuals = new LocalisationMetrics(interMapping, ruggedA, ruggedB, unit);
        System.out.format("Max: %1.4e Mean: %1.4e %s%n",residuals.getMaxResidual(),residuals.getMeanResidual(), stUnit);
        System.out.format("LOS distance Max: %1.4e Mean: %1.4e %s%n",residuals.getLosMaxDistance(),residuals.getLosMeanDistance(), stUnit);
        System.out.format("Earth distance Max: %1.4e Mean: %1.4e %s%n",residuals.getEarthMaxDistance(),residuals.getEarthMeanDistance(), stUnit);
    }

    /** Reset a model
     * @param rugged Rugged instance
     * @param sensorName line sensor name
     * @param isSelected flag to known if factor parameter is selected or not
     * @throws RuggedException
     */
    public void resetModel(final Rugged rugged, final String sensorName, final boolean isSelected) throws RuggedException {

        rugged.
        getLineSensor(sensorName).
        getParametersDrivers().
        filter(driver -> driver.getName().equals(sensorName + rollSuffix)
               || driver.getName().equals(sensorName + pitchSuffix)).
        forEach(driver -> {
            try {
                driver.setSelected(true);
                driver.setValue(0.0);
            } catch (OrekitException e) {
                throw new OrekitExceptionWrapper(e);
            }
        });

        rugged.
        getLineSensor(sensorName).
        getParametersDrivers().
        filter(driver -> driver.getName().equals(factorName)).
        forEach(driver -> {
            try {
                driver.setSelected(isSelected);

                // default value: no Z scale factor applied
                driver.setValue(1.0);
            } catch (OrekitException e) {
                throw new OrekitExceptionWrapper(e);
            }
        });
    }

    /** Start optimization to  adjust parameters (Ground Control Points GCP study).
     * @param maxIterations iterations max
     * @param convergenceThreshold threshold of convergence
     * @param measurements ground measurements
     * @param rugged Rugged instance
     * @throws RuggedException
     */
    public void optimization(final int maxIterations, final double convergenceThreshold,
    		                 final Observables measurements, final Rugged rugged) throws RuggedException {

        System.out.format("Iterations max: %d\tconvergence threshold: %3.6e \n", maxIterations, convergenceThreshold);

        AdjustmentContext adjustmentContext = new AdjustmentContext(Collections.singletonList(rugged), measurements);
        Optimum optimum = adjustmentContext.estimateFreeParameters(Collections.singletonList(rugged.getName()), maxIterations, convergenceThreshold);

        // Print statistics
        System.out.format("Max value: %3.6e %n", optimum.getResiduals().getMaxValue());
        System.out.format("Optimization performed in %d iterations \n", optimum.getIterations());
        System.out.format("Optimization performed with %d evaluations of model (objective) function \n", optimum.getEvaluations());
        System.out.format("RMSE: %f \n", optimum.getRMS());
    }

    /** Start optimization to  adjust parameters (tie points study).
     * @param maxIterations iterations max
     * @param convergenceThreshold threshold of convergence
     * @param measurements measurements
     * @param ruggeds Rugged instances A and B
     * @throws RuggedException
     */
    public void optimization(final int maxIterations, final double convergenceThreshold,
    		                 final Observables measurements,
    		                 final Collection<Rugged> ruggeds) throws RuggedException {

        System.out.format("Iterations max: %d\tconvergence threshold: %3.6e \n", maxIterations, convergenceThreshold);

        if(ruggeds.size()!= 2 ) {
            throw new RuggedException(RuggedMessages.UNSUPPORTED_REFINING_CONTEXT,ruggeds.size());
        }

        AdjustmentContext adjustmentContext = new AdjustmentContext(ruggeds, measurements);

        List<String> ruggedNameList = new ArrayList<String>();
        for(Rugged rugged : ruggeds) {
            ruggedNameList.add(rugged.getName());
        }

        Optimum optimum = adjustmentContext.estimateFreeParameters(ruggedNameList, maxIterations, convergenceThreshold);

        // Print statistics
        System.out.format("Max value: %3.6e %n", optimum.getResiduals().getMaxValue());
        System.out.format("Optimization performed in %d iterations \n", optimum.getIterations());
        System.out.format("Optimization performed with %d evaluations of model (objective) function \n", optimum.getEvaluations());
        System.out.format("RMSE: %f \n", optimum.getRMS());
    }

    /** Check adjusted parameters of an acquisition
     * @param rugged Rugged instance
     * @param sensorName line sensor name
     * @param rollValue rotation on roll value
     * @param pitchValue rotation on pitch value
     * @param factorValue scale factor
     * @throws RuggedException
     */
    public void paramsEstimation(final Rugged rugged, final String sensorName,
    		                     final double rollValue, final double pitchValue, final double factorValue)
        throws RuggedException {

        // Estimate Roll
        double estimatedRoll = rugged.getLineSensor(sensorName).
                        getParametersDrivers().
                        filter(driver -> driver.getName().equals(sensorName + rollSuffix)).
                        findFirst().get().getValue();

        double rollError = (estimatedRoll - rollValue);
        System.out.format("Estimated roll: %3.5f\troll error: %3.6e %n", estimatedRoll, rollError);

        // Estimate pitch
        double estimatedPitch = rugged.getLineSensor(sensorName).
                        getParametersDrivers().
                        filter(driver -> driver.getName().equals(sensorName + pitchSuffix)).
                        findFirst().get().getValue();

        double pitchError = (estimatedPitch - pitchValue);
        System.out.format("Estimated pitch: %3.5f\tpitch error: %3.6e %n", estimatedPitch, pitchError);

        // Estimate scale factor
        double estimatedFactor = rugged.getLineSensor(sensorName).
                        getParametersDrivers().
                        filter(driver -> driver.getName().equals(factorName)).
                        findFirst().get().getValue();

        double factorError = (estimatedFactor - factorValue);
        System.out.format("Estimated factor: %3.5f\tfactor error: %3.6e %n", estimatedFactor, factorError);
    }
    
    /**
     * Part of the name of parameter drivers 
     */
    static final String rollSuffix = "_roll";
    static final String pitchSuffix = "_pitch";
    static final String factorName = "factor";
    
    public static String getRollsuffix() {
        return rollSuffix;
    }

    public static String getPitchsuffix() {
        return pitchSuffix;
    }

    public static String getFactorname() {
        return factorName;
    }
    
    
}
