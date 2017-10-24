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
package RefiningPleiades;

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
import org.orekit.rugged.refining.generators.GroundMeasureGenerator;
import org.orekit.rugged.refining.generators.InterMeasureGenerator;
import org.orekit.rugged.refining.measures.Noise;
import org.orekit.rugged.refining.measures.Observables;
import org.orekit.rugged.refining.measures.SensorToGroundMapping;
import org.orekit.rugged.refining.measures.SensorToSensorMapping;
import org.orekit.rugged.refining.metrics.LocalisationMetrics;

/**
 * Class for refining problems common methods
 * @author Jonathan Guinet
 * @author Lucie Labat-Allee
 * @author Guylaine Prat
 * @see SensorToGroundMapping
 * @see SensorToSensorMapping
 * @see GroundMeasureGenerator
 * @see InterMeasureGenerator
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
    public void applyDisruptions(Rugged rugged, String sensorName,
                                 double rollValue, double pitchValue, double factorValue)
        throws OrekitException, RuggedException {

        final String commonFactorName = "factor";

        rugged.
        getLineSensor(sensorName).
        getParametersDrivers().
        filter(driver -> driver.getName().equals(sensorName+"_roll")).
        findFirst().get().setValue(rollValue);

        rugged.
        getLineSensor(sensorName).
        getParametersDrivers().
        filter(driver -> driver.getName().equals(sensorName+"_pitch")).
        findFirst().get().setValue(pitchValue);

        rugged.
        getLineSensor(sensorName).
        getParametersDrivers().
        filter(driver -> driver.getName().equals(commonFactorName)).
        findFirst().get().setValue(factorValue);
    }

    /** Generate measurements without noise (sensor to ground mapping)
     * @param lineSampling line sampling
     * @param pixelSampling pixel sampling
     * @param rugged Rugged instance
     * @param sensorName line sensor name
     * @param dimension number of line of the sensor
     * @return ground measures generator (sensor to ground mapping)
     * @throws RuggedException
     */
    public GroundMeasureGenerator generatePoints(int lineSampling, int pixelSampling,
                                                 Rugged rugged, String sensorName,
                                                 int dimension) throws RuggedException {

        GroundMeasureGenerator measures = new GroundMeasureGenerator(rugged, sensorName, dimension);

        System.out.format("\n**** Generate measures (without noise; sensor to ground mapping) **** %n");

        // Generation measures without noise
        measures.createMeasure(lineSampling, pixelSampling);

        System.out.format("Number of tie points generated: %d %n", measures.getMeasureCount());

        return measures;
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
     * @return inter measures generator (sensor to sensor mapping)
     * @throws RuggedException
     */
    public InterMeasureGenerator generatePoints(int lineSampling, int pixelSampling,
                                                Rugged ruggedA, String sensorNameA, int dimensionA,
                                                Rugged ruggedB, String sensorNameB, int dimensionB) throws RuggedException {

    	// Outliers control
    	final double outlierValue = 1e+2;
    	// Earth constraint weight
    	final double earthConstraintWeight = 0.1;

        // Generate measures with constraints on outliers control and Earth distance
        InterMeasureGenerator measures = new InterMeasureGenerator(ruggedA, sensorNameA, dimensionA,
                                                                   ruggedB, sensorNameB, dimensionB,
                                                                   outlierValue,
                                                                   earthConstraintWeight);

        System.out.format("\n**** Generate measures (without noise; sensor to sensor mapping) **** %n");

        // Generation measures without noise
        measures.createMeasure(lineSampling, pixelSampling);

        System.out.format("Number of tie points generated: %d %n", measures.getMeasureCount());

        return measures;
    }

    /** Generate noisy measurements (sensor to ground mapping)
     * @param lineSampling line sampling
     * @param pixelSampling pixel sampling
     * @param rugged Rugged instance
     * @param sensorName line sensor name
     * @param dimension dimension
     * @param noise Noise structure to generate noisy measures
     * @return ground measures generator (sensor to ground mapping)
     * @throws RuggedException
     */
    public GroundMeasureGenerator generateNoisyPoints(int lineSampling, int pixelSampling,
                                                      Rugged rugged, String sensorName, int dimension,
                                                      Noise noise) throws RuggedException {

        // Generate ground measures
    	GroundMeasureGenerator measures = new GroundMeasureGenerator(rugged, sensorName, dimension);

        System.out.format("\n**** Generate noisy measures (sensor to ground mapping) **** %n");

        // Generate noisy measures
        measures.createNoisyMeasure(lineSampling, pixelSampling, noise);

        System.out.format("Number of tie points generated: %d %n", measures.getMeasureCount());

        return measures;
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
     * @param noise noise structure to generate noisy measures
     * @return inter-measures generator (sensor to sensor mapping)
     * @throws RuggedException
     */
    public InterMeasureGenerator generateNoisyPoints(int lineSampling, int pixelSampling,
                                                     Rugged ruggedA, String sensorNameA, int dimensionA,
                                                     Rugged ruggedB, String sensorNameB, int dimensionB,
                                                     Noise noise) throws RuggedException {

    	// Outliers control
    	final double outlierValue = 1.e+2;    
    	
        // Earth constraint weight
        final double earthConstraintWeight = 0.1;  

        // Generate measures with constraints on Earth distance and outliers control
        InterMeasureGenerator measures = new InterMeasureGenerator(ruggedA, sensorNameA, dimensionA,
                                                                   ruggedB, sensorNameB, dimensionB,
                                                                   outlierValue,
                                                                   earthConstraintWeight);
        System.out.format("\n**** Generate noisy measures (sensor to sensor mapping) **** %n");

        // Generation noisy measures
        measures.createNoisyMeasure(lineSampling, pixelSampling, noise);

        System.out.format("Number of tie points generated: %d %n", measures.getMeasureCount());

        return measures;
    }

    /** Compute metrics to evaluate geometric performances in location,
     * for fulcrum points study.
     * @param groundMapping sensor to ground mapping
     * @param rugged Rugged instance
     * @param unit flag to know if distance is computed in meters (false) or with angular (true)
     * @throws RuggedException
     */
    public void computeMetrics(SensorToGroundMapping groundMapping,
                               Rugged rugged, boolean unit) throws RuggedException {

        String stUnit = null;
        if(unit) {
        	stUnit="degrees";
        } else {
        	stUnit="meters";
        }

        LocalisationMetrics residues = new LocalisationMetrics(groundMapping, rugged, unit);
        System.out.format("Max: %3.4e Mean: %3.4e %s%n",residues.getMaxResidual(),residues.getMeanResidual(), stUnit);
    }

    /** Compute metrics to evaluate geometric performances in location,
     * for liaison points study.
     * @param interMapping sensor to sensor mapping
     * @param ruggedA Rugged instance A
     * @param ruggedB Rugged instance B
     * @param unit flag to know if distance is computed in meters (false) or with angular (true)
     * @throws RuggedException
     */
    public void computeMetrics(SensorToSensorMapping interMapping,
                               Rugged ruggedA, Rugged ruggedB,
                               boolean unit) throws RuggedException {

        String stUnit = null;
        if(unit) stUnit="degrees";
        else stUnit="meters";

        LocalisationMetrics residues = new LocalisationMetrics(interMapping, ruggedA, ruggedB, unit);
        System.out.format("Max: %1.4e Mean: %1.4e %s%n",residues.getMaxResidual(),residues.getMeanResidual(), stUnit);
        System.out.format("LOS distance Max: %1.4e Mean: %1.4e %s%n",residues.getLosMaxDistance(),residues.getLosMeanDistance(), stUnit);
        System.out.format("Earth distance Max: %1.4e Mean: %1.4e %s%n",residues.getEarthMaxDistance(),residues.getEarthMeanDistance(), stUnit);
    }

    /** Reset a model
     * @param rugged Rugged instance
     * @param sensorName line sensor name
     * @param isSelected flag to known if factor parameter is selected or not
     * @throws RuggedException
     */
    public void resetModel(Rugged rugged, String sensorName, boolean isSelected) throws RuggedException {

        final String commonFactorName = "factor";

        rugged.
        getLineSensor(sensorName).
        getParametersDrivers().
        filter(driver -> driver.getName().equals(sensorName+"_roll")
               || driver.getName().equals(sensorName+"_pitch")).
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
        filter(driver -> driver.getName().equals(commonFactorName)).
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

    /** Start optimization to  adjust parameters (fulcrum points study).
     * @param maxIterations iterations max
     * @param convergenceThreshold threshold of convergence
     * @param measures ground measures
     * @param rugged Rugged instance
     * @throws RuggedException
     */
    public void optimization(int maxIterations, double convergenceThreshold,
                             Observables measures, Rugged rugged) throws RuggedException {

        System.out.format("Iterations max: %d\tconvergence threshold: %3.6e \n", maxIterations, convergenceThreshold);

        AdjustmentContext adjustmentContext = new AdjustmentContext(Collections.singletonList(rugged), measures);
        Optimum optimum = adjustmentContext.estimateFreeParameters(Collections.singletonList(rugged.getName()), maxIterations, convergenceThreshold);

        // Print statistics
        System.out.format("Max value: %3.6e %n", optimum.getResiduals().getMaxValue());
        System.out.format("Optimization performed in %d iterations \n", optimum.getEvaluations());
        System.out.format("RMSE: %f \n", optimum.getRMS());
    }

    /** Start optimization to  adjust parameters (liaison points study).
     * @param maxIterations iterations max
     * @param convergenceThreshold threshold of convergence
     * @param measures measures
     * @param ruggeds Rugged instances A and B
     * @throws RuggedException
     */
    public void optimization(int maxIterations, double convergenceThreshold,
                             Observables measures,
                             Collection<Rugged> ruggeds) throws RuggedException {

        System.out.format("Iterations max: %d\tconvergence threshold: %3.6e \n", maxIterations, convergenceThreshold);
        
        if(ruggeds.size()!= 2 ) {
            throw new RuggedException(RuggedMessages.UNSUPPORTED_REFINING_CONTEXT,ruggeds.size());
        }

        AdjustmentContext adjustmentContext = new AdjustmentContext(ruggeds, measures);

        List<String> ruggedNameList = new ArrayList<String>();
        for(Rugged rugged : ruggeds) {
            ruggedNameList.add(rugged.getName());
        }

        Optimum optimum = adjustmentContext.estimateFreeParameters(ruggedNameList, maxIterations, convergenceThreshold);

        // Print statistics
        System.out.format("Max value: %3.6e %n", optimum.getResiduals().getMaxValue());
        System.out.format("Optimization performed in %d iterations \n", optimum.getEvaluations());
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
    public void paramsEstimation(Rugged rugged, String sensorName,
                                 double rollValue, double pitchValue, double factorValue)
        throws RuggedException {

    	final String commonFactorName = "factor";

        // Estimate Roll
        double estimatedRoll = rugged.getLineSensor(sensorName).
                        getParametersDrivers().
                        filter(driver -> driver.getName().equals(sensorName+"_roll")).
                        findFirst().get().getValue();

        double rollError = (estimatedRoll - rollValue);
        System.out.format("Estimated roll: %3.5f\troll error: %3.6e %n", estimatedRoll, rollError);

        // Estimate pitch
        double estimatedPitch = rugged.getLineSensor(sensorName).
                        getParametersDrivers().
                        filter(driver -> driver.getName().equals(sensorName+"_pitch")).
                        findFirst().get().getValue();

        double pitchError = (estimatedPitch - pitchValue);
        System.out.format("Estimated pitch: %3.5f\tpitch error: %3.6e %n", estimatedPitch, pitchError);

        // Estimate scale factor
        double estimatedFactor = rugged.getLineSensor(sensorName).
                        getParametersDrivers().
                        filter(driver -> driver.getName().equals(commonFactorName)).
                        findFirst().get().getValue();

        double factorError = (estimatedFactor - factorValue);
        System.out.format("Estimated factor: %3.5f\tfactor error: %3.6e %n", estimatedFactor, factorError);
    }
}
