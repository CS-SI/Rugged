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
package AffinagePleiades;

import org.hipparchus.optim.nonlinear.vector.leastsquares.LeastSquaresOptimizer.Optimum;
import java.util.Collections;

import org.orekit.errors.OrekitException;
import org.orekit.errors.OrekitExceptionWrapper;
import org.orekit.rugged.api.Rugged;
import org.orekit.rugged.errors.RuggedException;
import org.orekit.rugged.refining.generators.GroundMeasureGenerator;
import org.orekit.rugged.refining.generators.InterMeasureGenerator;
import org.orekit.rugged.refining.measures.Noise;
import org.orekit.rugged.refining.measures.SensorToGroundMapping;
import org.orekit.rugged.refining.measures.SensorToSensorMapping;
import org.orekit.rugged.refining.metrics.LocalisationMetrics;



/**
 * Class for refining
 * @author Jonathan Guinet
 * @author Lucie Labat-Allee
 * @see SensorToGroundMapping
 * @see SensorToSensorMapping
 * @see GroundMeasureGenerator
 * @see InterMeasureGenerator
 */
public class Refining {
    
    
    /**
     * Constructor
     */
    public Refining() throws RuggedException, OrekitException {
    
    }
    
    
    /** Apply disruptions on acquisition
     * @param rugged Rugged instance
     * @param sensorName line sensor name
     * @param  rollValue rotation on roll value
     * @param  pitchValue rotation on pitch value
     * @param  factorValue scale factor
     * @throws OrekitException 
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
    
    
    /** Generate measurements without noise
     * @param lineSampling line sampling
     * @param pixelSampling pixel sampling
     * @param rugged Rugged instance
     * @param sensorName line sensor name
     * @param dimension dimension
     * @throws OrekitException 
     * @throws RuggedException 
     * @return ground measures generator (sensor to ground mapping)
     */
    public GroundMeasureGenerator generatePoints(int lineSampling, int pixelSampling,
                                                 Rugged rugged, String sensorName, 
                                                 int dimension) throws OrekitException, RuggedException {
        
        GroundMeasureGenerator measures = new GroundMeasureGenerator(rugged, sensorName, dimension);
        
        System.out.format("\n**** Generate measures (without noise) **** %n");
        
        // Generation measures without noise
        measures.createMeasure(lineSampling, pixelSampling);
        
        System.out.format("Number of tie points generated: %d %n", measures.getMeasureCount());

        return measures;        
    }
    

    /** Generate measurements without noise
     * @param lineSampling line sampling
     * @param pixelSampling pixel sampling
     * @param ruggedA Rugged instance of acquisition A
     * @param sensorNameA line sensor name A
     * @param dimensionA dimension for acquisition A 
     * @param ruggedB Rugged instance of acquisition B
     * @param sensorNameB line sensor name B
     * @param dimensionB dimension for acquisition B
     * @throws OrekitException 
     * @throws RuggedException 
     * @return inter measures generator (sensor to sensor mapping)
     */
    public InterMeasureGenerator generatePoints(int lineSampling, int pixelSampling, 
                                                Rugged ruggedA, String sensorNameA, int dimensionA, 
                                                Rugged ruggedB, String sensorNameB, int dimensionB) throws OrekitException, RuggedException {
        
        final double outlierValue = 1e+2;           // outliers control
        final double earthConstraintWeight = 0.1;   // earth constraint weight

        /* Generate measures with constraints on Earth distance and outliers control */
        InterMeasureGenerator measures = new InterMeasureGenerator(ruggedA, sensorNameA, dimensionA,
                                                                   ruggedB, sensorNameB, dimensionB,
                                                                   outlierValue,
                                                                   earthConstraintWeight);
      
        System.out.format("\n**** Generate measures (without noise) **** %n");
        
        // Generation measures without noise
        measures.createMeasure(lineSampling, pixelSampling);
        
        System.out.format("Number of tie points generated: %d %n", measures.getMeasureCount());

        return measures;     
    }
    
    
    /** Generate noisy measurements
     * @param lineSampling line sampling
     * @param pixelSampling pixel sampling
     * @param rugged Rugged instance
     * @param sensorName line sensor name
     * @param dimension dimension
     * @param noise Noise structure to generate noisy measures
     * @throws OrekitException 
     * @throws RuggedException 
     * @return ground measures generator (sensor to ground mapping)
     */
    public GroundMeasureGenerator generateNoisyPoints(int lineSampling, int pixelSampling,
                                                      Rugged rugged, String sensorName, int dimension, 
                                                      Noise noise) throws OrekitException, RuggedException {
        
        GroundMeasureGenerator measures = new GroundMeasureGenerator(rugged, sensorName, dimension);
        
        System.out.format("\n**** Generate noisy measures **** %n");
            
        // Generation noisy measures
        measures.createNoisyMeasure(lineSampling, pixelSampling, noise); 
    
        System.out.format("Number of tie points generated: %d %n", measures.getMeasureCount());

        return measures;
    }

    
    /** Generate noisy measurements
     * @param lineSampling line sampling
     * @param pixelSampling pixel sampling
     * @param ruggedA Rugged instance of acquisition A
     * @param sensorNameA line sensor name A
     * @param dimensionA dimension for acquisition A
     * @param ruggedB Rugged instance of acquisition B
     * @param sensorNameB line sensor name B
     * @param dimensionB dimension for acquisition B
     * @param noise Noise structure to generate noisy measures
     * @throws OrekitException 
     * @throws RuggedException 
     * @return inter measures generator (sensor to sensor mapping)
     */
    public InterMeasureGenerator generateNoisyPoints(int lineSampling, int pixelSampling, 
                                                     Rugged ruggedA, String sensorNameA, int dimensionA, 
                                                     Rugged ruggedB, String sensorNameB, int dimensionB,
                                                     Noise noise) throws OrekitException, RuggedException {
        
        final double outlierValue = 1e+2;           // outliers control
        final double earthConstraintWeight = 0.1;   // earth constraint weight

        /* Generate measures with constraints on Earth distance and outliers control */
        InterMeasureGenerator measures = new InterMeasureGenerator(ruggedA, sensorNameA, dimensionA,
                                                                   ruggedB, sensorNameB, dimensionB,
                                                                   outlierValue,
                                                                   earthConstraintWeight);
        System.out.format("\n**** Generate noisy measures **** %n");
            
        // Generation noisy measures
        measures.createNoisyMeasure(lineSampling, pixelSampling, noise); 
            
        System.out.format("Number of tie points generated: %d %n", measures.getMeasureCount());

        return measures;
    }

    
    /** Compute metrics to evaluate geometric performances in location, 
     * for fulcrum points study.
     * @param groundMapping sensor to ground mapping
     * @param rugged Rugged instance
     * @param unit flag to known distance's unit
     * @throws OrekitException 
     * @throws RuggedException 
     */
    public void computeMetrics(SensorToGroundMapping groundMapping,
                               Rugged rugged, boolean unit) throws RuggedException {
        
        String stUnit = null;
        if(unit) stUnit="degrees";
        else stUnit="meters";
        
        LocalisationMetrics residues = new LocalisationMetrics(groundMapping, rugged, unit);
        System.out.format("Max: %3.4e Mean: %3.4e %s%n",residues.getMaxResidual(),residues.getMeanResidual(), stUnit);
    }

    
    /** Compute metrics to evaluate geometric performances in location, 
     * for liaison points study.
     * @param interMapping sensor to sensor mapping
     * @param ruggedA Rugged instance A
     * @param ruggedB Rugged instance B
     * @param unit flag to known distance's unit
     * @throws OrekitException 
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
     * @throws OrekitException 
     * @throws RuggedException 
     */
    public void resetModel(Rugged rugged, String sensorName, boolean isSelected) throws OrekitException, RuggedException {
    
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
                driver.setValue(1.0);       // default value: no Z scale factor applied
            } catch (OrekitException e) {
                throw new OrekitExceptionWrapper(e);
            }
        });
        
    }
    
    
    /** Start optimization to  adjust parameters (fulcrum points study).
     * @param maxIterations iterations max
     * @param convergenceThreshold threshold of convergence
     * @param groundMapping sensor to ground mapping
     * @param rugged Rugged instance
     * @throws OrekitException 
     * @throws RuggedException 
     */
    public void optimization(int maxIterations, double convergenceThreshold,
                             SensorToGroundMapping groundMapping, 
                             Rugged rugged) throws OrekitException, RuggedException {
    
        
        System.out.format("Iterations max: %d\tconvergence threshold: %3.6e \n",maxIterations, convergenceThreshold);

        // Adapt parameters
        Optimum optimum = rugged.estimateFreeParameters(Collections.singletonList(groundMapping), 
                                                        maxIterations, convergenceThreshold);
        
        // Print statistics 
        System.out.format("Max value: %3.6e %n",optimum.getResiduals().getMaxValue());
        System.out.format("Optimization performed in %d iterations \n",optimum.getEvaluations());
        System.out.format("RMSE: %f \n",optimum.getRMS());
    }

    
    /** Start optimization to  adjust parameters (liaison points study).
     * @param maxIterations iterations max
     * @param convergenceThreshold threshold of convergence
     * @param interMapping sensor to sensor mapping
     * @param ruggedA Rugged instance A
     * @param ruggedB Rugged instance B
     * @throws OrekitException 
     * @throws RuggedException 
     */
    public void optimization(int maxIterations, double convergenceThreshold,
                             SensorToSensorMapping interMapping, 
                             Rugged ruggedA, Rugged ruggedB) throws OrekitException, RuggedException {
    
        
        System.out.format("Iterations max: %d\tconvergence threshold: %3.6e \n",maxIterations, convergenceThreshold);

        // Note: estimateFreeParams2Models() is supposed to be applying on ruggedB, not on ruggedA (to be compliant with notations)
        
        // Adapt parameters
        Optimum optimum = ruggedB.estimateFreeParams2Models(Collections.singletonList(interMapping), 
                                                            maxIterations,convergenceThreshold, ruggedA);
        
        // Print statistics 
        System.out.format("Max value: %3.6e %n",optimum.getResiduals().getMaxValue());
        System.out.format("Optimization performed in %d iterations \n",optimum.getEvaluations());
        System.out.format("RMSE: %f \n",optimum.getRMS());
    }

    
    /** Check adjusted parameters of an acquisition
     * @param rugged Rugged instance
     * @param sensorName line sensor name
     * @param  rollValue rotation on roll value
     * @param  pitchValue rotation on pitch value
     * @param  factorValue scale factor
     * @throws OrekitException 
     * @throws RuggedException 
     */
    public void paramsEstimation(Rugged rugged, String sensorName,
                                 double rollValue, double pitchValue, double factorValue) 
                                 throws OrekitException, RuggedException {
        
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

        // Estimate factor
        double estimatedFactor = rugged.getLineSensor(sensorName).
                                        getParametersDrivers().
                                        filter(driver -> driver.getName().equals(commonFactorName)).
                                        findFirst().get().getValue();
        
        double factorError = (estimatedFactor - factorValue);
        System.out.format("Estimated factor: %3.5f\tfactor error: %3.6e %n", estimatedFactor, factorError);
    }
}
