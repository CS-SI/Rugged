/* Copyright 2013-2022 CS GROUP
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

import java.lang.reflect.Field;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import org.hipparchus.optim.nonlinear.vector.leastsquares.LeastSquaresOptimizer.Optimum;

import org.junit.After;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

import org.orekit.rugged.adjustment.measurements.Observables;
import org.orekit.rugged.adjustment.measurements.SensorMapping;
import org.orekit.rugged.adjustment.util.InitInterRefiningTest;
import org.orekit.rugged.api.Rugged;
import org.orekit.rugged.errors.RuggedException;
import org.orekit.rugged.errors.RuggedMessages;
import org.orekit.rugged.linesensor.SensorPixel;

public class AdjustmentContextTest {
    
    @Before
    public void setUp() {
        
        try {
            // One must set a context for the adjustment ... Here we choose an inter sensors optimization problem
            InitInterRefiningTest refiningTest = new InitInterRefiningTest();
            refiningTest.initRefiningTest();

            ruggedList = refiningTest.getRuggedList();

            int lineSampling = 1000;
            int pixelSampling = 1000;
            
            double earthConstraintWeight = 0.1;

            measurements = refiningTest.generateNoisyPoints(lineSampling, pixelSampling, earthConstraintWeight, false);
            
        }  catch (RuggedException re) {
            Assert.fail(re.getLocalizedMessage());
        }
    }

    @Test
    public void testAdjustmentContext() throws NoSuchFieldException, SecurityException, IllegalArgumentException, IllegalAccessException {
        
        AdjustmentContext adjustmentContext = new AdjustmentContext(ruggedList, measurements);
        
        // Check if the default OptimizerId is the expected one (use reflectivity)
        Field optimizerId = adjustmentContext.getClass().getDeclaredField("optimizerID");
        optimizerId.setAccessible(true);
        OptimizerId defaultOptimizerId = (OptimizerId) optimizerId.get(adjustmentContext);
        Assert.assertTrue((defaultOptimizerId == OptimizerId.GAUSS_NEWTON_QR));
        
        // Check if the change of the default OptimizerId is correct
        adjustmentContext.setOptimizer(OptimizerId.GAUSS_NEWTON_LU);
        OptimizerId modifiedOptimizerId = (OptimizerId) optimizerId.get(adjustmentContext);
        Assert.assertTrue((modifiedOptimizerId == OptimizerId.GAUSS_NEWTON_LU));
        
        // Check if the change of the default OptimizerId is correct
        adjustmentContext.setOptimizer(OptimizerId.LEVENBERG_MARQUADT);
        modifiedOptimizerId = (OptimizerId) optimizerId.get(adjustmentContext);
        Assert.assertTrue((modifiedOptimizerId == OptimizerId.LEVENBERG_MARQUADT));

 
    }

    @Test
    public void testEstimateFreeParameters() throws NoSuchFieldException, SecurityException, IllegalArgumentException, IllegalAccessException {
        
        AdjustmentContext adjustmentContext = new AdjustmentContext(ruggedList, measurements);
        
        for (OptimizerId optimizer : OptimizerId.values()) {
            
            // Set the optimizer
            adjustmentContext.setOptimizer(optimizer);

            List<String> ruggedNameList = new ArrayList<String>();
            for(Rugged rugged : ruggedList) {
                ruggedNameList.add(rugged.getName());
            }
            final int maxIterations = 120;
            final double convergenceThreshold = 1.e-7;

            Optimum optimum = adjustmentContext.estimateFreeParameters(ruggedNameList, maxIterations, convergenceThreshold);
            
            Field optimizerId = adjustmentContext.getClass().getDeclaredField("optimizerID");
            optimizerId.setAccessible(true);
            OptimizerId usedOptimizerId = (OptimizerId) optimizerId.get(adjustmentContext);

            if (usedOptimizerId == OptimizerId.GAUSS_NEWTON_QR || usedOptimizerId == OptimizerId.GAUSS_NEWTON_LU) {
                // For Gauss Newton, the number of evaluations is equal to the number of iterations
                Assert.assertTrue(optimum.getEvaluations() == optimum.getIterations());
            } else if (usedOptimizerId == OptimizerId.LEVENBERG_MARQUADT) {
                // For Levenberg Marquadt, the number of evaluations is slightly greater than the number of iterations
                Assert.assertTrue(optimum.getEvaluations() >= optimum.getIterations());
            }

        } // loop on OptimizerId
    }

    @Test
    public void testInvalidRuggedName() {
        try {

            AdjustmentContext adjustmentContext = new AdjustmentContext(ruggedList, measurements);

            List<String> ruggedNameList = new ArrayList<String>();
            // the list must not have a null value
            Iterator<Rugged> it = ruggedList.iterator();
            while (it.hasNext()) {
                ruggedNameList.add(null);
                it.next();
            }
            final int maxIterations = 1;
            final double convergenceThreshold = 1.e-7;

            adjustmentContext.estimateFreeParameters(ruggedNameList, maxIterations, convergenceThreshold);
            Assert.fail("An exception should have been thrown");

        } catch (RuggedException re) {
            Assert.assertEquals(RuggedMessages.INVALID_RUGGED_NAME,re.getSpecifier());
        }
    }

    @Test
    public void testUnsupportedRefiningContext() {
        try {
            
            AdjustmentContext adjustmentContext = new AdjustmentContext(ruggedList, measurements);
            
            List<String> ruggedNameList = new ArrayList<String>();
            // Add too many rugged name: the list must have 1 or 2 items 
            for(Rugged rugged : ruggedList) {
                ruggedNameList.add(rugged.getName());
                ruggedNameList.add(rugged.getName());
                ruggedNameList.add(rugged.getName());
            }
            final int maxIterations = 1;
            final double convergenceThreshold = 1.e-7;

            adjustmentContext.estimateFreeParameters(ruggedNameList, maxIterations, convergenceThreshold);
            Assert.fail("An exception should have been thrown");
            
        } catch (RuggedException re) {
            Assert.assertEquals(RuggedMessages.UNSUPPORTED_REFINING_CONTEXT,re.getSpecifier());
        }
    }
    
    @Test
    public void testSensorMapping() throws NoSuchFieldException, SecurityException, IllegalArgumentException, IllegalAccessException {
 
        String sensorGiven = "lineSensor";
        String ruggedGiven = "Rugged";
        SensorMapping<SensorPixel> simpleMapping = new SensorMapping<SensorPixel>(sensorGiven) ;
        
        Field ruggedField = simpleMapping.getClass().getDeclaredField("ruggedName");
        ruggedField.setAccessible(true);
        String ruggedRead = (String) ruggedField.get(simpleMapping);
        
        Field sensorField = simpleMapping.getClass().getDeclaredField("sensorName");
        sensorField.setAccessible(true);
        String sensorRead = (String) sensorField.get(simpleMapping);

        Assert.assertTrue(ruggedGiven.equals(ruggedRead));
        Assert.assertTrue(sensorGiven.equals(sensorRead));
    }
    
    @After
    public void tearDown() {
        measurements = null;
        ruggedList = null;
    }
    
    private Observables measurements;
    private List<Rugged> ruggedList;

}
