/* Copyright 2013-2018 CS Systèmes d'Information
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

import static org.junit.Assert.assertTrue;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;

import org.hipparchus.optim.nonlinear.vector.leastsquares.LeastSquaresOptimizer.Optimum;
import org.junit.After;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;
import org.orekit.rugged.adjustment.measurements.Observables;
import org.orekit.rugged.api.Rugged;
import org.orekit.rugged.errors.RuggedException;
import org.orekit.rugged.utils.RefiningTest;

public class AdjustmentContextTest {
    
    @Before
    public void setUp() {
        
        try {
            RefiningTest refiningTest = new RefiningTest();
            refiningTest.initRefiningTest();

            ruggedList = refiningTest.getRuggedList();

            int lineSampling = 1000;
            int pixelSampling = 1000;

            measurements = refiningTest.generateNoisyPoints(lineSampling, pixelSampling);
            numberOfParameters = refiningTest.getParameterToAdjust();
            
        }  catch (RuggedException re) {
            Assert.fail(re.getLocalizedMessage());
        }
    }

    @Test
    public void testAdjustmentContext() throws RuggedException, NoSuchFieldException, SecurityException, IllegalArgumentException, IllegalAccessException {
        
        AdjustmentContext adjustmentContext = new AdjustmentContext(ruggedList, measurements);
        
        // Check if the default OptimizerId is the expected one
        Field optimizerId = adjustmentContext.getClass().getDeclaredField("optimizerID");
        optimizerId.setAccessible(true);
        OptimizerId defaultOptimizerId = (OptimizerId) optimizerId.get(adjustmentContext);
        
        System.out.println(defaultOptimizerId);
        assertTrue((defaultOptimizerId == OptimizerId.GAUSS_NEWTON_QR));
        
//        // Check if the change of the default OptimizerId is correct
//        adjustmentContext.setOptimizer(OptimizerId.GAUSS_NEWTON_LU);
//        OptimizerId modifiedOptimizerId = (OptimizerId) optimizerId.get(adjustmentContext);
//        System.out.println(modifiedOptimizerId);
//        assertTrue((modifiedOptimizerId == OptimizerId.GAUSS_NEWTON_LU));
        
//        // Check if the change of the default OptimizerId is correct
//        adjustmentContext.setOptimizer(OptimizerId.GAUSS_NEWTON_LU);
//        OptimizerId modifiedOptimizerId = (OptimizerId) optimizerId.get(adjustmentContext);
//        System.out.println(modifiedOptimizerId);
//        assertTrue((modifiedOptimizerId == OptimizerId.GAUSS_NEWTON_LU));

        
//        OptimizerId.valueOf(OptimizerId.LEVENBERG_MARQUADT.toString());
    }
    
 

    @Test
    public void testEstimateFreeParameters() throws RuggedException, NoSuchFieldException, SecurityException, IllegalArgumentException, IllegalAccessException {
        
        AdjustmentContext adjustmentContext = new AdjustmentContext(ruggedList, measurements);
        
        List<String> ruggedNameList = new ArrayList<String>();
        for(Rugged rugged : ruggedList) {
            ruggedNameList.add(rugged.getName());
        }
        final int maxIterations = 120;
        final double convergenceThreshold = 1.e-7;

        Optimum optimum = adjustmentContext.estimateFreeParameters(ruggedNameList, maxIterations, convergenceThreshold);
        
        Assert.assertTrue(optimum.getIterations() < 20);
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

        final double expectedMaxValue = 3.324585e-03;
        Assert.assertEquals(expectedMaxValue, optimum.getResiduals().getMaxValue(), 1.0e-6);

        final double expectedRMS = 0.069669;
        Assert.assertEquals(expectedRMS, optimum.getRMS(), 1.0e-6);

        System.out.format("Chi sqaure %3.8e %n", optimum.getChiSquare());
        
        assertTrue(numberOfParameters == optimum.getPoint().getDimension());
        
        System.out.format("residuals %d \n", optimum.getResiduals().getDimension());
//        int measureCount = 0;
//        while (measurements.getInterMappings().iterator().hasNext()) {
//            measureCount++; 
//            System.out.println("measure " + measureCount);
//        }
//        System.out.format(" measurements %d \n", measureCount);
        System.out.format("cost %3.8e \n", optimum.getCost());
        

        
    }

    @After
    public void tearDown() {
        measurements = null;
        ruggedList = null;
    }
    
    private Observables measurements;
    private List<Rugged> ruggedList;
    private int numberOfParameters;

}
