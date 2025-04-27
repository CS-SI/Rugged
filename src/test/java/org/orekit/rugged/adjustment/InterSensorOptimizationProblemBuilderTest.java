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
import java.util.Collection;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;

import org.hipparchus.optim.nonlinear.vector.leastsquares.LeastSquaresOptimizer.Optimum;
import org.hipparchus.optim.nonlinear.vector.leastsquares.LeastSquaresProblem;
import org.junit.After;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;
import org.orekit.rugged.TestUtils;
import org.orekit.rugged.adjustment.measurements.Observables;
import org.orekit.rugged.adjustment.measurements.SensorToSensorMapping;
import org.orekit.rugged.adjustment.util.InitInterRefiningTest;
import org.orekit.rugged.api.Rugged;
import org.orekit.rugged.errors.RuggedException;
import org.orekit.rugged.errors.RuggedMessages;
import org.orekit.rugged.linesensor.LineSensor;
import org.orekit.rugged.linesensor.SensorPixel;

public class InterSensorOptimizationProblemBuilderTest {

    @Before
    public void setUp() {

        try {
            refiningTest = new InitInterRefiningTest();
            refiningTest.initRefiningTest();

            ruggedList = refiningTest.getRuggedList();

            earthConstraintWeight = 0.1;

            measurements = refiningTest.generateNoisyPoints(lineSampling, pixelSampling, earthConstraintWeight, false);
            numberOfParameters = refiningTest.getParameterToAdjust();

        }  catch (RuggedException re) {
            Assert.fail(re.getLocalizedMessage());
        }
    }

    @Test
    public void testEstimateFreeParameters() throws SecurityException, IllegalArgumentException {

        AdjustmentContext adjustmentContext = new AdjustmentContext(ruggedList, measurements);

        List<String> ruggedNameList = new ArrayList<String>();
        for(Rugged rugged : ruggedList) {
            ruggedNameList.add(rugged.getName());
        }
        final int maxIterations = 100;
        final double convergenceThreshold = 1.e-7;

        Optimum optimum = adjustmentContext.estimateFreeParameters(ruggedNameList, maxIterations, convergenceThreshold);

        Assert.assertTrue(optimum.getIterations() < maxIterations);

        // The default optimizer is a Gauss Newton.
        // For Gauss Newton, the number of evaluations is equal to the number of iterations
        Assert.assertTrue(optimum.getEvaluations() == optimum.getIterations());

        final double expectedMaxValue = 1.924769e-03;
        Assert.assertEquals(expectedMaxValue, optimum.getResiduals().getMaxValue(), 1.0e-6);

        final double expectedRMS = 0.069302;
        Assert.assertEquals(expectedRMS, optimum.getRMS(), 1.0e-6);

        final double expectedCost = 3.597014;
        Assert.assertEquals(expectedCost, optimum.getCost(), 2.5e-6);

        Assert.assertTrue(numberOfParameters == optimum.getPoint().getDimension());
        
        final int sensorToSensorMappingSize = 1347;
        Collection<SensorToSensorMapping> ssm = measurements.getInterMappings();
        Iterator<SensorToSensorMapping> it = ssm.iterator();
        while (it.hasNext()) {
            SensorToSensorMapping ssmit = it.next();
            Assert.assertTrue(sensorToSensorMappingSize == ssmit.getMapping().size());
        }        
        Assert.assertTrue(sensorToSensorMappingSize*2 == optimum.getResiduals().getDimension());

    }
    
    @Test
    public void testEarthConstraintPostponed() {

        // Get the measurements as computed in other tests
        Collection<SensorToSensorMapping> sensorToSensorMapping = measurements.getInterMappings();
        int nbModels = measurements.getNbModels();

        // Recompute the measurements in another way ... but that must be the same after all
        Observables measurementsPostponed = refiningTest.generateNoisyPoints(lineSampling, pixelSampling, earthConstraintWeight, true);
        Collection<SensorToSensorMapping> sensorToSensorMappingPostponed = measurementsPostponed.getInterMappings();
        int nbModelsPostponed = measurementsPostponed.getNbModels();

        // Compare the two collections of measurements
        Assert.assertEquals(nbModels, nbModelsPostponed);

        Assert.assertEquals(sensorToSensorMapping.size(), sensorToSensorMappingPostponed.size());

        // There is only one item
        SensorToSensorMapping arraySensorToSensorMapping          = (SensorToSensorMapping) sensorToSensorMapping.toArray()[0];
        SensorToSensorMapping arraySensorToSensorMappingPostponed = (SensorToSensorMapping) sensorToSensorMappingPostponed.toArray()[0];

        List<Double> listBody = arraySensorToSensorMapping.getBodyDistances();

        // Convert List<Double> to double[]
        double[] arrayBody = listBody.stream().mapToDouble(Double::doubleValue).toArray(); //via method reference
        // Explanations:
        //      get the Stream<Double> from the list
        //      map each double instance to its primitive value, resulting in a DoubleStream
        //      call toArray() to get the array.
        // Other method:       double[] arrayBody = listBody.stream().mapToDouble(d -> d).toArray(); //identity function, Java unboxes automatically to get the double value

        List<Double> listBodyPostponed = arraySensorToSensorMappingPostponed.getBodyDistances();
        double[] arrayBodyPostponed  = listBodyPostponed.stream().mapToDouble(Double::doubleValue).toArray();

        Assert.assertEquals(listBody.size(), listBodyPostponed.size());
        Assert.assertArrayEquals(arrayBody, arrayBodyPostponed, 3.e-3);

        List<Double> listLos = arraySensorToSensorMapping.getLosDistances();
        double[] arrayLos = listLos.stream().mapToDouble(Double::doubleValue).toArray();
        List<Double> listLosPostponed = arraySensorToSensorMappingPostponed.getLosDistances();
        double[] arrayLosPostponed = listLosPostponed.stream().mapToDouble(Double::doubleValue).toArray();

        Assert.assertEquals(listLos.size(), listLosPostponed.size());
        Assert.assertArrayEquals(arrayLos, arrayLosPostponed, 1.e-6);

        // Check if the two set are the same
        Set<Entry<SensorPixel, SensorPixel>> mapping          = arraySensorToSensorMapping.getMapping();
        Set<Entry<SensorPixel, SensorPixel>> mappingPostponed = arraySensorToSensorMappingPostponed.getMapping();

        Iterator<Entry<SensorPixel, SensorPixel>> itMapping = mapping.iterator();
        while(itMapping.hasNext()) {
            Entry<SensorPixel, SensorPixel> current = itMapping.next();
            SensorPixel key = current.getKey();
            SensorPixel value = current.getValue();

            // Will search in mappingPostponed if we can find the (key,value) found in mapping 
            Boolean found = false;
            Iterator<Entry<SensorPixel, SensorPixel>> itMappingPost = mappingPostponed.iterator();
            while(itMappingPost.hasNext()) {
                Entry<SensorPixel, SensorPixel> currentPost = itMappingPost.next();
                SensorPixel keyPost = currentPost.getKey();
                SensorPixel valuePost = currentPost.getValue();

                // Comparison of each SensorPixel (for the key part and the value part)
                if (TestUtils.sameSensorPixels(key, keyPost, 3.e-3) &&
                    TestUtils.sameSensorPixels(value, valuePost, 3.e-3)) {
                    // we found a match ...
                    found = true;
                }
            } // end iteration on mappingPostponed

            if (!found) { // the current (key,value) of the mapping was not found in the mappingPostponed
                Assert.assertTrue(found);
            }
        } // end on iteration on mapping

        Assert.assertEquals(arraySensorToSensorMapping.getRuggedNameA(),arraySensorToSensorMappingPostponed.getRuggedNameA());
        Assert.assertEquals(arraySensorToSensorMapping.getRuggedNameB(),arraySensorToSensorMappingPostponed.getRuggedNameB());;
        Assert.assertEquals(arraySensorToSensorMapping.getSensorNameA(),arraySensorToSensorMappingPostponed.getSensorNameA());;
        Assert.assertEquals(arraySensorToSensorMapping.getSensorNameB(),arraySensorToSensorMappingPostponed.getSensorNameB());
    } 
    
    @Test
    public void testDefaultRuggedNames() {
        
        // In that case there are no body distance set at construction

        // Generate intermapping with simple constructor of SensorToSensorMapping with Earth Constraint Weight given at construction
        Observables measurementsWithWeight = refiningTest.generateSimpleInterMapping(lineSampling, pixelSampling, earthConstraintWeight, false);
        Collection<SensorToSensorMapping> sensorToSensorMappingWithWeight = measurementsWithWeight.getInterMappings();
        int nbModelsWithWeight = measurementsWithWeight.getNbModels();

        // Generate intermapping with simple constructor of SensorToSensorMapping with Earth Constraint Weight given after construction
        Observables measurementsWithoutWeight = refiningTest.generateSimpleInterMapping(lineSampling, pixelSampling, earthConstraintWeight, true);        
        Collection<SensorToSensorMapping> sensorToSensorMappingPostponed = measurementsWithoutWeight.getInterMappings();
        int nbModelsPostponed = measurementsWithoutWeight.getNbModels();

        // Compare the two collections of measurements
        Assert.assertEquals(nbModelsWithWeight, nbModelsPostponed);

        Assert.assertEquals(sensorToSensorMappingWithWeight.size(), sensorToSensorMappingPostponed.size());

        // There is only one item
        SensorToSensorMapping arraySensorToSensorMappingWithWeight          = (SensorToSensorMapping) sensorToSensorMappingWithWeight.toArray()[0];
        SensorToSensorMapping arraySensorToSensorMappingPostponed = (SensorToSensorMapping) sensorToSensorMappingPostponed.toArray()[0];


        List<Double> listLosWithWeight = arraySensorToSensorMappingWithWeight.getLosDistances();
        double[] arrayLosWithWeight = listLosWithWeight.stream().mapToDouble(Double::doubleValue).toArray();
        List<Double> listLosPostponed = arraySensorToSensorMappingPostponed.getLosDistances();
        double[] arrayLosPostponed = listLosPostponed.stream().mapToDouble(Double::doubleValue).toArray();

        Assert.assertEquals(listLosWithWeight.size(), listLosPostponed.size());
        Assert.assertArrayEquals(arrayLosWithWeight, arrayLosPostponed, 1.e-6);

        // Check if the two set are the same
        Set<Entry<SensorPixel, SensorPixel>> mappingWithWeight = arraySensorToSensorMappingWithWeight.getMapping();
        Set<Entry<SensorPixel, SensorPixel>> mappingPostponed  = arraySensorToSensorMappingPostponed.getMapping();

        Iterator<Entry<SensorPixel, SensorPixel>> itMapping = mappingWithWeight.iterator();
        while(itMapping.hasNext()) {
            Entry<SensorPixel, SensorPixel> current = itMapping.next();
            SensorPixel key = current.getKey();
            SensorPixel value = current.getValue();

            // Will search in mappingPostponed if we can find the (key,value) found in mapping 
            Boolean found = false;
            Iterator<Entry<SensorPixel, SensorPixel>> itMappingPost = mappingPostponed.iterator();
            while(itMappingPost.hasNext()) {
                Entry<SensorPixel, SensorPixel> currentPost = itMappingPost.next();
                SensorPixel keyPost = currentPost.getKey();
                SensorPixel valuePost = currentPost.getValue();

                // Comparison of each SensorPixel (for the key part and the value part)
                if (TestUtils.sameSensorPixels(key, keyPost, 1.e-3) &&
                    TestUtils.sameSensorPixels(value, valuePost, 1.e-3)) {
                    // we found a match ...
                    found = true;
                }
            } // end iteration on mappingPostponed

            if (!found) { // the current (key,value) of the mapping was not found in the mappingPostponed
                Assert.assertTrue(found);
            }
        } // end on iteration on mapping

        Assert.assertEquals(arraySensorToSensorMappingWithWeight.getRuggedNameA(),arraySensorToSensorMappingPostponed.getRuggedNameA());
        Assert.assertEquals(arraySensorToSensorMappingWithWeight.getRuggedNameB(),arraySensorToSensorMappingPostponed.getRuggedNameB());;
        Assert.assertEquals(arraySensorToSensorMappingWithWeight.getSensorNameA(),arraySensorToSensorMappingPostponed.getSensorNameA());;
        Assert.assertEquals(arraySensorToSensorMappingWithWeight.getSensorNameB(),arraySensorToSensorMappingPostponed.getSensorNameB());
    }
    
    @Test
    public void testNoReferenceMapping() throws NoSuchFieldException, SecurityException, IllegalArgumentException, IllegalAccessException {

        try {
            final int maxIterations = 120;
            final double convergenceThreshold = 1.e-7;

            final List<LineSensor> selectedSensors = new ArrayList<LineSensor>();
            for (Rugged rugged : ruggedList) {
                selectedSensors.addAll(rugged.getLineSensors());
            }
            final InterSensorsOptimizationProblemBuilder interSensorsOptimizationProblem = 
                    new InterSensorsOptimizationProblemBuilder(selectedSensors, measurements, ruggedList);

            Field sensorToSensorMapping = interSensorsOptimizationProblem.getClass().getDeclaredField("sensorToSensorMappings");
            sensorToSensorMapping.setAccessible(true);
            sensorToSensorMapping.set(interSensorsOptimizationProblem,new ArrayList<SensorToSensorMapping>());

            interSensorsOptimizationProblem.build(maxIterations, convergenceThreshold);
            Assert.fail("An exception should have been thrown");

        } catch  (RuggedException re) {
            Assert.assertEquals(RuggedMessages.NO_REFERENCE_MAPPINGS,re.getSpecifier());
        }
    }

    @Test
    public void testInvalidRuggedNames() throws NoSuchFieldException, SecurityException, IllegalArgumentException, IllegalAccessException {

            final int maxIterations = 120;
            final double convergenceThreshold = 1.e-7;

            final List<LineSensor> selectedSensors = new ArrayList<LineSensor>();
            for (Rugged rugged : ruggedList) {
                selectedSensors.addAll(rugged.getLineSensors());
            }
            final InterSensorsOptimizationProblemBuilder interSensorsOptimizationProblem = 
                    new InterSensorsOptimizationProblemBuilder(selectedSensors, measurements, ruggedList);

            Field ruggedMapField = interSensorsOptimizationProblem.getClass().getDeclaredField("ruggedMap");
            ruggedMapField.setAccessible(true);
            @SuppressWarnings("unchecked")
            Map<String,Rugged> ruggedMap = (Map<String,Rugged>) ruggedMapField.get(interSensorsOptimizationProblem);
            
            // Set first RuggedB to null to get the right exception ...
            try {
                ruggedMap.put("RuggedB",null);

                ruggedMapField.set(interSensorsOptimizationProblem,ruggedMap);

                final LeastSquareAdjuster adjuster = new LeastSquareAdjuster(OptimizerId.GAUSS_NEWTON_QR);
                LeastSquaresProblem theProblem = interSensorsOptimizationProblem.build(maxIterations, convergenceThreshold);
                adjuster.optimize(theProblem);
                Assert.fail("An exception should have been thrown");

            } catch  (RuggedException re) {
                Assert.assertEquals(RuggedMessages.INVALID_RUGGED_NAME,re.getSpecifier());
            }
            
            // Then set RuggedA to null to get the right exception ...
            try {
                ruggedMap.put("RuggedA",null);

                ruggedMapField.set(interSensorsOptimizationProblem,ruggedMap);

                final LeastSquareAdjuster adjuster = new LeastSquareAdjuster(OptimizerId.GAUSS_NEWTON_QR);
                LeastSquaresProblem theProblem = interSensorsOptimizationProblem.build(maxIterations, convergenceThreshold);
                adjuster.optimize(theProblem);
                Assert.fail("An exception should have been thrown");

            } catch  (RuggedException re) {
                Assert.assertEquals(RuggedMessages.INVALID_RUGGED_NAME,re.getSpecifier());
            }
    }

    
    @After
    public void tearDown() {
        measurements = null;
        ruggedList = null;
    }

    private InitInterRefiningTest refiningTest;
    private Observables measurements;
    private List<Rugged> ruggedList;
    private int numberOfParameters;
    private double earthConstraintWeight; 
    
    private  final int lineSampling = 1000;
    private final int pixelSampling = 1000;
}
