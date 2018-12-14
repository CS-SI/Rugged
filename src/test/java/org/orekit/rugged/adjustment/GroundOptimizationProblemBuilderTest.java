package org.orekit.rugged.adjustment;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.Iterator;
import java.util.List;
import java.util.Map.Entry;
import java.util.Set;

import org.hipparchus.optim.nonlinear.vector.leastsquares.LeastSquaresOptimizer.Optimum;
import org.junit.After;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.rugged.TestUtils;
import org.orekit.rugged.adjustment.measurements.Observables;
import org.orekit.rugged.adjustment.measurements.SensorToGroundMapping;
import org.orekit.rugged.adjustment.util.InitGroundRefiningTest;
import org.orekit.rugged.adjustment.util.RefiningParametersDriver;
import org.orekit.rugged.api.Rugged;
import org.orekit.rugged.errors.RuggedException;
import org.orekit.rugged.errors.RuggedMessages;
import org.orekit.rugged.linesensor.LineSensor;
import org.orekit.rugged.linesensor.SensorPixel;

public class GroundOptimizationProblemBuilderTest {

    @Before
    public void setUp() {

        try {
            refiningTest = new InitGroundRefiningTest();
            refiningTest.initGroundRefiningTest();
            
            rugged = refiningTest.getRugged();
            // get the only line sensor 
            LineSensor sensor = rugged.getLineSensors().iterator().next();
            sensorName = sensor.getName();

            measurements = refiningTest.generateNoisyPoints(lineSampling, pixelSampling, false);
            numberOfParameters = refiningTest.getParameterToAdjust();

        }  catch (RuggedException re) {
            Assert.fail(re.getLocalizedMessage());
        }
    }
    
    @Test
    public void testEstimateFreeParameters() {
        
        AdjustmentContext adjustmentContext = new AdjustmentContext(Collections.singletonList(rugged), measurements);
        final int maxIterations = 50;
        final double convergenceThreshold = 1.e-11;
        Optimum optimum = adjustmentContext.estimateFreeParameters(Collections.singletonList(rugged.getName()), maxIterations, convergenceThreshold);
        
        Assert.assertTrue(optimum.getIterations() < maxIterations);

        // The default optimizer is a Gauss Newton.
        // For Gauss Newton, the number of evaluations is equal to the number of iterations
        Assert.assertTrue(optimum.getEvaluations() == optimum.getIterations());

        final double expectedMaxValue = 39200.0;
        Assert.assertEquals(expectedMaxValue, optimum.getResiduals().getMaxValue(), 1.0e-6);

        final double expectedRMS = 5067.112098;
        Assert.assertEquals(expectedRMS, optimum.getRMS(), 1.0e-6);

        final double expectedCost = 286639.1460351976;
        Assert.assertEquals(expectedCost, optimum.getCost(), 1.0e-6);

        Assert.assertTrue(numberOfParameters == optimum.getPoint().getDimension());

        final int sensorToGroundMappingSize = 1600;
        Collection<SensorToGroundMapping> ssm = measurements.getGroundMappings();
        Iterator<SensorToGroundMapping> it = ssm.iterator();
        while (it.hasNext()) {
            SensorToGroundMapping ssmit = it.next();
            Assert.assertTrue(sensorToGroundMappingSize == ssmit.getMapping().size());
        }        
        Assert.assertTrue(sensorToGroundMappingSize*2 == optimum.getResiduals().getDimension());
    }

    @Test
    public void testNoParametersSelected() {
        try {
            RefiningParametersDriver.unselectRoll(rugged, sensorName);
            RefiningParametersDriver.unselectPitch(rugged, sensorName);
            
            AdjustmentContext adjustmentContext = new AdjustmentContext(Collections.singletonList(rugged), measurements);
            final int maxIterations = 50;
            final double convergenceThreshold = 1.e-11;
            
            adjustmentContext.estimateFreeParameters(Collections.singletonList(rugged.getName()), maxIterations, convergenceThreshold);
            Assert.fail("An exception should have been thrown");

        } catch (RuggedException re) {
            Assert.assertEquals(RuggedMessages.NO_PARAMETERS_SELECTED,re.getSpecifier());
        }
    }
    
    @Test
    public void testNoReferenceMapping() throws NoSuchFieldException, SecurityException, IllegalArgumentException, IllegalAccessException {

        try {
            final int maxIterations = 50;
            final double convergenceThreshold = 1.e-11;

            final List<LineSensor> selectedSensors = new ArrayList<LineSensor>();
            selectedSensors.addAll(rugged.getLineSensors());
            
            final GroundOptimizationProblemBuilder groundOptimizationProblem = 
                    new GroundOptimizationProblemBuilder(selectedSensors, measurements, rugged);

            Field sensorToGroundMapping = groundOptimizationProblem.getClass().getDeclaredField("sensorToGroundMappings");
            sensorToGroundMapping.setAccessible(true);
            sensorToGroundMapping.set(groundOptimizationProblem,new ArrayList<SensorToGroundMapping>());

            groundOptimizationProblem.build(maxIterations, convergenceThreshold);
            Assert.fail("An exception should have been thrown");

        } catch  (RuggedException re) {
            Assert.assertEquals(RuggedMessages.NO_REFERENCE_MAPPINGS,re.getSpecifier());
        }
    }
    
    @Test
    public void testDefaultRuggedName(){
        
        // Get the measurements as computed in other tests
        Collection<SensorToGroundMapping> sensorToGroundMapping = measurements.getGroundMappings();
        int nbModels = measurements.getNbModels();

        // Recompute the measurements in another way ... but that must be the same after all
        Observables measurementsNoName = refiningTest.generateNoisyPoints(lineSampling, pixelSampling, true);
        Collection<SensorToGroundMapping> sensorToGroundMappingNoName = measurementsNoName.getGroundMappings();
        int nbModelsWithWeight = measurementsNoName.getNbModels();
        
        // Compare the two collections of measurements
        Assert.assertEquals(nbModels, nbModelsWithWeight);

        Assert.assertEquals(sensorToGroundMapping.size(), sensorToGroundMappingNoName.size());

        // There is only one item
        SensorToGroundMapping arraySensorToGroundMapping          = (SensorToGroundMapping) sensorToGroundMapping.toArray()[0];
        SensorToGroundMapping arraySensorToGroundMappingNoName = (SensorToGroundMapping) sensorToGroundMappingNoName.toArray()[0];


        // Check if the two set are the same
        Set<Entry<SensorPixel, GeodeticPoint>> mapping          = arraySensorToGroundMapping.getMapping();
        Set<Entry<SensorPixel, GeodeticPoint>> mappingNoName = arraySensorToGroundMappingNoName.getMapping();

        Iterator<Entry<SensorPixel, GeodeticPoint>> itMapping = mapping.iterator();
        while(itMapping.hasNext()) {
            Entry<SensorPixel, GeodeticPoint> current = itMapping.next();
            SensorPixel key = current.getKey();
            GeodeticPoint value = current.getValue();

            // Will search in mappingNoName if we can find the (key,value) found in mapping 
            Boolean found = false;
            Iterator<Entry<SensorPixel, GeodeticPoint>> itMappingNoName = mappingNoName.iterator();
            while(itMappingNoName.hasNext()) {
                Entry<SensorPixel, GeodeticPoint> currentNoName = itMappingNoName.next();
                SensorPixel keyNoName = currentNoName.getKey();
                GeodeticPoint valueNoName = currentNoName.getValue();

                // Comparison of each SensorPixel (for the key part and the value part)
                if (TestUtils.sameSensorPixels(key, keyNoName, 1.e-3) &&
                    TestUtils.sameGeodeticPoints(value, valueNoName, 1.e-3)) {
                    // we found a match ...
                    found = true;
                }
            } // end iteration on mappingNoName

            if (!found) { // the current (key,value) of the mapping was not found in the mappingNoName
                Assert.assertTrue(found);
            }
        } // end on iteration on mapping

        Assert.assertEquals(arraySensorToGroundMapping.getRuggedName(),arraySensorToGroundMappingNoName.getRuggedName());
        Assert.assertEquals(arraySensorToGroundMapping.getSensorName(),arraySensorToGroundMappingNoName.getSensorName());;
    }
    
    
    @After
    public void tearDown() {
        measurements = null;
    }

    private InitGroundRefiningTest refiningTest;
    private Observables measurements;
    private Rugged rugged;
    private String sensorName;
    private int numberOfParameters;
    
    private  final int lineSampling = 1000;
    private final int pixelSampling = 1000;
}
