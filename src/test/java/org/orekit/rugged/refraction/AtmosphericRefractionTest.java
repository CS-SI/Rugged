package org.orekit.rugged.refraction;

import static org.junit.Assert.assertFalse;

import java.util.ArrayList;

import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.junit.Assert;
import org.junit.Test;
import org.orekit.rugged.errors.RuggedException;
import org.orekit.rugged.errors.RuggedMessages;
import org.orekit.rugged.linesensor.LineSensor;
import org.orekit.rugged.los.LOSBuilder;
import org.orekit.rugged.los.TimeDependentLOS;

public class AtmosphericRefractionTest {

    @Test
    public void testBadConfig() {

        int dimension = 400;
        
        TimeDependentLOS los = new LOSBuilder(new ArrayList<Vector3D>(dimension)).build();
        LineSensor lineSensor = new LineSensor("line", null, Vector3D.ZERO, los);
        
        // Defines atmospheric refraction model (with the default multi layers model)
        AtmosphericRefraction atmosphericRefraction = new MultiLayerModel(null);

        // Check the context
        atmosphericRefraction.setGridSteps(100, 100);
        atmosphericRefraction.configureCorrectionGrid(lineSensor, 0, 300);
        assertFalse(atmosphericRefraction.isSameContext("otherSensor", 0, 300));
        assertFalse(atmosphericRefraction.isSameContext("line", 42, 300));
        assertFalse(atmosphericRefraction.isSameContext("line", 0, 42));

        // Check the test of validity of min / max line vs line step
        try {
            atmosphericRefraction.setGridSteps(100, 100);
            atmosphericRefraction.configureCorrectionGrid(lineSensor, 0, 100);
            Assert.fail("An exception should have been thrown");
    
        } catch (RuggedException re) {
            Assert.assertEquals(RuggedMessages.INVALID_RANGE_FOR_LINES,re.getSpecifier());
        }
        
        // Bad pixel step
        try {
            atmosphericRefraction.setGridSteps(-5, 100);
            atmosphericRefraction.configureCorrectionGrid(lineSensor, 0, 100);
            Assert.fail("An exception should have been thrown");

        } catch (RuggedException re) {
            Assert.assertEquals(RuggedMessages.INVALID_STEP,re.getSpecifier());
        }
     
        // Bad line step
        try {
            atmosphericRefraction.setGridSteps(10, -42);
            atmosphericRefraction.configureCorrectionGrid(lineSensor, 0, 100);
            Assert.fail("An exception should have been thrown");

        } catch (RuggedException re) {
            Assert.assertEquals(RuggedMessages.INVALID_STEP,re.getSpecifier());
        }
    }
}
