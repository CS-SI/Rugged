package org.orekit.rugged.utils;

import org.junit.jupiter.api.Test;
import org.orekit.rugged.adjustment.OptimizerId;
import org.orekit.rugged.api.BodyRotatingFrameId;
import org.orekit.rugged.api.EllipsoidId;
import org.orekit.rugged.api.InertialFrameId;

/** Tests to obtain 100% of coverage for enum class with jacoco tool.
 * Even though one use each enumerate of an enum class, 
 * there is no way to obtain a full 100% coverage of the class
 * unless ... to perform a simple test like
 * TheEnumClass.valueOf(TheEnumClass.OneEnumValue..toString());
 * @author Guylaine Prat
 */
public class EnumeratesTest {

    @Test
    public void testEnumOptimizerId() {
        // Test to have 100% coverage of enum class
        OptimizerId.valueOf(OptimizerId.LEVENBERG_MARQUADT.toString());
    }
    
    @Test
    public void testBodyRotatingFrameId() {
        // Test to have 100% coverage of enum class
        BodyRotatingFrameId.valueOf(BodyRotatingFrameId.ITRF.toString());
    }

    @Test
    public void testEllipsoidId() {
        // Test to have 100% coverage of enum class
        EllipsoidId.valueOf(EllipsoidId.IERS2003.toString());
    }

    @Test
    public void testInertialFrameId() {
        // Test to have 100% coverage of enum class
        InertialFrameId.valueOf(InertialFrameId.GCRF.toString());
    }
}
