package org.orekit.rugged.adjustment;

import java.lang.reflect.Field;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

public class LeastSquareAdjusterTest {

    @Test
    public void testLeastSquareAdjuster() throws NoSuchFieldException, SecurityException, IllegalArgumentException, IllegalAccessException {
        
        final LeastSquareAdjuster adjusterWithOptimizer = new LeastSquareAdjuster(OptimizerId.GAUSS_NEWTON_QR);
        final LeastSquareAdjuster adjusterWithDefaultOptimizer = new LeastSquareAdjuster();
        
        Field optimizerIdWithOptimizer = adjusterWithOptimizer.getClass().getDeclaredField("optimizerID");
        optimizerIdWithOptimizer.setAccessible(true);
        OptimizerId getOptimizerIdWithOptimizer = (OptimizerId) optimizerIdWithOptimizer.get(adjusterWithOptimizer);

        Field optimizerIdWithDefaultOptimizer = adjusterWithDefaultOptimizer.getClass().getDeclaredField("optimizerID");
        optimizerIdWithDefaultOptimizer.setAccessible(true);
        OptimizerId getOptimizerIdWithDefaultOptimizer = (OptimizerId) optimizerIdWithDefaultOptimizer.get(adjusterWithDefaultOptimizer);

        Assertions.assertTrue(getOptimizerIdWithDefaultOptimizer == getOptimizerIdWithOptimizer);
    }
}
