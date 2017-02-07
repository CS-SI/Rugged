package org.orekit.rugged.refining.generators;

import org.orekit.rugged.errors.RuggedException;
import org.orekit.rugged.refining.measures.Noise;

/**
 * For measures generator
 * @author Lucie Labat-Allee
 */

public interface Measurable {

    int  getMeasureCount() throws RuggedException;
    
    void createMeasure(final int lineSampling,final int pixelSampling)  throws RuggedException;
    
    void createNoisyMeasure(final int lineSampling,final int pixelSampling, Noise noise)  throws RuggedException;
    
}