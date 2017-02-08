/**
 * 
 */
package org.orekit.rugged.adjustment;


import java.util.Collection;
import java.util.Set;

import org.orekit.rugged.refining.measures.SensorToGroundMapping;
import org.orekit.rugged.refining.measures.SensorToSensorMapping;

import org.hipparchus.optim.nonlinear.vector.leastsquares.LeastSquaresProblem;
import java.lang.Object;
import org.orekit.utils.ParameterDriver;
import org.orekit.utils.ParameterDriversList;
import org.hipparchus.optim.ConvergenceChecker;
import org.hipparchus.optim.nonlinear.vector.leastsquares.ParameterValidator;

import org.orekit.errors.OrekitException;
import org.orekit.errors.OrekitExceptionWrapper;

/** Builder for optimization problem
 * <p>
* </p>
 * @author Jonathan Guinet
 */
public class OptimizationProblemBuilder {
    
    final ParameterDriversList drivers;

    public OptimizationProblemBuilder(final ParameterDriversList drivers) {
        this.drivers = drivers;
        
    }

    /** Create the convergence check
     * <p>
     * check Linf distance of parameters variation between previous and current iteration
     * </p>
     * @param parametersConvergenceThreshold convergence threshold
     * @return the checker
     */
    final ConvergenceChecker<LeastSquaresProblem.Evaluation>
        createChecker(final float parametersConvergenceThreshold) {
        final ConvergenceChecker<LeastSquaresProblem.Evaluation> checker = (iteration,
                                                                            previous,
                                                                            current) -> current
                                                                                .getPoint()
                                                                                .getLInfDistance(previous
                                                                                    .getPoint()) <= parametersConvergenceThreshold;
        return checker;
    }

    /** create start point for optimization algorithm.
     * @return start parameters values
     */
    final double[] createStartTab() {
        int iStart = 0;
        // get start point (as a normalized value)
        final double[] start = new double[this.drivers.getNbParams()];
        for (final ParameterDriver driver : this.drivers.getDrivers()) {
            start[iStart++] = driver.getNormalizedValue();
        }
        return start;

    }

    final Set<double[]> createTargetAndWeight(SensorToSensorMapping references) {
       return Set<>;  
    }
    
    final Set<double[]> createTargetAndWeight(SensorToGroundMapping references) {
        return Set<>;  
     }    
    
    
    /** create parameter validator.
     * @return parameter validator
     */
    final ParameterValidator createParameterValidator() {
        // prevent parameters to exceed their prescribed bounds
        final ParameterValidator validator = params -> {
            try {
                int i = 0;
                for (final ParameterDriver driver : this.drivers.getDrivers()) {
                    // let the parameter handle min/max clipping
                    driver.setNormalizedValue(params.getEntry(i));
                    params.setEntry(i++, driver.getNormalizedValue());
                }
                return params;
            } catch (OrekitException oe) {
                throw new OrekitExceptionWrapper(oe);
            }
        };
        return validator;
    }
}
