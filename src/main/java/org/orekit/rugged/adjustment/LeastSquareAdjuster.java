/* Copyright 2013-2017 CS Systèmes d'Information
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

import org.hipparchus.optim.nonlinear.vector.leastsquares.GaussNewtonOptimizer;
import org.hipparchus.optim.nonlinear.vector.leastsquares.LeastSquaresOptimizer;
import org.hipparchus.optim.nonlinear.vector.leastsquares.LeastSquaresOptimizer.Optimum;
import org.hipparchus.optim.nonlinear.vector.leastsquares.LeastSquaresProblem;
import org.hipparchus.optim.nonlinear.vector.leastsquares.LevenbergMarquardtOptimizer;
import org.orekit.rugged.errors.RuggedException;

/** TODO GP description a completer 
 * @author Guylaine Prat
 * @since 2.0
 */
public class LeastSquareAdjuster {

    /** Least square optimizer.*/
    private final LeastSquaresOptimizer adjuster;

    /** Least square optimizer choice.*/
    private final OptimizerId optimizerID;

    /** Constructor.
     * @param optimizerID optimizer choice
     */
    // TODO GP public protected ???
    LeastSquareAdjuster(final OptimizerId optimizerID) {
        this.optimizerID = optimizerID;
        this.adjuster = this.selectOptimizer();
    }

    /** Default constructor with Gauss Newton with QR decomposition algorithm.*/
    // TODO GP public protected ???
    LeastSquareAdjuster() {
        this.optimizerID = OptimizerId.GAUSS_NEWTON_QR;
        this.adjuster = this.selectOptimizer();
    }

    /** Solve the least square problem.
     * @param problem the least square problem
     * @return the solution
     */
    public Optimum optimize(final LeastSquaresProblem problem) {
        return this.adjuster.optimize(problem);
    }

    /** Create the optimizer.
     * @return the least square optimizer
     */
    private LeastSquaresOptimizer selectOptimizer() {
    	
        // Set up the optimizer
        switch (this.optimizerID) {
        
        case LEVENBERG_MARQUADT:
            return new LevenbergMarquardtOptimizer();
            
        case GAUSS_NEWTON_LU :
            return new GaussNewtonOptimizer().withDecomposition(GaussNewtonOptimizer.Decomposition.LU);
            
        case GAUSS_NEWTON_QR :
            return new GaussNewtonOptimizer().withDecomposition(GaussNewtonOptimizer.Decomposition.QR);
            
        default :
            // this should never happen
            throw RuggedException.createInternalError(null);
        }
    }
}
