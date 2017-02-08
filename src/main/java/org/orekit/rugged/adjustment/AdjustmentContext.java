/* Copyright 2013-2016 CS Systèmes d'Information
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

import org.orekit.rugged.refining.measures.Observables;
import org.orekit.rugged.api.Rugged;
import org.orekit.rugged.errors.RuggedException;

import org.hipparchus.optim.ConvergenceChecker;
import org.hipparchus.optim.nonlinear.vector.leastsquares.GaussNewtonOptimizer;
import org.hipparchus.optim.nonlinear.vector.leastsquares.LevenbergMarquardtOptimizer;
import org.hipparchus.optim.nonlinear.vector.leastsquares.LeastSquaresBuilder;
import org.hipparchus.optim.nonlinear.vector.leastsquares.LeastSquaresProblem;
import org.hipparchus.optim.nonlinear.vector.leastsquares.LeastSquaresOptimizer;
import org.hipparchus.optim.nonlinear.vector.leastsquares.LeastSquaresOptimizer.Optimum;
import org.hipparchus.optim.nonlinear.vector.leastsquares.MultivariateJacobianFunction;
import org.hipparchus.optim.nonlinear.vector.leastsquares.ParameterValidator;
import org.orekit.rugged.adjustment.OptimizerId;

import org.hipparchus.linear.DiagonalMatrix;
import org.hipparchus.optim.ConvergenceChecker;


import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Create adjustment context for viewing model refining refining
 *
 * @author Lucie LabatAllee
 * @author Jonathan Guinet
 */
public class AdjustmentContext {

    /** List of Rugged instances to optimize. */
    private final Map<String, Rugged> viewingModel;

    /** set of measures. */
    private final Observables measures;

    /** Build a new instance.
     * @param viewingModel viewingModel
     * @param measures control and tie points
     */
    public AdjustmentContext(final List<Rugged> viewingModel, final Observables measures) {
        this.viewingModel = new HashMap<String, Rugged>();
        for (final Rugged r : viewingModel) {
            /** TODO this.viewingModel.put(r.getName(), r); */
            this.viewingModel.put("Rugged", r);
        }
        this.measures = measures;
    }

    
    public Optimum estimateFreeParameters(String RuggedName) {
        Optimum optimum;
        return optimum;
    }
    
    public Optimum estimateFreeParameters2Viewing(String RuggedNameA, String RuggedNameB) {
        Optimum optimum;
        return optimum;
    }
    
    /** Create the LeastSquareProblem.
     * @param maxEvaluations maximum iterations and evaluations in optimization problem
     * @param weight Measures weight matrix
     * @param start
     * @param target
     * @param validator
     * @param checker
     * @param model
     * @return the least square problem
     */
    public LeastSquaresProblem LeastSquaresProblemBuilder(final int maxEvaluations, final double[] weight,
                                                          final double[] start, final double[] target,
                                                          final ParameterValidator validator, 
                                                          final ConvergenceChecker<LeastSquaresProblem.Evaluation> checker,
                                                          final MultivariateJacobianFunction model) {
        final LeastSquaresProblem problem = new LeastSquaresBuilder()
                        .lazyEvaluation(false).maxIterations(maxEvaluations)
                        .maxEvaluations(maxEvaluations).weight(new DiagonalMatrix(weight)).start(start)
                        .target(target).parameterValidator(validator).checker(checker)
                        .model(model).build();
        return problem;
    }

    /** Create the optimizer.
     * @param optimizerID reference optimizer identifier
     * @return optimizer
     */
    private LeastSquaresOptimizer selectOptimizer(final OptimizerId optimizerID) {
        // set up the optimizer
        switch (optimizerID) {
        case LEVENBERG_MARQUADT:
            return new LevenbergMarquardtOptimizer();
        case GAUSS_NEWTON_LU :
            return new GaussNewtonOptimizer()
                            .withDecomposition(GaussNewtonOptimizer.Decomposition.LU);
        case GAUSS_NEWTON_QR :
            return new GaussNewtonOptimizer()
            .withDecomposition(GaussNewtonOptimizer.Decomposition.QR);
        default :
            // this should never happen
            throw RuggedException.createInternalError(null);
        }

    }

}
