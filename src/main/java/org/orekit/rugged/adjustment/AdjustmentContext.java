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


import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.hipparchus.optim.nonlinear.vector.leastsquares.LeastSquaresOptimizer.Optimum;
import org.orekit.errors.OrekitException;
import org.orekit.errors.OrekitExceptionWrapper;
import org.orekit.rugged.api.Rugged;
import org.orekit.rugged.errors.RuggedException;
import org.orekit.rugged.errors.RuggedExceptionWrapper;
import org.orekit.rugged.errors.RuggedMessages;
import org.orekit.rugged.linesensor.LineSensor;
import org.orekit.rugged.refining.measures.Observables;
import org.orekit.utils.ParameterDriver;

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

    /** least square optimizer choice.*/
    private OptimizerId optimizerID;


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
        this.optimizerID = OptimizerId.GAUSS_NEWTON_QR;
    }


    /** setter for optimizer algorithm.
     * @param optimizerId the algorithm
     */
    public void setOptimizer(final OptimizerId optimizerId)
    {
        this.optimizerID = optimizerId;
    }

    /**
     * Estimate the free parameters in viewing model to match specified sensor
     * to ground mappings.
     * <p>
     * This method is typically used for calibration of on-board sensor
     * parameters, like rotation angles polynomial coefficients.
     * </p>
     * <p>
     * Before using this method, the {@link ParameterDriver viewing model
     * parameters} retrieved by calling the
     * {@link LineSensor#getParametersDrivers() getParametersDrivers()} method
     * on the desired sensors must be configured. The parameters that should be
     * estimated must have their {@link ParameterDriver#setSelected(boolean)
     * selection status} set to {@link true} whereas the parameters that should
     * retain their current value must have their
     * {@link ParameterDriver#setSelected(boolean) selection status} set to
     * {@link false}. If needed, the {@link ParameterDriver#setValue(double)
     * value} of the estimated/selected parameters can also be changed before
     * calling the method, as this value will serve as the initial value in the
     * estimation process.
     * </p>
     * <p>
     * The method solves a least-squares problem to minimize the residuals
     * between test locations and the reference mappings by adjusting the
     * selected viewing models parameters.
     * </p>
     * <p>
     * The estimated parameters can be retrieved after the method completes by
     * calling again the {@link LineSensor#getParametersDrivers()
     * getParametersDrivers()} method on the desired sensors and checking the
     * updated values of the parameters. In fact, as the values of the
     * parameters are already updated by this method, if users want to use the
     * updated values immediately to perform new direct/inverse locations, they
     * can do so without looking at the parameters: the viewing models are
     * already aware of the updated parameters.
     * </p>
     *
     * @param references reference mappings between sensors pixels and ground
     *        point that should ultimately be reached by adjusting selected
     *        viewing models parameters
     * @param maxEvaluations maximum number of evaluations
     * @param parametersConvergenceThreshold convergence threshold on normalized
     *        parameters (dimensionless, related to parameters scales)
     * @return optimum of the least squares problem
     * @exception RuggedException if several parameters with the same name
     *            exist, if no parameters have been selected for estimation, or
     *            if parameters cannot be estimated (too few measurements,
     *            ill-conditioned problem ...)
     */
    public Optimum estimateFreeParameters(final String RuggedName, final int maxEvaluations, final double parametersConvergenceThreshold)
                    throws RuggedException {
        try {

            final Rugged rugged = this.viewingModel.get(RuggedName);
            if (rugged == null) {
                throw new RuggedException(RuggedMessages.INVALID_RUGGED_NAME);
            }


            final String sensorName = measures.getGroundMapping().getSensorName();

            final List<LineSensor> selectedSensors = new ArrayList<>();
            //TODO loop over all sensors
            //for (final SensorToGroundMapping reference : references) {
            selectedSensors.add(rugged.getLineSensor(sensorName));
            //}

            /** builder */
            final GroundOptimizationProblemBuilder optimizationProblem = new GroundOptimizationProblemBuilder(selectedSensors, measures, rugged);


            final LeastSquareAdjuster adjuster = new LeastSquareAdjuster(this.optimizerID);

            return adjuster.optimize(optimizationProblem.build(maxEvaluations, parametersConvergenceThreshold));


        } catch (RuggedExceptionWrapper rew) {
            throw rew.getException();
        } catch (OrekitExceptionWrapper oew) {
            final OrekitException oe = oew.getException();
            throw new RuggedException(oe, oe.getSpecifier(), oe.getParts());
        }
    }

    /*public Optimum estimateFreeParameters(final String RuggedNameA, final String RuggedNameB, final int maxEvaluations, final float parametersConvergenceThreshold)*/

}