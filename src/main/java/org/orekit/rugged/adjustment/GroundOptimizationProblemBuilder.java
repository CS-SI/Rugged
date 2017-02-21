package org.orekit.rugged.adjustment;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.hipparchus.analysis.differentiation.DerivativeStructure;
import org.hipparchus.linear.Array2DRowRealMatrix;
import org.hipparchus.linear.ArrayRealVector;
import org.hipparchus.linear.RealMatrix;
import org.hipparchus.linear.RealVector;
import org.hipparchus.optim.ConvergenceChecker;
import org.hipparchus.optim.nonlinear.vector.leastsquares.LeastSquaresBuilder;
import org.hipparchus.optim.nonlinear.vector.leastsquares.LeastSquaresProblem;
import org.hipparchus.optim.nonlinear.vector.leastsquares.MultivariateJacobianFunction;
import org.hipparchus.optim.nonlinear.vector.leastsquares.ParameterValidator;
import org.hipparchus.util.FastMath;
import org.hipparchus.util.Pair;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.errors.OrekitException;
import org.orekit.errors.OrekitExceptionWrapper;
import org.orekit.rugged.api.Rugged;
import org.orekit.rugged.errors.RuggedException;
import org.orekit.rugged.errors.RuggedExceptionWrapper;
import org.orekit.rugged.errors.RuggedMessages;
import org.orekit.rugged.linesensor.LineSensor;
import org.orekit.rugged.linesensor.SensorPixel;
import org.orekit.rugged.refining.measures.Observables;
import org.orekit.rugged.refining.measures.SensorToGroundMapping;
import org.orekit.utils.ParameterDriver;



public class GroundOptimizationProblemBuilder
extends OptimizationProblemBuilder {

    /** rugged instance to refine.*/
    private final Rugged rugged;

    /** sensorToGround mapping to generate target tab for optimization.*/
    private List<SensorToGroundMapping> sensorToGroundMappings;

    /** minimum line for inverse location estimation.*/
    private int minLine;

    /** maximum line for inverse location estimation.*/
    private int maxLine;

    private HashMap<String, double[] > targetAndWeight;

    /**
     * @param sensors list of sensors to refine
     * @param measures set of observables
     * @param rugged name of rugged to refine
     * @throws RuggedException an exception is generated if no parameters has been selected for refining
     */
    public GroundOptimizationProblemBuilder(final List<LineSensor> sensors,
                                            final Observables measures, final Rugged rugged)
                                                            throws RuggedException {
        super(sensors, measures);
        this.rugged = rugged;
        this.sensorToGroundMappings = new ArrayList<SensorToGroundMapping>();
        for (Map.Entry<String, SensorToGroundMapping> entry : this.measures.getGroundMappings()) {
            this.sensorToGroundMappings.add(entry.getValue());
        }

    }


    @Override
    protected void createTargetAndWeight()
                    throws RuggedException {
        try {
            int n = 0;
            for (final SensorToGroundMapping reference : this.sensorToGroundMappings) {
                n += reference.getMapping().size();
            }

            if (n == 0) {
                throw new RuggedException(RuggedMessages.NO_REFERENCE_MAPPINGS);
            }
            final double[] target = new double[2 * n];
            final double[] weight = new double[2 * n];

            double min = Double.POSITIVE_INFINITY;
            double max = Double.NEGATIVE_INFINITY;
            int k = 0;

            for (final SensorToGroundMapping reference : this.sensorToGroundMappings) {
                for (final Map.Entry<SensorPixel, GeodeticPoint> mapping : reference.getMapping()) {
                    final SensorPixel sp = mapping.getKey();
                    weight[k] = 1.0;
                    target[k++] = sp.getLineNumber();
                    weight[k] = 1.0;
                    target[k++] = sp.getPixelNumber();
                    min = FastMath.min(min, sp.getLineNumber());
                    max = FastMath.max(max, sp.getLineNumber());
                }
            }

            this.minLine = (int) FastMath
                            .floor(min - ESTIMATION_LINE_RANGE_MARGIN);
            this.maxLine = (int) FastMath
                            .ceil(max - ESTIMATION_LINE_RANGE_MARGIN);
            this.targetAndWeight = new HashMap<String, double[]>();
            this.targetAndWeight.put("Target", target);
            this.targetAndWeight.put("Weight", weight);

        } catch  (RuggedExceptionWrapper rew) {
            throw rew.getException();
        }
    }

    @Override
    protected MultivariateJacobianFunction createFunction()
    {
        // model function
        final MultivariateJacobianFunction model = point -> {
            try {

                // set the current parameters values
                int i = 0;
                for (final ParameterDriver driver : this.drivers.getDrivers()) {
                    driver.setNormalizedValue(point.getEntry(i++));
                }

                final double[] target = this.targetAndWeight.get("Target");

                // compute inverse loc and its partial derivatives
                final RealVector value = new ArrayRealVector(target.length);
                final RealMatrix jacobian = new Array2DRowRealMatrix(target.length, this.nbParams);
                int l = 0;
                for (final SensorToGroundMapping reference : this.sensorToGroundMappings) {
                    for (final Map.Entry<SensorPixel, GeodeticPoint> mapping : reference.getMapping()) {
                        final GeodeticPoint gp = mapping.getValue();
                        final DerivativeStructure[] ilResult = this.rugged.inverseLocationDerivatives(reference.getSensorName(), gp, minLine, maxLine, generator);

                        if (ilResult == null) {
                            value.setEntry(l, minLine - 100.0); // arbitrary
                            // line far
                            // away
                            value.setEntry(l + 1, -100.0); // arbitrary
                            // pixel far away
                        } else {
                            // extract the value
                            value.setEntry(l, ilResult[0].getValue());
                            value.setEntry(l + 1, ilResult[1].getValue());

                            // extract the Jacobian
                            final int[] orders = new int[this.nbParams];
                            int m = 0;
                            for (final ParameterDriver driver : this.drivers.getDrivers()) {
                                final double scale = driver.getScale();
                                orders[m] = 1;
                                jacobian.setEntry(l, m,
                                                  ilResult[0]
                                                                  .getPartialDerivative(orders) *
                                                                  scale);
                                jacobian.setEntry(l + 1, m,
                                                  ilResult[1]
                                                                  .getPartialDerivative(orders) *
                                                                  scale);
                                orders[m] = 0;
                                m++;
                            }
                        }

                        l += 2;

                    }
                }


                // inverse loc result with Jacobian for all reference points
                return new Pair<RealVector, RealMatrix>(value, jacobian);

            } catch (RuggedException re) {
                throw new RuggedExceptionWrapper(re);
            } catch (OrekitException oe) {
                throw new OrekitExceptionWrapper(oe);
            }
        };
        return model;
    }


    /** leastsquare problem builder.
     * @param maxEvaluations maxIterations and evaluations
     * @param convergenceThreshold parameter convergence threshold
     * @throws RuggedException if sensor is not found
     * @return
     */
    @Override
    public final LeastSquaresProblem build(final int maxEvaluations, final double convergenceThreshold) throws RuggedException {

        this.createTargetAndWeight();
        final double[] target = this.targetAndWeight.get("Target");
        final double[] start = this.createStartTab();
        final ParameterValidator validator = this.createParameterValidator();
        final ConvergenceChecker<LeastSquaresProblem.Evaluation> checker = this.createChecker(convergenceThreshold);
        final MultivariateJacobianFunction model = this.createFunction();
        return new LeastSquaresBuilder()
                        .lazyEvaluation(false).maxIterations(maxEvaluations)
                        .maxEvaluations(maxEvaluations).weight(null).start(start)
                        .target(target).parameterValidator(validator).checker(checker)
                        .model(model).build();
    }

}
