/* Copyright 2013-2020 CS GROUP
 * Licensed to CS GROUP (CS) under one or more
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
package org.orekit.rugged.los;

import java.util.stream.Stream;

import org.hipparchus.analysis.differentiation.Derivative;
import org.hipparchus.geometry.euclidean.threed.FieldVector3D;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.util.FastMath;
import org.orekit.rugged.utils.DerivativeGenerator;
import org.orekit.utils.ParameterDriver;
import org.orekit.utils.ParameterObserver;

/** {@link TimeIndependentLOSTransform LOS transform} based on a homothety along the Z axis.
 * @author Lucie Labatallee
 * @author Guylaine Prat
 * @see LOSBuilder
 * @since 2.0
 */
public class FixedZHomothety implements TimeIndependentLOSTransform {

    /** Parameters scaling factor.
     * <p>
     * We use a power of 2 to avoid numeric noise introduction
     * in the multiplications/divisions sequences.
     * </p>
     */
    private final double SCALE = FastMath.scalb(1.0, 0);

    /** Homothety factor. */
    private double factor;

    /** Underlying homothety with derivatives. */
    private Derivative<?> factorDS;

    /** Driver for homothety factor. */
    private final ParameterDriver factorDriver;

    /** Simple constructor.
     * <p>
     * The single parameter is the homothety factor.
     * </p>
     * @param name name of the homothety (used for estimated parameters identification)
     * @param factorvalue homothety factor
     */
    public FixedZHomothety(final String name, final double factorvalue) {

        this.factor   = factorvalue;
        this.factorDS = null;
        this.factorDriver = new ParameterDriver(name, factorvalue, SCALE, 0, Double.POSITIVE_INFINITY);
        factorDriver.addObserver(new ParameterObserver() {
            @Override
            public void valueChanged(final double previousValue, final ParameterDriver driver) {
                // reset factor to zero, they will be evaluated lazily if needed
                factor = 0.0;
                factorDS = null;
            }
        });
    }

    /** {@inheritDoc} */
    @Override
    public Stream<ParameterDriver> getParametersDrivers() {
        return Stream.of(factorDriver);
    }

    /** {@inheritDoc} */
    @Override
    public Vector3D transformLOS(final int i, final Vector3D los) {

        if (factor == 0.0) {
            // lazy evaluation of the homothety
            factor = factorDriver.getValue();
        }
        return new Vector3D(los.getX(), los.getY(), factor * los.getZ());
    }

    /** {@inheritDoc} */
    @SuppressWarnings("unchecked")
    @Override
    public <T extends Derivative<T>> FieldVector3D<T> transformLOS(final int i, final FieldVector3D<T> los,
                                                                   final DerivativeGenerator<T> generator) {
        final T factorD;
        if (factorDS == null || !factorDS.getField().equals(generator.getField())) {

            // lazy evaluation of the homothety
            factorD = generator.variable(factorDriver);

            // cache evaluated homothety
            factorDS = factorD;

        } else {
            // reuse cached value
            factorD  = (T) factorDS;
        }

        return new FieldVector3D<>(los.getX(), los.getY(), factorD.multiply(los.getZ()));

    }

}
