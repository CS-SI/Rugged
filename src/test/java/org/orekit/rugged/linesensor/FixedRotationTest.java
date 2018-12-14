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
package org.orekit.rugged.linesensor;

import java.net.URISyntaxException;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import org.hipparchus.analysis.UnivariateMatrixFunction;
import org.hipparchus.analysis.differentiation.DSFactory;
import org.hipparchus.analysis.differentiation.DerivativeStructure;
import org.hipparchus.analysis.differentiation.FiniteDifferencesDifferentiator;
import org.hipparchus.analysis.differentiation.UnivariateDifferentiableMatrixFunction;
import org.hipparchus.geometry.euclidean.threed.FieldVector3D;
import org.hipparchus.geometry.euclidean.threed.Rotation;
import org.hipparchus.geometry.euclidean.threed.RotationConvention;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.random.UncorrelatedRandomVectorGenerator;
import org.hipparchus.random.UniformRandomGenerator;
import org.hipparchus.random.Well19937a;
import org.hipparchus.util.FastMath;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;
import org.orekit.rugged.los.FixedRotation;
import org.orekit.rugged.los.LOSBuilder;
import org.orekit.rugged.los.TimeDependentLOS;
import org.orekit.rugged.utils.DSGenerator;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.ParameterDriver;

public class FixedRotationTest {

    private List<Vector3D> raw;

    @Test
    public void testIdentity() {
        UniformRandomGenerator            rng = new UniformRandomGenerator(new Well19937a(0xaba71348a77d77cbl));
        UncorrelatedRandomVectorGenerator rvg = new UncorrelatedRandomVectorGenerator(3, rng);
        for (int k = 0; k < 20; ++k) {
            LOSBuilder builder = new LOSBuilder(raw);
            builder.addTransform(new FixedRotation("identity",
                                                   new Vector3D(rvg.nextVector()),
                                                   0.0));
            TimeDependentLOS tdl = builder.build();
            for (int i = 0; i < raw.size(); ++i) {
                Assert.assertEquals(0.0,
                                    Vector3D.distance(raw.get(i), tdl.getLOS(i, AbsoluteDate.J2000_EPOCH)),
                                    2.0e-15);
            }

            Assert.assertEquals(1, tdl.getParametersDrivers().count());
            Assert.assertEquals("identity", tdl.getParametersDrivers().findFirst().get().getName());

        }
    }

    @Test
    public void testCombination() {
        UniformRandomGenerator            rng = new UniformRandomGenerator(new Well19937a(0xefac03d9be4d24b9l));
        UncorrelatedRandomVectorGenerator rvg = new UncorrelatedRandomVectorGenerator(3, rng);
        for (int k = 0; k < 20; ++k) {

            LOSBuilder builder = new LOSBuilder(raw);

            Vector3D axis1 = new Vector3D(rvg.nextVector());
            double angle1  = 2 * FastMath.PI * rng.nextNormalizedDouble() / FastMath.sqrt(3);
            builder.addTransform(new FixedRotation("r1", axis1, angle1));
            Rotation r1 = new Rotation(axis1, angle1, RotationConvention.VECTOR_OPERATOR);

            Vector3D axis2 = new Vector3D(rvg.nextVector());
            double angle2  = 2 * FastMath.PI * rng.nextNormalizedDouble() / FastMath.sqrt(3);
            builder.addTransform(new FixedRotation("r2", axis2, angle2));
            Rotation r2 = new Rotation(axis2, angle2, RotationConvention.VECTOR_OPERATOR);

            Vector3D axis3 = new Vector3D(rvg.nextVector());
            double angle3  = 2 * FastMath.PI * rng.nextNormalizedDouble() / FastMath.sqrt(3);
            builder.addTransform(new FixedRotation("r3", axis3, angle3));
            Rotation r3 = new Rotation(axis3, angle3, RotationConvention.VECTOR_OPERATOR);

            TimeDependentLOS tdl = builder.build();
            Rotation combined = r3.applyTo(r2.applyTo(r1));

            for (int i = 0; i < raw.size(); ++i) {
                Assert.assertEquals(0.0,
                                    Vector3D.distance(combined.applyTo(raw.get(i)),
                                                      tdl.getLOS(i, AbsoluteDate.J2000_EPOCH)),
                                    2.0e-15);
            }

            List<ParameterDriver> drivers = tdl.getParametersDrivers().collect(Collectors.toList());
            Assert.assertEquals(3, drivers.size());
            ParameterDriver driver1 = drivers.get(0);
            ParameterDriver driver2 = drivers.get(1);
            ParameterDriver driver3 = drivers.get(2);
            Assert.assertEquals("r1", driver1.getName());
            Assert.assertEquals(-2 * FastMath.PI, driver1.getMinValue(), 2.0e-15);
            Assert.assertEquals(+2 * FastMath.PI, driver1.getMaxValue(), 2.0e-15);
            Assert.assertEquals(angle1, driver1.getValue(), 2.0e-15);
            Assert.assertEquals("r2", driver2.getName());
            Assert.assertEquals(-2 * FastMath.PI, driver2.getMinValue(), 2.0e-15);
            Assert.assertEquals(+2 * FastMath.PI, driver2.getMaxValue(), 2.0e-15);
            Assert.assertEquals(angle2, driver2.getValue(), 2.0e-15);
            Assert.assertEquals("r3", driver3.getName());
            Assert.assertEquals(-2 * FastMath.PI, driver3.getMinValue(), 2.0e-15);
            Assert.assertEquals(+2 * FastMath.PI, driver3.getMaxValue(), 2.0e-15);
            Assert.assertEquals(angle3, driver3.getValue(), 2.0e-15);

            driver1.setValue(0.0);
            driver2.setValue(0.0);
            driver3.setValue(0.0);

            for (int i = 0; i < raw.size(); ++i) {
                Assert.assertEquals(0.0,
                                    Vector3D.distance(raw.get(i),
                                                      tdl.getLOS(i, AbsoluteDate.J2000_EPOCH)),
                                    2.0e-15);
            }

        }
    }

    @Test
    public void testDerivatives() {
        UniformRandomGenerator            rng = new UniformRandomGenerator(new Well19937a(0xddae2b46b2207e08l));
        UncorrelatedRandomVectorGenerator rvg = new UncorrelatedRandomVectorGenerator(3, rng);
        for (int k = 0; k < 20; ++k) {

            LOSBuilder builder = new LOSBuilder(raw);

            builder.addTransform(new FixedRotation("r1",
                                                   new Vector3D(rvg.nextVector()),
                                                   2 * FastMath.PI * rng.nextNormalizedDouble() / FastMath.sqrt(3)));
            builder.addTransform(new FixedRotation("r2",
                                                   new Vector3D(rvg.nextVector()),
                                                   2 * FastMath.PI * rng.nextNormalizedDouble() / FastMath.sqrt(3)));
            builder.addTransform(new FixedRotation("r3",
                                                   new Vector3D(rvg.nextVector()),
                                                   2 * FastMath.PI * rng.nextNormalizedDouble() / FastMath.sqrt(3)));
            TimeDependentLOS tdl = builder.build();
            final List<ParameterDriver> selected = tdl.getParametersDrivers().collect(Collectors.toList());
            for (final ParameterDriver driver : selected) {
                driver.setSelected(true);
            }
            final DSFactory factoryS = new DSFactory(selected.size(), 1);
            DSGenerator generator = new DSGenerator() {

                /** {@inheritDoc} */
                @Override
                public List<ParameterDriver> getSelected() {
                    return selected;
                }

                /** {@inheritDoc} */
                @Override
                public DerivativeStructure constant(final double value) {
                    return factoryS.constant(value);
                }

                /** {@inheritDoc} */
                @Override
                public DerivativeStructure variable(final ParameterDriver driver) {
                    int index = 0;
                    for (ParameterDriver d : getSelected()) {
                        if (d == driver) {
                            return factoryS.variable(index, driver.getValue());
                        }
                        ++index;
                    }
                    return constant(driver.getValue());
                }

            };
            Assert.assertEquals(3, generator.getSelected().size());

            FiniteDifferencesDifferentiator differentiator =
                            new FiniteDifferencesDifferentiator(4, 0.001);
            int index = 0;
            DSFactory factory11 = new DSFactory(1, 1);
            for (final ParameterDriver driver : selected) {
                int[] orders = new int[selected.size()];
                orders[index] = 1;
                UnivariateDifferentiableMatrixFunction f =
                        differentiator.differentiate((UnivariateMatrixFunction) x -> {
                            double oldX = driver.getValue();
                            double[][] matrix = new double[raw.size()][];
                            driver.setValue(x);
                            for (int i = 0 ; i < raw.size(); ++i) {
                                matrix[i] = tdl.getLOS(i, AbsoluteDate.J2000_EPOCH).toArray();
                            }
                            driver.setValue(oldX);
                            return matrix;
                        });
                DerivativeStructure[][] mDS = f.value(factory11.variable(0, driver.getValue()));
                for (int i = 0; i < raw.size(); ++i) {
                    Vector3D los = tdl.getLOS(i, AbsoluteDate.J2000_EPOCH);
                    FieldVector3D<DerivativeStructure> losDS =
                                    tdl.getLOSDerivatives(i, AbsoluteDate.J2000_EPOCH, generator);
                    Assert.assertEquals(los.getX(), losDS.getX().getValue(), 2.0e-15);
                    Assert.assertEquals(los.getY(), losDS.getY().getValue(), 2.0e-15);
                    Assert.assertEquals(los.getZ(), losDS.getZ().getValue(), 2.0e-15);
                    Assert.assertEquals(mDS[i][0].getPartialDerivative(1), losDS.getX().getPartialDerivative(orders), 2.0e-12);
                    Assert.assertEquals(mDS[i][1].getPartialDerivative(1), losDS.getY().getPartialDerivative(orders), 2.0e-12);
                    Assert.assertEquals(mDS[i][2].getPartialDerivative(1), losDS.getZ().getPartialDerivative(orders), 2.0e-12);
                }
                ++index;
            }
        }

    }

    @Before
    public void setUp() throws URISyntaxException {

        final Vector3D normal    = Vector3D.PLUS_I;
        final Vector3D fovCenter = Vector3D.PLUS_K;
        final Vector3D cross     = Vector3D.crossProduct(normal, fovCenter);

        // build lists of pixels regularly spread on a perfect plane
        raw = new ArrayList<Vector3D>();
        for (int i = -100; i <= 100; ++i) {
            final double alpha = i * 0.17 / 1000;
            raw.add(new Vector3D(FastMath.cos(alpha), fovCenter, FastMath.sin(alpha), cross));
        }

    }

}
