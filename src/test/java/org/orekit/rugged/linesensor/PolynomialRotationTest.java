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
import org.hipparchus.analysis.polynomials.PolynomialFunction;
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
import org.orekit.errors.OrekitException;
import org.orekit.errors.OrekitExceptionWrapper;
import org.orekit.rugged.errors.RuggedException;
import org.orekit.rugged.los.PolynomialRotation;
import org.orekit.rugged.los.LOSBuilder;
import org.orekit.rugged.los.TimeDependentLOS;
import org.orekit.rugged.utils.DSGenerator;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.ParameterDriver;

public class PolynomialRotationTest {

    private List<Vector3D> raw;

    @Test
    public void testIdentity() throws RuggedException, OrekitException {
        UniformRandomGenerator            rng = new UniformRandomGenerator(new Well19937a(0xbe0d9b530fe7f53cl));
        UncorrelatedRandomVectorGenerator rvg = new UncorrelatedRandomVectorGenerator(3, rng);
        for (int k = 0; k < 20; ++k) {
            LOSBuilder builder = new LOSBuilder(raw);
            builder.addTransform(new PolynomialRotation("identity",
                                                   new Vector3D(rvg.nextVector()),
                                                   AbsoluteDate.J2000_EPOCH, 0.0));
            TimeDependentLOS tdl = builder.build();
            for (int i = 0; i < raw.size(); ++i) {
                Assert.assertEquals(0.0,
                                    Vector3D.distance(raw.get(i), tdl.getLOS(i, AbsoluteDate.J2000_EPOCH)),
                                    2.0e-15);
            }

            Assert.assertEquals(1, tdl.getParametersDrivers().count());
            Assert.assertEquals("identity[0]", tdl.getParametersDrivers().findFirst().get().getName());

        }
    }

    @Test
    public void testFixedCombination() throws RuggedException, OrekitException {
        UniformRandomGenerator            rng = new UniformRandomGenerator(new Well19937a(0xdc4cfdea38edd2bbl));
        UncorrelatedRandomVectorGenerator rvg = new UncorrelatedRandomVectorGenerator(3, rng);
        for (int k = 0; k < 20; ++k) {

            LOSBuilder builder = new LOSBuilder(raw);

            Vector3D axis1 = new Vector3D(rvg.nextVector());
            double angle1  = 2 * FastMath.PI * rng.nextNormalizedDouble() / FastMath.sqrt(3);
            builder.addTransform(new PolynomialRotation("r1", axis1, AbsoluteDate.J2000_EPOCH, angle1));
            Rotation r1 = new Rotation(axis1, angle1, RotationConvention.VECTOR_OPERATOR);

            Vector3D axis2 = new Vector3D(rvg.nextVector());
            double angle2  = 2 * FastMath.PI * rng.nextNormalizedDouble() / FastMath.sqrt(3);
            builder.addTransform(new PolynomialRotation("r2", axis2, AbsoluteDate.J2000_EPOCH, angle2));
            Rotation r2 = new Rotation(axis2, angle2, RotationConvention.VECTOR_OPERATOR);

            Vector3D axis3 = new Vector3D(rvg.nextVector());
            double angle3  = 2 * FastMath.PI * rng.nextNormalizedDouble() / FastMath.sqrt(3);
            builder.addTransform(new PolynomialRotation("r3", axis3, AbsoluteDate.J2000_EPOCH, angle3));
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
            Assert.assertEquals("r1[0]", driver1.getName());
            Assert.assertTrue(Double.isInfinite(driver1.getMinValue()));
            Assert.assertTrue(driver1.getMinValue() < 0);
            Assert.assertTrue(Double.isInfinite(driver1.getMaxValue()));
            Assert.assertTrue(driver1.getMaxValue() > 0);
            Assert.assertEquals(angle1, driver1.getValue(), 2.0e-15);
            Assert.assertEquals("r2[0]", driver2.getName());
            Assert.assertTrue(Double.isInfinite(driver2.getMinValue()));
            Assert.assertTrue(driver2.getMinValue() < 0);
            Assert.assertTrue(Double.isInfinite(driver2.getMaxValue()));
            Assert.assertTrue(driver2.getMaxValue() > 0);
            Assert.assertEquals(angle2, driver2.getValue(), 2.0e-15);
            Assert.assertEquals("r3[0]", driver3.getName());
            Assert.assertTrue(Double.isInfinite(driver3.getMinValue()));
            Assert.assertTrue(driver3.getMinValue() < 0);
            Assert.assertTrue(Double.isInfinite(driver3.getMaxValue()));
            Assert.assertTrue(driver3.getMaxValue() > 0);
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
    public void testDerivatives() throws RuggedException, OrekitException {
        UniformRandomGenerator            rng = new UniformRandomGenerator(new Well19937a(0xc60acfc04eb27935l));
        UncorrelatedRandomVectorGenerator rvg = new UncorrelatedRandomVectorGenerator(3, rng);
        for (int k = 0; k < 20; ++k) {

            LOSBuilder builder = new LOSBuilder(raw);

            builder.addTransform(new PolynomialRotation("r1",
                                                   new Vector3D(rvg.nextVector()),
                                                   AbsoluteDate.J2000_EPOCH,
                                                   2 * FastMath.PI * rng.nextNormalizedDouble() / FastMath.sqrt(3),
                                                   1.0e-4 * 2 * FastMath.PI * rng.nextNormalizedDouble() / FastMath.sqrt(3)));
            builder.addTransform(new PolynomialRotation("r2",
                                                   new Vector3D(rvg.nextVector()),
                                                   AbsoluteDate.J2000_EPOCH,
                                                   new PolynomialFunction(new double[] {
                                                       2 * FastMath.PI * rng.nextNormalizedDouble() / FastMath.sqrt(3),
                                                       1.0e-4 * 2 * FastMath.PI * rng.nextNormalizedDouble() / FastMath.sqrt(3),
                                                       1.0e-8 * 2 * FastMath.PI * rng.nextNormalizedDouble() / FastMath.sqrt(3)
                                                   })));
            builder.addTransform(new PolynomialRotation("r3",
                                                   new Vector3D(rvg.nextVector()),
                                                   AbsoluteDate.J2000_EPOCH,
                                                   2 * FastMath.PI * rng.nextNormalizedDouble() / FastMath.sqrt(3),
                                                   1.0e-4 * 2 * FastMath.PI * rng.nextNormalizedDouble() / FastMath.sqrt(3)));
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
            Assert.assertEquals(7, generator.getSelected().size());

            FiniteDifferencesDifferentiator differentiator =
                            new FiniteDifferencesDifferentiator(4, 0.0001);
            int index = 0;
            DSFactory factory11 = new DSFactory(1, 1);
            final AbsoluteDate date = AbsoluteDate.J2000_EPOCH.shiftedBy(7.0);
            for (final ParameterDriver driver : selected) {
                int[] orders = new int[selected.size()];
                orders[index] = 1;
                UnivariateDifferentiableMatrixFunction f =
                                differentiator.differentiate((UnivariateMatrixFunction) x -> {
                                    try {
                                        double oldX = driver.getValue();
                                        double[][] matrix = new double[raw.size()][];
                                        driver.setValue(x);
                                        for (int i = 0 ; i < raw.size(); ++i) {
                                            matrix[i] = tdl.getLOS(i, date).toArray();
                                        }
                                        driver.setValue(oldX);
                                        return matrix;
                                    } catch (OrekitException oe) {
                                        throw new OrekitExceptionWrapper(oe);
                                    }
                                });
                DerivativeStructure[][] mDS = f.value(factory11.variable(0, driver.getValue()));
                for (int i = 0; i < raw.size(); ++i) {
                    Vector3D los = tdl.getLOS(i, date);
                    FieldVector3D<DerivativeStructure> losDS = tdl.getLOSDerivatives(i, date, generator);
                    Assert.assertEquals(los.getX(), losDS.getX().getValue(), 2.0e-15);
                    Assert.assertEquals(los.getY(), losDS.getY().getValue(), 2.0e-15);
                    Assert.assertEquals(los.getZ(), losDS.getZ().getValue(), 2.0e-15);
                    Assert.assertEquals(mDS[i][0].getPartialDerivative(1), losDS.getX().getPartialDerivative(orders), 2.0e-10);
                    Assert.assertEquals(mDS[i][1].getPartialDerivative(1), losDS.getY().getPartialDerivative(orders), 2.0e-10);
                    Assert.assertEquals(mDS[i][2].getPartialDerivative(1), losDS.getZ().getPartialDerivative(orders), 2.0e-10);
                }
                ++index;
            }
        }

    }

    @Before
    public void setUp() throws OrekitException, URISyntaxException {

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
