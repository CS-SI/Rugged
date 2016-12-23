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
package org.orekit.rugged.refraction;

import org.hipparchus.analysis.UnivariateFunction;
import org.hipparchus.geometry.euclidean.threed.Rotation;
import org.hipparchus.geometry.euclidean.threed.RotationConvention;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.util.FastMath;
import org.junit.Assert;
import org.junit.Test;
import org.orekit.errors.OrekitException;
import org.orekit.rugged.errors.RuggedException;
import org.orekit.rugged.errors.RuggedMessages;
import org.orekit.rugged.intersection.AbstractAlgorithmTest;
import org.orekit.rugged.intersection.IntersectionAlgorithm;
import org.orekit.rugged.intersection.duvenhage.DuvenhageAlgorithm;
import org.orekit.rugged.raster.TileUpdater;
import org.orekit.rugged.utils.NormalizedGeodeticPoint;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class MultiLayerModelTest extends AbstractAlgorithmTest {

    @Test
    public void testAlmostNadir() throws OrekitException, RuggedException {

        setUpMayonVolcanoContext();
        final IntersectionAlgorithm algorithm = createAlgorithm(updater, 8);
        final Vector3D position = new Vector3D(-3787079.6453602533, 5856784.405679551, 1655869.0582939098);
        final Vector3D los = new Vector3D( 0.5127552821932051, -0.8254313129088879, -0.2361041470463311);
        final NormalizedGeodeticPoint rawIntersection =
                        algorithm.refineIntersection(earth, position, los,
                                                     algorithm.intersection(earth, position, los));

        MultiLayerModel model = new MultiLayerModel(earth);
        NormalizedGeodeticPoint correctedIntersection = model.applyCorrection(position, los, rawIntersection, algorithm);
        double distance = Vector3D.distance(earth.transform(rawIntersection), earth.transform(correctedIntersection));

        // this is almost a Nadir observation (LOS deviates between 1.4 and 1.6 degrees from vertical)
        // so the refraction correction is small
        Assert.assertEquals(0.0553796, distance, 1.0e-6);

    }

    @Test
    public void testNoOpRefraction() throws OrekitException, RuggedException {

        setUpMayonVolcanoContext();
        final IntersectionAlgorithm   algorithm       = createAlgorithm(updater, 8);
        final Vector3D                position        = new Vector3D(-3787079.6453602533, 5856784.405679551, 1655869.0582939098);
        final Vector3D                los             = los(position, FastMath.toRadians(50.0));
        final NormalizedGeodeticPoint rawIntersection = algorithm.refineIntersection(earth, position, los,
                                                                                     algorithm.intersection(earth, position, los));

        MultiLayerModel model = new MultiLayerModel(earth);
        NormalizedGeodeticPoint correctedIntersection = model.applyCorrection(position, los, rawIntersection, algorithm);
        double distance = Vector3D.distance(earth.transform(rawIntersection), earth.transform(correctedIntersection));

        // a test with indices all set to 1.0 - correction must be zero
        final int numberOfLayers = 16;
        List<ConstantRefractionLayer> refractionLayers = new ArrayList<ConstantRefractionLayer>(numberOfLayers);
        for(int i = numberOfLayers - 1; i >= 0; i--) {
            refractionLayers.add(new ConstantRefractionLayer(i * 1.0e4, 1.0));
        }
        model = new MultiLayerModel(earth, refractionLayers);
        correctedIntersection = model.applyCorrection(position, los, rawIntersection, algorithm);
        distance = Vector3D.distance(earth.transform(rawIntersection), earth.transform(correctedIntersection));
        Assert.assertEquals(0.0, distance, 1.7e-9);

    }

    @Test
    public void testReversedAtmosphere()
        throws OrekitException, RuggedException, IllegalArgumentException,
               IllegalAccessException, NoSuchFieldException, SecurityException {

        setUpMayonVolcanoContext();
        final IntersectionAlgorithm   algorithm       = createAlgorithm(updater, 8);
        final Vector3D                position        = new Vector3D(-3787079.6453602533, 5856784.405679551, 1655869.0582939098);
        final Vector3D                los             = los(position, FastMath.toRadians(50.0));
        final NormalizedGeodeticPoint rawIntersection = algorithm.refineIntersection(earth, position, los,
                                                                                     algorithm.intersection(earth, position, los));

        MultiLayerModel baseModel = new MultiLayerModel(earth);
        NormalizedGeodeticPoint correctedIntersection = baseModel.applyCorrection(position, los, rawIntersection, algorithm);

        // an intentionally flawed atmosphere with refractive indices decreasing with altitude,
        // that should exhibit a LOS bending upwards
        Field refractionLayersField = MultiLayerModel.class.getDeclaredField("refractionLayers");
        refractionLayersField.setAccessible(true);
        @SuppressWarnings("unchecked")
        List<ConstantRefractionLayer> baseRefractionLayers =
                        (List<ConstantRefractionLayer>) refractionLayersField.get(baseModel);
        List<ConstantRefractionLayer> denserRefractionLayers = new ArrayList<>();
        for (final ConstantRefractionLayer layer : baseRefractionLayers) {
            denserRefractionLayers.add(new ConstantRefractionLayer(layer.getLowestAltitude(),
                                                               1.0 / layer.getRefractiveIndex()));
        }
        MultiLayerModel reversedModel = new MultiLayerModel(earth, denserRefractionLayers);
        NormalizedGeodeticPoint reversedIntersection = reversedModel.applyCorrection(position, los, rawIntersection, algorithm);
        double anglePosRawIntersection       = Vector3D.angle(position, earth.transform(rawIntersection));
        double anglePosCorrectedIntersection = Vector3D.angle(position, earth.transform(correctedIntersection));
        double anglePosReversedIntersection  = Vector3D.angle(position, earth.transform(reversedIntersection));

        // with regular atmosphere, the ray bends downwards,
        // so the ground point is closer to the sub-satellite point than the raw intersection
        Assert.assertTrue(anglePosCorrectedIntersection < anglePosRawIntersection);

        // with reversed atmosphere, the ray bends upwards,
        // so the ground point is farther from the sub-satellite point than the raw intersection
        Assert.assertTrue(anglePosReversedIntersection > anglePosRawIntersection);

        // the points are almost aligned (for distances around 20m, Earth curvature is small enough)
        double dRawCorrected      = Vector3D.distance(earth.transform(rawIntersection), earth.transform(correctedIntersection));
        double dRawReversed       = Vector3D.distance(earth.transform(rawIntersection), earth.transform(reversedIntersection));
        double dReversedCorrected = Vector3D.distance(earth.transform(reversedIntersection), earth.transform(correctedIntersection));
        Assert.assertEquals(dRawCorrected + dRawReversed, dReversedCorrected, 1.0e-12 * dReversedCorrected);

    }

    @Test
    public void testTwoAtmospheres()
        throws OrekitException, RuggedException, IllegalArgumentException,
               IllegalAccessException, NoSuchFieldException, SecurityException {

        setUpMayonVolcanoContext();
        final IntersectionAlgorithm   algorithm       = createAlgorithm(updater, 8);
        final Vector3D                position        = new Vector3D(-3787079.6453602533, 5856784.405679551, 1655869.0582939098);
        final Vector3D                los             = los(position, FastMath.toRadians(50.0));
        final NormalizedGeodeticPoint rawIntersection = algorithm.refineIntersection(earth, position, los,
                                                                                     algorithm.intersection(earth, position, los));

        // a comparison between two atmospheres, one more dense than the other and showing correction
        // is more important with high indices
        MultiLayerModel baseModel = new MultiLayerModel(earth);
        Field refractionLayersField = MultiLayerModel.class.getDeclaredField("refractionLayers");
        refractionLayersField.setAccessible(true);
        @SuppressWarnings("unchecked")
        List<ConstantRefractionLayer> baseRefractionLayers =
                        (List<ConstantRefractionLayer>) refractionLayersField.get(baseModel);
        List<ConstantRefractionLayer> denserRefractionLayers = new ArrayList<>();
        double previousBaseN   = 1.0;
        double previousDenserN = 1.0;
        double factor          = 1.00001;
        for (final ConstantRefractionLayer layer : baseRefractionLayers) {
            final double currentBaseN   = layer.getRefractiveIndex();
            final double baseRatio      = currentBaseN / previousBaseN;
            final double currentDenserN = previousDenserN * factor * baseRatio;
            denserRefractionLayers.add(new ConstantRefractionLayer(layer.getLowestAltitude(),
                                                                   currentDenserN));
            previousBaseN   = currentBaseN;
            previousDenserN = currentDenserN;
        }
        MultiLayerModel denserModel = new MultiLayerModel(earth, denserRefractionLayers);
        NormalizedGeodeticPoint baseIntersection   = baseModel.applyCorrection(position, los, rawIntersection, algorithm);
        NormalizedGeodeticPoint denserIntersection = denserModel.applyCorrection(position, los, rawIntersection, algorithm);
        double baseDistance   = Vector3D.distance(earth.transform(rawIntersection),
                                                  earth.transform(baseIntersection));
        double denserDistance = Vector3D.distance(earth.transform(rawIntersection),
                                                  earth.transform(denserIntersection));
        Assert.assertTrue(denserDistance > baseDistance);

    }

    @Test
    public void testMissingLayers() throws OrekitException, RuggedException {

        setUpMayonVolcanoContext();
        final IntersectionAlgorithm   algorithm       = createAlgorithm(updater, 8);
        final Vector3D                position        = new Vector3D(-3787079.6453602533, 5856784.405679551, 1655869.0582939098);
        final Vector3D                los             = los(position, FastMath.toRadians(50.0));
        final NormalizedGeodeticPoint rawIntersection = algorithm.refineIntersection(earth, position, los,
                                                                                     algorithm.intersection(earth, position, los));
        final double h = rawIntersection.getAltitude();

        MultiLayerModel model = new MultiLayerModel(earth,
                                                    Collections.singletonList(new ConstantRefractionLayer(h + 100.0,
                                                                                                          1.5)));
        try {
            model.applyCorrection(position, los, rawIntersection, algorithm);
            Assert.fail("an exception should have been thrown");
        } catch (RuggedException re) {
            Assert.assertEquals(RuggedMessages.NO_LAYER_DATA, re.getSpecifier());
            Assert.assertEquals(h,         ((Double) re.getParts()[0]).doubleValue(), 1.0e-6);
            Assert.assertEquals(h + 100.0, ((Double) re.getParts()[1]).doubleValue(), 1.0e-6);
        }

    }

    @Test
    public void testLayersBelowDEM() throws OrekitException, RuggedException {

        setUpMayonVolcanoContext();
        final IntersectionAlgorithm   algorithm       = createAlgorithm(updater, 8);
        final Vector3D                position        = new Vector3D(-3787079.6453602533, 5856784.405679551, 1655869.0582939098);
        final Vector3D                los             = los(position, FastMath.toRadians(50.0));
        final NormalizedGeodeticPoint rawIntersection = algorithm.refineIntersection(earth, position, los,
                                                                                     algorithm.intersection(earth, position, los));

        MultiLayerModel model = new MultiLayerModel(earth,
                                                    Collections.singletonList(new ConstantRefractionLayer(rawIntersection.getAltitude() - 100.0,
                                                                                                          1.5)));
        NormalizedGeodeticPoint correctedIntersection = model.applyCorrection(position, los, rawIntersection, algorithm);
        double distance = Vector3D.distance(earth.transform(rawIntersection), earth.transform(correctedIntersection));
        Assert.assertEquals(0.0, distance, 1.3e-9);

    }

    @Test
    public void testDivingAngleChange() throws OrekitException, RuggedException {

        setUpMayonVolcanoContext();
        final IntersectionAlgorithm algorithm = createAlgorithm(updater, 8);
        final Vector3D position = new Vector3D(-3787079.6453602533, 5856784.405679551, 1655869.0582939098);
        AtmosphericRefraction model = new MultiLayerModel(earth);

        // deviation should increase from 0 to about 17m
        // as the angle between los and nadir increases from 0 to 50 degrees
        // the reference model below has been obtained by fitting the test results themselves
        // it is NOT considered a full featured model, it's just a helper function for this specific test
        UnivariateFunction reference = alpha -> 1.17936 * FastMath.tan((2.94613 - 1.40162 * alpha) * alpha);

        for (double alpha = 0; alpha < FastMath.toRadians(50.0); alpha += 0.1) {
            final Vector3D rotatingLos = los(position, alpha);
            final NormalizedGeodeticPoint rawIntersection = algorithm.refineIntersection(earth, position, rotatingLos,
                                                                                         algorithm.intersection(earth, position, rotatingLos));

            final NormalizedGeodeticPoint correctedIntersection = model.applyCorrection(position, rotatingLos, rawIntersection, algorithm);
            final double distance = Vector3D.distance(earth.transform(rawIntersection),
                                                      earth.transform(correctedIntersection));
            Assert.assertEquals(reference.value(alpha), distance, 0.12);
        }

    }

    private Vector3D los(final Vector3D position, final double angleFromNadir)
        throws OrekitException {
        final Vector3D nadir       = earth.transform(position, earth.getBodyFrame(), null).getNadir();
        final Rotation losRotation = new Rotation(nadir.orthogonal(), angleFromNadir,
                                                  RotationConvention.VECTOR_OPERATOR);
        return losRotation.applyTo(nadir);
    }

    @Override
    protected IntersectionAlgorithm createAlgorithm(TileUpdater updater, int maxCachedTiles) {
        return new DuvenhageAlgorithm(updater, maxCachedTiles, false);
    }

}
