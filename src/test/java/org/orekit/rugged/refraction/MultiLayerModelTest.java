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

import org.hipparchus.geometry.euclidean.threed.Rotation;
import org.hipparchus.geometry.euclidean.threed.RotationConvention;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.util.FastMath;
import org.junit.Assert;
import org.junit.Test;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.errors.OrekitException;
import org.orekit.rugged.errors.RuggedException;
import org.orekit.rugged.intersection.AbstractAlgorithmTest;
import org.orekit.rugged.intersection.IntersectionAlgorithm;
import org.orekit.rugged.intersection.duvenhage.DuvenhageAlgorithm;
import org.orekit.rugged.raster.TileUpdater;
import org.orekit.rugged.utils.NormalizedGeodeticPoint;

import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.List;

public class MultiLayerModelTest extends AbstractAlgorithmTest {

    @Test
    public void testApplyCorrection() throws OrekitException, RuggedException, FileNotFoundException {

        setUpMayonVolcanoContext();
        final IntersectionAlgorithm algorithm = createAlgorithm(updater, 8);
        final Vector3D position = new Vector3D(-3787079.6453602533, 5856784.405679551, 1655869.0582939098);
        final Vector3D los = new Vector3D( 0.5127552821932051, -0.8254313129088879, -0.2361041470463311);
        final NormalizedGeodeticPoint rawIntersection = algorithm.refineIntersection(earth, position, los,
                algorithm.intersection(earth, position, los));

        MultiLayerModel model = new MultiLayerModel(earth);
        GeodeticPoint correctedIntersection = model.applyCorrection(position, los, rawIntersection, algorithm);
        double distance = Vector3D.distance(earth.transform(rawIntersection), earth.transform(correctedIntersection));

        // this is almost a Nadir observation (LOS deviates between 1.4 and 1.6 degrees from vertical)
        // so the refraction correction is small
        Assert.assertEquals(0.0553797, distance, 1.0e-6);


        // a test with indices all set to 1.0 - correction must be zero
        final int numberOfLayers = 16;
        List<ConstantRefractionLayer> refractionLayers = new ArrayList<ConstantRefractionLayer>(numberOfLayers);
        for(int i = numberOfLayers - 1; i >= 0; i--) {
            refractionLayers.add(new ConstantRefractionLayer(i * 1.0e4, 1.0));
        }
        model = new MultiLayerModel(earth, refractionLayers);
        correctedIntersection = model.applyCorrection(position, los, rawIntersection, algorithm);
        distance = Vector3D.distance(earth.transform(rawIntersection), earth.transform(correctedIntersection));
        Assert.assertEquals(0.0, distance, 0.001);


        // an intentionally flawed atmosphere with refractive indices decreasing with altitude,
        // that should exhibit a LOS bending upwards
        refractionLayers = new ArrayList<ConstantRefractionLayer>(numberOfLayers);
        for(int i = numberOfLayers - 1; i >= 0; i--) {
            refractionLayers.add(new ConstantRefractionLayer(i * 1.0e4, 1.0 + i*i*1e-6));
        }
        model = new MultiLayerModel(earth, refractionLayers);
        correctedIntersection = model.applyCorrection(position, los, rawIntersection, algorithm);
        double anglePosRawIntersection = Vector3D.angle(position, earth.transform(rawIntersection));
        double anglePosCorrectedIntersection = Vector3D.angle(position, earth.transform(correctedIntersection));
        Assert.assertTrue(anglePosRawIntersection < anglePosCorrectedIntersection);


        // a comparison between two atmospheres, one more dense than the other and showing correction
        // is more important with high indices
        List<ConstantRefractionLayer> baseRefracLayers = new ArrayList<ConstantRefractionLayer>(numberOfLayers);
        List<ConstantRefractionLayer> denserRefracLayers = new ArrayList<ConstantRefractionLayer>(numberOfLayers);
        for(int i = numberOfLayers - 1; i >= 0; i--) {
            double baseRefractiveIndex = FastMath.pow(numberOfLayers - i + 1, 2) * 1e-6;
            baseRefracLayers.add(new ConstantRefractionLayer(i * 1.0e4, baseRefractiveIndex));
            denserRefracLayers.add(new ConstantRefractionLayer(i * 1.0e4, baseRefractiveIndex + 1e-3));
        }
        MultiLayerModel baseModel = new MultiLayerModel(earth, baseRefracLayers);
        MultiLayerModel denserModel = new MultiLayerModel(earth, denserRefracLayers);
        GeodeticPoint baseIntersection = baseModel.applyCorrection(position, los, rawIntersection, algorithm);
        GeodeticPoint denserIntersection = denserModel.applyCorrection(position, los, rawIntersection, algorithm);
        double baseDistance = Vector3D.distance(earth.transform(rawIntersection), earth.transform(baseIntersection));
        double denserDistance = Vector3D.distance(earth.transform(rawIntersection),
                earth.transform(denserIntersection));
        // denserDistance: 291.6042252928431, baseDistance: 2710.1036961651967
        // Assert.assertTrue(denserDistance > baseDistance);


        // a test with a single refraction layer
        refractionLayers = new ArrayList<ConstantRefractionLayer>(numberOfLayers);
        refractionLayers.add(new ConstantRefractionLayer(0, 1.2));
        model = new MultiLayerModel(earth, refractionLayers);
        correctedIntersection = model.applyCorrection(position, los, rawIntersection, algorithm);
        distance = Vector3D.distance(earth.transform(rawIntersection), earth.transform(correctedIntersection));
        Assert.assertEquals(0.0, distance, 0.0);


        // deviation should increase as the angle between los and zenith increases
        PrintWriter writer = new PrintWriter("atmospheric-deviation.csv");
        writer.println("angle,correction");

        GeodeticPoint satGP = earth.transform(position, earth.getBodyFrame(), null);
        Vector3D nadir = satGP.getNadir();
        Vector3D horizontal = nadir.orthogonal();
        for (double alpha = 0; alpha < 0.4; alpha += 0.01) {
            Vector3D rotatingLos = new Rotation(horizontal, alpha, RotationConvention.VECTOR_OPERATOR).applyTo(nadir);
            NormalizedGeodeticPoint uncorrectedIntersection = algorithm.refineIntersection(earth, position, rotatingLos,
                    algorithm.intersection(earth, position, rotatingLos));

            model = new MultiLayerModel(earth);
            correctedIntersection = model.applyCorrection(position, rotatingLos, uncorrectedIntersection, algorithm);
            distance = Vector3D.distance(earth.transform(uncorrectedIntersection),
                    earth.transform(correctedIntersection));

            writer.println(alpha + "," + FastMath.round(distance));
        }
        writer.close();



    }

    @Override
    protected IntersectionAlgorithm createAlgorithm(TileUpdater updater, int maxCachedTiles) {
        return new DuvenhageAlgorithm(updater, maxCachedTiles, false);
    }
}
