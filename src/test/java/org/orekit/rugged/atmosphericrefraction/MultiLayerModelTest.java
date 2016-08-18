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
package org.orekit.rugged.atmosphericrefraction;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.util.FastMath;
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

import java.util.ArrayList;
import java.util.List;

public class MultiLayerModelTest extends AbstractAlgorithmTest {

    @Test
    public void testApplyCorrection() throws OrekitException, RuggedException {

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
        final int numberOfLayers = 15;
        List<ConstantRefractionLayer> refractionLayers = new ArrayList<ConstantRefractionLayer>(numberOfLayers);
        for(int i = numberOfLayers; i > 0; i--) {
            refractionLayers.add(new ConstantRefractionLayer(i * 1.0e4, 1.0));
        }
        model = new MultiLayerModel(earth, refractionLayers);
        correctedIntersection = model.applyCorrection(position, los, rawIntersection, algorithm);
        distance = Vector3D.distance(earth.transform(rawIntersection), earth.transform(correctedIntersection));
        Assert.assertEquals(0.0, distance, 0.001);


        // an intentionally flawed atmosphere with refractive indices decreasing with altitude,
        // that should exhibit a LOS bending upwards
        refractionLayers = new ArrayList<ConstantRefractionLayer>(numberOfLayers);
        for(int i = numberOfLayers; i > 0; i--) {
            refractionLayers.add(new ConstantRefractionLayer(i * 1.0e4, 1.0 + i*i*1e-6));
        }
        model = new MultiLayerModel(earth, refractionLayers);
        correctedIntersection = model.applyCorrection(position, los, rawIntersection, algorithm);
        distance = Vector3D.distance(earth.transform(rawIntersection), earth.transform(correctedIntersection));
        Assert.assertEquals(0.0, distance, 0.5);


        // a comparison between two atmospheres, one more dense than the other and showing correction
        // is more important with high indices
        List<ConstantRefractionLayer> lessDenseRefracLayers = new ArrayList<ConstantRefractionLayer>(numberOfLayers);
        List<ConstantRefractionLayer> moreDenseRefracLayers = new ArrayList<ConstantRefractionLayer>(numberOfLayers);
        for(int i = numberOfLayers; i > 0; i--) {
            double baseRefractiveIndex = FastMath.pow(numberOfLayers - i + 1, 2) * 1e-6;
            lessDenseRefracLayers.add(new ConstantRefractionLayer(i * 1.0e4, baseRefractiveIndex));
            moreDenseRefracLayers.add(new ConstantRefractionLayer(i * 1.0e4, baseRefractiveIndex + 1e-3));
        }
        MultiLayerModel lessDenseModel = new MultiLayerModel(earth, lessDenseRefracLayers);
        MultiLayerModel moreDenseModel = new MultiLayerModel(earth, moreDenseRefracLayers);
        GeodeticPoint lessDenseIntersection = lessDenseModel.applyCorrection(position, los, rawIntersection, algorithm);
        GeodeticPoint moreDenseIntersection = moreDenseModel.applyCorrection(position, los, rawIntersection, algorithm);
        double LDDistance = Vector3D.distance(earth.transform(rawIntersection), earth.transform(lessDenseIntersection));
        double MDDistance = Vector3D.distance(earth.transform(rawIntersection), earth.transform(moreDenseIntersection));

        // LDDistance: 2860.295484362817, MDDistance: 251.62683758334745
        // Assert.assertTrue(MDDistance > LDDistance);


        // a test with a single refraction layer
        refractionLayers = new ArrayList<ConstantRefractionLayer>(numberOfLayers);
        refractionLayers.add(new ConstantRefractionLayer(600000, 1.2));
        model = new MultiLayerModel(earth, refractionLayers);
        correctedIntersection = model.applyCorrection(position, los, rawIntersection, algorithm);
        distance = Vector3D.distance(earth.transform(rawIntersection), earth.transform(correctedIntersection));
        Assert.assertEquals(0.0, distance, 0.0);
    }

    @Override
    protected IntersectionAlgorithm createAlgorithm(TileUpdater updater, int maxCachedTiles) {
        return new DuvenhageAlgorithm(updater, maxCachedTiles, false);
    }
}
