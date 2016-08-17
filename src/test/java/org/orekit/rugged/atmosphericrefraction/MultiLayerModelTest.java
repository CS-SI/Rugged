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
import org.junit.Assert;
import org.junit.Test;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.errors.OrekitException;
import org.orekit.rugged.errors.RuggedException;
import org.orekit.rugged.intersection.AbstractAlgorithmTest;
import org.orekit.rugged.intersection.IntersectionAlgorithm;
import org.orekit.rugged.intersection.duvenhage.DuvenhageAlgorithm;
import org.orekit.rugged.intersection.duvenhage.MinMaxTreeTile;
import org.orekit.rugged.intersection.duvenhage.MinMaxTreeTileFactory;
import org.orekit.rugged.raster.Tile;
import org.orekit.rugged.raster.TileUpdater;
import org.orekit.rugged.raster.TilesCache;
import org.orekit.rugged.utils.NormalizedGeodeticPoint;

public class MultiLayerModelTest extends AbstractAlgorithmTest {

    @Test
    public void testApplyCorrection() throws OrekitException, RuggedException {

        setUpMayonVolcanoContext();
        final IntersectionAlgorithm algorithm = createAlgorithm(updater, 8);
        Vector3D position = new Vector3D(-3787079.6453602533, 5856784.405679551, 1655869.0582939098);
        Vector3D los = new Vector3D( 0.5127552821932051, -0.8254313129088879, -0.2361041470463311);
        NormalizedGeodeticPoint rawIntersection = algorithm.refineIntersection(earth, position, los,
                algorithm.intersection(earth, position, los));

        MultiLayerModel model = new MultiLayerModel(earth);
        GeodeticPoint correctedIntersection = model.applyCorrection(position, los, rawIntersection, algorithm);

        double distance = Vector3D.distance(earth.transform(rawIntersection), earth.transform(correctedIntersection));

        System.out.println("DISTANCE: " + distance);

        // with the current code, this check fails, the distance is about 800m instead of a couple meters
        Assert.assertEquals(0.0, distance, 2.0);
    }

    @Override
    protected IntersectionAlgorithm createAlgorithm(TileUpdater updater, int maxCachedTiles) {
        return new DuvenhageAlgorithm(updater, maxCachedTiles, false);
    }
}
