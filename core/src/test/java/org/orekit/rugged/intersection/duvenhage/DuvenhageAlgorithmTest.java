/* Copyright 2002-2014 CS Systèmes d'Information
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
package org.orekit.rugged.intersection.duvenhage;


import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.util.FastMath;
import org.junit.Assert;
import org.junit.Test;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.errors.OrekitException;
import org.orekit.rugged.api.RuggedException;
import org.orekit.rugged.api.RuggedMessages;
import org.orekit.rugged.intersection.AbstractAlgorithmTest;
import org.orekit.rugged.intersection.IntersectionAlgorithm;
import org.orekit.rugged.raster.TileUpdater;
import org.orekit.rugged.raster.UpdatableTile;

public class DuvenhageAlgorithmTest extends AbstractAlgorithmTest {

    protected IntersectionAlgorithm createAlgorithm(final TileUpdater updater, final int maxCachedTiles) {
        return new DuvenhageAlgorithm(updater, maxCachedTiles, false);
    }

    @Test
    public void testNumericalIssueAtTileExit() throws RuggedException, OrekitException {
        setUpMayonVolcanoContext();
        final IntersectionAlgorithm algorithm = createAlgorithm(updater, 8);
        Vector3D position = new Vector3D(-3787079.6453602533, 5856784.405679551, 1655869.0582939098);
        Vector3D los = new Vector3D( 0.5127552821932051, -0.8254313129088879, -0.2361041470463311);
        GeodeticPoint intersection = algorithm.refineIntersection(earth, position, los,
                                                                  algorithm.intersection(earth, position, los));
        checkIntersection(position, los, intersection);
    }

    @Test
    public void testCrossingBeforeLineSegmentStart() throws RuggedException, OrekitException {
        setUpMayonVolcanoContext();
        final IntersectionAlgorithm algorithm = createAlgorithm(updater, 8);
        Vector3D position = new Vector3D(-3787079.6453602533, 5856784.405679551, 1655869.0582939098);
        Vector3D los = new Vector3D( 0.42804005978915904, -0.8670291034054828, -0.2550338037664377);
        GeodeticPoint intersection = algorithm.refineIntersection(earth, position, los,
                                                                  algorithm.intersection(earth, position, los));
        checkIntersection(position, los, intersection);
    }

    @Test
    public void testWrongPositionMissesGround() throws RuggedException, OrekitException {
        setUpMayonVolcanoContext();
        final IntersectionAlgorithm algorithm = createAlgorithm(updater, 8);
        Vector3D position = new Vector3D(7.551889113912788E9, -3.173692685491814E10, 1.5727517321541348E9);
        Vector3D los = new Vector3D(0.010401349221417867, -0.17836068905951286, 0.9839101973923178);
        try {
            algorithm.intersection(earth, position, los);
            Assert.fail("an exception should have been thrown");
        } catch (RuggedException re) {
            Assert.assertEquals(RuggedMessages.LINE_OF_SIGHT_DOES_NOT_REACH_GROUND, re.getSpecifier());
        }
    }

    @Test
    public void testInconsistentTileUpdater() throws RuggedException, OrekitException {
        final int n = 1201;
        final double size = FastMath.toRadians(1.0);
        updater = new TileUpdater() {
            public void updateTile(double latitude, double longitude, UpdatableTile tile)
                throws RuggedException {
                double step = size / (n - 1);
                // this geometry is incorrect:
                // the specified latitude/longitude belong to rows/columns [1, n-1]
                // and not [0, n-2].
                tile.setGeometry(size * FastMath.floor(latitude / size) - 0.5 * step,
                                 size * FastMath.floor(longitude / size) - 0.5 * step,
                                 step, step, n, n);
                for (int i = 0; i < n; ++i) {
                    for (int j = 0; j < n; ++j) {
                        tile.setElevation(i, j, ((i + j) % 2 == 0) ? -7.0 : 224);
                    }
                }
            }
        };
        final IntersectionAlgorithm algorithm = createAlgorithm(updater, 8);
        try {
            algorithm.intersection(earth,
                                   new Vector3D(-3010311.9672771087, 5307094.8081077365, 1852867.7919871407),
                                   new Vector3D(0.3953329359154183, -0.8654901360032332, -0.30763402650162286));
        } catch (RuggedException re) {
            Assert.assertEquals(RuggedMessages.WRONG_TILE, re.getSpecifier());
        }
    }

}
