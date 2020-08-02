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
package org.orekit.rugged.intersection.duvenhage;


import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.util.FastMath;

import static org.junit.Assert.assertEquals;

import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;

import org.junit.Assert;
import org.junit.Test;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.errors.OrekitException;
import org.orekit.rugged.api.AlgorithmId;
import org.orekit.rugged.errors.RuggedException;
import org.orekit.rugged.errors.RuggedMessages;
import org.orekit.rugged.intersection.AbstractAlgorithmTest;
import org.orekit.rugged.intersection.IntersectionAlgorithm;
import org.orekit.rugged.raster.CheckedPatternElevationUpdater;
import org.orekit.rugged.raster.Tile;
import org.orekit.rugged.raster.TileUpdater;
import org.orekit.rugged.raster.UpdatableTile;
import org.orekit.rugged.utils.ExtendedEllipsoid;
import org.orekit.rugged.utils.NormalizedGeodeticPoint;

public class DuvenhageAlgorithmTest extends AbstractAlgorithmTest {

    protected IntersectionAlgorithm createAlgorithm(final TileUpdater updater, final int maxCachedTiles) {
        return new DuvenhageAlgorithm(updater, maxCachedTiles, false);
    }

    @Test
    public void testNumericalIssueAtTileExit() {
        setUpMayonVolcanoContext();
        final IntersectionAlgorithm algorithm = createAlgorithm(updater, 8);
        Vector3D position = new Vector3D(-3787079.6453602533, 5856784.405679551, 1655869.0582939098);
        Vector3D los = new Vector3D( 0.5127552821932051, -0.8254313129088879, -0.2361041470463311);
        GeodeticPoint intersection = algorithm.refineIntersection(earth, position, los,
                                                                  algorithm.intersection(earth, position, los));
        checkIntersection(position, los, intersection);
    }

    @Test
    public void testCrossingBeforeLineSegmentStart() {
        setUpMayonVolcanoContext();
        final IntersectionAlgorithm algorithm = createAlgorithm(updater, 8);
        Vector3D position = new Vector3D(-3787079.6453602533, 5856784.405679551, 1655869.0582939098);
        Vector3D los = new Vector3D( 0.42804005978915904, -0.8670291034054828, -0.2550338037664377);
        GeodeticPoint intersection = algorithm.refineIntersection(earth, position, los,
                                                                  algorithm.intersection(earth, position, los));
        checkIntersection(position, los, intersection);
    }

    @Test
    public void testWrongPositionMissesGround() {
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
    public void testInconsistentTileUpdater() {
        final int n = 1201;
        final double size = FastMath.toRadians(1.0);
        updater = new TileUpdater() {
            public void updateTile(double latitude, double longitude, UpdatableTile tile) {
                
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
            Assert.assertEquals(RuggedMessages.TILE_WITHOUT_REQUIRED_NEIGHBORS_SELECTED, re.getSpecifier());
        }
    }

    @Test
    public void testPureEastWestLOS() {
        updater = new CheckedPatternElevationUpdater(FastMath.toRadians(1.0),1201, 41.0, 1563.0);
        final IntersectionAlgorithm algorithm = createAlgorithm(updater, 8);
        NormalizedGeodeticPoint gp =
            algorithm.intersection(earth,
                                   new Vector3D(-3041185.154503948, 6486750.132281409, -32335.022880173332),
                                   new Vector3D(0.5660218606298548 , -0.8233939240951769, 0.040517885584811814));
        Assert.assertEquals(1164.35, gp.getAltitude(), 0.02);
    }

    @Test
    public void testParallelLOS() {
        double size       = 0.125;
        int    n          = 129;
        double elevation1 = 0.0;
        double elevation2 = 100.0;
        updater = new CheckedPatternElevationUpdater(size, n, elevation1, elevation2);
        MinMaxTreeTile northTile = new MinMaxTreeTileFactory().createTile();
        updater.updateTile((3 * size) / 2, (3 * size) / 2, northTile);
        MinMaxTreeTile southTile = new MinMaxTreeTileFactory().createTile();
        updater.updateTile((-3 * size) / 2, (3 * size) / 2, southTile);
        IntersectionAlgorithm algorithm = createAlgorithm(updater, 8);

        // line of sight in the South West corner
        Assert.assertEquals(northTile.getMinimumLongitude() - 0.0625 * northTile.getLongitudeStep(),
                            findExit(algorithm, northTile,
                                     new Vector3D( 6278799.86896170100,   788574.17965500170,   796074.70414069280),
                                     new Vector3D( 0.09416282233912959,  0.01183204230132312, -0.99548649697728680)).getLongitude(),
                                     1.0e-6);

        // line of sight in the West column
        Assert.assertEquals(northTile.getMinimumLongitude() - 0.0625 * northTile.getLongitudeStep(),
                            findExit(algorithm, northTile,
                                     new Vector3D(6278799.868961701, 788574.17965500171, 796074.7041406928),
                                     new Vector3D(0.09231669916268806, 0.011600067441452849, -0.9956621241621375)).getLongitude(),
                                     1.0e-6);

        // line of sight in the North-West corner
        Assert.assertEquals(northTile.getMinimumLongitude() - 0.0625 * northTile.getLongitudeStep(),
                            findExit(algorithm, northTile,
                                     new Vector3D( 6133039.79342824500,   770267.71434489540,  1568158.38266382620),
                                     new Vector3D(-0.52028845147300570, -0.06537691642830394, -0.85148446025875800)).getLongitude(),
                                     1.0e-6);
        // line of sight in the North-East corner
        Assert.assertEquals(northTile.getMaximumLongitude() + 0.0625 * northTile.getLongitudeStep(),
                            findExit(algorithm, northTile,
                                     new Vector3D( 5988968.17708294100,  1529624.01701343130,  1568158.38266382620),
                                     new Vector3D(-0.93877408645552440, -0.23970837882807683, -0.24747344851359457)).getLongitude(),
                                     1.0e-6);

        // line of sight in the East column
        Assert.assertEquals(northTile.getMaximumLongitude() + 0.0625 * northTile.getLongitudeStep(),
                            findExit(algorithm, northTile,
                                     new Vector3D( 6106093.15406747100,  1559538.54861392200,   979886.66862965740),
                                     new Vector3D(-0.18115090486319424, -0.04625542007869719,  0.98236693031707310)).getLongitude(),
                                     1.0e-6);

        // line of sight in the South-East corner
        Assert.assertEquals(northTile.getMaximumLongitude() + 0.0625 * northTile.getLongitudeStep(),
                            findExit(algorithm, northTile,
                                     new Vector3D( 6131304.19368509600,  1565977.62301751650,   796074.70414069280),
                                     new Vector3D( 0.09195297594530785,  0.02347944953986664, -0.99548649697728530)).getLongitude(),
                                     1.0e-6);

        // line of sight in the South row
        Assert.assertEquals(northTile.getMinimumLatitude() - 0.0625 * northTile.getLatitudeStep(),
                            findExit(algorithm, northTile,
                                     new Vector3D(6251729.731998736, 984354.4963428857, 789526.5774750853),
                                     new Vector3D(-0.15561499277355603, 0.9878177838164719, 0.0)).getLatitude(),
                                     1.0e-6);

        // line of sight in the North row
        Assert.assertEquals(southTile.getMaximumLatitude() + 0.0625 * southTile.getLatitudeStep(),
                            findExit(algorithm, southTile,
                                     new Vector3D(6251729.731998736, 984354.4963428857, -789526.5774750853),
                                     new Vector3D(-0.15561499277355603, 0.9878177838164719, 0.0)).getLatitude(),
                                     1.0e-6);

    }
    
    @Test
    public void testAlgorithmId() {
        setUpMayonVolcanoContext();
 
        final IntersectionAlgorithm algorithm = new DuvenhageAlgorithm(updater, 8, false);
        assertEquals(AlgorithmId.DUVENHAGE, algorithm.getAlgorithmId());
        
        final IntersectionAlgorithm algorithmFlatBody = new DuvenhageAlgorithm(updater, 8, true);
        assertEquals(AlgorithmId.DUVENHAGE_FLAT_BODY, algorithmFlatBody.getAlgorithmId());
    }

    private NormalizedGeodeticPoint findExit(IntersectionAlgorithm algorithm, Tile tile, Vector3D position, Vector3D los) {

        try {
            Method findExit = DuvenhageAlgorithm.class.getDeclaredMethod("findExit",
                                                                         Tile.class,
                                                                         ExtendedEllipsoid.class,
                                                                         Vector3D.class, Vector3D.class);
            findExit.setAccessible(true);
            Object limitPoint = findExit.invoke(algorithm, tile, earth, position, los);
            Class<?> limitPointCls = null;
            for (Class<?> c : DuvenhageAlgorithm.class.getDeclaredClasses()) {
                if (c.getName().endsWith("LimitPoint")) {
                    limitPointCls = c;
                }
            }
            Field pointField = limitPointCls.getDeclaredField("point");
            pointField.setAccessible(true);
            return (NormalizedGeodeticPoint) pointField.get(limitPoint);

        } catch (NoSuchMethodException nsme) {
            Assert.fail(nsme.getLocalizedMessage());
        } catch (SecurityException se) {
            Assert.fail(se.getLocalizedMessage());
        } catch (IllegalAccessException iae) {
            Assert.fail(iae.getLocalizedMessage());
        } catch (IllegalArgumentException iae) {
            Assert.fail(iae.getLocalizedMessage());
        } catch (NoSuchFieldException nsfe) {
            Assert.fail(nsfe.getLocalizedMessage());
        } catch (InvocationTargetException e) {
            if (e.getCause() instanceof RuggedException) {
                throw (RuggedException) e.getCause();
            } else {
                throw (OrekitException) e.getCause();
            }
        }
        return null;
    }

}
