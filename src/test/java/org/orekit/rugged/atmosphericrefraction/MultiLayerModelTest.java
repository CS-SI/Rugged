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
import org.junit.Test;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.bodies.OneAxisEllipsoid;
import org.orekit.errors.OrekitException;
import org.orekit.frames.FramesFactory;
import org.orekit.rugged.errors.RuggedException;
import org.orekit.rugged.intersection.AbstractAlgorithmTest;
import org.orekit.rugged.intersection.IntersectionAlgorithm;
import org.orekit.rugged.intersection.duvenhage.MinMaxTreeTile;
import org.orekit.rugged.raster.TileUpdater;
import org.orekit.rugged.utils.ExtendedEllipsoid;
import org.orekit.rugged.utils.NormalizedGeodeticPoint;
import org.orekit.utils.Constants;
import org.orekit.utils.IERSConventions;

public class MultiLayerModelTest extends AbstractAlgorithmTest {

    @Test
    public void testGetPointOnGround() throws OrekitException, RuggedException {

//        Vector3D position = new Vector3D(-3787079.6453602533, 5856784.405679551, 1655869.0582939098);
//        Vector3D los = new Vector3D( 0.5127552821932051, -0.8254313129088879, -0.2361041470463311);
//
//        OneAxisEllipsoid oneAxisEllipsoid = new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
//                Constants.WGS84_EARTH_FLATTENING, FramesFactory.getITRF(IERSConventions.IERS_2010, true));
//        ExtendedEllipsoid ellipsoid = new ExtendedEllipsoid(oneAxisEllipsoid.getEquatorialRadius(), ellipsoid.getFlattening(),
//                ellipsoid.getBodyFrame());
//
//
//        // compute intersection with ellipsoid
//        final NormalizedGeodeticPoint gp0 = ellipsoid.pointOnGround(position, los, 0.0);
//        // locate the entry tile along the line-of-sight
//        MinMaxTreeTile tile = cache.getTile(gp0.getLatitude(), gp0.getLongitude());

        // check mayon volcano tiles



        setUpMayonVolcanoContext();
        final IntersectionAlgorithm algorithm = createAlgorithm(updater, 8);
        Vector3D position = new Vector3D(-3787079.6453602533, 5856784.405679551, 1655869.0582939098);
        Vector3D los = new Vector3D( 0.5127552821932051, -0.8254313129088879, -0.2361041470463311);
        GeodeticPoint intersection = algorithm.refineIntersection(earth, position, los,
                algorithm.intersection(earth, position, los));



        MultiLayerModel model = new MultiLayerModel();



        // Vector3D initialPos, Vector3D initialLos, Vector3D initialZenith, double altitude, Tile tile
        // model.getPointOnGround();



    }

    @Override
    protected IntersectionAlgorithm createAlgorithm(TileUpdater updater, int maxCachedTiles) {
        return null;
    }
}
