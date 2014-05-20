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
package org.orekit.rugged.core;


import java.io.File;
import java.net.URISyntaxException;

import org.apache.commons.math3.geometry.euclidean.threed.Line;
import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.util.FastMath;
import org.junit.After;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;
import org.orekit.attitudes.Attitude;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.data.DataProvidersManager;
import org.orekit.data.DirectoryCrawler;
import org.orekit.errors.OrekitException;
import org.orekit.frames.FramesFactory;
import org.orekit.frames.Transform;
import org.orekit.orbits.CartesianOrbit;
import org.orekit.propagation.SpacecraftState;
import org.orekit.rugged.api.RuggedException;
import org.orekit.rugged.api.TileUpdater;
import org.orekit.rugged.core.duvenhage.MinMaxTreeTile;
import org.orekit.rugged.core.duvenhage.MinMaxTreeTileFactory;
import org.orekit.rugged.raster.CliffsElevationUpdater;
import org.orekit.rugged.raster.VolcanicConeElevationUpdater;
import org.orekit.rugged.raster.IntersectionAlgorithm;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeScalesFactory;
import org.orekit.utils.Constants;
import org.orekit.utils.IERSConventions;
import org.orekit.utils.PVCoordinates;

public abstract class AbstractAlgorithmTest {

    protected abstract IntersectionAlgorithm createAlgorithm(TileUpdater updater, int maxCachedTiles);

    @Test
    public void testMayonVolcanoOnSubTileCorner()
        throws RuggedException, OrekitException {

        setUpMayonVolcanoContext();

        // test point approximately 1.6km North-North-West and 800 meters below volcano summit
        // note that this test point is EXACTLY at a pixel corner, and even at corners of
        // middle level (12 and above) sub-tiles
        final double latitude  = FastMath.toRadians(13.27);
        final double longitude = FastMath.toRadians(123.68);
        MinMaxTreeTile tile = new MinMaxTreeTileFactory().createTile();
        updater.updateTile(latitude, longitude, tile);
        double altitude = tile.interpolateElevation(latitude, longitude);
        final GeodeticPoint groundGP = new GeodeticPoint(latitude, longitude, altitude);
        Vector3D groundP = earth.transform(groundGP);

        final IntersectionAlgorithm algorithm = createAlgorithm(updater, 8);

        // preliminary check: the point has been chosen in the spacecraft (YZ) plane
        Transform earthToSpacecraft = new Transform(state.getDate(),
                                                    earth.getBodyFrame().getTransformTo(state.getFrame(), state.getDate()),
                                                    state.toTransform());
        Vector3D pointInSpacecraftFrame = earthToSpacecraft.transformPosition(groundP);
        Assert.assertEquals(     0.000, pointInSpacecraftFrame.getX(), 1.0e-3);
        Assert.assertEquals(-87754.914, pointInSpacecraftFrame.getY(), 1.0e-3);
        Assert.assertEquals(790330.254, pointInSpacecraftFrame.getZ(), 1.0e-3);

        // test direct localization
        Vector3D      position = state.getPVCoordinates(earth.getBodyFrame()).getPosition();
        Vector3D      los      = groundP.subtract(position);
        GeodeticPoint result   = algorithm.refineIntersection(earth, position, los,
                                                              algorithm.intersection(earth, position, los));
        checkIntersection(position, los, result);
        Assert.assertEquals(0.0, groundP.distance(earth.transform(result)), 2.0e-9);

    }

    @Test
    public void testMayonVolcanoWithinPixel()
        throws RuggedException, OrekitException {

        setUpMayonVolcanoContext();

        final double latitude  = FastMath.toRadians(13.2696);
        final double longitude = FastMath.toRadians(123.6803);
        MinMaxTreeTile tile = new MinMaxTreeTileFactory().createTile();
        updater.updateTile(latitude, longitude, tile);
        double altitude = tile.interpolateElevation(latitude, longitude);
        final GeodeticPoint groundGP = new GeodeticPoint(latitude, longitude, altitude);
        Vector3D groundP = earth.transform(groundGP);

        final IntersectionAlgorithm algorithm = createAlgorithm(updater, 8);

        // test direct localization
        Vector3D      position = state.getPVCoordinates(earth.getBodyFrame()).getPosition();
        Vector3D      los      = groundP.subtract(position);
        GeodeticPoint result   = algorithm.refineIntersection(earth, position, los,
                                                              algorithm.intersection(earth, position, los));
        checkIntersection(position, los, result);
        Assert.assertEquals(0.0, groundP.distance(earth.transform(result)), 2.0e-9);

    }

    @Test
    public void testCliffsOfMoher()
        throws RuggedException, OrekitException {

        setUpCliffsOfMoherContext();

        // test point on top the cliffs, roughly 15m East of edge (inland)
        final double latitude  = FastMath.toRadians(52.98045);
        final double longitude = FastMath.toRadians(-9.421826199814143);
        MinMaxTreeTile tile = new MinMaxTreeTileFactory().createTile();
        updater.updateTile(latitude, longitude, tile);
        double altitude = tile.interpolateElevation(latitude, longitude);
        final GeodeticPoint groundGP = new GeodeticPoint(latitude, longitude, altitude);
        Vector3D groundP = earth.transform(groundGP);

        final IntersectionAlgorithm algorithm = createAlgorithm(updater, 8);

        // preliminary check: the point has been chosen in the spacecraft (YZ) plane
        Transform earthToSpacecraft = new Transform(state.getDate(),
                                                    earth.getBodyFrame().getTransformTo(state.getFrame(), state.getDate()),
                                                    state.toTransform());
        Vector3D pointInSpacecraftFrame = earthToSpacecraft.transformPosition(groundP);
        Assert.assertEquals(     0.000, pointInSpacecraftFrame.getX(), 1.0e-3);
        Assert.assertEquals( 66702.419, pointInSpacecraftFrame.getY(), 1.0e-3);
        Assert.assertEquals(796873.178, pointInSpacecraftFrame.getZ(), 1.0e-3);

        Vector3D      position = state.getPVCoordinates(earth.getBodyFrame()).getPosition();
        Vector3D      los      = groundP.subtract(position);
        GeodeticPoint result   = algorithm.refineIntersection(earth, position, los,
                                                              algorithm.intersection(earth, position, los));
        checkIntersection(position, los, result);
        Assert.assertEquals(0.0, groundP.distance(earth.transform(result)), 2.0e-9);

    }

    protected void checkIntersection(Vector3D position, Vector3D los, GeodeticPoint intersection)
        throws RuggedException {

        // check the point is on the line
        Line line = new Line(position, new Vector3D(1, position, 1e6, los), 1.0e-12);
        Assert.assertEquals(0.0, line.distance(earth.transform(intersection)), 3.0e-9);

        // check the point is on the Digital Elevation Model
        MinMaxTreeTile tile = new MinMaxTreeTileFactory().createTile();
        updater.updateTile(intersection.getLatitude(), intersection.getLongitude(), tile);
        Assert.assertEquals(tile.interpolateElevation(intersection.getLatitude(), intersection.getLongitude()),
                            intersection.getAltitude(), 2.0e-9);

    }

    protected void setUpMayonVolcanoContext()
        throws RuggedException, OrekitException {

        // Mayon Volcano location according to Wikipedia: 13°15′24″N 123°41′6″E
        GeodeticPoint summit =
                new GeodeticPoint(FastMath.toRadians(13.25667), FastMath.toRadians(123.685), 2463.0);
        updater = new VolcanicConeElevationUpdater(summit,
                                                   FastMath.toRadians(30.0), 16.0,
                                                   FastMath.toRadians(1.0), 1201);

        // some orbital parameters have been computed using Orekit
        // tutorial about phasing, using the following configuration:
        //
        //  orbit.date                          = 2012-01-01T00:00:00.000
        //  phasing.orbits.number               = 143
        //  phasing.days.number                 =  10
        //  sun.synchronous.reference.latitude  = 0
        //  sun.synchronous.reference.ascending = false
        //  sun.synchronous.mean.solar.time     = 10:30:00
        //  gravity.field.degree                = 12
        //  gravity.field.order                 = 12
        //
        // the resulting phased orbit has then been propagated to a date corresponding
        // to test point lying in the spacecraft (YZ) plane (with nadir pointing and yaw compensation)
        AbsoluteDate crossing = new AbsoluteDate("2012-01-06T02:27:15.942757185", TimeScalesFactory.getUTC());
        state = new SpacecraftState(new CartesianOrbit(new PVCoordinates(new Vector3D( -649500.423763743,
                                                                                       -6943715.537565755,
                                                                                       1657929.137063380),
                                                                         new Vector3D(-1305.453711368668,
                                                                                      -1600.627551928136,
                                                                                      -7167.286855869801)),
                                                       FramesFactory.getEME2000(),
                                                       crossing,
                                                       Constants.EIGEN5C_EARTH_MU),
                                                       new Attitude(crossing,
                                                                    FramesFactory.getEME2000(),
                                                                    new Rotation(-0.40904880353552850,
                                                                                  0.46125295378582530,
                                                                                 -0.63525007056319790,
                                                                                 -0.46516893361386025,
                                                                                 true),
                                                                    new Vector3D(-7.048568391860185e-05,
                                                                                 -1.043582650222194e-03,
                                                                                  1.700400341147713e-05)));

    }

    protected void setUpCliffsOfMoherContext()
        throws RuggedException, OrekitException {

        // cliffs of Moher location according to Wikipedia: 52°56′10″N 9°28′15″ W
        GeodeticPoint north = new GeodeticPoint(FastMath.toRadians(52.9984),
                                                FastMath.toRadians(-9.4072),
                                                120.0);
        GeodeticPoint south = new GeodeticPoint(FastMath.toRadians(52.9625),
                                                FastMath.toRadians(-9.4369),
                                                120.0);

        // pixels are about 10m x 10m here and a tile covers 1km x 1km
        updater = new CliffsElevationUpdater(north, south,
                                             120.0, 0.0,
                                             FastMath.toRadians(0.015), 101);

        // some orbital parameters have been computed using Orekit
        // tutorial about phasing, using the following configuration:
        //
        //  orbit.date                          = 2012-01-01T00:00:00.000
        //  phasing.orbits.number               = 143
        //  phasing.days.number                 =  10
        //  sun.synchronous.reference.latitude  = 0
        //  sun.synchronous.reference.ascending = false
        //  sun.synchronous.mean.solar.time     = 10:30:00
        //  gravity.field.degree                = 12
        //  gravity.field.order                 = 12
        //
        // the resulting phased orbit has then been propagated to a date corresponding
        // to test point lying in the spacecraft (YZ) plane (with nadir pointing and yaw compensation)
        AbsoluteDate crossing = new AbsoluteDate("2012-01-07T11:50:04.935272115", TimeScalesFactory.getUTC());
        state = new SpacecraftState(new CartesianOrbit(new PVCoordinates(new Vector3D(  412324.544397459,
                                                                                      -4325872.329311633,
                                                                                       5692124.593989491),
                                                                         new Vector3D(-1293.174701214779,
                                                                                      -5900.764863603793,
                                                                                      -4378.671036383179)),
                                                       FramesFactory.getEME2000(),
                                                       crossing,
                                                       Constants.EIGEN5C_EARTH_MU),
                                                       new Attitude(crossing,
                                                                    FramesFactory.getEME2000(),
                                                                    new Rotation(-0.17806699079182878,
                                                                                  0.60143347387211290,
                                                                                 -0.73251248177468900,
                                                                                 -0.26456641385623986,
                                                                                 true),
                                                                    new Vector3D(-4.289600857433520e-05,
                                                                                 -1.039151496480297e-03,
                                                                                  5.811423736843181e-05)));

    }

    @Before
    public void setUp()
            throws OrekitException, URISyntaxException {
        String path = getClass().getClassLoader().getResource("orekit-data").toURI().getPath();
        DataProvidersManager.getInstance().addProvider(new DirectoryCrawler(new File(path)));
        earth = new ExtendedEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                      Constants.WGS84_EARTH_FLATTENING,
                                      FramesFactory.getITRF(IERSConventions.IERS_2010, true));

    }

    @After
    public void tearDown() {
        earth     = null;
        updater   = null;
        state     = null;
    }

    protected ExtendedEllipsoid earth;
    protected TileUpdater       updater;
    protected SpacecraftState   state;

}
