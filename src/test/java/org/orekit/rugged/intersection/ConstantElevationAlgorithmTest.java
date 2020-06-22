/* Copyright 2013-2019 CS Systèmes d'Information
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
package org.orekit.rugged.intersection;


import java.io.File;
import java.net.URISyntaxException;

import org.hipparchus.geometry.euclidean.threed.Rotation;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.util.FastMath;
import org.junit.After;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;
import org.orekit.attitudes.Attitude;
import org.orekit.data.DataContext;
import org.orekit.data.DirectoryCrawler;
import org.orekit.frames.FramesFactory;
import org.orekit.orbits.CartesianOrbit;
import org.orekit.propagation.SpacecraftState;
import org.orekit.rugged.intersection.duvenhage.DuvenhageAlgorithm;
import org.orekit.rugged.raster.CheckedPatternElevationUpdater;
import org.orekit.rugged.raster.TileUpdater;
import org.orekit.rugged.utils.ExtendedEllipsoid;
import org.orekit.rugged.utils.NormalizedGeodeticPoint;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeScalesFactory;
import org.orekit.utils.Constants;
import org.orekit.utils.IERSConventions;
import org.orekit.utils.PVCoordinates;

public class ConstantElevationAlgorithmTest {

    @Test
    public void testDuvenhageComparison() {
        final Vector3D los = new Vector3D(-0.626242839, 0.0124194184, -0.7795291301);
        IntersectionAlgorithm duvenhage = new DuvenhageAlgorithm(new CheckedPatternElevationUpdater(FastMath.toRadians(1.0),
                                                                                                    256, 150.0, 150.0),
                                                                 8, false);
        IntersectionAlgorithm constantElevation = new ConstantElevationAlgorithm(150.0);
        NormalizedGeodeticPoint gpRef = duvenhage.intersection(earth, state.getPVCoordinates().getPosition(), los);
        NormalizedGeodeticPoint gpConst = constantElevation.intersection(earth, state.getPVCoordinates().getPosition(), los);
        Assert.assertEquals(gpRef.getLatitude(),  gpConst.getLatitude(),  1.0e-6);
        Assert.assertEquals(gpRef.getLongitude(), gpConst.getLongitude(), 1.0e-6);
        Assert.assertEquals(gpRef.getAltitude(),  gpConst.getAltitude(),  1.0e-3);
        Assert.assertEquals(150.0,  constantElevation.getElevation(0.0, 0.0),  1.0e-3);

        // shift longitude 2π
        NormalizedGeodeticPoint shifted =
                constantElevation.refineIntersection(earth, state.getPVCoordinates().getPosition(), los,
                                                     new NormalizedGeodeticPoint(gpConst.getLatitude(),
                                                                                 gpConst.getLongitude(),
                                                                                 gpConst.getAltitude(),
                                                                                 2 * FastMath.PI));
        Assert.assertEquals(2 * FastMath.PI + gpConst.getLongitude(), shifted.getLongitude(), 1.0e-6);

    }

    @Test
    public void testIgnoreDEMComparison() {
        final Vector3D los = new Vector3D(-0.626242839, 0.0124194184, -0.7795291301);
        IntersectionAlgorithm ignore = new IgnoreDEMAlgorithm();
        IntersectionAlgorithm constantElevation = new ConstantElevationAlgorithm(0.0);
        NormalizedGeodeticPoint gpRef = ignore.intersection(earth, state.getPVCoordinates().getPosition(), los);
        NormalizedGeodeticPoint gpConst = constantElevation.intersection(earth, state.getPVCoordinates().getPosition(), los);
        Assert.assertEquals(gpRef.getLatitude(),  gpConst.getLatitude(),  1.0e-6);
        Assert.assertEquals(gpRef.getLongitude(), gpConst.getLongitude(), 1.0e-6);
        Assert.assertEquals(gpRef.getAltitude(),  gpConst.getAltitude(),  1.0e-3);
        Assert.assertEquals(0.0,  constantElevation.getElevation(0.0, 0.0),  1.0e-3);

        // shift longitude 2π
        NormalizedGeodeticPoint shifted =
                constantElevation.refineIntersection(earth, state.getPVCoordinates().getPosition(), los,
                                                     new NormalizedGeodeticPoint(gpConst.getLatitude(),
                                                                                 gpConst.getLongitude(),
                                                                                 gpConst.getAltitude(),
                                                                                 2 * FastMath.PI));
        Assert.assertEquals(2 * FastMath.PI + gpConst.getLongitude(), shifted.getLongitude(), 1.0e-6);

        // Simple test for test coverage purpose
        double elevation0 = ignore.getElevation(gpRef.getLatitude(), gpConst.getLatitude());
        Assert.assertEquals(elevation0, 0.0, 1.e-15);
    }

    @Before
    public void setUp() throws  URISyntaxException {
        String path = getClass().getClassLoader().getResource("orekit-data").toURI().getPath();
        DataContext.getDefault().getDataProvidersManager().addProvider(new DirectoryCrawler(new File(path)));
        earth = new ExtendedEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                      Constants.WGS84_EARTH_FLATTENING,
                                      FramesFactory.getITRF(IERSConventions.IERS_2010, true));

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
                                                                                  5.811423736843181e-05),
                                                                    Vector3D.ZERO));

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
