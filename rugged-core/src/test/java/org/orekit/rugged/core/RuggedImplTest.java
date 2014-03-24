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
import java.util.Arrays;
import java.util.List;

import org.apache.commons.math3.ode.nonstiff.DormandPrince853Integrator;
import org.apache.commons.math3.util.FastMath;
import org.junit.Assert;
import org.junit.Test;
import org.orekit.attitudes.AttitudeProvider;
import org.orekit.attitudes.NadirPointing;
import org.orekit.attitudes.YawCompensation;
import org.orekit.bodies.BodyShape;
import org.orekit.bodies.CelestialBodyFactory;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.bodies.OneAxisEllipsoid;
import org.orekit.data.DataProvidersManager;
import org.orekit.data.DirectoryCrawler;
import org.orekit.errors.OrekitException;
import org.orekit.forces.gravity.HolmesFeatherstoneAttractionModel;
import org.orekit.forces.gravity.ThirdBodyAttraction;
import org.orekit.forces.gravity.potential.GravityFieldFactory;
import org.orekit.forces.gravity.potential.NormalizedSphericalHarmonicsProvider;
import org.orekit.frames.Frame;
import org.orekit.frames.FramesFactory;
import org.orekit.orbits.CircularOrbit;
import org.orekit.orbits.Orbit;
import org.orekit.orbits.OrbitType;
import org.orekit.orbits.PositionAngle;
import org.orekit.propagation.Propagator;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.numerical.NumericalPropagator;
import org.orekit.rugged.api.Rugged;
import org.orekit.rugged.api.RuggedException;
import org.orekit.rugged.api.SatellitePV;
import org.orekit.rugged.api.SatelliteQ;
import org.orekit.rugged.core.raster.CliffsElevationUpdater;
import org.orekit.rugged.core.raster.VolcanicConeElevationUpdater;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeScalesFactory;
import org.orekit.utils.Constants;
import org.orekit.utils.IERSConventions;

public class RuggedImplTest {

    @Test
    public void testSetContextWithoutOrekit()
        throws RuggedException, OrekitException, URISyntaxException {

        List<SatellitePV> pv = Arrays.asList(
            new SatellitePV( 0.000, -1545168.478, -7001985.361,       0.000, -1095.152224, 231.344922, -7372.851944),
            new SatellitePV( 1.000, -1546262.794, -7001750.226,   -7372.851, -1093.478904, 238.925123, -7372.847995),
            new SatellitePV( 2.000, -1547355.435, -7001507.511,  -14745.693, -1091.804408, 246.505033, -7372.836044),
            new SatellitePV( 3.000, -1548446.402, -7001257.216,  -22118.520, -1090.128736, 254.084644, -7372.816090),
            new SatellitePV( 4.000, -1549535.693, -7000999.342,  -29491.323, -1088.451892, 261.663949, -7372.788133),
            new SatellitePV( 5.000, -1550623.306, -7000733.888,  -36864.094, -1086.773876, 269.242938, -7372.752175),
            new SatellitePV( 6.000, -1551709.240, -7000460.856,  -44236.825, -1085.094690, 276.821604, -7372.708214),
            new SatellitePV( 7.000, -1552793.495, -7000180.245,  -51609.507, -1083.414336, 284.399938, -7372.656251),
            new SatellitePV( 8.000, -1553876.068, -6999892.056,  -58982.134, -1081.732817, 291.977932, -7372.596287),
            new SatellitePV( 9.000, -1554956.960, -6999596.289,  -66354.697, -1080.050134, 299.555578, -7372.528320),
            new SatellitePV(10.000, -1556036.168, -6999292.945,  -73727.188, -1078.366288, 307.132868, -7372.452352),
            new SatellitePV(11.000, -1557113.692, -6998982.024,  -81099.599, -1076.681282, 314.709792, -7372.368382),
            new SatellitePV(12.000, -1558189.530, -6998663.526,  -88471.922, -1074.995118, 322.286344, -7372.276411),
            new SatellitePV(13.000, -1559263.682, -6998337.451,  -95844.150, -1073.307797, 329.862513, -7372.176439),
            new SatellitePV(14.000, -1560336.145, -6998003.801, -103216.273, -1071.619321, 337.438294, -7372.068466),
            new SatellitePV(15.000, -1561406.920, -6997662.575, -110588.284, -1069.929692, 345.013676, -7371.952492),
            new SatellitePV(16.000, -1562476.004, -6997313.774, -117960.175, -1068.238912, 352.588652, -7371.828517),
            new SatellitePV(17.000, -1563543.398, -6996957.398, -125331.938, -1066.546983, 360.163213, -7371.696542),
            new SatellitePV(18.000, -1564609.098, -6996593.447, -132703.565, -1064.853906, 367.737352, -7371.556566),
            new SatellitePV(19.000, -1565673.105, -6996221.923, -140075.049, -1063.159684, 375.311060, -7371.408591),
            new SatellitePV(20.000, -1566735.417, -6995842.825, -147446.380, -1061.464319, 382.884328, -7371.252616));
        List<SatelliteQ> q = Arrays.asList(
            new SatelliteQ( 0.000, 0.516354347549, -0.400120145429,  0.583012133139,  0.483093065155),
            new SatelliteQ( 1.000, 0.516659035405, -0.399867643627,  0.582741754688,  0.483302551263),
            new SatelliteQ( 2.000, 0.516963581177, -0.399615033309,  0.582471217473,  0.483511904409),
            new SatelliteQ( 3.000, 0.517267984776, -0.399362314553,  0.582200521577,  0.483721124530),
            new SatelliteQ( 4.000, 0.517572246112, -0.399109487434,  0.581929667081,  0.483930211565),
            new SatelliteQ( 5.000, 0.517876365096, -0.398856552030,  0.581658654071,  0.484139165451),
            new SatelliteQ( 6.000, 0.518180341637, -0.398603508416,  0.581387482627,  0.484347986126),
            new SatelliteQ( 7.000, 0.518484175647, -0.398350356669,  0.581116152834,  0.484556673529),
            new SatelliteQ( 8.000, 0.518787867035, -0.398097096866,  0.580844664773,  0.484765227599),
            new SatelliteQ( 9.000, 0.519091415713, -0.397843729083,  0.580573018530,  0.484973648272),
            new SatelliteQ(10.000, 0.519394821590, -0.397590253397,  0.580301214186,  0.485181935488),
            new SatelliteQ(11.000, 0.519698084578, -0.397336669885,  0.580029251825,  0.485390089185),
            new SatelliteQ(12.000, 0.520001204587, -0.397082978623,  0.579757131530,  0.485598109301),
            new SatelliteQ(13.000, 0.520304181527, -0.396829179688,  0.579484853385,  0.485805995775),
            new SatelliteQ(14.000, 0.520607015311, -0.396575273158,  0.579212417473,  0.486013748545),
            new SatelliteQ(15.000, 0.520909705847, -0.396321259108,  0.578939823877,  0.486221367550),
            new SatelliteQ(16.000, 0.521212253049, -0.396067137616,  0.578667072681,  0.486428852729),
            new SatelliteQ(17.000, 0.521514656825, -0.395812908759,  0.578394163969,  0.486636204020),
            new SatelliteQ(18.000, 0.521816917089, -0.395558572613,  0.578121097824,  0.486843421362),
            new SatelliteQ(19.000, 0.522119033749, -0.395304129256,  0.577847874330,  0.487050504694),
            new SatelliteQ(20.000, 0.522421006719, -0.395049578765,  0.577574493570,  0.487257453954));
        String path = getClass().getClassLoader().getResource("orekit-data").toURI().getPath();
        RuggedImpl rugged = new RuggedImpl();
        rugged.setGeneralContext(new File(path),
                                 "2012-01-01T00:00:00",
                                 Rugged.Algorithm.DUVENHAGE,
                                 Rugged.Ellipsoid.WGS84,
                                 Rugged.InertialFrame.EME2000,
                                 Rugged.BodyRotatingFrame.ITRF,
                                 pv, 8, q, 8);

        Assert.assertEquals(new AbsoluteDate("2012-01-01T00:00:00", TimeScalesFactory.getUTC()),
                            rugged.getReferenceDate());

    }

    @Test
    public void testSetContextWithOrekit()
        throws RuggedException, OrekitException, URISyntaxException {

        String path = getClass().getClassLoader().getResource("orekit-data").toURI().getPath();
        DataProvidersManager.getInstance().addProvider(new DirectoryCrawler(new File(path)));
        BodyShape  earth                                  = createEarth();
        NormalizedSphericalHarmonicsProvider gravityField = createGravityField();
        Orbit      orbit                                  = createOrbit(gravityField);
        Propagator propagator                             = createPropagator(earth, gravityField, orbit);

        RuggedImpl rugged = new RuggedImpl();
        rugged.setGeneralContext(null,
                                 propagator.getInitialState().getDate(),
                                 Rugged.Algorithm.DUVENHAGE,
                                 Rugged.Ellipsoid.WGS84,
                                 Rugged.InertialFrame.EME2000,
                                 Rugged.BodyRotatingFrame.ITRF,
                                 propagator);

        Assert.assertEquals(propagator.getInitialState().getDate(), rugged.getReferenceDate());

    }

    @Test
    public void testMayonVolcano()
        throws RuggedException, OrekitException, URISyntaxException {

        String path = getClass().getClassLoader().getResource("orekit-data").toURI().getPath();
        DataProvidersManager.getInstance().addProvider(new DirectoryCrawler(new File(path)));
        BodyShape  earth                                  = createEarth();
        NormalizedSphericalHarmonicsProvider gravityField = createGravityField();
        Orbit      orbit                                  = createOrbit(gravityField);
        Propagator propagator                             = createPropagator(earth, gravityField, orbit);

        // Mayon Volcano location according to Wikipedia: 13°15′24″N 123°41′6″E
        GeodeticPoint summit =
                new GeodeticPoint(FastMath.toRadians(13.25667), FastMath.toRadians(123.685), 2463.0);
        VolcanicConeElevationUpdater updater =
                new VolcanicConeElevationUpdater(summit,
                                                 FastMath.toRadians(30.0), 16.0,
                                                 FastMath.toRadians(1.0), 1201);
        AbsoluteDate crossing = new AbsoluteDate("2012-01-06T02:27:16.139", TimeScalesFactory.getUTC());

        // TODO: test the direct localization

    }

    @Test
    public void testCliffsOfMoher()
        throws RuggedException, OrekitException, URISyntaxException {

        String path = getClass().getClassLoader().getResource("orekit-data").toURI().getPath();
        DataProvidersManager.getInstance().addProvider(new DirectoryCrawler(new File(path)));
        final BodyShape  earth                                  = createEarth();
        NormalizedSphericalHarmonicsProvider gravityField = createGravityField();
        Orbit      orbit                                  = createOrbit(gravityField);
        Propagator propagator                             = createPropagator(earth, gravityField, orbit);

        // cliffs of Moher location according to Wikipedia: 52°56′10″N 9°28′15″ W
        GeodeticPoint north = new GeodeticPoint(FastMath.toRadians(52.9984),
                                                FastMath.toRadians(-9.4072),
                                                120.0);
        GeodeticPoint south = new GeodeticPoint(FastMath.toRadians(52.9625),
                                                FastMath.toRadians(-9.4369),
                                                120.0);
        CliffsElevationUpdater updater = new CliffsElevationUpdater(north, south,
                                                                    120.0, 0.0,
                                                                    FastMath.toRadians(1.0), 1201);

        AbsoluteDate crossing = new AbsoluteDate("2012-01-07T11:50:05.778", TimeScalesFactory.getUTC());

        // TODO: test the direct localization

    }

    private BodyShape createEarth()
       throws OrekitException {
        return new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                               Constants.WGS84_EARTH_FLATTENING,
                                               FramesFactory.getITRF(IERSConventions.IERS_2010, true));
    }

    private NormalizedSphericalHarmonicsProvider createGravityField()
        throws OrekitException {
        return GravityFieldFactory.getNormalizedProvider(12, 12);
    }

    private Orbit createOrbit(NormalizedSphericalHarmonicsProvider gravityField)
        throws OrekitException {
        // the following orbital parameters have been computed using
        // Orekit tutorial about phasing, using the following configuration:
        //
        //  orbit.date                          = 2012-01-01T00:00:00.000
        //  phasing.orbits.number               = 143
        //  phasing.days.number                 =  10
        //  sun.synchronous.reference.latitude  = 0
        //  sun.synchronous.reference.ascending = false
        //  sun.synchronous.mean.solar.time     = 10:30:00
        //  gravity.field.degree                = 12
        //  gravity.field.order                 = 12
        AbsoluteDate date = new AbsoluteDate("2012-01-01T00:00:00.000", TimeScalesFactory.getUTC());
        Frame eme2000 = FramesFactory.getEME2000();
        return new CircularOrbit(7173352.811913891,
                                 -4.029194321683225E-4, 0.0013530362644647786,
                                 FastMath.toRadians(98.63218182243709),
                                 FastMath.toRadians(77.55565567747836),
                                 FastMath.PI, PositionAngle.TRUE,
                                 eme2000, date, gravityField.getMu());
    }

    private Propagator createPropagator(BodyShape earth,
                                        NormalizedSphericalHarmonicsProvider gravityField,
                                        Orbit orbit)
        throws OrekitException {

        AttitudeProvider yawCompensation = new YawCompensation(new NadirPointing(earth));
        SpacecraftState state = new SpacecraftState(orbit,
                                                    yawCompensation.getAttitude(orbit,
                                                                                orbit.getDate(),
                                                                                orbit.getFrame()),
                                                    1180.0);

        // numerical model for improving orbit
        OrbitType type = OrbitType.CIRCULAR;
        double[][] tolerances = NumericalPropagator.tolerances(0.1, orbit, type);
        DormandPrince853Integrator integrator =
                new DormandPrince853Integrator(1.0e-4 * orbit.getKeplerianPeriod(),
                                               1.0e-1 * orbit.getKeplerianPeriod(),
                                               tolerances[0], tolerances[1]);
        integrator.setInitialStepSize(1.0e-2 * orbit.getKeplerianPeriod());
        NumericalPropagator numericalPropagator = new NumericalPropagator(integrator);
        numericalPropagator.addForceModel(new HolmesFeatherstoneAttractionModel(earth.getBodyFrame(), gravityField));
        numericalPropagator.addForceModel(new ThirdBodyAttraction(CelestialBodyFactory.getSun()));
        numericalPropagator.addForceModel(new ThirdBodyAttraction(CelestialBodyFactory.getMoon()));
        numericalPropagator.setOrbitType(type);
        numericalPropagator.setInitialState(state);
        numericalPropagator.setAttitudeProvider(yawCompensation);
        return numericalPropagator;

    }

}

