/* Copyright 2013-2025 CS GROUP
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
package org.orekit.rugged.utils;

import java.io.File;
import java.net.URISyntaxException;

import org.hipparchus.geometry.euclidean.threed.Line;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.util.FastMath;
import org.junit.After;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.data.DataContext;
import org.orekit.data.DirectoryCrawler;
import org.orekit.errors.OrekitException;
import org.orekit.frames.Frame;
import org.orekit.frames.FramesFactory;
import org.orekit.rugged.errors.RuggedException;
import org.orekit.rugged.errors.RuggedMessages;
import org.orekit.utils.Constants;
import org.orekit.utils.IERSConventions;

public class ExtendedEllipsoidTest {

    @Test
    public void testPointAtLongitude() {

        Vector3D p = new Vector3D(3220103.0, 69623.0, -6449822.0);
        Vector3D d = new Vector3D(1.0, 2.0, 3.0);

        for (double longitude = -1.0; longitude < 1.0; longitude += 0.01) {
            GeodeticPoint gp = ellipsoid.transform(ellipsoid.pointAtLongitude(p, d, longitude),
                                                   ellipsoid.getBodyFrame(), null);
            Assert.assertEquals(longitude, gp.getLongitude(), 1.0e-15);
        }

    }

    @Test
    public void testPointAtLongitudeError() {

        Vector3D p = new Vector3D(3220103.0, 69623.0, -6449822.0);
        double longitude = 1.25;
        Vector3D parallelToLongitudePlane = new Vector3D(FastMath.cos(longitude),
                                                         FastMath.sin(longitude),
                                                         -2.4);
        try {
            ellipsoid.pointAtLongitude(p, parallelToLongitudePlane, longitude);
            Assert.fail("an error should have been triggered");
        } catch (RuggedException re) {
            Assert.assertEquals(RuggedMessages.LINE_OF_SIGHT_NEVER_CROSSES_LONGITUDE, re.getSpecifier());
            Assert.assertEquals(FastMath.toDegrees(longitude), re.getParts()[0]);
        }

    }

    @Test
    public void testPointAtLatitude() {

        Vector3D p = new Vector3D(3220103.0, 69623.0, -6449822.0);
        Vector3D d = new Vector3D(1.0, 2.0, 3.0);

        double epsilon = 5.0e-15;
        for (double latitude = -d.getDelta() + 1.0e-5; latitude < d.getDelta(); latitude += 0.1) {
            GeodeticPoint gp = ellipsoid.transform(ellipsoid.pointAtLatitude(p, d, latitude, p),
                                                   ellipsoid.getBodyFrame(), null);
            Assert.assertEquals(latitude, gp.getLatitude(), epsilon);
        }

    }

    @Test
    public void testPointAtLatitudeTwoPointsSameSide() {

        // the line of sight is almost parallel an iso-latitude cone generatrix
        // the spacecraft is at latitude lTarget - 0.951", and altitude 794.6km
        // so at a latitude slightly less than the target
        // the line of sight crosses the latitude cone first about 70km along line of sight
        // (so still at a very high altitude) and a second time about 798km along line of sight,
        // only a few hundreds meters above allipsoid
        // Note that this happens despite the line of sight is not along nadir, the longitudes
        // of the spacecraft and crossing points span in a 0.88° wide longitude range
        Vector3D position     = new Vector3D(-748528.2769999998, -5451658.432000002, 4587158.354);
        Vector3D los          = new Vector3D(0.010713435156834539, 0.7688536080293823, -0.6393350856376809);
        double h              = ellipsoid.transform(position, ellipsoid.getBodyFrame(), null).getAltitude();
        double lTarget        = 0.6978408125890662;

        // spacecraft is in LEO
        Assert.assertEquals(h, 794652.782, 0.001);

        Vector3D pHigh        = ellipsoid.pointAtLatitude(position, los, lTarget, position);
        GeodeticPoint gpHigh  = ellipsoid.transform(pHigh, ellipsoid.getBodyFrame(), null);
        Assert.assertEquals(lTarget, gpHigh.getLatitude(), 1.0e-12);
        // first crossing point is high, but below spacecraft and along positive line of sight
        Assert.assertEquals(724335.409, gpHigh.getAltitude(), 0.001);
        Assert.assertTrue(Vector3D.dotProduct(pHigh.subtract(position), los) > 0);

        Vector3D pLow         = ellipsoid.pointAtLatitude(position, los, lTarget,
                                                          new Vector3D(1, position, 900000, los));
        GeodeticPoint gpLow   = ellipsoid.transform(pLow, ellipsoid.getBodyFrame(), null);
        Assert.assertEquals(lTarget, gpLow.getLatitude(), 1.0e-12);
        // second crossing point is almost on ground, also along positive line of sight
        Assert.assertEquals(492.804, gpLow.getAltitude(), 0.001);
        Assert.assertTrue(Vector3D.dotProduct(pLow.subtract(position), los) > 0);

    }

    @Test
    public void testPointAtLatitudeTwoPointsOppositeSides() {

        Vector3D p = new Vector3D(3220103.0, 69623.0, -6449822.0);
        Vector3D d = new Vector3D(1.0, 2.0, 0.1);
        double latitude = -0.5;

        Vector3D pPlus = ellipsoid.pointAtLatitude(p, d, latitude, new Vector3D(1, p, +2.0e7, d));
        GeodeticPoint gpPlus = ellipsoid.transform(pPlus, ellipsoid.getBodyFrame(), null);
        Assert.assertEquals(latitude, gpPlus.getLatitude(), 4.0e-16);
        Assert.assertEquals(20646364.047, Vector3D.dotProduct(d, pPlus.subtract(p)), 0.001);

        Vector3D pMinus = ellipsoid.pointAtLatitude(p, d, latitude, new Vector3D(1, p, -3.0e7, d));
        GeodeticPoint gpMinus = ellipsoid.transform(pMinus, ellipsoid.getBodyFrame(), null);
        Assert.assertEquals(latitude, gpMinus.getLatitude(), 3.0e-16);
        Assert.assertEquals(-31797895.234, Vector3D.dotProduct(d, pMinus.subtract(p)), 0.001);

    }

    @Test
    public void testPointAtLatitudeAlmostEquator() {
        Vector3D      p              = new Vector3D(5767483.098580201, 4259689.325372237, -41553.67750784925);
        Vector3D      d              = new Vector3D(-0.7403523952347795, -0.6701811835520302, 0.05230212180799747);
        double        latitude       = -3.469446951953614E-18;
        Vector3D      closeReference = new Vector3D(5177991.74844521, 3726070.452427455, 90.88067547897226);
        Vector3D      intersection   = ellipsoid.pointAtLatitude(p, d, latitude, closeReference);
        GeodeticPoint gp             = ellipsoid.transform(intersection, ellipsoid.getBodyFrame(), null);
        Assert.assertEquals(latitude, gp.getLatitude(), 1.0e-10);
        Assert.assertEquals(2866.297, gp.getAltitude(), 1.0e-3);
    }

    @Test
    public void testPointAtLatitudeErrorQuadraticEquation() {

        Vector3D p = new Vector3D(3220103.0, 69623.0, -6449822.0);
        Vector3D d = new Vector3D(1.0, 2.0, 3.0);
        double latitude = -1.4;

        try {
            ellipsoid.pointAtLatitude(p, d, latitude, p);
            Assert.fail("an error should have been triggered");
        } catch (RuggedException re) {
            Assert.assertEquals(RuggedMessages.LINE_OF_SIGHT_NEVER_CROSSES_LATITUDE, re.getSpecifier());
            Assert.assertEquals(FastMath.toDegrees(latitude), re.getParts()[0]);
        }

    }

    @Test
    public void testPointAtLatitudeErrorNappe() {

        Vector3D p = new Vector3D(3220103.0, 69623.0, -6449822.0);
        Vector3D d = new Vector3D(1.0, 2.0, 0.1);
        double latitude = 0.5;

        try {
            ellipsoid.pointAtLatitude(p, d, latitude, p);
            Assert.fail("an error should have been triggered");
        } catch (RuggedException re) {
            Assert.assertEquals(RuggedMessages.LINE_OF_SIGHT_NEVER_CROSSES_LATITUDE, re.getSpecifier());
            Assert.assertEquals(FastMath.toDegrees(latitude), re.getParts()[0]);
        }

    }

    @Test
    public void testPointAtAltitude() {

        Vector3D p = new Vector3D(3220103.0, 69623.0, -6449822.0);
        Vector3D d = new Vector3D(1.0, 2.0, 3.0);
        for (double altitude = -500000; altitude < 800000.0; altitude += 100) {
            GeodeticPoint gp = ellipsoid.transform(ellipsoid.pointAtAltitude(p, d, altitude),
                                                   ellipsoid.getBodyFrame(), null);
            Assert.assertEquals(altitude, gp.getAltitude(), 1.0e-3);
        }

    }

    @Test
    public void testPointAtAltitudeStartInside() {

        Vector3D p = new Vector3D(322010.30, 6962.30, -644982.20);
        Vector3D d = new Vector3D(-1.0, -2.0, -3.0);
        for (double altitude = -500000; altitude < 800000.0; altitude += 100) {
            GeodeticPoint gp = ellipsoid.transform(ellipsoid.pointAtAltitude(p, d, altitude),
                                                   ellipsoid.getBodyFrame(), null);
            Assert.assertEquals(altitude, gp.getAltitude(), 1.0e-3);
        }

    }

    @Test
    public void testPointAtAltitudeError() {

        Vector3D p = new Vector3D(3220103.0, 69623.0, -6449822.0);
        Vector3D d = new Vector3D(1.0, 2.0, 3.0);
        double altitude = -580000.0;
        try {
            ellipsoid.pointAtAltitude(p, d, altitude);
            Assert.fail("an error should have been triggered");
        } catch (RuggedException re) {
            Assert.assertEquals(RuggedMessages.LINE_OF_SIGHT_NEVER_CROSSES_ALTITUDE, re.getSpecifier());
            Assert.assertEquals(altitude, re.getParts()[0]);
        }

    }

    @Test
    public void testConvertLOS() {

        GeodeticPoint gp   = new GeodeticPoint(-0.2, 1.8, 2400.0);
        Vector3D p         = ellipsoid.transform(gp);
        Vector3D los       = new Vector3D(-1, -2, -3);
        Vector3D converted = ellipsoid.convertLos(gp, los);
        Line line = new Line(p, new Vector3D(1.0, p, 1000, los), 1.0e-10);

        for (double delta = 0.1; delta < 100.0; delta += 0.1) {
            GeodeticPoint shifted = new GeodeticPoint(gp.getLatitude()  + delta * converted.getY(),
                                                      gp.getLongitude() + delta * converted.getX(),
                                                      gp.getAltitude()  + delta * converted.getZ());
            Vector3D converted2 = ellipsoid.convertLos(p, ellipsoid.transform(shifted));
            Assert.assertEquals(0.0, Vector3D.distance(converted, converted2), 3.0e-5 * converted.getNorm());
            Assert.assertEquals(0.0, line.distance(ellipsoid.transform(shifted)), 8.0e-4);
        }

    }

    @Test
    public void testPointAtLatitudeError() {

        Vector3D p = new Vector3D(-3052690.88784496, 6481300.309857268, 25258.7478104745);
        Vector3D d = new Vector3D(0.6, -0.8, 0.0);
        double latitude = 0.1;
        Vector3D c = new Vector3D(-2809972.5765414005, 5727461.020250551, 26.163518446261833);

        try {
            ellipsoid.pointAtLatitude(p, d, latitude, c);
            Assert.fail("an error should have been triggered");
        } catch (RuggedException re) {
            Assert.assertEquals(RuggedMessages.LINE_OF_SIGHT_NEVER_CROSSES_LATITUDE, re.getSpecifier());
            Assert.assertEquals(FastMath.toDegrees(latitude), re.getParts()[0]);
        }

    }

    @Test
    public void testPointAtLatitudeIssue1() {

        Vector3D position = new Vector3D(-1988136.619268088, -2905373.394638188, 6231185.484365295);
        Vector3D los = new Vector3D(0.3489121277213534, 0.3447806500507106, -0.8714279261531437);
        Vector3D close = new Vector3D(-1709383.0948608494, -2630206.8820586684, 5535282.169189105);
        double latitude =  1.0581058590215624;

        Vector3D s = ellipsoid.pointAtLatitude(position, los, latitude, close);
        GeodeticPoint gp = ellipsoid.transform(s, ellipsoid.getBodyFrame(), null);
        Assert.assertEquals(latitude, gp.getLatitude(), 1.0e-15);

    }

    @Before
    public void setUp() {
        try {

            String path = getClass().getClassLoader().getResource("orekit-data").toURI().getPath();
            DataContext.getDefault().getDataProvidersManager().addProvider(new DirectoryCrawler(new File(path)));

            Frame itrf = FramesFactory.getITRF(IERSConventions.IERS_2010, true);
            ellipsoid = new ExtendedEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                              Constants.WGS84_EARTH_FLATTENING,
                                              itrf);
        } catch (OrekitException oe) {
            Assert.fail(oe.getLocalizedMessage());
        } catch (URISyntaxException use) {
            Assert.fail(use.getLocalizedMessage());
        }
    }

    @After
    public void tearDown() {
        ellipsoid = null;
    }

    private ExtendedEllipsoid ellipsoid;

}
