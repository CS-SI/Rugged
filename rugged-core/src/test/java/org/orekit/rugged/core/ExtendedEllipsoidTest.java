/* Copyright 2013-2014 CS Systèmes d'Information
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

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.util.FastMath;
import org.junit.After;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.data.DataProvidersManager;
import org.orekit.data.DirectoryCrawler;
import org.orekit.errors.OrekitException;
import org.orekit.frames.Frame;
import org.orekit.frames.FramesFactory;
import org.orekit.rugged.api.RuggedException;
import org.orekit.rugged.api.RuggedMessages;
import org.orekit.rugged.core.ExtendedEllipsoid;
import org.orekit.utils.Constants;
import org.orekit.utils.IERSConventions;

public class ExtendedEllipsoidTest {

    @Test
    public void testPointAtLongitude() throws RuggedException, OrekitException {

        Vector3D p = new Vector3D(3220103.0, 69623.0, -6449822.0);
        Vector3D d = new Vector3D(1.0, 2.0, 3.0);

        for (double longitude = -1.0; longitude < 1.0; longitude += 0.01) {
            GeodeticPoint gp = ellipsoid.transform(ellipsoid.pointAtLongitude(p, d, longitude),
                                                   ellipsoid.getBodyFrame(), null);
            Assert.assertEquals(longitude, gp.getLongitude(), 1.0e-15);
        }

    }

    @Test
    public void testPointAtLongitudeError() throws RuggedException, OrekitException {

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
    public void testPointAtLatitude() throws RuggedException, OrekitException {

        Vector3D p = new Vector3D(3220103.0, 69623.0, -6449822.0);
        Vector3D d = new Vector3D(1.0, 2.0, 3.0);

        for (double latitude = -d.getDelta() + 1.0e-6; latitude < d.getDelta(); latitude += 0.1) {
            GeodeticPoint gp = ellipsoid.transform(ellipsoid.pointAtLatitude(p, d, latitude),
                                                   ellipsoid.getBodyFrame(), null);
            Assert.assertEquals(latitude, gp.getLatitude(), 1.0e-12);
        }

    }

    @Test
    public void testPointAtLatitudeTwoPointsInGoodNappe() throws RuggedException, OrekitException {

        Vector3D p = new Vector3D(3220103.0, 69623.0, -6449822.0);
        Vector3D d = new Vector3D(1.0, 2.0, 0.1);
        double latitude = -0.5;

        GeodeticPoint gpPlus = ellipsoid.transform(ellipsoid.pointAtLatitude(p, d, latitude),
                                                   ellipsoid.getBodyFrame(), null);
        Assert.assertEquals(latitude, gpPlus.getLatitude(), 1.0e-12);

        GeodeticPoint gpMinus = ellipsoid.transform(ellipsoid.pointAtLatitude(p, d.negate(), latitude),
                                                    ellipsoid.getBodyFrame(), null);
        Assert.assertEquals(latitude, gpMinus.getLatitude(), 1.0e-12);

    }

    @Test
    public void testPointAtLatitudeErrorQuadraticEquation() throws RuggedException, OrekitException {

        Vector3D p = new Vector3D(3220103.0, 69623.0, -6449822.0);
        Vector3D d = new Vector3D(1.0, 2.0, 3.0);
        double latitude = -1.4;

        try {
            ellipsoid.pointAtLatitude(p, d, latitude);
            Assert.fail("an error should have been triggered");
        } catch (RuggedException re) {
            Assert.assertEquals(RuggedMessages.LINE_OF_SIGHT_NEVER_CROSSES_LATITUDE, re.getSpecifier());
            Assert.assertEquals(FastMath.toDegrees(latitude), re.getParts()[0]);
        }

    }

    @Test
    public void testPointAtLatitudeErrorNappe() throws RuggedException, OrekitException {

        Vector3D p = new Vector3D(3220103.0, 69623.0, -6449822.0);
        Vector3D d = new Vector3D(1.0, 2.0, 0.1);
        double latitude = 0.5;

        try {
            ellipsoid.pointAtLatitude(p, d, latitude);
            Assert.fail("an error should have been triggered");
        } catch (RuggedException re) {
            Assert.assertEquals(RuggedMessages.LINE_OF_SIGHT_NEVER_CROSSES_LATITUDE, re.getSpecifier());
            Assert.assertEquals(FastMath.toDegrees(latitude), re.getParts()[0]);
        }

    }

    @Test
    public void testPointAtAltitude() throws RuggedException, OrekitException {

        Vector3D p = new Vector3D(3220103.0, 69623.0, -6449822.0);
        Vector3D d = new Vector3D(1.0, 2.0, 3.0);
        for (double altitude = -500000; altitude < 800000.0; altitude += 100) {
            GeodeticPoint gp = ellipsoid.transform(ellipsoid.pointAtAltitude(p, d, altitude),
                                                   ellipsoid.getBodyFrame(), null);
            Assert.assertEquals(altitude, gp.getAltitude(), 1.0e-3);
        }

    }

    @Test
    public void testPointAtAltitudeStartInside() throws RuggedException, OrekitException {

        Vector3D p = new Vector3D(322010.30, 6962.30, -644982.20);
        Vector3D d = new Vector3D(-1.0, -2.0, -3.0);
        for (double altitude = -500000; altitude < 800000.0; altitude += 100) {
            GeodeticPoint gp = ellipsoid.transform(ellipsoid.pointAtAltitude(p, d, altitude),
                                                   ellipsoid.getBodyFrame(), null);
            Assert.assertEquals(altitude, gp.getAltitude(), 1.0e-3);
        }

    }

    @Test
    public void testPointAtAltitudeError() throws RuggedException, OrekitException {

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

    @Before
    public void setUp() {
        try {

            String path = getClass().getClassLoader().getResource("orekit-data").toURI().getPath();
            DataProvidersManager.getInstance().addProvider(new DirectoryCrawler(new File(path)));

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
