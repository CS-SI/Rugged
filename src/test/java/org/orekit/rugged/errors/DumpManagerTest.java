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
package org.orekit.rugged.errors;


import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.net.URISyntaxException;

import org.hipparchus.geometry.euclidean.threed.Rotation;
import org.hipparchus.geometry.euclidean.threed.RotationConvention;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.util.FastMath;
import org.junit.Assert;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;
import org.orekit.bodies.BodyShape;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.data.DataProvidersManager;
import org.orekit.data.DirectoryCrawler;
import org.orekit.errors.OrekitException;
import org.orekit.orbits.Orbit;
import org.orekit.rugged.TestUtils;
import org.orekit.rugged.api.AlgorithmId;
import org.orekit.rugged.api.BodyRotatingFrameId;
import org.orekit.rugged.api.EllipsoidId;
import org.orekit.rugged.api.InertialFrameId;
import org.orekit.rugged.api.Rugged;
import org.orekit.rugged.api.RuggedBuilder;
import org.orekit.rugged.linesensor.LineDatation;
import org.orekit.rugged.linesensor.LineSensor;
import org.orekit.rugged.linesensor.LinearLineDatation;
import org.orekit.rugged.linesensor.SensorPixel;
import org.orekit.rugged.los.TimeDependentLOS;
import org.orekit.rugged.raster.RandomLandscapeUpdater;
import org.orekit.rugged.raster.TileUpdater;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeScalesFactory;
import org.orekit.utils.AngularDerivativesFilter;
import org.orekit.utils.CartesianDerivativesFilter;
import org.orekit.utils.Constants;

public class DumpManagerTest {

    @Rule
    public TemporaryFolder tempFolder = new TemporaryFolder();

    @Test
    public void testDump() throws URISyntaxException, IOException, OrekitException, RuggedException {

        File dump = tempFolder.newFile();
        DumpManager.activate(dump);
        variousRuggedCalls();
        DumpManager.deactivate();

        int countAlgorithm        = 0;
        int countEllipsoid        = 0;
        int countDirectLoc        = 0;
        int countDirectLocResult  = 0;
        int countSpan             = 0;
        int countTransform        = 0;
        int countDEMTile          = 0;
        int countDEMCell          = 0;
        int countInverseLoc       = 0;
        int countInverseLocResult = 0;
        int countSensor           = 0;
        int countSensorMeanPlane  = 0;
        int countSensorLOS        = 0;
        int countSensorDatation   = 0;
        int countSensorRate       = 0;
        try (final FileReader     fr = new FileReader(dump);
             final BufferedReader br = new BufferedReader(fr)) {
            for (String line = br.readLine(); line != null; line = br.readLine()) {
                String trimmed = line.trim();
                if (trimmed.length() > 0 && !trimmed.startsWith("#")){
                    if (trimmed.startsWith("algorithm:")) {
                        ++countAlgorithm;
                    } else if (trimmed.startsWith("ellipsoid:")) {
                        ++countEllipsoid;
                    } else if (trimmed.startsWith("direct location:")) {
                        ++countDirectLoc;
                    } else if (trimmed.startsWith("direct location result:")) {
                        ++countDirectLocResult;
                    } else if (trimmed.startsWith("span:")) {
                        ++countSpan;
                    } else if (trimmed.startsWith("transform:")) {
                        ++countTransform;
                    } else if (trimmed.startsWith("DEM tile:")) {
                        ++countDEMTile;
                    } else if (trimmed.startsWith("DEM cell:")) {
                        ++countDEMCell;
                    } else if (trimmed.startsWith("inverse location:")) {
                        ++countInverseLoc;
                    } else if (trimmed.startsWith("inverse location result:")) {
                        ++countInverseLocResult;
                    } else if (trimmed.startsWith("sensor:")) {
                        ++countSensor;
                    } else if (trimmed.startsWith("sensor mean plane:")) {
                        ++countSensorMeanPlane;
                    } else if (trimmed.startsWith("sensor LOS:")) {
                        ++countSensorLOS;
                    } else if (trimmed.startsWith("sensor datation:")) {
                        ++countSensorDatation;
                    } else if (trimmed.startsWith("sensor rate:")) {
                        ++countSensorRate;
                    } else {
                        Assert.fail(line);
                    }
                }
            }
        }

        Assert.assertEquals(1,   countAlgorithm);
        Assert.assertEquals(1,   countEllipsoid);
        Assert.assertEquals(400, countDirectLoc);
        Assert.assertEquals(400, countDirectLocResult);
        Assert.assertEquals(1,   countSpan);
        Assert.assertEquals(2,   countTransform);
        Assert.assertEquals(2,   countDEMTile);
        Assert.assertEquals(812, countDEMCell);
        Assert.assertEquals(5,   countInverseLoc);
        Assert.assertEquals(5,   countInverseLocResult);
        Assert.assertEquals(1,   countSensor);
        Assert.assertEquals(1,   countSensorMeanPlane);
        Assert.assertEquals(422, countSensorLOS);
        Assert.assertEquals(19,  countSensorDatation);
        Assert.assertEquals(6,   countSensorRate);

    }

   public void variousRuggedCalls()
        throws RuggedException, OrekitException, URISyntaxException {

       int dimension = 200;

       String path = getClass().getClassLoader().getResource("orekit-data").toURI().getPath();
       DataProvidersManager.getInstance().addProvider(new DirectoryCrawler(new File(path)));
       final BodyShape  earth = TestUtils.createEarth();
       final Orbit      orbit = TestUtils.createOrbit(Constants.EIGEN5C_EARTH_MU);

       AbsoluteDate crossing = new AbsoluteDate("2012-01-01T12:30:00.000", TimeScalesFactory.getUTC());

       // one line sensor
       // position: 1.5m in front (+X) and 20 cm above (-Z) of the S/C center of mass
       // los: swath in the (YZ) plane, looking at 50° roll, ±1° aperture
       Vector3D position = new Vector3D(1.5, 0, -0.2);
       TimeDependentLOS los = TestUtils.createLOSPerfectLine(new Rotation(Vector3D.PLUS_I,
                                                                          FastMath.toRadians(50.0),
                                                                          RotationConvention.VECTOR_OPERATOR).applyTo(Vector3D.PLUS_K),
                                                             Vector3D.PLUS_I, FastMath.toRadians(1.0), dimension).build();

       // linear datation model: at reference time we get line 100, and the rate is one line every 1.5ms
       LineDatation lineDatation = new LinearLineDatation(crossing, dimension / 2, 1.0 / 1.5e-3);
       int firstLine = 0;
       int lastLine  = dimension;
       LineSensor lineSensor = new LineSensor("line", lineDatation, position, los);
       AbsoluteDate minDate = lineSensor.getDate(firstLine);
       AbsoluteDate maxDate = lineSensor.getDate(lastLine);

       TileUpdater updater =
               new RandomLandscapeUpdater(0.0, 9000.0, 0.5, 0xf0a401650191f9f6l,
                                          FastMath.toRadians(1.0), 257);

       Rugged rugged = new RuggedBuilder().
               setDigitalElevationModel(updater, 8).
               setAlgorithm(AlgorithmId.DUVENHAGE).
               setEllipsoid(EllipsoidId.WGS84, BodyRotatingFrameId.ITRF).
               setTimeSpan(minDate, maxDate, 0.001, 5.0).
               setTrajectory(InertialFrameId.EME2000,
                             TestUtils.orbitToPV(orbit, earth, minDate.shiftedBy(-1.0), maxDate.shiftedBy(+1.0), 0.25),
                             8, CartesianDerivativesFilter.USE_PV,
                             TestUtils.orbitToQ(orbit, earth, minDate.shiftedBy(-1.0), maxDate.shiftedBy(+1.0), 0.25),
                             2, AngularDerivativesFilter.USE_R).
               addLineSensor(lineSensor).build();
       GeodeticPoint[] gpLine = rugged.directLocation("line", 100);

       for (int i = 0; i < gpLine.length; ++i) {
           GeodeticPoint gpPixel =
                   rugged.directLocation(lineSensor.getDate(100), lineSensor.getPosition(),
                                             lineSensor.getLOS(lineSensor.getDate(100), i));
           Assert.assertEquals(gpLine[i].getLatitude(),  gpPixel.getLatitude(),  1.0e-10);
           Assert.assertEquals(gpLine[i].getLongitude(), gpPixel.getLongitude(), 1.0e-10);
           Assert.assertEquals(gpLine[i].getAltitude(),  gpPixel.getAltitude(),  1.0e-10);

           if (i % 45 == 0) {
               SensorPixel sp = rugged.inverseLocation(lineSensor.getName(), gpPixel, 0, 179);
               Assert.assertEquals(100, sp.getLineNumber(), 1.0e-5);
               Assert.assertEquals(i, sp.getPixelNumber(), 6.0e-9);
           }
       }

    }

   @Test
   public void testAlreadyActive() throws URISyntaxException, IOException, OrekitException, RuggedException {

       DumpManager.activate(tempFolder.newFile());
       try {
           DumpManager.activate(tempFolder.newFile());
           Assert.fail("an exception should have been thrown");
       } catch (RuggedException re) {
           Assert.assertEquals(RuggedMessages.DEBUG_DUMP_ALREADY_ACTIVE, re.getSpecifier());
       } finally {
           DumpManager.deactivate();
       }
   }

   @Test
   public void testNotActive() throws URISyntaxException, IOException, OrekitException, RuggedException {
       try {
           DumpManager.deactivate();
           Assert.fail("an exception should have been thrown");
       } catch (RuggedException re) {
           Assert.assertEquals(RuggedMessages.DEBUG_DUMP_NOT_ACTIVE, re.getSpecifier());
       }
   }

   @Test
   public void testWriteError() throws URISyntaxException, IOException, OrekitException, RuggedException {
       try {
           File dump = tempFolder.newFile();
           dump.setReadOnly();
           DumpManager.activate(dump);
           Assert.fail("an exception should have been thrown");
       } catch (RuggedException re) {
           Assert.assertEquals(RuggedMessages.DEBUG_DUMP_ACTIVATION_ERROR, re.getSpecifier());
       }
   }

}
