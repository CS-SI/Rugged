/* Copyright 2013-2015 CS Systèmes d'Information
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


import java.io.File;
import java.io.IOException;
import java.net.URISyntaxException;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.junit.Assert;
import org.junit.Test;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.data.DataProvidersManager;
import org.orekit.data.DirectoryCrawler;
import org.orekit.errors.OrekitException;
import org.orekit.rugged.api.Rugged;
import org.orekit.rugged.linesensor.SensorPixel;

public class DumpReplayerTest {

    @Test
    public void testDirectLoc01() throws URISyntaxException, IOException, OrekitException, RuggedException {

        String orekitPath = getClass().getClassLoader().getResource("orekit-data").toURI().getPath();
        DataProvidersManager.getInstance().addProvider(new DirectoryCrawler(new File(orekitPath)));

        String dumpPath = getClass().getClassLoader().getResource("replay/replay-direct-loc-01.txt").toURI().getPath();
        DumpReplayer replayer = new DumpReplayer();
        replayer.parse(new File(dumpPath));
        Rugged rugged = replayer.createRugged();
        DumpReplayer.Result[] results = replayer.execute(rugged);

        Assert.assertEquals(5, results.length);
        for (final DumpReplayer.Result result : results) {
            GeodeticPoint expectedGP = (GeodeticPoint) result.getExpected();
            GeodeticPoint replayedGP = (GeodeticPoint) result.getReplayed();
            double distance = Vector3D.distance(rugged.getEllipsoid().transform(expectedGP),
                                                rugged.getEllipsoid().transform(replayedGP));
            Assert.assertEquals(0.0, distance, 1.0e-8);
        }

    }

    @Test
    public void testDirectLoc02() throws URISyntaxException, IOException, OrekitException, RuggedException {

        String orekitPath = getClass().getClassLoader().getResource("orekit-data").toURI().getPath();
        DataProvidersManager.getInstance().addProvider(new DirectoryCrawler(new File(orekitPath)));

        String dumpPath = getClass().getClassLoader().getResource("replay/replay-direct-loc-02.txt").toURI().getPath();
        DumpReplayer replayer = new DumpReplayer();
        replayer.parse(new File(dumpPath));
        Rugged rugged = replayer.createRugged();
        DumpReplayer.Result[] results = replayer.execute(rugged);

        Assert.assertEquals(1, results.length);
        for (final DumpReplayer.Result result : results) {
            GeodeticPoint expectedGP = (GeodeticPoint) result.getExpected();
            GeodeticPoint replayedGP = (GeodeticPoint) result.getReplayed();
            double distance = Vector3D.distance(rugged.getEllipsoid().transform(expectedGP),
                                                rugged.getEllipsoid().transform(replayedGP));
            Assert.assertEquals(0.0, distance, 1.0e-8);
        }

    }

    @Test
    public void testInverseLoc01() throws URISyntaxException, IOException, OrekitException, RuggedException {

        String orekitPath = getClass().getClassLoader().getResource("orekit-data").toURI().getPath();
        DataProvidersManager.getInstance().addProvider(new DirectoryCrawler(new File(orekitPath)));

        String dumpPath = getClass().getClassLoader().getResource("replay/replay-inverse-loc-01.txt").toURI().getPath();
        DumpReplayer replayer = new DumpReplayer();
        replayer.parse(new File(dumpPath));
        Rugged rugged = replayer.createRugged();
        DumpReplayer.Result[] results = replayer.execute(rugged);

        Assert.assertEquals(1, results.length);
        for (final DumpReplayer.Result result : results) {
            SensorPixel expectedSP = (SensorPixel) result.getExpected();
            SensorPixel replayedSP = (SensorPixel) result.getReplayed();
            Assert.assertEquals(expectedSP.getLineNumber(),  replayedSP.getLineNumber(),  1.0e-6);
            Assert.assertEquals(expectedSP.getPixelNumber(), replayedSP.getPixelNumber(), 1.0e-6);
        }

    }

    @Test
    public void testInverseLoc02() throws URISyntaxException, IOException, OrekitException, RuggedException {

        String orekitPath = getClass().getClassLoader().getResource("orekit-data").toURI().getPath();
        DataProvidersManager.getInstance().addProvider(new DirectoryCrawler(new File(orekitPath)));

        String dumpPath = getClass().getClassLoader().getResource("replay/replay-inverse-loc-02.txt").toURI().getPath();
        DumpReplayer replayer = new DumpReplayer();
        replayer.parse(new File(dumpPath));
        Rugged rugged = replayer.createRugged();
        DumpReplayer.Result[] results = replayer.execute(rugged);

        Assert.assertEquals(3, results.length);
        for (final DumpReplayer.Result result : results) {
            SensorPixel expectedSP = (SensorPixel) result.getExpected();
            SensorPixel replayedSP = (SensorPixel) result.getReplayed();
            Assert.assertEquals(expectedSP.getLineNumber(),  replayedSP.getLineNumber(),  1.0e-6);
            Assert.assertEquals(expectedSP.getPixelNumber(), replayedSP.getPixelNumber(), 1.0e-6);
        }

    }

    @Test
    public void testInverseLoc03() throws URISyntaxException, IOException, OrekitException, RuggedException {

        String orekitPath = getClass().getClassLoader().getResource("orekit-data").toURI().getPath();
        DataProvidersManager.getInstance().addProvider(new DirectoryCrawler(new File(orekitPath)));

        String dumpPath = getClass().getClassLoader().getResource("replay/replay-inverse-loc-03.txt").toURI().getPath();
        DumpReplayer replayer = new DumpReplayer();
        replayer.parse(new File(dumpPath));
        Rugged rugged = replayer.createRugged();
        DumpReplayer.Result[] results = replayer.execute(rugged);

        Assert.assertEquals(1, results.length);
        for (final DumpReplayer.Result result : results) {
            RuggedException expectedSP = (RuggedException) result.getExpected();
            RuggedException replayedSP = (RuggedException) result.getReplayed();
            Assert.assertEquals(expectedSP.getSpecifier(), replayedSP.getSpecifier());
            Assert.assertEquals(expectedSP.getParts().length, replayedSP.getParts().length);
            for (int i = 0; i < expectedSP.getParts().length; ++i) {
                Assert.assertEquals(expectedSP.getParts()[i].toString(), replayedSP.getParts()[i].toString());
            }
        }

    }

}
