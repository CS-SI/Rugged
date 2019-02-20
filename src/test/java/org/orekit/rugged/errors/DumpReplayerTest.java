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
package org.orekit.rugged.errors;


import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.URISyntaxException;
import java.util.ArrayList;
import java.util.List;

import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.junit.Assert;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.data.DataProvidersManager;
import org.orekit.data.DirectoryCrawler;
import org.orekit.rugged.api.Rugged;
import org.orekit.rugged.linesensor.SensorPixel;

public class DumpReplayerTest {

    @Rule
    public TemporaryFolder tempFolder = new TemporaryFolder();

    @Test
    public void testDirectLoc01() throws URISyntaxException, IOException {

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
            Assert.assertEquals(0.0, distance, 6.0e-8);
        }

    }

    @Test
    public void testDirectLoc02() throws URISyntaxException, IOException {

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
    public void testDirectLoc03() throws URISyntaxException, IOException {

        String orekitPath = getClass().getClassLoader().getResource("orekit-data").toURI().getPath();
        DataProvidersManager.getInstance().addProvider(new DirectoryCrawler(new File(orekitPath)));

        String dumpPath = getClass().getClassLoader().getResource("replay/replay-direct-loc-03.txt").toURI().getPath();
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
    public void testDirectLoc04() throws URISyntaxException, IOException {

        String orekitPath = getClass().getClassLoader().getResource("orekit-data").toURI().getPath();
        DataProvidersManager.getInstance().addProvider(new DirectoryCrawler(new File(orekitPath)));

        String dumpPath = getClass().getClassLoader().getResource("replay/replay-direct-loc-04.txt").toURI().getPath();
        File dump = tempFolder.newFile();
        DumpManager.activate(dump);
        DumpReplayer replayer = new DumpReplayer();
        replayer.parse(new File(dumpPath));
        Rugged rugged = replayer.createRugged();
        DumpReplayer.Result[] results = replayer.execute(rugged);
        DumpManager.deactivate();

        Assert.assertEquals(3, results.length);
        for (final DumpReplayer.Result result : results) {
            GeodeticPoint expectedGP = (GeodeticPoint) result.getExpected();
            GeodeticPoint replayedGP = (GeodeticPoint) result.getReplayed();
            double distance = Vector3D.distance(rugged.getEllipsoid().transform(expectedGP),
                                                rugged.getEllipsoid().transform(replayedGP));
            Assert.assertEquals(0.0, distance, 1.0e-8);
        }

        try (FileReader fr = new FileReader(dump);
             BufferedReader br = new BufferedReader(fr)) {
            Assert.assertEquals(12, br.lines().count());
        }

    }

    @Test
    public void testInverseLoc01() throws URISyntaxException, IOException {

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
    public void testInverseLoc02() throws URISyntaxException, IOException {

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
    public void testInverseLoc03() throws URISyntaxException, IOException {

        String orekitPath = getClass().getClassLoader().getResource("orekit-data").toURI().getPath();
        DataProvidersManager.getInstance().addProvider(new DirectoryCrawler(new File(orekitPath)));

        String dumpPath = getClass().getClassLoader().getResource("replay/replay-inverse-loc-03.txt").toURI().getPath();
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
    public void testCorruptedFiles() throws URISyntaxException, IOException {

        String orekitPath = getClass().getClassLoader().getResource("orekit-data").toURI().getPath();
        DataProvidersManager.getInstance().addProvider(new DirectoryCrawler(new File(orekitPath)));

        File folder = new File(getClass().getClassLoader().getResource("replay/replay-direct-loc-01.txt").toURI().getPath()).getParentFile();
        for (final File file : folder.listFiles()) {

            // split all data lines into fields
            final List<String[]> lines = new ArrayList<>();
            try (FileInputStream fis = new FileInputStream(file);
                 InputStreamReader isr = new InputStreamReader(fis, "UTF-8");
                 BufferedReader br = new BufferedReader(isr)) {
                br.lines().
                   filter(line -> {
                       String trimmed = line.trim();
                       return trimmed.length() > 0 && !trimmed.startsWith("#");
                   }).
                   forEach(line -> lines.add(line.split("\\s+")));
            }

            // for each field of each line, delete the field and check parsing fails
            for (int i = 0; i < lines.size(); ++i) {
                for (int j = 0; j < lines.get(i).length; ++j) {
                    final File corrupted = tempFolder.newFile();
                    try (PrintWriter pw = new PrintWriter(corrupted, "UTF-8")) {
                        for (int k = 0; k < lines.size(); ++k) {
                            for (int l = 0; l < lines.get(k).length; ++l) {
                                if (k != i || l != j) {
                                    pw.print(' ');
                                    pw.print(lines.get(k)[l]);
                                }
                            }
                            pw.println();
                        }
                    }
                    try {
                        new DumpReplayer().parse(corrupted);
                        Assert.fail("an exception should have been thrown");
                    } catch (RuggedException re) {
                        Assert.assertEquals(RuggedMessages.CANNOT_PARSE_LINE, re.getSpecifier());
                        Assert.assertEquals(i + 1, ((Integer) re.getParts()[0]).intValue());
                        Assert.assertEquals(corrupted, re.getParts()[1]);
                    }
                    corrupted.delete();
                }
            }
        }

    }

    @Test
    public void testDirectLocIssue376_01() throws URISyntaxException, IOException {

        String orekitPath = getClass().getClassLoader().getResource("orekit-data").toURI().getPath();
        DataProvidersManager.getInstance().addProvider(new DirectoryCrawler(new File(orekitPath)));

        String dumpPath = getClass().getClassLoader().getResource("replay/replay-direct-loc-Issue376-01.txt").toURI().getPath();

        DumpReplayer replayer = new DumpReplayer();
        replayer.parse(new File(dumpPath));
        Rugged rugged = replayer.createRugged();
        DumpReplayer.Result[] results = replayer.execute(rugged);

        GeodeticPoint expectedGP = (GeodeticPoint) results[0].getExpected();
        GeodeticPoint replayedGP = (GeodeticPoint) results[0].getReplayed();
        double distance = Vector3D.distance(rugged.getEllipsoid().transform(expectedGP),
                                            rugged.getEllipsoid().transform(replayedGP));
        Assert.assertEquals(0.0, distance, 1.0e-8);
    }

    @Test
    public void testDirectLocIssue376_02() throws URISyntaxException, IOException {

        String orekitPath = getClass().getClassLoader().getResource("orekit-data").toURI().getPath();
        DataProvidersManager.getInstance().addProvider(new DirectoryCrawler(new File(orekitPath)));

        String dumpPath = getClass().getClassLoader().getResource("replay/replay-direct-loc-Issue376-02.txt").toURI().getPath();

        DumpReplayer replayer = new DumpReplayer();
        replayer.parse(new File(dumpPath));
        Rugged rugged = replayer.createRugged();
        DumpReplayer.Result[] results = replayer.execute(rugged);

        GeodeticPoint expectedGP = (GeodeticPoint) results[0].getExpected();
        GeodeticPoint replayedGP = (GeodeticPoint) results[0].getReplayed();
        double distance = Vector3D.distance(rugged.getEllipsoid().transform(expectedGP),
                                            rugged.getEllipsoid().transform(replayedGP));
        Assert.assertEquals(0.0, distance, 1.0e-8);
    }

    @Test
    public void testDirectLocIssue376_03() throws URISyntaxException, IOException {

        String orekitPath = getClass().getClassLoader().getResource("orekit-data").toURI().getPath();
        DataProvidersManager.getInstance().addProvider(new DirectoryCrawler(new File(orekitPath)));

        String dumpPath = getClass().getClassLoader().getResource("replay/replay-direct-loc-Issue376-03.txt").toURI().getPath();

        DumpReplayer replayer = new DumpReplayer();
        replayer.parse(new File(dumpPath));
        Rugged rugged = replayer.createRugged();
        DumpReplayer.Result[] results = replayer.execute(rugged);

        for (int i= 0; i < results.length; i++) {
            GeodeticPoint expectedGP = (GeodeticPoint) results[i].getExpected();
            GeodeticPoint replayedGP = (GeodeticPoint) results[i].getReplayed();
            double distance = Vector3D.distance(rugged.getEllipsoid().transform(expectedGP),
                    rugged.getEllipsoid().transform(replayedGP));
            Assert.assertEquals(0.0, distance, 5.0e-8);
        }
    }

}
