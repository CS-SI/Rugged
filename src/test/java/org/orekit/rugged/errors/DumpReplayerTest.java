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


import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.io.PrintWriter;
import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.net.URISyntaxException;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import org.hipparchus.analysis.differentiation.DSFactory;
import org.hipparchus.analysis.differentiation.DerivativeStructure;
import org.hipparchus.geometry.euclidean.threed.FieldVector3D;
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
import org.orekit.rugged.refraction.MultiLayerModel;
import org.orekit.rugged.utils.DSGenerator;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeScalesFactory;
import org.orekit.utils.ParameterDriver;

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
    public void testDirectLocNull() throws URISyntaxException, IOException {

        String orekitPath = getClass().getClassLoader().getResource("orekit-data").toURI().getPath();
        DataProvidersManager.getInstance().addProvider(new DirectoryCrawler(new File(orekitPath)));

        File tempFile = tempFolder.newFile();
        try (FileOutputStream   fos = new FileOutputStream(tempFile);
             OutputStreamWriter osw = new OutputStreamWriter(fos, StandardCharsets.UTF_8);
             BufferedWriter     bw  = new BufferedWriter(osw)) {

            // Create a dump file with NULL result for direct location
            createDataForCreateRugged(bw);
            bw.write("direct location: date 2012-01-01T12:29:30.85Z position 0.0e+00  0.0e+00  0.0e+00 " + 
                     "los -2.2e-02 -9.1e-02 9.9e-01 lightTime false aberration false refraction false");
            bw.newLine();
            bw.write("direct location result: NULL");           
         }
        DumpReplayer replayer = new DumpReplayer();
        replayer.parse(tempFile);
        Rugged rugged = replayer.createRugged();
        DumpReplayer.Result[] results = replayer.execute(rugged);

        assertTrue(results[0].getExpected() == null);
        tempFile.delete();
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
    public void testInverseLocNull() throws URISyntaxException, IOException {

        String orekitPath = getClass().getClassLoader().getResource("orekit-data").toURI().getPath();
        DataProvidersManager.getInstance().addProvider(new DirectoryCrawler(new File(orekitPath)));

        File tempFile = tempFolder.newFile();
        try (FileOutputStream   fos = new FileOutputStream(tempFile);
             OutputStreamWriter osw = new OutputStreamWriter(fos, StandardCharsets.UTF_8);
             BufferedWriter     bw  = new BufferedWriter(osw)) {

            // Create a dump file with NULL result for inverse location
            createDataForInvLocResult(bw);
            bw.write("inverse location result: NULL");           
         }
        DumpReplayer replayer = new DumpReplayer();
        replayer.parse(tempFile);
        Rugged rugged = replayer.createRugged();
        DumpReplayer.Result[] results = replayer.execute(rugged);

        assertTrue(results[0].getExpected() == null);
        tempFile.delete();
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

    @Test
    public void testCreateRuggedWithAtmosphere() throws URISyntaxException, IOException {

        String orekitPath = getClass().getClassLoader().getResource("orekit-data").toURI().getPath();
        DataProvidersManager.getInstance().addProvider(new DirectoryCrawler(new File(orekitPath)));

        File tempFile = tempFolder.newFile();
        try (FileOutputStream   fos = new FileOutputStream(tempFile);
             OutputStreamWriter osw = new OutputStreamWriter(fos, StandardCharsets.UTF_8);
             BufferedWriter     bw  = new BufferedWriter(osw)) {

            // CreateRugged with atmospheric refraction
            bw.write("direct location: date 2012-01-01T12:30:00.0Z position 1.5e+00  0.e+00 -2.e-01 "+ 
                     "los  0.e+00 -7.5e-01  6.5e-01 lightTime true aberration true refraction true");
            bw.newLine();
            createDataForCreateRugged(bw);
         }
        DumpReplayer replayer = new DumpReplayer();
        replayer.parse(tempFile);
        Rugged rugged = replayer.createRugged();
        
        assertTrue(rugged.getRefractionCorrection().getClass().isInstance(new MultiLayerModel(rugged.getEllipsoid())));
        tempFile.delete();
    }

    @Test
    public void testCreateRuggedNoDEMdata() throws URISyntaxException, IOException {

        String orekitPath = getClass().getClassLoader().getResource("orekit-data").toURI().getPath();
        DataProvidersManager.getInstance().addProvider(new DirectoryCrawler(new File(orekitPath)));

        File tempFile = tempFolder.newFile();
        try (FileOutputStream   fos = new FileOutputStream(tempFile);
             OutputStreamWriter osw = new OutputStreamWriter(fos, StandardCharsets.UTF_8);
             BufferedWriter     bw  = new BufferedWriter(osw)) {

            // CreateRugged with atmospheric refraction
            bw.write("direct location: date 2012-01-01T12:30:00.0Z position 1.5e+00  0.e+00 -2.e-01 "+ 
                     "los  0.e+00 -7.5e-01  6.5e-01 lightTime true aberration true refraction false");
            bw.newLine();
            createDataForCreateRugged(bw);
            bw.write("algorithm: DUVENHAGE");
         }
        DumpReplayer replayer = new DumpReplayer();
        replayer.parse(tempFile);
        Rugged rugged = replayer.createRugged();

        try {
            replayer.execute(rugged);
            Assert.fail("an exception should have been thrown");
        } catch (RuggedException re) {
            // as the execution stops in the TilesCache: one must reset the DumpManager state
            DumpManager.endNicely();
            Assert.assertEquals(RuggedMessages.NO_DEM_DATA, re.getSpecifier());
        } 
        tempFile.delete();
    }
    
    @Test
    public void testLineParserBadKey() throws URISyntaxException, IOException {

        String orekitPath = getClass().getClassLoader().getResource("orekit-data").toURI().getPath();
        DataProvidersManager.getInstance().addProvider(new DirectoryCrawler(new File(orekitPath)));

        File tempFile = tempFolder.newFile();
        try (FileOutputStream   fos = new FileOutputStream(tempFile);
             OutputStreamWriter osw = new OutputStreamWriter(fos, StandardCharsets.UTF_8);
             BufferedWriter     bw  = new BufferedWriter(osw)) {
            // this empty line to ensure coverage
            bw.write("");
            bw.newLine();
            // LineParser.parse: case bad key
            bw.write("dummy : dummy");
        }
        DumpReplayer replayer = new DumpReplayer();
        try {
            replayer.parse(tempFile);
            Assert.fail("an exception should have been thrown");
        } catch (RuggedException re) {
            Assert.assertEquals(RuggedMessages.CANNOT_PARSE_LINE, re.getSpecifier());
            Assert.assertEquals(2, ((Integer) re.getParts()[0]).intValue());
            Assert.assertEquals(tempFile, re.getParts()[1]);
            Assert.assertTrue(re.getParts()[2].toString().contains("dummy : dummy"));
        }
        tempFile.delete();
    }
    
    @Test
    public void testLineParserEndColon() throws URISyntaxException, IOException {

        String orekitPath = getClass().getClassLoader().getResource("orekit-data").toURI().getPath();
        DataProvidersManager.getInstance().addProvider(new DirectoryCrawler(new File(orekitPath)));

        File tempFile = tempFolder.newFile();
        try (FileOutputStream   fos = new FileOutputStream(tempFile);
             OutputStreamWriter osw = new OutputStreamWriter(fos, StandardCharsets.UTF_8);
             BufferedWriter     bw  = new BufferedWriter(osw)) {
            // LineParser.parse: case colon at end of line
            bw.write("direct location result:");
        }
        DumpReplayer replayer = new DumpReplayer();
        try {
            replayer.parse(tempFile);
            Assert.fail("an exception should have been thrown");
        } catch (RuggedException re) {
            Assert.assertEquals(RuggedMessages.CANNOT_PARSE_LINE, re.getSpecifier());
            Assert.assertEquals(1, ((Integer) re.getParts()[0]).intValue());
            Assert.assertEquals(tempFile, re.getParts()[1]);
        }
        tempFile.delete();
    }

    @Test
    public void testLineParserNoColon() throws URISyntaxException, IOException {

        String orekitPath = getClass().getClassLoader().getResource("orekit-data").toURI().getPath();
        DataProvidersManager.getInstance().addProvider(new DirectoryCrawler(new File(orekitPath)));

        File tempFile = tempFolder.newFile();
        try (FileOutputStream   fos = new FileOutputStream(tempFile);
                OutputStreamWriter osw = new OutputStreamWriter(fos, StandardCharsets.UTF_8);
                BufferedWriter     bw  = new BufferedWriter(osw)) {
            // LineParser.parse case no colon
            bw.write("sensorName s0 nbPixels 200 position  1.5e+00  0.0e+00 -2.0e-01");
        }
        DumpReplayer replayer = new DumpReplayer();
        try {
            replayer.parse(tempFile);
            Assert.fail("an exception should have been thrown");
        } catch (RuggedException re) {
            Assert.assertEquals(RuggedMessages.CANNOT_PARSE_LINE, re.getSpecifier());
            Assert.assertEquals(1, ((Integer) re.getParts()[0]).intValue());
            Assert.assertEquals(tempFile, re.getParts()[1]);
        }
        tempFile.delete();
    }

    @Test
    public void testParsedSensorGetDateGetLineCoverage() throws URISyntaxException, ClassNotFoundException, InstantiationException, 
                                                                IllegalAccessException, IllegalArgumentException, 
                                                                InvocationTargetException, NoSuchMethodException, SecurityException {

        String orekitPath = getClass().getClassLoader().getResource("orekit-data").toURI().getPath();
        DataProvidersManager.getInstance().addProvider(new DirectoryCrawler(new File(orekitPath)));

        // ParsedSensor inner class
        Class<?> innerClass = Class.forName("org.orekit.rugged.errors.DumpReplayer$ParsedSensor");
        Constructor<?>[] constructors = innerClass.getDeclaredConstructors();
        constructors[0].setAccessible(true);
        Object parsedSensor = constructors[0].newInstance("dummy");


        Method getLine = innerClass.getDeclaredMethod("getLine", AbsoluteDate.class);
        getLine.setAccessible(true);
        Method getDate = innerClass.getDeclaredMethod("getDate", double.class);
        getDate.setAccessible(true);
        Method setDatation = innerClass.getDeclaredMethod("setDatation", double.class, AbsoluteDate.class);
        setDatation.setAccessible(true);

        // datation with only one data
        AbsoluteDate date0 = new AbsoluteDate("2012-01-06T02:27:16.139", TimeScalesFactory.getUTC());
        setDatation.invoke(parsedSensor, 100., date0);

        AbsoluteDate date = date0.shiftedBy(5.);
        double foundLine = (double) getLine.invoke(parsedSensor, date);
        assertEquals(100., foundLine, 1.e-15);

        double line = 105.;
        AbsoluteDate foundDate = (AbsoluteDate) getDate.invoke(parsedSensor, line);
        assertEquals("2012-01-06T02:27:16.139",foundDate.toString(TimeScalesFactory.getUTC()));

        // add datations data
        AbsoluteDate date1 = date0.shiftedBy(10.);
        AbsoluteDate date2 = date1.shiftedBy(10.);

        setDatation.invoke(parsedSensor, 120., date1);
        setDatation.invoke(parsedSensor, 150., date2);
        foundLine = (double) getLine.invoke(parsedSensor, date);
        assertEquals(110., foundLine, 1.e-15);

        date = date2.shiftedBy(5.);
        foundLine = (double) getLine.invoke(parsedSensor, date);
        assertEquals(165., foundLine, 1.e-15);

        date = date0.shiftedBy(-5.);
        foundLine = (double) getLine.invoke(parsedSensor, date);
        assertEquals(90., foundLine, 1.e-15);

    }
    
    @Test
    public void testParsedSensorGetLOSCoverage() throws URISyntaxException, ClassNotFoundException, InstantiationException, 
                                                        IllegalAccessException, IllegalArgumentException, 
                                                        InvocationTargetException, NoSuchMethodException, SecurityException {

        String orekitPath = getClass().getClassLoader().getResource("orekit-data").toURI().getPath();
        DataProvidersManager.getInstance().addProvider(new DirectoryCrawler(new File(orekitPath)));

        // ParsedSensor inner class
        Class<?> innerClass = Class.forName("org.orekit.rugged.errors.DumpReplayer$ParsedSensor");
        Constructor<?>[] constructors = innerClass.getDeclaredConstructors();
        constructors[0].setAccessible(true);
        Object parsedSensor = constructors[0].newInstance("dummy");

        AbsoluteDate date = new AbsoluteDate("2012-01-06T02:27:16.139", TimeScalesFactory.getUTC());

        // ParsedSensor.getLOS RunTimeException
        Method getLos = innerClass.getDeclaredMethod("getLOS", int.class, AbsoluteDate.class);
        getLos.setAccessible(true);
        try {
            getLos.invoke(parsedSensor, 1, date);
            Assert.fail("an exception should have been thrown");
        } catch (InvocationTargetException ite) {
            RuggedInternalError rie = (RuggedInternalError) ite.getTargetException();
            assertEquals(RuggedMessages.INTERNAL_ERROR, rie.getSpecifier());
            assertEquals("https://gitlab.orekit.org/orekit/rugged/issues", rie.getParts()[0]);
            assertTrue(rie.getMessage(Locale.FRENCH).startsWith("erreur interne"));
        }
    }
    
    @Test
    public void testParsedSensorLOSCoverage() throws URISyntaxException, ClassNotFoundException, InstantiationException, 
                                                     IllegalAccessException, IllegalArgumentException, 
                                                     InvocationTargetException, NoSuchMethodException, SecurityException {

        String orekitPath = getClass().getClassLoader().getResource("orekit-data").toURI().getPath();
        DataProvidersManager.getInstance().addProvider(new DirectoryCrawler(new File(orekitPath)));

        // ParsedSensor inner class
        Class<?> innerClass = Class.forName("org.orekit.rugged.errors.DumpReplayer$ParsedSensor");
        Constructor<?>[] constructors = innerClass.getDeclaredConstructors();
        constructors[0].setAccessible(true);
        Object parsedSensor = constructors[0].newInstance("dummy");

        AbsoluteDate date0 = new AbsoluteDate("2012-01-06T02:27:16.139", TimeScalesFactory.getUTC());
        AbsoluteDate date = date0.shiftedBy(5.);
        AbsoluteDate date1 = date0.shiftedBy(10.);
        AbsoluteDate date2 = date1.shiftedBy(10.);

        // ParsedSensor.setLos
        Method setLos = innerClass.getDeclaredMethod("setLOS", AbsoluteDate.class, int.class, Vector3D.class);
        setLos.setAccessible(true);
        setLos.invoke(parsedSensor, date, 1, new Vector3D(0, 0, 0));
        setLos.invoke(parsedSensor, date1, 100, new Vector3D(1, 1, 1));
        setLos.invoke(parsedSensor, date2, 200, new Vector3D(2, 2, 2));
        setLos.invoke(parsedSensor, date2.shiftedBy(10.), 200, new Vector3D(3, 3, 3));
 
        // ParsedSensor.getLOSDerivatives
        // Needs some LOS to be set
        Method getLOSDerivatives = innerClass.getDeclaredMethod("getLOSDerivatives", int.class, AbsoluteDate.class, DSGenerator.class);
        getLOSDerivatives.setAccessible(true);

        final DSFactory factory = new DSFactory(1, 1);
        DSGenerator generator = new DSGenerator() {
            @Override
            public List<ParameterDriver> getSelected() {
                return null;
            }
            @Override
            public DerivativeStructure constant(final double value) {
                return factory.constant(value);
            }
            @Override
            public DerivativeStructure variable(final ParameterDriver driver) {
                return null;
            }
        };
        @SuppressWarnings("unchecked")
        FieldVector3D<DerivativeStructure> fv= (FieldVector3D<DerivativeStructure>) getLOSDerivatives.invoke(parsedSensor, 1, date, generator);
        assertEquals(0., fv.getX().getValue(), 1.e-15);
        assertEquals(0., fv.getY().getValue(), 1.e-15);
        assertEquals(0., fv.getZ().getValue(), 1.e-15);
        
        
       // ParsedSensor.getParametersDrivers
       Method getParametersDrivers = innerClass.getDeclaredMethod("getParametersDrivers");
       getParametersDrivers.setAccessible(true);
       
       getParametersDrivers.invoke(parsedSensor);
    }
    
    private void createDataForCreateRugged(BufferedWriter bw) throws IOException {
        
        bw.write("ellipsoid: ae  6.378137e+06 f  3.35e-03 frame ITRF_CIO_CONV_2010_SIMPLE_EOP");
        bw.newLine();
        bw.write("span: minDate 2012-01-01T12:29:00.85Z maxDate 2012-01-01T12:30:00.15Z " + 
                 "tStep  1.e-03 tolerance  5.e+00 inertialFrame EME2000");
        bw.newLine();
        bw.write("transform: index 150 body r -8.0e-01 -3.4e-04  4.8e-04 -5.8e-01 Ω -8.7e-08  1.2e-09 -7.3e-05 " + 
                 "ΩDot -1.6e-16  8.9e-17  1.9e-19 spacecraft p  1.3e+04  3.1e+03 -7.1e+06 v -3.1e+01 -8.0e+00  8.2e+00 " + 
                 "a -9.3e-01 -8.3e+00  1.3e-03 r -6.8e-01  4.1e-01 -3.8e-01  4.6e-01 Ω -1.e-03  1.9e-04  1.6e-04 " + 
                 "ΩDot -3.6e-07  2.0e-07 -1.2e-06");
        bw.newLine();
    }
    
    private void createDataForInvLocResult(BufferedWriter bw) throws IOException {
        
        bw.write("inverse location: sensorName s0 latitude  1.4e+00 longitude -8.8e-01 elevation 3.1e+01 minLine -23040 maxLine 39851 " +
                 "lightTime false aberration false refraction false");
        bw.newLine();
        bw.write("ellipsoid: ae  6.378e+06 f 3.35e-03 frame ITRF_CIO_CONV_2010_SIMPLE_EOP");
        bw.newLine();
        bw.write("span: minDate 2015-07-07T18:38:55.0Z maxDate 2015-07-07T18:40:35.8Z tStep 1.e-01 tolerance 1.e+01 inertialFrame EME2000");
        bw.newLine();
        bw.write("transform: index 516 body r -2.2e-01 -7.3e-04 1.8e-04 -9.7e-01 Ω -1.1e-07 3.6e-09 -7.2e-05 " + 
                 "ΩDot 0. 0. 0. spacecraft p -3.6e+02 -4.2e+02 -7.1e+06 v -7.4e+01 -3.4e+02 -1.8e-01 " + 
                 "a 0. 0. 0. r -6.2e-02 7.4e-01 6.5e-01 4.1e-02 Ω 0. 0. 0. " + 
                 "ΩDot 0. 0. 0.");
        bw.newLine();
        bw.write("sensor: sensorName s0 nbPixels 2552 position  0. 0. 0.");
        bw.newLine();
        bw.write("sensor mean plane: sensorName s0 minLine -23040 maxLine 39851 maxEval 50 accuracy 1.e-02 " + 
                 "normal 9.e-01 -2.6e-02  1.8e-02 cachedResults 1 lineNumber 2.4e+04 date 2015-07-07T18:40:12.4Z " + 
                 "target 5.8e+05 -7.1e+05 6.2e+06 targetDirection -1.5e-02 8.9e-02 9.9e-01 -2.0e-07 2.1e-08 -2.0e-07");
        bw.newLine();
        bw.write("sensor datation: sensorName s0 lineNumber 8.4e+03 date 2015-07-07T18:39:46.5Z");
        bw.newLine();
        bw.write("sensor rate: sensorName s0 lineNumber 2.4e+04 rate 6.3e+02");
        bw.newLine();
    }
}
