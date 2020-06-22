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

import static org.junit.Assert.assertTrue;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.net.URISyntaxException;
import java.nio.charset.StandardCharsets;

import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.data.DataContext;
import org.orekit.data.DirectoryCrawler;
import org.orekit.frames.Frame;
import org.orekit.frames.FramesFactory;
import org.orekit.frames.Transform;
import org.orekit.rugged.api.Rugged;
import org.orekit.rugged.linesensor.SensorPixel;

public class DumpTest {

    @Rule
    public TemporaryFolder tempFolder = new TemporaryFolder();
    
    @Test
    public void testGetKeyOrNameCoverage() throws NoSuchMethodException, SecurityException, IllegalAccessException, 
                                          IllegalArgumentException, InvocationTargetException, IOException {
        File tempFile = tempFolder.newFile();
        PrintWriter pw = new PrintWriter(tempFile, "UTF-8");
        Dump dump = new Dump(pw);
        
        Method getKeyOrName = dump.getClass().getDeclaredMethod("getKeyOrName", Frame.class);
        getKeyOrName.setAccessible(true);
        
        String dummyName = "dummy";
        Frame frame = new Frame(FramesFactory.getEME2000(), Transform.IDENTITY, dummyName);
        
        String foundName = (String) getKeyOrName.invoke(dump, frame);
        
        assertTrue(foundName.equals(dummyName));
    }
    
    @Test
    public void testInverseLocNull() throws NoSuchMethodException, SecurityException, IllegalAccessException, 
                                            IllegalArgumentException, InvocationTargetException, IOException {
        File tempFile = tempFolder.newFile();
        PrintWriter pw = new PrintWriter(tempFile, "UTF-8");
        Dump dump = new Dump(pw);
        
        Method dumpInverseLocationResult = dump.getClass().getDeclaredMethod("dumpInverseLocationResult", SensorPixel.class);
        dumpInverseLocationResult.setAccessible(true);

        SensorPixel px = null;
        // just to ensure the test coverage
        dumpInverseLocationResult.invoke(dump, px);
        
        dump.deactivate();
        
        // Check that the created file contains the line "inverse location result: NULL"
        try (FileInputStream   fis = new FileInputStream(tempFile);
             InputStreamReader isr = new InputStreamReader(fis, StandardCharsets.UTF_8);
             BufferedReader    br  = new BufferedReader(isr)) {
               for (String line = br.readLine(); line != null; line = br.readLine()) {
                   final String trimmed = line.trim();
                   if (!(trimmed.length() == 0 || trimmed.startsWith("#"))) {
                       assertTrue(line.contains("inverse location result: NULL"));
                   }
               }
           }
    }
    
    @Test
    public void testDirectLocNull() throws NoSuchMethodException, SecurityException, IllegalAccessException, 
                                            IllegalArgumentException, InvocationTargetException, IOException {
        File tempFile = tempFolder.newFile();
        PrintWriter pw = new PrintWriter(tempFile, "UTF-8");
        Dump dump = new Dump(pw);
        
        Method dumpDirectLocationResult = dump.getClass().getDeclaredMethod("dumpDirectLocationResult", GeodeticPoint.class);
        dumpDirectLocationResult.setAccessible(true);

        GeodeticPoint gp = null;
        // just to ensure the coverage
        dumpDirectLocationResult.invoke(dump, gp);
        dump.deactivate();
        
        // Check that the created file contains the line "inverse location result: NULL"
        try (FileInputStream   fis = new FileInputStream(tempFile);
             InputStreamReader isr = new InputStreamReader(fis, StandardCharsets.UTF_8);
             BufferedReader    br  = new BufferedReader(isr)) {
               for (String line = br.readLine(); line != null; line = br.readLine()) {
                   final String trimmed = line.trim();
                   if (!(trimmed.length() == 0 || trimmed.startsWith("#"))) {
                       assertTrue(line.contains("direct location result: NULL"));
                   }
               }
           }
    }
    
    @Test
    public void testSetMeanPlane() throws NoSuchMethodException, SecurityException, IllegalAccessException, 
                                            IllegalArgumentException, InvocationTargetException, IOException, URISyntaxException {

        String orekitPath = getClass().getClassLoader().getResource("orekit-data").toURI().getPath();
        DataContext.getDefault().getDataProvidersManager().addProvider(new DirectoryCrawler(new File(orekitPath)));

        String dumpPath = getClass().getClassLoader().getResource("replay/replay-inverse-loc-02.txt").toURI().getPath();
        
        // Regenerate a dump in order to write the "sensor mean plane: sensorName xxx lineNumber xxx targetDirection xxx targetDirection xxx ...."
        File dummyDump = tempFolder.newFile();
        DumpManager.activate(dummyDump);
        
        DumpReplayer replayer = new DumpReplayer();
        replayer.parse(new File(dumpPath));
        Rugged rugged = replayer.createRugged();
        replayer.execute(rugged);
        
        DumpManager.deactivate();
        
        // Check that the created dump contains the line " lineNumber xxx targetDirection xxx targetDirection xxx ...."
        try (FileInputStream   fis = new FileInputStream(dummyDump);
             InputStreamReader isr = new InputStreamReader(fis, StandardCharsets.UTF_8);
             BufferedReader    br  = new BufferedReader(isr)) {
            for (String line = br.readLine(); line != null; line = br.readLine()) {
                final String trimmed = line.trim();
                if (!(trimmed.length() == 0 || trimmed.startsWith("#"))) {
                    if (line.contains("lineNumber ")&& line.contains("targetDirection ")) {
                        assertTrue(line.split("targetDirection").length == 6);
                    }
                }
            }
        }
    }
}
