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
package org.orekit.rugged.aster;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.net.URISyntaxException;
import java.net.URL;
import java.util.ArrayList;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import org.apache.commons.math3.util.FastMath;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;
import org.orekit.rugged.api.RuggedException;
import org.orekit.rugged.raster.SimpleTile;
import org.orekit.rugged.raster.SimpleTileFactory;
import org.orekit.rugged.raster.TileFactory;


public class AsterTileUpdaterTest {

    @Test
    public void testAster() throws RuggedException {

        File folder = warningFile.getParentFile();
        List<int[]> corners = getCorners(folder);
        if (corners.isEmpty()) {
            // no ASTER data available in the test resources
            // warn user, but don't allow the test to fail
            displayWarning();
        } else {

            TileFactory<SimpleTile> factory = new SimpleTileFactory();
            AsterTileUpdater updater = new AsterTileUpdater(folder);
            for (int[] corner : corners) {

                SimpleTile tile = factory.createTile();
                updater.updateTile(FastMath.toRadians(corner[0] + 0.2),
                                   FastMath.toRadians(corner[1] + 0.7),
                                   tile);
                tile.tileUpdateCompleted();

                Assert.assertEquals(corner[0] + 1.0 / 7200.0, FastMath.toDegrees(tile.getMinimumLatitude()),  1.0e-10);
                Assert.assertEquals(corner[1] - 1.0 / 7200.0, FastMath.toDegrees(tile.getMinimumLongitude()), 1.0e-10);
                Assert.assertEquals(1.0 / 3600.0, FastMath.toDegrees(tile.getLatitudeStep()), 1.0e-10);
                Assert.assertEquals(1.0 / 3600.0, FastMath.toDegrees(tile.getLongitudeStep()), 1.0e-10);
                Assert.assertTrue(tile.getMinElevation() <  9000.0);
                Assert.assertTrue(tile.getMaxElevation() > -1000.0);

            }

        }

    }

    @Before
    public void setUp() {
        try {
            String warningResource = "org/orekit/rugged/geotiff/ASTER-files-warning.txt";
            URL url = AsterTileUpdaterTest.class.getClassLoader().getResource(warningResource);
            warningFile = new File(url.toURI().getPath());
        } catch (URISyntaxException urise) {
            Assert.fail(urise.getLocalizedMessage());
        }
    }

    private List<int[]> getCorners(File folder) {
        Pattern patter = Pattern.compile("ASTGTM2_([NS]\\d\\d)([EW]\\d\\d\\d)\\.zip$");
        List<int[]> asterCorners = new ArrayList<int[]>();
        for (final File file : folder.listFiles()) {
            Matcher matcher = patter.matcher(file.getName());
            if (matcher.matches()) {
                int latCode = (matcher.group(1).startsWith("N") ? 1 : -1) * Integer.parseInt(matcher.group(1).substring(1));
                int lonCode = (matcher.group(2).startsWith("E") ? 1 : -1) * Integer.parseInt(matcher.group(2).substring(1));
                asterCorners.add(new int[] { latCode, lonCode });
            }
        }
        return asterCorners;
    }

    private void displayWarning() {
        try {
            System.err.println("######  " + warningFile.getAbsolutePath() + "  ######");
            BufferedReader reader = new BufferedReader(new FileReader(warningFile));
            for (String line = reader.readLine(); line != null; line = reader.readLine()) {
                System.err.println(line);
            }
            reader.close();
            System.err.println("######  " + warningFile.getAbsolutePath() + "  ######");
        } catch (IOException ioe) {
            Assert.fail(ioe.getLocalizedMessage());
        }
    }

    private File warningFile;

}
