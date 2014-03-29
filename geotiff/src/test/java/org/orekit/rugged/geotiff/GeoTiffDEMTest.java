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
package org.orekit.rugged.geotiff;

import java.io.IOException;
import java.net.URISyntaxException;
import java.util.Enumeration;
import java.util.zip.ZipEntry;
import java.util.zip.ZipFile;

import org.apache.commons.imaging.ImageReadException;
import org.junit.Assert;
import org.junit.Test;
import org.orekit.rugged.geotiff.AngulerUnits;
import org.orekit.rugged.geotiff.GeoTiffDEM;
import org.orekit.rugged.geotiff.GeographicCoordinateSystemType;
import org.orekit.rugged.geotiff.LinearUnits;
import org.orekit.rugged.geotiff.ModelType;
import org.orekit.rugged.geotiff.RasterType;


public class GeoTiffDEMTest {

    @Test
    public void testAster() throws URISyntaxException, IOException, ImageReadException {
        String zipFile = "org/orekit/rugged/geotiff/ASTGTM2_S21E165.zip";
        String zipPath = GeoTiffDEMTest.class.getClassLoader().getResource(zipFile).toURI().getPath();
        ZipFile zip = new ZipFile(zipPath);
        for (Enumeration<? extends ZipEntry> e = zip.entries(); e.hasMoreElements();) {
            ZipEntry entry = e.nextElement();
            if ((!entry.isDirectory()) && entry.getName().endsWith("_dem.tif")) {
                GeoTiffDEM geoTiffDEM = new GeoTiffDEM(zip.getInputStream(entry), entry.getName());
                Assert.assertEquals(ModelType.GEOGRAPHIC,                      geoTiffDEM.getModelType());
                Assert.assertEquals(RasterType.RASTER_PIXEL_IS_AREA,           geoTiffDEM.getRasterType());
                Assert.assertEquals(GeographicCoordinateSystemType.GCS_WGS_84, geoTiffDEM.getCSType());
                Assert.assertEquals(LinearUnits.METER,                         geoTiffDEM.getLinearUnits());
                Assert.assertEquals(AngulerUnits.DEGREE,                       geoTiffDEM.getAngularUnits());
                Assert.assertEquals(6, geoTiffDEM.getTiePoint().length);
                Assert.assertEquals(  0.0,               geoTiffDEM.getTiePoint()[0], 1.0e-10);
                Assert.assertEquals(  0.0,               geoTiffDEM.getTiePoint()[1], 1.0e-10);
                Assert.assertEquals(  0.0,               geoTiffDEM.getTiePoint()[2], 1.0e-10);
                Assert.assertEquals(164.9998611111111,   geoTiffDEM.getTiePoint()[3], 1.0e-10);
                Assert.assertEquals(-19.999861111111112, geoTiffDEM.getTiePoint()[4], 1.0e-10);
                Assert.assertEquals(  0.0,               geoTiffDEM.getTiePoint()[5], 1.0e-10);
            }
        }
        zip.close();
    }

}
