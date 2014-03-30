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

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;

import org.apache.commons.imaging.FormatCompliance;
import org.apache.commons.imaging.ImageReadException;
import org.apache.commons.imaging.common.bytesource.ByteSource;
import org.apache.commons.imaging.common.bytesource.ByteSourceInputStream;
import org.apache.commons.imaging.formats.tiff.TiffContents;
import org.apache.commons.imaging.formats.tiff.TiffField;
import org.apache.commons.imaging.formats.tiff.TiffReader;
import org.apache.commons.imaging.formats.tiff.constants.GeoTiffTagConstants;

/** Loader for GeoTiff Digital Elevation Model similar to ASTER files.
 * @author Luc Maisonobe
 */
public class GeoTiffDEM {

    /** Pixels scales. */
    private double[] pixelsScales;

    /** Tie point. */
    private double[] tiePoint;

    /** Model type. */
    private ModelType modelType;

    /** Raster type. */
    private RasterType rasterType;

    /** Geographic type. */
    private GeographicCoordinateSystemType csType;

    /** Linear units. */
    private LinearUnits linearUnits;

    /** Angular units. */
    private AngulerUnits angulerUnits;

    /** Load a Digital Elevation Model.
     * @param file file containing the Digital Elevation Model
     * @exception IOException if file cannot be loaded
     * @exception ImageReadException if TIFF data cannot be extracted
     */
    public GeoTiffDEM(final File file) throws IOException, ImageReadException {
        InputStream stream = null;
        try {
            stream = new FileInputStream(file);
            load(stream, file.getName());
        } finally {
            if (stream != null) {
                stream.close();
            }
        }
    }

    /** Load a Digital Elevation Model.
     * @param stream stream containing the Digital Elevation Model
     * @param fileName name of the file
     * @exception IOException if file cannot be loaded
     * @exception ImageReadException if TIFF data cannot be extracted
     */
    public GeoTiffDEM(final InputStream stream, final String fileName)
        throws IOException, ImageReadException {
        load(stream, fileName);
    }

    /** Load a Digital Elevation Model.
     * @param stream stream containing the Digital Elevation Model
     * @param fileName name of the file
     * @exception IOException if file cannot be loaded
     * @exception ImageReadException if TIFF data cannot be extracted
     */
    private void load(final InputStream stream, final String fileName)
        throws IOException, ImageReadException {

        // read TIFF
        final TiffReader reader = new TiffReader(true);
        final ByteSource source = new ByteSourceInputStream(stream, fileName);
        final TiffContents content = reader.readContents(source, null, FormatCompliance.getDefault());

        // extract GeoTiff data
        pixelsScales = content.findField(GeoTiffTagConstants.EXIF_TAG_MODEL_PIXEL_SCALE_TAG).getDoubleArrayValue();
        tiePoint     = content.findField(GeoTiffTagConstants.EXIF_TAG_MODEL_TIEPOINT_TAG).getDoubleArrayValue();

        final TiffField geoAsciiParams  = content.findField(GeoTiffTagConstants.EXIF_TAG_GEO_ASCII_PARAMS_TAG);
        final TiffField geoDoubleParams = content.findField(GeoTiffTagConstants.EXIF_TAG_GEO_DOUBLE_PARAMS_TAG);
        final int[] geoKeyDirectory     = content.findField(GeoTiffTagConstants.EXIF_TAG_GEO_KEY_DIRECTORY_TAG).getIntArrayValue();
        final int keyDirectoryVersion = geoKeyDirectory[0];
        final int keyRevision         = geoKeyDirectory[1];
        final int minorRevision       = geoKeyDirectory[2];
        final int numberOfKeys        = geoKeyDirectory[3];
        for (int i = 0; i < numberOfKeys; ++i) {
            final GeoKey geoKey   = GeoKey.getKey(geoKeyDirectory[4 * i + 4]);
            final int location    = geoKeyDirectory[4 * i + 5];
            final int count       = geoKeyDirectory[4 * i + 6];
            final int valueOffset = geoKeyDirectory[4 * i + 7];
            switch(geoKey) {
            case GT_MODEL_TYPE:
                modelType = ModelType.getType(getShort(geoKey, location, count, valueOffset));
                break;
            case GT_RASTER_TYPE:
                rasterType = RasterType.getType(getShort(geoKey, location, count, valueOffset));
                break;
            case GEOGRAPHIC_TYPE:
                csType = GeographicCoordinateSystemType.getType(getShort(geoKey, location, count, valueOffset));
                break;
            case GEOG_LINEAR_UNITS:
                linearUnits = LinearUnits.getUnits(getShort(geoKey, location, count, valueOffset));
                break;
            case GEOG_ANGULAR_UNITS:
                angulerUnits = AngulerUnits.getUnits(getShort(geoKey, location, count, valueOffset));
                break;
            default:
                // TODO: support the other geo keys
                throw new ImageReadException(geoKey + " unsupported for now");
            }
        }

    }

    /** Get a short from the geo key directory.
     * @param key being parsed
     * @param location location of the short
     * @param count number of elements
     * @param valueOffset offset of the value
     * @return short value
     * @exception ImageReadException if value cannot be retrieved
     */
    private int getShort(final GeoKey key, final int location, final int count, final int valueOffset)
        throws ImageReadException {
        if (location != 0 && count != 1) {
            throw new ImageReadException("cannot retrieve value for " + key);
        }
        return valueOffset;
    }

    /** Get the pixels scales.
     * @return pixels scales
     */
    public double[] getPixelScales() {
        return pixelsScales.clone();
    }

    /** Get the tie point.
     * @return tie point
     */
    public double[] getTiePoint() {
        return tiePoint.clone();
    }

    /** Get the model type.
     * @return model type
     */
    public ModelType getModelType() {
        return modelType;
    }

    /** Get the raster type.
     * @return raster type
     */
    public RasterType getRasterType() {
        return rasterType;
    }

    /** Get the geographic coordinate system type.
     * @return geographic coordinate system type
     */
    public GeographicCoordinateSystemType getCSType() {
        return csType;
    }

    /** Get the linear units.
     * @return linear units
     */
    public LinearUnits getLinearUnits() {
        return linearUnits;
    }

    /** Get the angular units.
     * @return angular units
     */
    public AngulerUnits getAngularUnits() {
        return angulerUnits;
    }


}
