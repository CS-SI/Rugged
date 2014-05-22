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
import java.io.IOException;
import java.io.InputStream;
import java.nio.ByteOrder;
import java.util.Enumeration;
import java.util.zip.ZipEntry;
import java.util.zip.ZipFile;

import org.apache.commons.imaging.FormatCompliance;
import org.apache.commons.imaging.ImageReadException;
import org.apache.commons.imaging.common.bytesource.ByteSource;
import org.apache.commons.imaging.common.bytesource.ByteSourceInputStream;
import org.apache.commons.imaging.formats.tiff.TiffContents;
import org.apache.commons.imaging.formats.tiff.TiffDirectory;
import org.apache.commons.imaging.formats.tiff.TiffField;
import org.apache.commons.imaging.formats.tiff.TiffReader;
import org.apache.commons.imaging.formats.tiff.constants.GeoTiffTagConstants;
import org.apache.commons.imaging.formats.tiff.constants.TiffTagConstants;
import org.apache.commons.math3.util.FastMath;
import org.orekit.rugged.api.RuggedException;
import org.orekit.rugged.raster.TileUpdater;
import org.orekit.rugged.raster.UpdatableTile;

/** Digital Elevation Model Updater using Aster data.
 * @author Luc Maisonobe
 */
public class AsterTileUpdater implements TileUpdater {

    /** Directory for Aster data. */
    private final File directory;

    /** Simple constructor.
     * @param directory directory holding Aster data.
     */
    public AsterTileUpdater(final File directory) {
        this.directory = directory;
    }

    /** {@inheritDoc} */
    @Override
    public void updateTile(final double latitude, final double longitude,
                           final UpdatableTile tile)
        throws RuggedException {

        final String name = fileName(latitude, longitude);

        try {

            // build the file name
            final File file = new File(directory, name);
            if (!file.exists()) {
                throw new RuggedException(AsterMessages.NO_DEM_DATA_FOR_POINT,
                                          FastMath.toDegrees(latitude), FastMath.toDegrees(longitude),
                                          name);
            }

            final ZipFile zipFile = new ZipFile(file);
            for (final Enumeration<? extends ZipEntry> e = zipFile.entries(); e.hasMoreElements();) {
                final ZipEntry entry = e.nextElement();
                if ((!entry.isDirectory()) && entry.getName().endsWith("_dem.tif")) {
                    load(zipFile.getInputStream(entry), file.getAbsolutePath(), tile);
                }
            }
            zipFile.close();

        } catch (IOException ioe) {
            final RuggedException re =
                    new RuggedException(AsterMessages.ERROR_PARSING_FILE, name, ioe.getLocalizedMessage());
            re.initCause(ioe);
            throw re;
        }

    }

    /** Build the file name covering a point.
     * @param latitude point latitude
     * @param longitude point longitude
     * @return file name
     */
    private String fileName(final double latitude, final double longitude) {
        final int latCode = (int) FastMath.floor(FastMath.toDegrees(latitude));
        final int lonCode = (int) FastMath.floor(FastMath.toDegrees(longitude));
        if (latCode < 0) {
            if (lonCode < 0) {
                return String.format("ASTGTM2_S%2dW%3d.zip", -latCode, -lonCode);
            } else {
                return String.format("ASTGTM2_S%2dE%3d.zip", -latCode,  lonCode);
            }
        } else {
            if (lonCode < 0) {
                return String.format("ASTGTM2_N%2dW%3d.zip",  latCode, -lonCode);
            } else {
                return String.format("ASTGTM2_N%2dE%3d.zip",  latCode,  lonCode);
            }
        }
    }

    /** Load a Digital Elevation Model.
     * @param stream stream containing the Digital Elevation Model
     * @param fileName name of the file
     * @param tile tile to update
     * @exception IOException if file cannot be loaded
     * @exception RuggedException if ASTER data cannot be extracted
     */
    private void load(final InputStream stream, final String fileName, final UpdatableTile tile)
        throws RuggedException, IOException {

        try {

            // read TIFF
            final TiffReader reader = new TiffReader(true);
            final ByteSource source = new ByteSourceInputStream(stream, fileName);
            final TiffContents contents = reader.readContents(source, null, FormatCompliance.getDefault());

            // extract GeoTiff data
            final TiffField scalesField = contents.findField(GeoTiffTagConstants.EXIF_TAG_MODEL_PIXEL_SCALE_TAG);
            if (scalesField == null) {
                throw new RuggedException(AsterMessages.MISSING_PIXEL_SCALE, fileName);
            }
            final double[] pixelsScales = scalesField.getDoubleArrayValue();

            final TiffField tieField = contents.findField(GeoTiffTagConstants.EXIF_TAG_MODEL_TIEPOINT_TAG);
            if (tieField == null) {
                throw new RuggedException(AsterMessages.MISSING_TIE_POINT, fileName);
            }
            final double[] tiePoint = tieField.getDoubleArrayValue();

            final int    latitudeRows     = contents.findField(TiffTagConstants.TIFF_TAG_IMAGE_LENGTH).getIntValue();
            final int    longitudeColumns = contents.findField(TiffTagConstants.TIFF_TAG_IMAGE_WIDTH).getIntValue();

            final int[] geoKeyDirectory   = contents.findField(GeoTiffTagConstants.EXIF_TAG_GEO_KEY_DIRECTORY_TAG).getIntArrayValue();
            final int keyDirectoryVersion = geoKeyDirectory[0];
            final int keyRevision         = geoKeyDirectory[1];
            final int minorRevision       = geoKeyDirectory[2];
            final int numberOfKeys        = geoKeyDirectory[3];
            if (keyDirectoryVersion != 1 || keyRevision != 1 || minorRevision != 0) {
                throw new RuggedException(AsterMessages.UNSUPPORTED_GEOTIFF_VERSION,
                                          keyDirectoryVersion, keyRevision, minorRevision, fileName,
                                          1, 1, 0);
            }

            AngulerUnits angulerUnits = AngulerUnits.DEGREE;
            for (int i = 0; i < numberOfKeys; ++i) {
                final GeoKey geoKey   = GeoKey.getKey(geoKeyDirectory[4 * i + 4]);
                final int location    = geoKeyDirectory[4 * i + 5];
                final int count       = geoKeyDirectory[4 * i + 6];
                final int valueOffset = geoKeyDirectory[4 * i + 7];
                switch(geoKey) {
                case GT_MODEL_TYPE: {
                    final ModelType modelType = ModelType.getType(getShort(geoKey, location, count, valueOffset, fileName));
                    if (modelType != ModelType.GEOGRAPHIC) {
                        throw new RuggedException(AsterMessages.UNEXPECTED_GEOKEY_VALUE,
                                                  geoKey, fileName, modelType, ModelType.GEOCENTRIC);
                    }
                    break;
                }
                case GT_RASTER_TYPE: {
                    final RasterType rasterType = RasterType.getType(getShort(geoKey, location, count, valueOffset, fileName));
                    if (rasterType != RasterType.RASTER_PIXEL_IS_AREA) {
                        throw new RuggedException(AsterMessages.UNEXPECTED_GEOKEY_VALUE,
                                                  geoKey, fileName, rasterType, RasterType.RASTER_PIXEL_IS_AREA);
                    }
                    break;
                }
                case GEOGRAPHIC_TYPE: {
                    final GeographicCoordinateSystemType csType =
                            GeographicCoordinateSystemType.getType(getShort(geoKey, location, count, valueOffset, fileName));
                    if (csType != GeographicCoordinateSystemType.GCS_WGS_84) {
                        throw new RuggedException(AsterMessages.UNEXPECTED_GEOKEY_VALUE,
                                                  geoKey, fileName, csType, GeographicCoordinateSystemType.GCS_WGS_84);
                    }
                    break;
                }
                case GEOG_LINEAR_UNITS: {
                    final LinearUnits linearUnits =
                            LinearUnits.getUnits(getShort(geoKey, location, count, valueOffset, fileName));
                    if (linearUnits != LinearUnits.METER) {
                        throw new RuggedException(AsterMessages.UNEXPECTED_GEOKEY_VALUE,
                                                  geoKey, fileName, linearUnits, LinearUnits.METER);
                    }
                    break;
                }
                case GEOG_ANGULAR_UNITS: {
                    angulerUnits = AngulerUnits.getUnits(getShort(geoKey, location, count, valueOffset, fileName));
                    break;
                }
                default:
                    throw new RuggedException(AsterMessages.UNEXPECTED_GEOKEY, geoKey, fileName);
                }
            }

            // set up geometry
            final double latitudeStep  = angulerUnits.toRadians(pixelsScales[1]);
            final double longitudeStep = angulerUnits.toRadians(pixelsScales[0]);
            final double minLatitude   = angulerUnits.toRadians(tiePoint[4]) - (latitudeRows - 1) * latitudeStep;
            final double minLongitude  = angulerUnits.toRadians(tiePoint[3]);
            tile.setGeometry(minLatitude, minLongitude, latitudeStep, longitudeStep,
                             latitudeRows, longitudeColumns);

            // read the raster data
            final int msbOffset = reader.getByteOrder() == ByteOrder.BIG_ENDIAN ? 0 : 1;
            final int lsbOffset = reader.getByteOrder() == ByteOrder.BIG_ENDIAN ? 1 : 0;
            int i = 0;
            for (final TiffDirectory tiffDirectory : contents.directories) {
                for (final TiffDirectory.ImageDataElement element : tiffDirectory.getTiffRawImageDataElements()) {
                    final byte[] bytes = source.getBlock(element.offset, element.length);
                    for (int longitudeIndex = 0; longitudeIndex < longitudeColumns; ++longitudeIndex) {
                        final int msb = bytes[2 * longitudeIndex + msbOffset];
                        final int lsb = 0xff & bytes[2 * longitudeIndex + lsbOffset];
                        final int elevation = (msb << 8) | lsb;
                        tile.setElevation(latitudeRows - 1 - i, longitudeIndex, elevation);
                    }
                    i++;
                }
            }

        } catch (ImageReadException ire) {
            throw new RuggedException(AsterMessages.ERROR_PARSING_FILE,
                                      fileName, ire.getLocalizedMessage());
        }

    }

    /** Get a short from the geo key directory.
     * @param key being parsed
     * @param location location of the short
     * @param count number of elements
     * @param valueOffset offset of the value
     * @param fileName name of the file
     * @return short value
     * @exception RuggedException if value cannot be retrieved
     */
    private int getShort(final GeoKey key, final int location, final int count, final int valueOffset,
                         final String fileName)
        throws RuggedException {
        if (location != 0 && count != 1) {
            throw new RuggedException(AsterMessages.UNABLE_TO_RETRIEVE_VALUE_FOR_KEY, key, fileName);
        }
        return valueOffset;
    }

}
