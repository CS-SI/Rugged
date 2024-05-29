/* Copyright 2013-2022 CS GROUP
 * Licensed to CS GROUP (CS) under one or more
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

import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.net.URL;
import java.net.URLConnection;
import java.nio.charset.StandardCharsets;
import java.util.Locale;
import java.util.MissingResourceException;
import java.util.PropertyResourceBundle;
import java.util.ResourceBundle;

import org.hipparchus.exception.Localizable;


/**
 * Enumeration for localized messages formats.
 * <p>
 * The constants in this enumeration represent the available
 * formats as localized strings. These formats are intended to be
 * localized using simple properties files, using the constant
 * name as the key and the property value as the message format.
 * The source English format is provided in the constants themselves
 * to serve both as a reminder for developers to understand the parameters
 * needed by each format, as a basis for translators to create
 * localized properties files, and as a default format if some
 * translation is missing.
 * </p>
 * <p>
 * This class is heavily based on {@code OrekitMessages},
 * which is distributed under the terms of the Apache License V2.
 * </p>
 */
public enum RuggedMessages implements Localizable {


    /** INTERNAL_ERROR. */
    INTERNAL_ERROR("internal error, please notify development team by creating an issue at {0}"),
    /** OUT_OF_TILE_INDICES. */
    OUT_OF_TILE_INDICES("no data at indices [{0}, {1}], tile only covers from [0, 0] to [{2}, {3}] (inclusive)"),
    /** OUT_OF_TILE_ANGLES. */
    OUT_OF_TILE_ANGLES("no data at latitude {0} and longitude {1}, tile covers only latitudes {2} to {3} and longitudes {4} to {5}"),
    /** NO_DEM_DATA. */
    NO_DEM_DATA("no Digital Elevation Model data at latitude {0} and longitude {1}"),
    /** TILE_WITHOUT_REQUIRED_NEIGHBORS_SELECTED. */
    TILE_WITHOUT_REQUIRED_NEIGHBORS_SELECTED("the tile selected for latitude {0} and longitude {1} does not contain required point neighborhood"),
    /** OUT_OF_TIME_RANGE. */
    OUT_OF_TIME_RANGE("date {0} is out of time span [{1}, {2}] (UTC)"),
    /** UNINITIALIZED_CONTEXT. */
    UNINITIALIZED_CONTEXT("general context has not been initialized (missing call to {0})"),
    /** EMPTY_TILE. */
    EMPTY_TILE("tile is empty: {0} ⨉ {1}"),
    /** UNKNOWN_SENSOR. */
    UNKNOWN_SENSOR("unknown sensor {0}"),
    /** LINE_OF_SIGHT_DOES_NOT_REACH_GROUND. */
    LINE_OF_SIGHT_DOES_NOT_REACH_GROUND("line-of-sight does not reach ground"),
    /** LINE_OF_SIGHT_NEVER_CROSSES_LATITUDE. */
    LINE_OF_SIGHT_NEVER_CROSSES_LATITUDE("line-of-sight never crosses latitude {0}"),
    /** LINE_OF_SIGHT_NEVER_CROSSES_LONGITUDE. */
    LINE_OF_SIGHT_NEVER_CROSSES_LONGITUDE("line-of-sight never crosses longitude {0}"),
    /** LINE_OF_SIGHT_NEVER_CROSSES_ALTITUDE. */
    LINE_OF_SIGHT_NEVER_CROSSES_ALTITUDE("line-of-sight never crosses altitude {0}"),
    /** DEM_ENTRY_POINT_IS_BEHIND_SPACECRAFT. */
    DEM_ENTRY_POINT_IS_BEHIND_SPACECRAFT("line-of-sight enters the Digital Elevation Model behind spacecraft!"),
    /** FRAMES_MISMATCH_WITH_INTERPOLATOR_DUMP. */
    FRAMES_MISMATCH_WITH_INTERPOLATOR_DUMP("frame {0} does not match frame {1} from interpolator dump"),
    /** NOT_INTERPOLATOR_DUMP_DATA. */
    NOT_INTERPOLATOR_DUMP_DATA("data is not an interpolator dump"),
    /** DEBUG_DUMP_ALREADY_ACTIVE. */
    DEBUG_DUMP_ALREADY_ACTIVE("debug dump is already active for this thread"),
    /** DEBUG_DUMP_ACTIVATION_ERROR. */
    DEBUG_DUMP_ACTIVATION_ERROR("unable to active debug dump with file {0}: {1}"),
    /** DEBUG_DUMP_NOT_ACTIVE. */
    DEBUG_DUMP_NOT_ACTIVE("debug dump is not active for this thread"),
    /** CANNOT_PARSE_LINE. */
    CANNOT_PARSE_LINE("cannot parse line {0}, file {1}: {2}"),
    /** LIGHT_TIME_CORRECTION_REDEFINED. */
    LIGHT_TIME_CORRECTION_REDEFINED("light time correction redefined, line {0}, file {1}: {2}"),
    /** ABERRATION_OF_LIGHT_CORRECTION_REDEFINED. */
    ABERRATION_OF_LIGHT_CORRECTION_REDEFINED("aberration of light correction redefined, line {0}, file {1}: {2}"),
    /** ATMOSPHERIC_REFRACTION_REDEFINED. */
    ATMOSPHERIC_REFRACTION_REDEFINED("atmospheric refraction correction redefined, line {0}, file {1}: {2}"),
    /** TILE_ALREADY_DEFINED. */
    TILE_ALREADY_DEFINED("tile {0} already defined, line {1}, file {2}: {3}"),
    /** UNKNOWN_TILE. */
    UNKNOWN_TILE("unknown tile {0}, line {1}, file {2}: {3}"),
    /** NO_PARAMETERS_SELECTED. */
    NO_PARAMETERS_SELECTED("no parameters have been selected for estimation"),
    /** NO_REFERENCE_MAPPINGS. */
    NO_REFERENCE_MAPPINGS("no reference mappings for parameters estimation"),
    /** DUPLICATED_PARAMETER_NAME. */
    DUPLICATED_PARAMETER_NAME("a different parameter with name {0} already exists"),
    /** INVALID_RUGGED_NAME. */
    INVALID_RUGGED_NAME("invalid rugged name"),
    /** UNSUPPORTED_REFINING_CONTEXT. */
    UNSUPPORTED_REFINING_CONTEXT("refining using {0} rugged instance is not handled"),
    /** NO_LAYER_DATA. */
    NO_LAYER_DATA("no atmospheric layer data at altitude {0} (lowest altitude: {1})"),
    /** INVALID_STEP. */
    INVALID_STEP("step {0} is not valid : {1}"),
    /** INVALID_RANGE_FOR_LINES. */
    INVALID_RANGE_FOR_LINES("range between min line {0} and max line {1} is invalid {2}"),
    /** SENSOR_PIXEL_NOT_FOUND_IN_RANGE_LINES. */
    SENSOR_PIXEL_NOT_FOUND_IN_RANGE_LINES("impossible to find sensor pixel in given range lines (with atmospheric refraction) between lines {0} and {1}"),
    /** SENSOR_PIXEL_NOT_FOUND_IN_PIXELS_LINE. */
    SENSOR_PIXEL_NOT_FOUND_IN_PIXELS_LINE("impossible to find sensor pixel: pixel {0} outside interval [ {1} , {2} [ (with atmospheric refraction margin = {3})");

    /** Base name of the resource bundle in classpath. */
    private static final String RESOURCE_BASE_NAME = "assets/org/orekit/rugged/RuggedMessages";

    /** Source English format. */
    private final String sourceFormat;

    /** Simple constructor.
     * @param sourceFormat source English format to use when no
     * localized version is available
     */
    RuggedMessages(final String sourceFormat) {
        this.sourceFormat = sourceFormat;
    }

    /** {@inheritDoc} */
    @Override
    public String getSourceString() {
        return sourceFormat;
    }

    /** {@inheritDoc} */
    @Override
    public String getLocalizedString(final Locale locale) {
        try {
            final ResourceBundle bundle =
                    ResourceBundle.getBundle(RESOURCE_BASE_NAME, locale, new UTF8Control());
            if (bundle.getLocale().getLanguage().equals(locale.getLanguage())) {
                final String translated = bundle.getString(name());
                if (translated.length() > 0 &&
                    !translated.toLowerCase(locale).contains("missing translation")) {
                    // the value of the resource is the translated format
                    return translated;
                }
            }

        } catch (MissingResourceException mre) {
            // do nothing here
        }

        // either the locale is not supported or the resource is not translated or
        // it is unknown: don't translate and fall back to using the source format
        return sourceFormat;

    }

    /** Control class loading properties in UTF-8 encoding.
     * <p>
     * This class has been very slightly adapted from BalusC answer to question: <a
     * href="http://stackoverflow.com/questions/4659929/how-to-use-utf-8-in-resource-properties-with-resourcebundle">
     * How to use UTF-8 in resource properties with ResourceBundle</a>.
     * </p>
     */
    public static class UTF8Control extends ResourceBundle.Control {

        /** {@inheritDoc} */
        @Override
        public ResourceBundle newBundle(final String baseName, final Locale locale, final String format,
                                        final ClassLoader loader, final boolean reload) throws IOException {
            // The below is a copy of the default implementation.
            final String bundleName = toBundleName(baseName, locale);
            final String resourceName = toResourceName(bundleName, "utf8");
            ResourceBundle bundle = null;
            InputStream stream = null;
            if (reload) {
                final URL url = loader.getResource(resourceName);
                if (url != null) {
                    final URLConnection connection = url.openConnection();
                    if (connection != null) {
                        connection.setUseCaches(false);
                        stream = connection.getInputStream();
                    }
                }
            } else {
                stream = loader.getResourceAsStream(resourceName);
            }
            if (stream != null) {
                try (InputStreamReader inputStreamReader = new InputStreamReader(stream, StandardCharsets.UTF_8)) {
                    // Only this line is changed to make it to read properties files as UTF-8.
                    bundle = new PropertyResourceBundle(inputStreamReader);
                }
            }
            return bundle;
        }

    }

}
