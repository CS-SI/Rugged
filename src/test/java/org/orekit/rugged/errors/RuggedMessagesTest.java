/* Copyright 2013-2025 CS GROUP
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


import java.text.MessageFormat;
import java.util.Enumeration;
import java.util.Locale;
import java.util.ResourceBundle;

import org.hipparchus.exception.UTF8Control;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

public class RuggedMessagesTest {

    private final String[] LANGUAGES_LIST = { "da", "de", "en", "es", "fr", "gl", "it", "no", "ro" } ;
    @Test
    public void testMessageNumber() {
        Assertions.assertEquals(35, RuggedMessages.values().length);
    }

    @Test
    public void testAllKeysPresentInPropertiesFiles() {
        for (final String language : LANGUAGES_LIST) {
            ResourceBundle bundle = ResourceBundle.getBundle("assets/org/orekit/rugged/RuggedMessages",
                                                      Locale.forLanguageTag(language),
                                                      new UTF8Control());

            for (RuggedMessages message : RuggedMessages.values()) {
                final String messageKey = message.toString();
                boolean keyPresent = false;
                for (final Enumeration<String> keys = bundle.getKeys(); keys.hasMoreElements();) {
                    keyPresent |= messageKey.equals(keys.nextElement());
                }
                Assertions.assertTrue(keyPresent,
                                  "missing key \"" + message.name() + "\" for language " + language);
            }
            Assertions.assertEquals(language, bundle.getLocale().getLanguage());
        }

    }

    @Test
    public void testAllPropertiesCorrespondToKeys() {
        for (final String language : LANGUAGES_LIST) {
            ResourceBundle bundle = ResourceBundle.getBundle("assets/org/orekit/rugged/RuggedMessages",
                                                      Locale.forLanguageTag(language),
                                                      new UTF8Control());
            for (final Enumeration<String> keys = bundle.getKeys(); keys.hasMoreElements();) {
                final String propertyKey = keys.nextElement();
                try {
                    Assertions.assertNotNull(RuggedMessages.valueOf(propertyKey));
                } catch (IllegalArgumentException iae) {
                    Assertions.fail("unknown key \"" + propertyKey + "\" in language " + language);
                }
            }
            Assertions.assertEquals(language, bundle.getLocale().getLanguage());
        }

    }

    @Test
    public void testNoMissingFrenchTranslation() {
        for (RuggedMessages message : RuggedMessages.values()) {
            String translated = message.getLocalizedString(Locale.FRENCH);
            Assertions.assertFalse(translated.toLowerCase().contains("missing translation"), message.name());
        }
    }

    @Test
    public void testNoOpEnglishTranslation() {
        for (RuggedMessages message : RuggedMessages.values()) {
            String translated = message.getLocalizedString(Locale.ENGLISH);
            Assertions.assertEquals(message.getSourceString(), translated);
        }
    }

    @Test
    public void testMissingLanguageFallback() {
        for (RuggedMessages message : RuggedMessages.values()) {
            String translated = message.getLocalizedString(Locale.TRADITIONAL_CHINESE);
            Assertions.assertEquals(message.getSourceString(), translated);
        }
    }

    @Test
    public void testMissingLanguageMissingTranslation() {
        Assertions.assertEquals(RuggedMessages.INTERNAL_ERROR.getSourceString(),
                            RuggedMessages.INTERNAL_ERROR.getLocalizedString(Locale.KOREAN));
        Assertions.assertEquals(RuggedMessages.NO_DEM_DATA.getSourceString(),
                            RuggedMessages.NO_DEM_DATA.getLocalizedString(Locale.KOREAN));
        Assertions.assertEquals("ABCDEF {0}",
                            RuggedMessages.UNKNOWN_SENSOR.getLocalizedString(Locale.KOREAN));
        Assertions.assertEquals(RuggedMessages.EMPTY_TILE.getSourceString(),
                            RuggedMessages.EMPTY_TILE.getLocalizedString(Locale.KOREAN));
    }

    @Test
    public void testVariablePartsConsistency() {
        for (final String language : LANGUAGES_LIST) {
            Locale locale = Locale.forLanguageTag(language);
            for (RuggedMessages message : RuggedMessages.values()) {
                MessageFormat source     = new MessageFormat(message.getSourceString());
                MessageFormat translated = new MessageFormat(message.getLocalizedString(locale));
                Assertions.assertEquals(source.getFormatsByArgumentIndex().length,
                                    translated.getFormatsByArgumentIndex().length,
                                    message.name() + " (" + language + ")");
            }
        }
    }

}
