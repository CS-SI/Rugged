/* Copyright 2013-2016 CS Systèmes d'Information
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

import java.text.MessageFormat;
import java.util.Locale;

import org.hipparchus.exception.Localizable;
import org.hipparchus.exception.LocalizedException;


/** This class is the base class for all specific exceptions thrown by
 * the rugged library classes.

 * <p>
 * This class is heavily based on Orekit {@link org.orekit.errors.OrekitException},
 * which is distributed under the terms of the Apache License V2.
 * </p>
 *
 * @author Luc Maisonobe

 */

public class RuggedException extends Exception implements LocalizedException {

    /** Serializable UID. */
    private static final long serialVersionUID = 20140309L;

    /** Format specifier (to be translated). */
    private final Localizable specifier;

    /** Parts to insert in the format (no translation). */
    private final Object[] parts;

    /** Simple constructor.
     * Build an exception with a translated and formatted message
     * @param specifier format specifier (to be translated)
     * @param parts parts to insert in the format (no translation)
     */
    public RuggedException(final Localizable specifier, final Object ... parts) {
        this.specifier = specifier;
        this.parts     = (parts == null) ? new Object[0] : parts.clone();
    }

    /** Simple constructor.
     * Build an exception from a cause and with a translated and formatted message
     * @param cause underlying cause
     * @param specifier format specifier (to be translated)
     * @param parts parts to insert in the format (no translation)
     */
    public RuggedException(final Throwable cause, final Localizable specifier,
                           final Object ... parts) {
        super(cause);
        this.specifier = specifier;
        this.parts     = (parts == null) ? new Object[0] : parts.clone();
    }

    /** {@inheritDoc} */
    @Override
    public String getMessage(final Locale locale) {
        return buildMessage(locale, specifier, parts);
    }

    /** {@inheritDoc} */
    @Override
    public String getMessage() {
        return getMessage(Locale.US);
    }

    /** {@inheritDoc} */
    @Override
    public String getLocalizedMessage() {
        return getMessage(Locale.getDefault());
    }

    /** Get the localizable specifier of the error message.
     * @return localizable specifier of the error message
     */
    public Localizable getSpecifier() {
        return specifier;
    }

    /** Get the variable parts of the error message.
     * @return a copy of the variable parts of the error message
     */
    public Object[] getParts() {
        return parts.clone();
    }

    /**
     * Builds a message string by from a pattern and its arguments.
     * @param locale Locale in which the message should be translated
     * @param specifier format specifier (to be translated)
     * @param parts parts to insert in the format (no translation)
     * @return a message string
     */
    private static String buildMessage(final Locale locale, final Localizable specifier,
                                       final Object ... parts) {
        return (specifier == null) ? "" : new MessageFormat(specifier.getLocalizedString(locale), locale).format(parts);
    }

    /** Create an {@link java.lang.RuntimeException} for an internal error.
     * @param cause underlying cause
     * @return an {@link java.lang.RuntimeException} for an internal error
     */
    public static RuntimeException createInternalError(final Throwable cause) {

        /** Format specifier (to be translated). */
        final Localizable specifier = RuggedMessages.INTERNAL_ERROR;

        /** Parts to insert in the format (no translation). */
        final String parts     = "rugged-developers@orekit.org";

        return new RuntimeException() {

            /** Serializable UID. */
            private static final long serialVersionUID = 20140309L;

            /** {@inheritDoc} */
            @Override
            public String getMessage() {
                return buildMessage(Locale.US, specifier, parts);
            }

            /** {@inheritDoc} */
            @Override
            public String getLocalizedMessage() {
                return buildMessage(Locale.getDefault(), specifier, parts);
            }

        };

    }

}
