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
package org.orekit.rugged.api;

import java.io.Serializable;

/** Container for sensor pixel.
 * <p>
 * Instances of this class are guaranteed to be immutable.
 * </p>
 * @author Luc Maisonobe
 */
public class SensorPixel implements Serializable {

    /** Serializable UID. */
    private static final long serialVersionUID = 20140309L;

    /** Line number. */
    private final int lineNumber;

    /** Pixel number. */
    private final int pixelNumber;

    /**
     * Build a new instance.
     *
     * @param lineNumber line number
     * @param pixelNumber pixel number
     */
    public SensorPixel(final int lineNumber, final int pixelNumber) {
        this.lineNumber  = lineNumber;
        this.pixelNumber = pixelNumber;
    }

    /** Get the line number.
     * @return line number
     */
    public int getLineNumber() {
        return lineNumber;
    }

    /** Get the pixel number.
     * @return pixel number
     */
    public int getPixelNumber() {
        return pixelNumber;
    }

}
