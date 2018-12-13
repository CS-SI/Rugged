/* Copyright 2013-2018 CS Systèmes d'Information
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
package org.orekit.rugged.los;

import java.io.Serializable;

import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.orekit.rugged.linesensor.SensorPixel;

/** Container for pixel line-of-sight.
 * @author Guylaine Prat
 * @since 3.0
 */
public class PixelLOS implements Serializable {

    /** Serializable UID. */
    private static final long serialVersionUID = -6674056279573271367L;

    /** Sensor pixel. */
    private final SensorPixel sensorPixel;

    /** Pixel line-of-sight in spacecraft frame. */
    private final Vector3D los;

    /**
     * Build a new instance.
     * @param sensorPixel the sensor pixel cell
     * @param los the pixel line-of-sight in spacecraft frame
     */
    public PixelLOS(final SensorPixel sensorPixel, final Vector3D los) {
        this.sensorPixel = sensorPixel;
        this.los = los;
    }

    /**
     * @return the sensorPixel
     */
    public SensorPixel getSensorPixel() {
        return sensorPixel;
    }

    /**
     * @return the lOS in spacecraft frame
     */
    public Vector3D getLOS() {
        return los;
    }
}
