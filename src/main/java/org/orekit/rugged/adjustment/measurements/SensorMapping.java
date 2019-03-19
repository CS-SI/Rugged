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
package org.orekit.rugged.adjustment.measurements;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Set;

import org.orekit.rugged.linesensor.SensorPixel;

/** Container for mapping sensor pixels with sensor pixels or ground points.
 * @author Lucie Labat-Allee
 * @author Guylaine Prat
 * @since 2.0
 */
public class SensorMapping<T> {

    /** Default name for Rugged. */
    private static final String RUGGED = "Rugged";

    /** Name of the sensor to which mapping applies. */
    private final String sensorName;

    /** Name of the Rugged to which mapping applies. */
    private final String ruggedName;

    /** Mapping from sensor to other (sensor or ground). */
    private final Map<SensorPixel, T> mapping;


    /** Build a new instance (with default Rugged name).
     * @param sensorName name of the sensor to which mapping applies
     */
    public SensorMapping(final String sensorName) {
        this(sensorName, RUGGED);
    }

    /** Build a new instance with a specific Rugged name.
     * @param sensorName name of the sensor to which mapping applies
     * @param ruggedName name of the Rugged to which mapping applies
     */
    public SensorMapping(final String sensorName, final String ruggedName) {

        this.sensorName     = sensorName;
        this.ruggedName     = ruggedName;
        this.mapping = new LinkedHashMap<SensorPixel, T>();
    }

    /** Get the name of the sensor to which mapping applies.
     * @return name of the sensor to which mapping applies
     */
    public String getSensorName() {
        return sensorName;
    }

    /** Get the name of the Rugged to which mapping applies.
     * @return name of the Rugged to which mapping applies
     */
    public String getRuggedName() {
        return ruggedName;
    }

    /** Add a mapping between a sensor pixel and another point (sensor pixel or ground point).
     * @param pixel sensor pixel
     * @param point sensor pixel or ground point corresponding to the sensor pixel
     */
    public void addMapping(final SensorPixel pixel, final T point) {
        mapping.put(pixel, point);
    }

    /** Get all the mapping entries.
     * @return an unmodifiable view of all mapping entries
     */
    public Set<Map.Entry<SensorPixel, T>> getMapping() {
        return Collections.unmodifiableSet(mapping.entrySet());
    }
}
