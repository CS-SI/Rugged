/* Copyright 2013-2017 CS Systèmes d'Information
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
import java.util.Map;
import java.util.Set;

import org.orekit.bodies.GeodeticPoint;
import org.orekit.rugged.linesensor.SensorPixel;

/** Container for mapping between sensor pixels and ground points.
 * @see SensorMapping
 * @author Luc Maisonobe
 * @author Lucie Labat-Allee
 * @author Guylaine Prat
 * @since 2.0
 */
public class SensorToGroundMapping {

    /** Default name for Rugged. */
    private static final String RUGGED = "Rugged";

    /** Name of the sensor to which mapping applies. */
    private final String sensorName;

    /** Mapping from sensor to ground. */
    private final SensorMapping<GeodeticPoint> groundMapping;

    
    /** Build a new instance (with default Rugged name).
     * @param sensorName name of the sensor to which mapping applies
     */
    public SensorToGroundMapping(final String sensorName) {
        this(sensorName, RUGGED);
    }

    /** Build a new instance with a specific Rugged name.
     * @param ruggedName name of the Rugged to which mapping applies
     * @param sensorName name of the sensor to which mapping applies
     */
    public SensorToGroundMapping(final String ruggedName, final String sensorName) {
        
        this.sensorName     = sensorName;
        this.groundMapping = new SensorMapping<GeodeticPoint>(sensorName, ruggedName);
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
        return this.groundMapping.getRuggedName();
    }

    /** Add a mapping between one sensor pixel and one ground point.
     * @param pixel sensor pixel
     * @param groundPoint ground point corresponding to the sensor pixel
     */
    public void addMapping(final SensorPixel pixel, final GeodeticPoint groundPoint) {
        groundMapping.addMapping(pixel, groundPoint);
    }

    /** Get all the mapping entries.
     * @return an unmodifiable view of all mapping entries
     */
    public Set<Map.Entry<SensorPixel, GeodeticPoint>> getMapping() {
        return Collections.unmodifiableSet(groundMapping.getMapping());
    }
}
