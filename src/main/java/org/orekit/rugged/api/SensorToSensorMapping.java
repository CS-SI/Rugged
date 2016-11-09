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
package org.orekit.rugged.api;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.IdentityHashMap;
import java.util.Map;
import java.util.Set;
import org.orekit.rugged.linesensor.SensorPixel;

/** Container for mapping between sensor A pixels and sensor B pixels 
 * and storing distance between the two LOS computed with @link {@link Rugged#distanceBetweenLOS(
 * String, org.orekit.time.AbsoluteDate, int, String, org.orekit.time.AbsoluteDate, int)}
 * @author Lucie LabatAllee
 * @since 2.0
 */
public class SensorToSensorMapping {

    /** Name of the sensors to which mapping applies. */
    private final String sensorNameA;
    private final String sensorNameB;

    /** Mapping from sensor A to sensor B. */
    private final Map<SensorPixel, SensorPixel> sensorToSensor;

    /** Distance between two LOS */
    private final Collection<Double> mapDistance;
    

    /** Build a new instance.
     * @param sensorNameA name of the sensor A to which mapping applies
     * @param sensorNameB name of the sensor B to which mapping applies
     */
    public SensorToSensorMapping(final String sensorNameA, final String sensorNameB) {
        this.sensorNameA     = sensorNameA;
        this.sensorNameB     = sensorNameB;
        this.sensorToSensor = new IdentityHashMap<>();
        this.mapDistance = new ArrayList<Double>();
    }

    /** Get the name of the sensor A to which mapping applies.
     * @return name of the sensor A to which mapping applies
     */
    public String getSensorNameA() {
        return sensorNameA;
    }

    /** Get the name of the sensor B to which mapping applies.
     * @return name of the sensor B to which mapping applies
     */
    public String getSensorNameB() {
        return sensorNameB;
    }

    /** Add a mapping between one sensor A pixel to one sensor B pixel and computed distance between both LOS
     * @param pixelA sensor A pixel
     * @param pixelB sensor B pixel corresponding to the sensor A pixel
     */
    public void addMapping(final SensorPixel pixelA, final SensorPixel pixelB, final double distance) {
        sensorToSensor.put(pixelA, pixelB);
        mapDistance.add(distance);
    }

        /** Get all the mapping entries.
     * @return an unmodifiable view of all mapping entries
     */
    public Set<Map.Entry<SensorPixel, SensorPixel>> getMappings() {
        return Collections.unmodifiableSet(sensorToSensor.entrySet());
    }
    /**
     * @return the mapDistance
     */
    public Collection<Double> getMapDistance() {
        return mapDistance;
    }  
}
