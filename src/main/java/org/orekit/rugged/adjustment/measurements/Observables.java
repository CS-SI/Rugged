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

import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.Map;

/** Class for measurements generation.
 * @see SensorToSensorMapping
 * @see SensorToGroundMapping
 * @author Lucie Labat-Allee
 * @author Guylaine Prat
 * @since 2.0
 */
public class Observables {

    /** Separator between Rugged name and sensor name. */
    private static final String RUGGED_SENSOR_SEPARATOR = "_";

    /** Separator between sensors. */
    private static final String SENSORS_SEPARATOR = "__";

    /** Sensor to ground mapping structure (example: for Ground Control Points GCP points).*/
    private final Map<String, SensorToGroundMapping> groundMappings;

    /** Sensor to sensor mappings structure (Tie points). */
    private final Map<String, SensorToSensorMapping> interMappings;

    /** Number of viewing models to map.*/
    private final int nbModels;


    /** Build a new instance.
     * @param nbModels number of viewing models to map
     */
    public Observables(final int nbModels) {

        this.groundMappings = new LinkedHashMap<String, SensorToGroundMapping>();
        this.interMappings = new LinkedHashMap<String, SensorToSensorMapping>();
        this.nbModels = nbModels;
    }

    /** Add a mapping between two viewing models.
     * @param interMapping sensor to sensor mapping
     */
    public void addInterMapping(final SensorToSensorMapping interMapping) {

        interMappings.put(this.createKey(interMapping), interMapping);
    }

    /** Add a ground mapping.
     * <p>
     * A ground mapping is defined by a set of GCPs.
     * </p>
     * @param groundMapping sensor to ground mapping
     */
    public void addGroundMapping(final SensorToGroundMapping groundMapping) {

        groundMappings.put(this.createKey(groundMapping), groundMapping);
    }

    /** Get all the ground mapping entries.
     * @return an unmodifiable view of all mapping entries
     */
    public Collection<SensorToGroundMapping> getGroundMappings() {
        return  groundMappings.values();
    }

    /**
     * Get a ground Mapping for a sensor.
     * @param ruggedName Rugged name
     * @param sensorName sensor name
     * @return selected ground mapping or null if sensor is not found
     */
    public SensorToGroundMapping getGroundMapping(final String ruggedName, final String sensorName) {

        final SensorToGroundMapping mapping = this.groundMappings.get(ruggedName + RUGGED_SENSOR_SEPARATOR + sensorName);
        return mapping;
    }

    /** Get the sensor to sensor values.
     * @return the inter-mappings
     */
    public Collection<SensorToSensorMapping> getInterMappings() {
        return interMappings.values();
    }

    /** Get the number of viewing models to map.
     * @return the number of viewing models to map
     */
    public int getNbModels() {
        return nbModels;
    }

    /**
     * Get a sensor mapping for a sensor.
     * <p>
     * returns sensor to sensor mapping associated with specific sensors and related rugged instance.
     * </p>
     * @param ruggedNameA Rugged name A
     * @param sensorNameA sensor name A
     * @param ruggedNameB Rugged name B
     * @param sensorNameB sensor name B
     * @return selected ground mapping or null if a sensor is not found
     */
    public SensorToSensorMapping getInterMapping(final String ruggedNameA, final String sensorNameA,
                                                 final String ruggedNameB, final String sensorNameB) {

        final String keyA = ruggedNameA + RUGGED_SENSOR_SEPARATOR + sensorNameA;
        final String keyB = ruggedNameB + RUGGED_SENSOR_SEPARATOR + sensorNameB;
        final SensorToSensorMapping mapping = interMappings.get(keyA + SENSORS_SEPARATOR + keyB);
        return mapping;
    }

    /** Create key for SensorToGroundMapping map.
     * @param groundMapping the ground mapping
     * @return the key
     */
    private String createKey(final SensorToGroundMapping groundMapping)
    {
        final String key = groundMapping.getRuggedName() + RUGGED_SENSOR_SEPARATOR + groundMapping.getSensorName();
        return key;
    }

    /** Create key for SensorToSensorMapping map.
     * @param sensorMapping the inter mapping
     * @return the key
     */
    private String createKey(final SensorToSensorMapping sensorMapping)
    {
        final String keyA = sensorMapping.getRuggedNameA() + RUGGED_SENSOR_SEPARATOR + sensorMapping.getSensorNameA();
        final String keyB = sensorMapping.getRuggedNameB() + RUGGED_SENSOR_SEPARATOR + sensorMapping.getSensorNameB();
        return keyA + SENSORS_SEPARATOR + keyB;
    }
}
