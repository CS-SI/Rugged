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
package org.orekit.rugged.refining.measures;

import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.Map;

/** Class for measures generation.
 * @author Lucie Labat-Allee
 * @see SensorToSensorMapping
 * @see SensorToGroundMapping
 * @since 2.0
 */
public class Observables {

    /** Sensor to ground mapping structure (example: for GCP points) */
    private final Map<String, SensorToGroundMapping> groundMappings;

    /** Sensor to sensor mappings structure (liaison points). */
    private final Map<String, SensorToSensorMapping> interMappings;

    /** Number of viewing models to map */
    private final int nbModels;


    /** Build a new instance.
     * @param sensorName name of the sensor to which mapping applies
     * @param nbMeasures number of viewing models
     */
    public Observables(final int nbModels) {
        this.groundMappings = new LinkedHashMap<String, SensorToGroundMapping>();
        this.interMappings = new LinkedHashMap<String, SensorToSensorMapping>();
        this.nbModels = nbModels;
    }

    /** Add a mapping between two viewing models
     * @param SensorToSensorMapping sensor to sensor mapping
     */
    public void addInterMapping(final SensorToSensorMapping interMapping) {
        final String key = this.createKey(interMapping);
        interMappings.put(this.createKey(interMapping),interMapping);
    }

    /** Add a  ground mapping between
     * @param SensorToGroundMapping sensor to ground mapping
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
     *
     * @param sensorName sensor name
     * @return selected ground mapping or null of sensorName is not found
     */
    public SensorToGroundMapping getGroundMapping(final String ruggedName, final String sensorName) {
        final SensorToGroundMapping mapping = this.groundMappings.get(ruggedName + "_" + sensorName);
        return mapping;
    }

    /**
     * @return the interMappings
     */
    public Collection<SensorToSensorMapping> getInterMappings() {
        return interMappings.values();
    }

    /**
     * Get a sensor  Mapping for a sensor.
     *
     * @param ruggedNameA rugged name A
     * @param sensorNameA sensor name A
     * @param ruggedNameB rugged name B
     * @param sensorNameB sensor name B
     * @return selected ground mapping or null of sensorName is not found
     */
    public SensorToSensorMapping getInterMapping(final String ruggedNameA, final String sensorNameA, final String ruggedNameB, final String sensorNameB) {

        final String keyA = ruggedNameA + "_" + sensorNameA;
        final String keyB = ruggedNameB + "_" + sensorNameB;
        final SensorToSensorMapping mapping = this.interMappings.get(keyA + "__" + keyB);
        return mapping;
    }


    /**
     * @return the nbModels
     */
    public int getNbModels() {
        return nbModels;
    }


    /** create key for GroudMapping map.
     * @param groundMapping the groundMapping
     * @return the key
     */
    private String createKey(final SensorToGroundMapping groundMapping)
    {
        final String key = groundMapping.getRuggedName() + "_" + groundMapping.getSensorName();
        return key;
    }

    /** create key for SensorToSensorMapping map.
     * @param groundMapping the groundMapping
     * @return the key
     */
    private String createKey(final SensorToSensorMapping sensorMapping)
    {
        final String keyA = sensorMapping.getRuggedNameA() + "_" + sensorMapping.getSensorNameA();
        final String keyB = sensorMapping.getRuggedNameB() + "_" + sensorMapping.getSensorNameB();
        return keyA + "__" + keyB;
    }


}
