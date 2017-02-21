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

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.orekit.rugged.errors.RuggedException;
import org.orekit.rugged.errors.RuggedMessages;

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
    private final List<SensorToSensorMapping> interMappings;

    /** Number of viewing models to map */
    private final int nbModels;


    /** Build a new instance.
     * @param sensorName name of the sensor to which mapping applies
     * @param nbMeasures number of viewing models
     */
    public Observables(final int nbModels) {
        this.groundMappings = new LinkedHashMap<String, SensorToGroundMapping>();
        this.interMappings = new ArrayList<SensorToSensorMapping>();
        this.nbModels = nbModels;
    }

    /** Add a mapping between two viewing models
     * @param SensorToSensorMapping sensor to sensor mapping
     */
    public void addInterMapping(final SensorToSensorMapping interMapping) {
        interMappings.add(interMapping);
    }

    /** Add a  ground mapping between
     * @param SensorToGroundMapping sensor to ground mapping
     */
    public void addGroundMapping(final SensorToGroundMapping groundMapping) {
        groundMappings.put(groundMapping.getSensorName(), groundMapping);
    }


    /** Get all the ground mapping entries.
     * @return an unmodifiable view of all mapping entries
     */
    public Set<Map.Entry<String, SensorToGroundMapping>> getGroundMappings() {
        return  Collections.unmodifiableSet(groundMappings.entrySet());
    }


    /**
     * Get a ground Mapping for a sensor.
     *
     * @param sensorName sensor name
     * @return selected ground mapping
     * @exception RuggedException if sensor is not known
     */
    public SensorToGroundMapping getGroundMapping(final String sensorName)
                    throws RuggedException {
        final SensorToGroundMapping mapping = this.groundMappings.get(sensorName);
        if (mapping == null) {
            throw new RuggedException(RuggedMessages.UNKNOWN_SENSOR,
                                      sensorName);
        }
        return mapping;
    }

    /**
     * @return the interMappings
     */
    public List<SensorToSensorMapping> getInterMappings() {
        return interMappings;
    }

    /**
     * @return the nbModels
     */
    public int getNbModels() {
        return nbModels;
    }

}
