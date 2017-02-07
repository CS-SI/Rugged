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
import java.util.List;

/** Class for measures generation.
 * @author Lucie Labat-Allee
 * @see SensorToSensorMapping
 * @see SensorToGroundMapping
 * @since 2.0
 */
public class Observables {

    /** Sensor to ground mapping structure (example: for GCP points) */
    private final SensorToGroundMapping groundMapping;

    /** Sensor to sensor mappings structure (liaison points). */
    private final List<SensorToSensorMapping> interMappings;

    /** Number of viewing models to map */
    private final int nbModels;


    /** Build a new instance.
     * @param sensorName name of the sensor to which mapping applies
     * @param nbMeasures number of viewing models
     */
    public Observables(final String sensorName, final int nbModels) {
        this.groundMapping = new SensorToGroundMapping(sensorName);
        this.interMappings = new ArrayList<SensorToSensorMapping>();
        this.nbModels = nbModels;
    }

    /** Add a mapping between two viewing models
     * @param SensorToSensorMapping sensor to sensor mapping
     */
    public void addInterMapping(final SensorToSensorMapping interMapping) {
        interMappings.add(interMapping);
    }

    
    /**
     * @return the groundMapping
     */
    public SensorToGroundMapping getGroundMapping() {
        return groundMapping;
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
