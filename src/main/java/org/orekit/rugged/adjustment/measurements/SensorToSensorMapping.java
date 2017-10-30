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

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.orekit.rugged.api.Rugged;
import org.orekit.rugged.linesensor.LineSensor;
import org.orekit.rugged.linesensor.SensorPixel;
import org.orekit.rugged.utils.SpacecraftToObservedBody;
import org.orekit.time.AbsoluteDate;

/** Container for mapping sensors pixels of two viewing models.
 * Store the distance between both lines of sight computed with 
 * {@link Rugged#distanceBetweenLOS(LineSensor, AbsoluteDate, double, SpacecraftToObservedBody, LineSensor, AbsoluteDate, double)}
 * <p> Constraints in relation to Earth distance can be added.
 * @see SensorMapping
 * @author Lucie LabatAllee
 * @author Guylaine Prat
 * @since 2.0
 */
public class SensorToSensorMapping {

    /** Default name for Rugged. */
    private static final String RUGGED = "Rugged";

    /** Name of the sensor B to which mapping applies. */
    private final String sensorNameB;

    /** Name of the Rugged B to which mapping applies. */
    private final String ruggedNameB;

    /** Mapping from sensor A to sensor B. */
    private final SensorMapping<SensorPixel> interMapping;

    /** Distances between two LOS. */
    private final List<Double> losDistances;

    /** Earth distances associated with pixel A. */
    private final List<Double> earthDistances;

    /** Earth constraint weight. */
    private double earthConstraintWeight;

    
    /** Build a new instance without Earth constraint (with default Rugged names).
     * @param sensorNameA name of the sensor A to which mapping applies
     * @param sensorNameB name of the sensor B to which mapping applies
     */
    public SensorToSensorMapping(final String sensorNameA, final String sensorNameB) {
        
        this(sensorNameA, RUGGED, sensorNameB, RUGGED, 0.0);
    }

    /** Build a new instance with Earth constraint.
     * @param sensorNameA name of the sensor A to which mapping applies
     * @param ruggedNameA name of the Rugged A to which mapping applies
     * @param sensorNameB name of the sensor B to which mapping applies
     * @param ruggedNameB name of the Rugged B to which mapping applies
     * @param earthConstraintWeight weight given to the Earth distance constraint
     * with respect to the LOS distance (between 0 and 1). 
     * <br>Weighting will be applied as follow :
     * <ul>
     *    <li>(1 - earthConstraintWeight) for LOS distance weighting</li>
     *    <li>earthConstraintWeight for Earth distance weighting</li>
     * </ul>
     */
    public SensorToSensorMapping(final String sensorNameA, final String ruggedNameA, 
                                 final String sensorNameB, final String ruggedNameB, 
                                 final double earthConstraintWeight) {
        
        this.interMapping = new SensorMapping<SensorPixel>(sensorNameA, ruggedNameA);
        this.sensorNameB = sensorNameB;
        this.ruggedNameB = ruggedNameB;
        this.losDistances = new ArrayList<Double>();
        this.earthDistances = new ArrayList<Double>();
        this.earthConstraintWeight = earthConstraintWeight;
    }

    /** Build a new instance without Earth constraints.
     * @param sensorNameA name of the sensor A to which mapping applies
     * @param ruggedNameA name of the Rugged A to which mapping applies
     * @param sensorNameB name of the sensor B to which mapping applies
     * @param ruggedNameB name of the Rugged B to which mapping applies
     */
    public SensorToSensorMapping(final String sensorNameA, final String ruggedNameA, 
                                 final String sensorNameB, final String ruggedNameB) {
        
        this(sensorNameA, ruggedNameA, sensorNameB, ruggedNameB, 0.0);
    }

    /** Build a new instance with Earth constraints  (with default Rugged names):
     * we want to minimize the distance between pixel A and Earth.
     * @param sensorNameA name of the sensor A to which mapping applies
     * @param sensorNameB name of the sensor B to which mapping applies
     * @param earthConstraintWeight weight given to the Earth distance constraint
     * with respect to the LOS distance (between 0 and 1). 
     * <br>Weighting will be applied as follow :
     * <ul>
     *    <li>(1 - earthConstraintWeight) for LOS distance weighting</li>
     *    <li>earthConstraintWeight for Earth distance weighting</li>
     * </ul>
     */
    public SensorToSensorMapping(final String sensorNameA, final String sensorNameB,
                                 final double earthConstraintWeight) {
        
        this(sensorNameA, RUGGED, sensorNameB, RUGGED, earthConstraintWeight);
    }

    /** Get the name of the sensor B to which mapping applies.
     * @return name of the sensor B to which mapping applies
     */
    public String getSensorNameB() {
        return sensorNameB;
    }

    /** Get the name of the sensor A to which mapping applies.
     * @return name of the sensor A to which mapping applies
     */
    public String getSensorNameA() {
        return interMapping.getSensorName();
    }

    /** Get the name of the Rugged B to which mapping applies.
     * @return name of the Rugged B to which mapping applies
     */
    public String getRuggedNameB() {
        return ruggedNameB;
    }

    /** Get the name of the Rugged A to which mapping applies.
     * @return name of the Rugged A to which mapping applies
     */
    public String getRuggedNameA() {
        return interMapping.getRuggedName();
    }

    /** Get all the inter-mapping entries.
     * @return an unmodifiable view of all mapping entries
     */
    public Set<Map.Entry<SensorPixel, SensorPixel>> getMapping() {
        return interMapping.getMapping();
    }

    /** Get distances between lines of sight (from both view).
     * @return the LOS distances
     */
    public  List<Double> getLosDistances() {
        return losDistances;
    }

    /** Get distances between Earth and pixel A (mapping with constraints).
     * @return the Earth distances
     */
    public List<Double> getEarthDistances() {
        return earthDistances;
    }

    /** Get the weight given to the Earth distance constraint with respect to the LOS distance.
     * @return the Earth constraint weight
     */
    public double getEarthConstraintWeight() {
        return earthConstraintWeight;
    }

    /** Get distance between Earth and pixel A, corresponding to the inter-mapping index.
     * @param idx inter-mapping index
     * @return the Earth distances at index idx
     */
    public Double getEarthDistance(final int idx) {
        return getEarthDistances().get(idx);
    }

    /** Get distance between LOS, corresponding to the inter-mapping index.
     * @param idx inter-mapping index
     * @return the LOS distance at index idx
     */
    public Double getLosDistance(final int idx) {
        return getLosDistances().get(idx);
    }

    /** Add a mapping between two sensor pixels (A and B) and corresponding distance between the LOS.
     * @param pixelA sensor pixel A
     * @param pixelB sensor pixel B corresponding to the sensor pixel A (by direct then inverse location)
     * @param losDistance distance between both line of sight
     */
    public void addMapping(final SensorPixel pixelA, final SensorPixel pixelB, final Double losDistance) {
        
        interMapping.addMapping(pixelA, pixelB);
        losDistances.add(losDistance);
    }

    /** Add a mapping between two sensor pixels (A and B) and corresponding distance between the LOS.
     *  and the Earth distance constraint associated with pixel A
     * @param pixelA sensor pixel A
     * @param pixelB sensor pixel B corresponding to the sensor pixel A (by direct then inverse location)
     * @param losDistance distance between both line of sight
     * @param earthDistance distance between Earth and pixel A
     */
    public void addMapping(final SensorPixel pixelA, final SensorPixel pixelB,
                           final Double losDistance, final Double earthDistance) {
        
        interMapping.addMapping(pixelA, pixelB);
        losDistances.add(losDistance);
        earthDistances.add(earthDistance);
    }

    /** Set the Earth constraint weight.
     * @param earthConstraintWeight the Earth constraint weight to set
     */
    public void setEarthConstraintWeight(final double earthConstraintWeight) {
        this.earthConstraintWeight = earthConstraintWeight;
    }
}
