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

import java.util.Collections;
import java.util.IdentityHashMap;
import java.util.Map;
import java.util.Set;

import org.orekit.bodies.GeodeticPoint;
import org.orekit.rugged.linesensor.LineSensor;
import org.orekit.rugged.linesensor.SensorPixel;

/** Container for mapping between sensor pixels and ground points.
 * @author Luc Maisonobe
 * @since 2.0
 */
public class SensorToGroundMapping {

    /** Sensor to which mapping applies. */
    private final LineSensor sensor;

    /** Mapping from sensor to ground. */
    private final Map<SensorPixel, GeodeticPoint> sensorToGround;

    /** Mapping from ground to sensor. */
    private final Map<GeodeticPoint, SensorPixel> groundToSensor;

    /** Build a new instance.
     * @param sensor sensor to which mapping applies
     */
    public SensorToGroundMapping(final LineSensor sensor) {
        this.sensor         = sensor;
        this.sensorToGround = new IdentityHashMap<>();
        this.groundToSensor = new IdentityHashMap<>();
    }

    /** Get the sensor to which mapping applies.
     * @return sensor to which mapping applies
     */
    public LineSensor getSensor() {
        return sensor;
    }

    /** Add a mapping between one sensor pixel and one ground point.
     * @param pixel sensor pixel
     * @param groundPoint ground point corresponding to the sensor pixel
     */
    public void addMapping(final SensorPixel pixel, final GeodeticPoint groundPoint) {
        sensorToGround.put(pixel, groundPoint);
        groundToSensor.put(groundPoint, pixel);
    }

    /** Get the ground point corresponding to a pixel.
     * @param pixel sensor pixel (it must one of the instances
     * passed to {@link #addMapping(SensorPixel, GeodeticPoint)},
     * not a pixel at the same location)
     * @return corresponding ground point, or null if the pixel was
     * not passed to {@link #addMapping(SensorPixel, GeodeticPoint)}
     */
    public GeodeticPoint getGroundPoint(final SensorPixel pixel) {
        return sensorToGround.get(pixel);
    }

    /** Get the sensor pixel corresponding to a ground point.
     * @param groundPoint ground point (it must one of the instances
     * passed to {@link #addMapping(SensorPixel, GeodeticPoint)},
     * not a ground point at the same location)
     * @return corresponding sensor pixel, or null if the ground point
     * was not passed to {@link #addMapping(SensorPixel, GeodeticPoint)}
     */
    public SensorPixel getPixel(final GeodeticPoint groundPoint) {
        return groundToSensor.get(groundPoint);
    }

    /** Get all the mapping entries.
     * @return an unmodifiable view of all mapping entries
     */
    public Set<Map.Entry<SensorPixel, GeodeticPoint>> getMappings() {
        return Collections.unmodifiableSet(sensorToGround.entrySet());
    }

}
