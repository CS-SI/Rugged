/* Copyright 2013-2014 CS Systèmes d'Information
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
package org.orekit.rugged.core;

import java.util.List;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.orekit.rugged.api.LineDatation;
import org.orekit.time.AbsoluteDate;

/** Container for sensor data.
 * <p>
 * Instances of this class are guaranteed to be immutable.
 * </p>
 * @author Luc Maisonobe
 */
class Sensor {

    /** Name of the sensor. */
    private final String name;

    /** Reference date. */
    private final AbsoluteDate referenceDate;

    /** Pixels positions. */
    private final List<Vector3D> positions;

    /** Pixels lines-of-sight. */
    private final List<Vector3D> los;

    /** Datation model. */
    private final LineDatation datationModel;

    /** Simple constructor.
     * @param name name of the sensor
     * @param referenceDate reference date
     * @param positions pixels positions
     * @param los pixels lines-of-sight
     * @param datationModel datation model
     */
    public Sensor(final String name,
                  final AbsoluteDate referenceDate, final LineDatation datationModel,
                  final List<Vector3D> positions, final List<Vector3D> los) {
        this.name          = name;
        this.referenceDate = referenceDate;
        this.positions     = positions;
        this.los           = los;
        this.datationModel = datationModel;
    }

    /** Get the name of the sensor.
     * @return name of the sensor
     */
    public String getName() {
        return name;
    }

    /** Get the number of pixels.
     * @return number of pixels
     */
    public int getNbPixels() {
        return positions.size();
    }

    /** Get pixel position.
     * @param i pixel index (must be between 0 and {@link #getNbPixels()}
     * @return pixel position
     */
    public Vector3D getPosition(final int i) {
        return positions.get(i);
    }

    /** Get the pixel line-of-sight.
     * @param i pixel index (must be between 0 and {@link #getNbPixels()}
     * @return pixel line-of-sight
     */
    public Vector3D getLos(final int i) {
        return los.get(i);
    }

    /** Get the date.
     * @param lineNumber line number
     * @return date corresponding to line number
     */
    public AbsoluteDate getDate(final double lineNumber) {
        return referenceDate.shiftedBy(datationModel.getDate(lineNumber));
    }

    /** Get the line number.
     * @param date date
     * @return line number corresponding to date
     */
    public double getLine(final AbsoluteDate date) {
        return datationModel.getLine(date.durationFrom(referenceDate));
    }

}
