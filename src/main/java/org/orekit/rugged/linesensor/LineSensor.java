/* Copyright 2013-2015 CS Systèmes d'Information
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
package org.orekit.rugged.linesensor;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.orekit.rugged.errors.DumpManager;
import org.orekit.rugged.errors.RuggedException;
import org.orekit.rugged.los.TimeDependentLOS;
import org.orekit.time.AbsoluteDate;

/** Line sensor model.
 * @author Luc Maisonobe
 */
public class LineSensor {

    /** Name of the sensor. */
    private final String name;

    /** Datation model. */
    private final LineDatation datationModel;

    /** Sensor position. */
    private final Vector3D position;

    /** Pixels lines-of-sight. */
    private final TimeDependentLOS los;

    /** Simple constructor.
     * @param name name of the sensor
     * @param datationModel datation model
     * @param position sensor position in spacecraft frame
     * @param los pixels lines-of-sight in spacecraft frame
     * @see org.orekit.rugged.los.LOSBuilder
     */
    public LineSensor(final String name, final LineDatation datationModel,
                      final Vector3D position, final TimeDependentLOS los) {

        this.name          = name;
        this.datationModel = datationModel;
        this.position      = position;
        this.los           = los;

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
        return los.getNbPixels();
    }

    /** Get the pixel normalized line-of-sight at some date.
     * @param date current date
     * @param i pixel index (must be between 0 and {@link #getNbPixels()} - 1
     * @return pixel normalized line-of-sight
     * @exception RuggedException if date cannot be handled
     */
    public Vector3D getLos(final AbsoluteDate date, final int i)
        throws RuggedException {
        final Vector3D l = los.getLOS(i, date);
        DumpManager.dumpSensorLOS(this, date, i, l);
        return l;
    }

    /** Get the date.
     * @param lineNumber line number
     * @return date corresponding to line number
     * @exception RuggedException if date cannot be handled
     */
    public AbsoluteDate getDate(final double lineNumber)
        throws RuggedException {
        final AbsoluteDate date = datationModel.getDate(lineNumber);
        DumpManager.dumpSensorDatation(this, lineNumber, date);
        return date;
    }

    /** Get the line number.
     * @param date date
     * @return line number corresponding to date
     * @exception RuggedException if date cannot be handled
     */
    public double getLine(final AbsoluteDate date)
        throws RuggedException {
        final double lineNumber = datationModel.getLine(date);
        DumpManager.dumpSensorDatation(this, lineNumber, date);
        return lineNumber;
    }

    /** Get the rate of lines scanning.
     * @param lineNumber line number
     * @return rate of lines scanning (lines / seconds)
     */
    public double getRate(final double lineNumber) {
        final double rate = datationModel.getRate(lineNumber);
        DumpManager.dumpSensorRate(this, lineNumber, rate);
        return rate;
    }

    /** Get the sensor position.
     * @return position
     */
    public Vector3D getPosition() {
        return position;
    }

}
