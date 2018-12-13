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
package org.orekit.rugged.linesensor;

import java.util.stream.Stream;

import org.hipparchus.analysis.differentiation.DerivativeStructure;
import org.hipparchus.geometry.euclidean.threed.FieldVector3D;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.util.FastMath;
import org.orekit.rugged.errors.DumpManager;
import org.orekit.rugged.errors.RuggedException;
import org.orekit.rugged.los.TimeDependentLOS;
import org.orekit.rugged.utils.DSGenerator;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.ParameterDriver;

/** Line sensor model.
 * @author Luc Maisonobe
 * @author Guylaine Prat
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

    /** Get the drivers for LOS parameters.
     * @return drivers for LOS parameters
     * @since 2.0
     */
    public Stream<ParameterDriver> getParametersDrivers() {
        return los.getParametersDrivers();
    }

    /** Get the pixel normalized line-of-sight at some date.
     * @param date current date
     * @param i pixel index (must be between 0 and {@link #getNbPixels()} - 1
     * @return pixel normalized line-of-sight
     */
    public Vector3D getLOS(final AbsoluteDate date, final int i) {
        final Vector3D l = los.getLOS(i, date);
        DumpManager.dumpSensorLOS(this, date, i, l);
        return l;
    }

    /** Get the pixel normalized interpolated line-of-sight at some date.
     * @param date current date
     * @param i pixel index (must be between 0 and {@link #getNbPixels()} - 1
     * @return pixel normalized line-of-sight
     * @exception RuggedException if date cannot be handled
     * @since 2.0
     */
    public Vector3D getLOS(final AbsoluteDate date, final double i) {

        final int iInf = FastMath.max(0, FastMath.min(getNbPixels() - 2, (int) FastMath.floor(i)));
        final int iSup = iInf + 1;
        final Vector3D interpolatedLos     = new Vector3D(iSup - i, los.getLOS(iInf, date),
                                                  i - iInf, los.getLOS(iSup, date));
        return interpolatedLos;
    }

    /** Get the pixel normalized line-of-sight at some date,
     * and their derivatives with respect to estimated parameters.
     * @param date current date
     * @param i pixel index (must be between 0 and {@link #getNbPixels()} - 1
     * @param generator generator to use for building {@link DerivativeStructure} instances
     * @return pixel normalized line-of-sight
     * @since 2.0
     */
    public FieldVector3D<DerivativeStructure> getLOSDerivatives(final AbsoluteDate date, final int i,
                                                                final DSGenerator generator) {
        return los.getLOSDerivatives(i, date, generator);
    }

    /** Get the pixel normalized line-of-sight at some date,
     * and their derivatives with respect to estimated parameters.
     * @param date current date
     * @param i pixel index (must be between 0 and {@link #getNbPixels()} - 1
     * @param generator generator to use for building {@link DerivativeStructure} instances
     * @return pixel normalized line-of-sight
     * @since 2.0
     */
    public FieldVector3D<DerivativeStructure> getLOSDerivatives(final AbsoluteDate date, final double i,
                                                                final DSGenerator generator) {

        // find surrounding pixels of pixelB (in order to interpolate LOS from pixelB (that is not an integer)
        final int iInf = FastMath.max(0, FastMath.min(getNbPixels() - 2, (int) FastMath.floor(i)));
        final int iSup = iInf + 1;

        final FieldVector3D<DerivativeStructure> interpolatedLos = new FieldVector3D<DerivativeStructure> (
                                                                    iSup - i,
                                                                    los.getLOSDerivatives(iInf, date, generator),
                                                                    i - iInf,
                                                                    los.getLOSDerivatives(iSup, date, generator)).normalize();
        return interpolatedLos;
    }

    /** Get the date.
     * @param lineNumber line number
     * @return date corresponding to line number
     */
    public AbsoluteDate getDate(final double lineNumber) {
        final AbsoluteDate date = datationModel.getDate(lineNumber);
        DumpManager.dumpSensorDatation(this, lineNumber, date);
        return date;
    }

    /** Get the line number.
     * @param date date
     * @return line number corresponding to date
     */
    public double getLine(final AbsoluteDate date) {
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
