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
package org.orekit.rugged.linesensor;

import java.util.stream.Stream;

import org.hipparchus.analysis.differentiation.DerivativeStructure;
import org.hipparchus.geometry.euclidean.threed.FieldVector3D;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.util.FastMath;
import org.orekit.errors.OrekitException;
import org.orekit.rugged.errors.DumpManager;
import org.orekit.rugged.errors.RuggedException;
import org.orekit.rugged.los.TimeDependentLOS;
import org.orekit.rugged.utils.ExtendedParameterDriver;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.ParameterDriver;
import org.orekit.utils.ParameterObserver;

/** Line sensor model.
 * @author Luc Maisonobe
 */
public class LineSensor {

    /** Parameters scaling factor.
     * <p>
     * We use a power of 2 to avoid numeric noise introduction
     * in the multiplications/divisions sequences.
     * </p>
     */
    private final double SCALE = FastMath.scalb(1.0, -3);

    /** Name of the sensor. */
    private final String name;

    /** Datation model. */
    private final LineDatation datationModel;

    /** Sensor position. */
    private Vector3D position;

    /** Sensor position, with derivatives. */
    private FieldVector3D<DerivativeStructure> positionDS;

    /** Pixels lines-of-sight. */
    private final TimeDependentLOS los;

    /** Driver for the sensor position parameter along X. */
    private final ExtendedParameterDriver xPos;

    /** Driver for the sensor position parameter along Y. */
    private final ExtendedParameterDriver yPos;

    /** Driver for the sensor position parameter along Z. */
    private final ExtendedParameterDriver zPos;

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
        this.los           = los;

        final ParameterObserver resettingObserver = new ParameterObserver() {
            /** {@inheritDoc} */
            @Override
            public void valueChanged(final double previousValue, final ParameterDriver driver) {
                LineSensor.this.position   = null;
                LineSensor.this.positionDS = null;
            }
        };

        try {
            xPos = new ExtendedParameterDriver(name + "-X", position.getX(), SCALE,
                                               Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
            xPos.addObserver(resettingObserver);
            yPos = new ExtendedParameterDriver(name + "-Y", position.getY(), SCALE,
                                               Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
            yPos.addObserver(resettingObserver);
            zPos = new ExtendedParameterDriver(name + "-Z", position.getZ(), SCALE,
                                               Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
            zPos.addObserver(resettingObserver);
        } catch (OrekitException oe) {
            // this should never happen
            throw RuggedException.createInternalError(oe);
        }

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
    public Stream<ExtendedParameterDriver> getExtendedParametersDrivers() {
        return Stream.<ExtendedParameterDriver>concat(Stream.of(xPos, yPos, zPos),
                                                      los.getExtendedParametersDrivers());
    }

    /** Get the pixel normalized line-of-sight at some date.
     * @param date current date
     * @param i pixel index (must be between 0 and {@link #getNbPixels()} - 1
     * @return pixel normalized line-of-sight
     * @exception RuggedException if date cannot be handled
     */
    public Vector3D getLOS(final AbsoluteDate date, final int i)
        throws RuggedException {
        final Vector3D l = los.getLOS(i, date);
        DumpManager.dumpSensorLOS(this, date, i, l);
        return l;
    }

    /** Get the pixel normalized line-of-sight at some date,
     * and their derivatives with respect to estimated parameters.
     * @param date current date
     * @param i pixel index (must be between 0 and {@link #getNbPixels()} - 1
     * @return pixel normalized line-of-sight
     */
    public FieldVector3D<DerivativeStructure> getLOSDerivatives(final AbsoluteDate date, final int i) {
        return los.getLOSDerivatives(i, date);
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
        if (position == null) {
            // lazy evaluation of the position
            position = new Vector3D(xPos.getValue(), yPos.getValue(), zPos.getValue());
        }
        return position;
    }

    /** Get the sensor position,
     * and its derivatives with respect to estimated parameters.
     * @return position
     */
    public FieldVector3D<DerivativeStructure> getPositionDerivatives() {
        if (positionDS == null) {
            // lazy evaluation of the position
            positionDS = new FieldVector3D<DerivativeStructure>(getCoordinate(xPos),
                                                                getCoordinate(yPos),
                                                                getCoordinate(zPos));
        }
        return positionDS;
    }

    /** Get a coordinate and its derivatives.
     * @param driver coordinate driver
     * @return coordinate value and its derivatives
     */
    private DerivativeStructure getCoordinate(final ExtendedParameterDriver driver) {
        final double value = driver.getValue();
        if (driver.isSelected()) {
            // the x coordinate of the sensor is estimated
            return new DerivativeStructure(driver.getNbEstimated(), 1, driver.getIndex(), value);
        } else {
            // the x coordinate of the sensor is not estimated
            return new DerivativeStructure(driver.getNbEstimated(), 1, value);
        }
    }

}
