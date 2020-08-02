/* Copyright 2013-2020 CS GROUP
 * Licensed to CS GROUP (CS) under one or more
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
package fr.cs.examples.refiningPleiades.models;

import java.util.ArrayList;
import java.util.List;

import org.hipparchus.geometry.euclidean.threed.Rotation;
import org.hipparchus.geometry.euclidean.threed.RotationConvention;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.util.FastMath;
import org.orekit.rugged.linesensor.LineDatation;
import org.orekit.rugged.linesensor.LineSensor;
import org.orekit.rugged.linesensor.LinearLineDatation;
import org.orekit.rugged.los.FixedRotation;
import org.orekit.rugged.los.FixedZHomothety;
import org.orekit.rugged.los.LOSBuilder;
import org.orekit.rugged.los.TimeDependentLOS;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeScale;
import org.orekit.time.TimeScalesFactory;

import fr.cs.examples.refiningPleiades.Refining;

/**
 * Pleiades viewing model class definition.
 * The aim of this class is to simulate PHR sensor.
 * @author Jonathan Guinet
 * @author Lucie Labat-Allee
 * @author Guylaine Prat
 * @since 2.0
 */

public class PleiadesViewingModel {

    /** Pleiades parameters. */
    private static final double FOV = 1.65; // 20km - alt 694km
    private static final int DIMENSION = 40000;
    private static final double LINE_PERIOD =  1.e-4; 

    private double angle;
    private LineSensor lineSensor;
    private String date;

    private String sensorName;


    /** Simple constructor.
     * <p>
     *  initialize PleiadesViewingModel with
     *  sensorName="line", incidenceAngle = 0.0, date = "2016-01-01T12:00:00.0"
     * </p>
     */
    public PleiadesViewingModel(final String sensorName) {
    	
        this(sensorName, 0.0, "2016-01-01T12:00:00.0");
    }

    /** PleiadesViewingModel constructor.
     * @param sensorName sensor name
     * @param incidenceAngle incidence angle
     * @param referenceDate reference date
     */
    public PleiadesViewingModel(final String sensorName, final double incidenceAngle, final String referenceDate) {
    	
        this.sensorName = sensorName;
        this.date = referenceDate;
        this.angle = incidenceAngle;
        this.createLineSensor();
    }

    /** Create raw fixed Line Of sight
     */
    public LOSBuilder rawLOS(final Vector3D center, final Vector3D normal, final double halfAperture, final int n) {

        final List<Vector3D> list = new ArrayList<Vector3D>(n);
        for (int i = 0; i < n; ++i) {
            final double alpha = (halfAperture * (2 * i + 1 - n)) / (n - 1);
            list.add(new Rotation(normal, alpha, RotationConvention.VECTOR_OPERATOR).applyTo(center));
        }

        return new LOSBuilder(list);
    }

    /** Build a LOS provider
     */
    public TimeDependentLOS buildLOS() {
    	
        final LOSBuilder losBuilder = rawLOS(new Rotation(Vector3D.PLUS_I,
                                                          FastMath.toRadians(this.angle),
                                                          RotationConvention.VECTOR_OPERATOR).applyTo(Vector3D.PLUS_K),
                                             Vector3D.PLUS_I, FastMath.toRadians(FOV / 2), DIMENSION);

        losBuilder.addTransform(new FixedRotation(sensorName + Refining.getRollsuffix(),  Vector3D.MINUS_I, 0.00));
        losBuilder.addTransform(new FixedRotation(sensorName + Refining.getPitchsuffix(), Vector3D.MINUS_J, 0.00));

        // factor is a common parameters shared between all Pleiades models
        losBuilder.addTransform(new FixedZHomothety(Refining.getFactorname(), 1.0));

        return losBuilder.build();
    }


    /** Get the reference date.
     */
    public AbsoluteDate getDatationReference() {

    	// We use Orekit for handling time and dates, and Rugged for defining the datation model:
    	final TimeScale utc = TimeScalesFactory.getUTC();

    	return new AbsoluteDate(date, utc);
    }

    /** Get the min date.
     */
   public  AbsoluteDate getMinDate() {
        return lineSensor.getDate(0);
    }

   /** Get the max date.
    */
   public  AbsoluteDate  getMaxDate() {
        return lineSensor.getDate(DIMENSION);
    }

   /** Get the line sensor.
    */
   public  LineSensor  getLineSensor() {
        return lineSensor;
    }

   /** Get the sensor name.
    */
   public  String getSensorName() {
        return sensorName;
    }

   /** Get the number of lines of the sensor
    * @return the number of lines of the sensor
    */
public int getDimension() {
        return DIMENSION;
    }

   /** Create the line sensor
    */
   private void createLineSensor() {

        // Offset of the MSI from center of mass of satellite
        // one line sensor
        // los: swath in the (YZ) plane, looking at 50° roll, 2.6" per pixel
        final Vector3D msiOffset = new Vector3D(0, 0, 0);

        final TimeDependentLOS lineOfSight = buildLOS();

        final double rate =  1. / LINE_PERIOD;
        // linear datation model: at reference time we get the middle line, and the rate is one line every 1.5ms

        final LineDatation lineDatation = new LinearLineDatation(getDatationReference(), DIMENSION / 2, rate);

        lineSensor = new LineSensor(sensorName, lineDatation, msiOffset, lineOfSight);
    }
}

