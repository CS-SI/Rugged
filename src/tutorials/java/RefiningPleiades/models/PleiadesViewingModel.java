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
package RefiningPleiades.models;

import org.hipparchus.geometry.euclidean.threed.Rotation;
import org.hipparchus.geometry.euclidean.threed.RotationConvention;
import org.hipparchus.geometry.euclidean.threed.Vector3D;

import org.hipparchus.util.FastMath;
import java.util.ArrayList;
import java.util.List;


import org.orekit.rugged.los.FixedRotation;
import org.orekit.rugged.los.FixedZHomothety;
import org.orekit.rugged.los.LOSBuilder;
import org.orekit.rugged.los.TimeDependentLOS;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeScale;
import org.orekit.time.TimeScalesFactory;
import org.orekit.rugged.linesensor.LinearLineDatation;
import org.orekit.rugged.linesensor.LineDatation;
import org.orekit.rugged.linesensor.LineSensor;

import org.orekit.rugged.errors.RuggedException;
import org.orekit.errors.OrekitException;

/**
 * TODO GP add comments for tuto 
 * Pleiades viewing model class definition.
 * @author Jonathan Guinet
 * @author Lucie Labat-Allee
 * @since 2.0
 */

public class PleiadesViewingModel {

    /** intrinsic Pleiades parameters. */
    private double fov = 1.65; // 20km - alt 694km
    
    // Number of line of the sensor
    private int dimension = 40000;

    private double angle;
    private LineSensor lineSensor;
    private String date;

    private String sensorName;

    private final double linePeriod =  1e-4;


    /** Simple constructor.
     * <p>
     *  initialize PleiadesViewingModel with
     *  sensorName="line", incidenceAngle = 0.0, date = "2016-01-01T12:00:00.0"
     * </p>
     */
    public PleiadesViewingModel(final String sensorName) throws RuggedException, OrekitException {
    	
        this(sensorName, 0.0, "2016-01-01T12:00:00.0");
    }

    /** PleiadesViewingModel constructor.
     * @param sensorName sensor name
     * @param incidenceAngle incidence angle
     * @param referenceDate reference date
     * @throws RuggedException
     * @throws OrekitException
     */
    public PleiadesViewingModel(final String sensorName, final double incidenceAngle, final String referenceDate)
        throws RuggedException, OrekitException {
    	
        this.sensorName = sensorName;
        this.date = referenceDate;
        this.angle = incidenceAngle;
        this.createLineSensor();
    }

    /** TODO GP add comments
     */
    public LOSBuilder rawLOS(final Vector3D center, final Vector3D normal, final double halfAperture, final int n) {

        final List<Vector3D> list = new ArrayList<Vector3D>(n);
        for (int i = 0; i < n; ++i) {
            final double alpha = (halfAperture * (2 * i + 1 - n)) / (n - 1);
            list.add(new Rotation(normal, alpha, RotationConvention.VECTOR_OPERATOR).applyTo(center));
        }

        return new LOSBuilder(list);
    }

    /** TODO GP add comments
     */
    public TimeDependentLOS buildLOS() {
    	
        final LOSBuilder losBuilder = rawLOS(new Rotation(Vector3D.PLUS_I,
                                                          FastMath.toRadians(this.angle),
                                                          RotationConvention.VECTOR_OPERATOR).applyTo(Vector3D.PLUS_K),
                                             Vector3D.PLUS_I, FastMath.toRadians(fov / 2), dimension);

        losBuilder.addTransform(new FixedRotation(sensorName + "_roll",  Vector3D.MINUS_I, 0.00));
        losBuilder.addTransform(new FixedRotation(sensorName + "_pitch", Vector3D.MINUS_J, 0.00));

        // factor is a common parameters shared between all Pleiades models
        losBuilder.addTransform(new FixedZHomothety("factor", 1.0));
        
        return losBuilder.build();
    }


    /** TODO GP add comments
     */
    public AbsoluteDate getDatationReference() throws OrekitException {

    	// We use Orekit for handling time and dates, and Rugged for defining the datation model:
    	final TimeScale utc = TimeScalesFactory.getUTC();

    	return new AbsoluteDate(date, utc);
    }

    /** TODO GP add comments
     */
   public  AbsoluteDate getMinDate() throws RuggedException {
        return lineSensor.getDate(0);
    }

   /** TODO GP add comments
    */
   public  AbsoluteDate  getMaxDate() throws RuggedException {
        return lineSensor.getDate(dimension);
    }

   /** TODO GP add comments
    */
   public  LineSensor  getLineSensor() {
        return lineSensor;
    }

   /** TODO GP add comments
    */
   public  String getSensorName() {
        return sensorName;
    }

   /** Get the number of lines of the sensor
    * @return the number of lines of the sensor
    */
public int getDimension() {
        return dimension;
    }

   /** TODO GP add comments
    */
   private void createLineSensor() throws RuggedException, OrekitException {

        // Offset of the MSI from center of mass of satellite
        // one line sensor
        // los: swath in the (YZ) plane, looking at 50° roll, 2.6" per pixel
        final Vector3D msiOffset = new Vector3D(0, 0, 0);

        // TODO build complex los
        final TimeDependentLOS lineOfSight = buildLOS();

        final double rate =  1 / linePeriod;
        // linear datation model: at reference time we get the middle line, and the rate is one line every 1.5ms

        final LineDatation lineDatation = new LinearLineDatation(getDatationReference(), dimension / 2, rate);
        
        lineSensor = new LineSensor(sensorName, lineDatation, msiOffset, lineOfSight);
    }
}

