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
package fr.cs.examples.refiningPleiades.generators;

import org.hipparchus.random.GaussianRandomGenerator;
import org.hipparchus.random.UncorrelatedRandomVectorGenerator;
import org.hipparchus.random.Well19937a;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.rugged.api.Rugged;
import org.orekit.rugged.errors.RuggedException;
import org.orekit.rugged.errors.RuggedMessages;
import org.orekit.rugged.linesensor.LineSensor;
import org.orekit.rugged.linesensor.SensorPixel;
import org.orekit.rugged.adjustment.measurements.Observables;
import org.orekit.rugged.adjustment.measurements.SensorToSensorMapping;
import org.orekit.rugged.utils.SpacecraftToObservedBody;
import org.orekit.time.AbsoluteDate;

import fr.cs.examples.refiningPleiades.metrics.DistanceTools;

/**
 * Inter-measurements generator (sensor to sensor mapping).
 * @author Jonathan Guinet
 * @author Lucie Labatallee
 * @author Guylaine Prat
 * @since 2.0
 */
public class InterMeasurementGenerator implements Measurable {

    /** Mapping from sensor A to sensor B. */
    private SensorToSensorMapping interMapping;

    /** Observables which contains sensor to sensor mapping.*/
    private Observables observables;

    /** Rugged instance corresponding to the viewing model A */
    private Rugged ruggedA;

    /** Rugged instance corresponding to the viewing model B */
    private Rugged ruggedB;

    /** Sensor A */
    private LineSensor sensorA;

    /** Sensor B */
    private LineSensor sensorB;

    /** Number of measurements */
    private int measurementCount;

    /** Sensor name B */
    private String sensorNameB;

    /** Number of line for acquisition A */
    private int dimensionA;

    /** Number of line for acquisition B */
    private int dimensionB;

    /** Limit value for outlier points */
    private double outlier;


    /** Default constructor: measurements generation without outlier points control
     * and without Earth distance constraint.
     * @param ruggedA Rugged instance corresponding to the viewing model A
     * @param sensorNameA sensor name A
     * @param dimensionA number of line for acquisition A
     * @param ruggedB Rugged instance corresponding to the viewing model B
     * @param sensorNameB sensor name B
     * @param dimensionB number of line for acquisition B
     */
    public InterMeasurementGenerator(final Rugged ruggedA, final String sensorNameA, final int dimensionA,
                                 final Rugged ruggedB, final String sensorNameB, final int dimensionB) {

        // Initialize parameters
        initParams(ruggedA, sensorNameA, dimensionA, ruggedB, sensorNameB, dimensionB);

        // Generate reference mapping, without Earth distance constraint
        interMapping = new SensorToSensorMapping(sensorNameA, sensorNameB);

        // Create observables for two models
        observables = new Observables(2);
    }


    /** Constructor for measurements generation taking into account outlier points control,
     * without Earth distance constraint.
     * @param ruggedA Rugged instance corresponding to the viewing model A
     * @param sensorNameA sensor name A
     * @param dimensionA dimension for acquisition A
     * @param ruggedB Rugged instance corresponding to the viewing model B
     * @param sensorNameB sensor name B
     * @param dimensionB dimension for acquisition B
     * @param outlier limit value for outlier points
     */
    public InterMeasurementGenerator(final Rugged ruggedA, final String sensorNameA, final int dimensionA,
                                 final Rugged ruggedB, final String sensorNameB, final int dimensionB,
                                 final double outlier) {

        this(ruggedA, sensorNameA, dimensionA, ruggedB, sensorNameB, dimensionB);
        this.outlier = outlier;
    }

    /** Constructor for measurements generation taking into account outlier points control,
     * and Earth distance constraint.
     * @param ruggedA Rugged instance corresponding to the viewing model A
     * @param sensorNameA sensor name A
     * @param dimensionA dimension for acquisition A
     * @param ruggedB Rugged instance corresponding to the viewing model B
     * @param sensorNameB sensor name B
     * @param dimensionB dimension for acquisition B
     * @param outlier limit value for outlier points
     * @param earthConstraintWeight weight given to the Earth distance constraint
     * with respect to the LOS distance (between 0 and 1).
     */
    public InterMeasurementGenerator(final Rugged ruggedA, final String sensorNameA, final int dimensionA,
                                 final Rugged ruggedB, final String sensorNameB, final int dimensionB,
                                 final double outlier, final double earthConstraintWeight) {

        // Initialize parameters
        initParams(ruggedA, sensorNameA, dimensionA, ruggedB, sensorNameB, dimensionB);

        // Generate reference mapping, with Earth distance constraints.
        // Weighting will be applied as follow :
        //     (1-earthConstraintWeight) for losDistance weighting
        //     earthConstraintWeight for earthDistance weighting
        interMapping = new SensorToSensorMapping(sensorNameA, ruggedA.getName(), sensorNameB, ruggedB.getName(), earthConstraintWeight);

        // Outlier points control
        this.outlier = outlier;

        // Observables which contains sensor to sensor mapping
        this.observables = new Observables(2);
    }

    /** Get the mapping from sensor A to sensor B
     * @return the mapping from sensor A to sensor B
     */
    public SensorToSensorMapping getInterMapping() {
        return interMapping;
    }

    /**  Get the observables which contains sensor to sensor mapping
     * @return the observables which contains sensor to sensor mapping
     */
    public Observables getObservables() {
        return observables;
    }

    @Override
    public int getMeasurementCount() {
        return measurementCount;
    }

    @Override
    public void createMeasurement(final int lineSampling, final int pixelSampling) {

        // Search the sensor pixel seeing point
        final int minLine = 0;
        final int maxLine = dimensionB - 1;

        for (double line = 0; line < dimensionA; line += lineSampling) {

            final AbsoluteDate dateA = sensorA.getDate(line);

            for (double pixelA = 0; pixelA < sensorA.getNbPixels(); pixelA += pixelSampling) {

                final GeodeticPoint gpA = ruggedA.directLocation(dateA, sensorA.getPosition(),
                                                                 sensorA.getLOS(dateA, pixelA));

                final SensorPixel sensorPixelB = ruggedB.inverseLocation(sensorNameB, gpA, minLine, maxLine);

                // We need to test if the sensor pixel is found in the prescribed lines
                // otherwise the sensor pixel is null
                if (sensorPixelB != null) {
                	
                    final AbsoluteDate dateB = sensorB.getDate(sensorPixelB.getLineNumber());
                    final double pixelB = sensorPixelB.getPixelNumber();
                    final SpacecraftToObservedBody scToBodyA = ruggedA.getScToBody();

                    final GeodeticPoint gpB = ruggedB.directLocation(dateB, sensorB.getPosition(),
                                                                     sensorB.getLOS(dateB, pixelB));

                    final double geoDistance = DistanceTools.computeDistanceInMeter(gpA, gpB);

                    if (geoDistance < this.outlier) {
                    	
                        final SensorPixel realPixelA = new SensorPixel(line, pixelA);
                        final SensorPixel realPixelB = new SensorPixel(sensorPixelB.getLineNumber(), sensorPixelB.getPixelNumber());

                        final AbsoluteDate realDateA = sensorA.getDate(realPixelA.getLineNumber());
                        final AbsoluteDate realDateB = sensorB.getDate(realPixelB.getLineNumber());
                        final double[] distanceLOSB = ruggedB.distanceBetweenLOS(sensorA, realDateA, realPixelA.getPixelNumber(), scToBodyA,
                                                                                 sensorB, realDateB, realPixelB.getPixelNumber());

                        final double losDistance = 0.0;
                        final double earthDistance = distanceLOSB[1];

                        interMapping.addMapping(realPixelA, realPixelB, losDistance, earthDistance);

                        // Increment the number of measurements
                        this.measurementCount++;
                    }
                }
            }
        }

        this.observables.addInterMapping(interMapping);
    }


    /** Tie point creation from direct 1ocation with Rugged A and inverse location with Rugged B
     * @param lineSampling sampling along lines
     * @param pixelSampling sampling along columns
     * @param noise errors to apply to measure for pixel A and pixel B
     */
    public void createNoisyMeasurement(final int lineSampling, final int pixelSampling, final Noise noise) {

    	// Get noise features (errors)
    	// [pixelA, pixelB] mean
    	final double[] mean = noise.getMean();
    	// [pixelA, pixelB] standard deviation
    	final double[] std = noise.getStandardDeviation();

    	// Search the sensor pixel seeing point
        final int minLine = 0;
        final int maxLine = dimensionB - 1;

        final double meanA[] = { mean[0], mean[0] };
        final double stdA[]  = { std[0], std[0] };
        final double meanB[] = { mean[1], mean[1] };
        final double stdB[]  = { std[1], std[1] };

        // Seed has been fixed for tests purpose
        final GaussianRandomGenerator rngA = new GaussianRandomGenerator(new Well19937a(0xefac03d9be4d24b9l));
        final UncorrelatedRandomVectorGenerator rvgA = new UncorrelatedRandomVectorGenerator(meanA, stdA, rngA);

        // Seed has been fixed for tests purpose
        final GaussianRandomGenerator rngB = new GaussianRandomGenerator(new Well19937a(0xdf1c03d9be0b34b9l));
        final UncorrelatedRandomVectorGenerator rvgB = new UncorrelatedRandomVectorGenerator(meanB, stdB, rngB);

        for (double line = 0; line < dimensionA; line += lineSampling) {

            final AbsoluteDate dateA = sensorA.getDate(line);
            for (double pixelA = 0; pixelA < sensorA.getNbPixels(); pixelA += pixelSampling) {

                final GeodeticPoint gpA = ruggedA.directLocation(dateA, sensorA.getPosition(),
                                                                 sensorA.getLOS(dateA, pixelA));
                final SensorPixel sensorPixelB = ruggedB.inverseLocation(sensorNameB, gpA, minLine, maxLine);

                // We need to test if the sensor pixel is found in the prescribed lines
                // otherwise the sensor pixel is null
                if (sensorPixelB != null) {
                	
                    final AbsoluteDate dateB = sensorB.getDate(sensorPixelB.getLineNumber());
                    final double pixelB = sensorPixelB.getPixelNumber();

                    // Get spacecraft to body transform of Rugged instance A
                    final SpacecraftToObservedBody scToBodyA = ruggedA.getScToBody();

                    final GeodeticPoint gpB = ruggedB.directLocation(dateB, sensorB.getPosition(),
                                                                     sensorB.getLOS(dateB, pixelB));
                    final double geoDistance = DistanceTools.computeDistanceInMeter(gpA, gpB);
                    // Create the inter mapping if distance is below outlier value
                    if (geoDistance < outlier) {

                        final double[] vecRandomA = rvgA.nextVector();
                        final double[] vecRandomB = rvgB.nextVector();

                        final SensorPixel realPixelA = new SensorPixel(line + vecRandomA[0], pixelA + vecRandomA[1]);
                        final SensorPixel realPixelB = new SensorPixel(sensorPixelB.getLineNumber() + vecRandomB[0],
                                                                       sensorPixelB.getPixelNumber() + vecRandomB[1]);
                        final AbsoluteDate realDateA = sensorA.getDate(realPixelA.getLineNumber());
                        final AbsoluteDate realDateB = sensorB.getDate(realPixelB.getLineNumber());
                        final double[] distanceLOSB = ruggedB.distanceBetweenLOS(sensorA, realDateA, realPixelA.getPixelNumber(), scToBodyA,
                                                                                 sensorB, realDateB, realPixelB.getPixelNumber());
                        final Double losDistance = 0.0;
                        final Double earthDistance = distanceLOSB[1];

                        interMapping.addMapping(realPixelA, realPixelB, losDistance, earthDistance);

                        // Increment the number of measurements
                        this.measurementCount++;
                        
                    } // end test if geoDistance < outlier
                } // end test if sensorPixelB != null
            } // end loop on pixel of sensorA
        } // end loop on line of sensorA

        this.observables.addInterMapping(interMapping);
    }

    /** Default constructor: measurements generation without outlier points control
     * and Earth distances constraint.
     * @param rA Rugged instance A
     * @param sNameA sensor name A
     * @param dimA dimension for acquisition A
     * @param rB Rugged instance B
     * @param sNameB sensor name B
     * @param dimB dimension for acquisition B
     */
    private void initParams(final Rugged rA, final String sNameA, final int dimA,
                            final Rugged rB, final String sNameB, final int dimB) {

        this.sensorNameB = sNameB;
        // Check that sensors's name is different
        if (sNameA.contains(sNameB)) {
           throw new RuggedException(RuggedMessages.DUPLICATED_PARAMETER_NAME, sNameA);
        }

        this.ruggedA = rA;
        this.ruggedB = rB;

        this.sensorA = rA.getLineSensor(sNameA);
        this.sensorB = rB.getLineSensor(sNameB);

        this.dimensionA = dimA;
        this.dimensionB = dimB;

        this.measurementCount = 0;
    }
}
