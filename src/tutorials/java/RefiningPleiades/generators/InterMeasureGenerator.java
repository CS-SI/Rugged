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
package RefiningPleiades.generators;

import org.hipparchus.random.GaussianRandomGenerator;
import org.hipparchus.random.UncorrelatedRandomVectorGenerator;
import org.hipparchus.random.Well19937a;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.rugged.api.Rugged;
import org.orekit.rugged.errors.RuggedException;
import org.orekit.rugged.errors.RuggedExceptionWrapper;
import org.orekit.rugged.errors.RuggedMessages;
import org.orekit.rugged.linesensor.LineSensor;
import org.orekit.rugged.linesensor.SensorPixel;
import org.orekit.rugged.refining.measures.Noise;
import org.orekit.rugged.refining.measures.Observables;
import org.orekit.rugged.refining.measures.SensorToSensorMapping;
import org.orekit.rugged.utils.SpacecraftToObservedBody;
import org.orekit.time.AbsoluteDate;

import RefiningPleiades.metrics.DistanceTools;

/**
 * Inter-measures generator (sensor to sensor mapping).
 * @author Jonathan Guinet
 * @author Lucie Labatallee
 * @author Guylaine Prat
 * @since 2.0
 */
public class InterMeasureGenerator implements Measurable {

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

    /** Number of measures */
    private int measureCount;

    // TODO GP pas utilise ... 
    //   private String sensorNameA;

    /** Sensor name B */
    private String sensorNameB;

    /** Number of line for acquisition A */
    private int dimensionA;

    /** Number of line for acquisition B */
    private int dimensionB;
    
    /** Limit value for outlier points */
    private double outlier;


    /** Default constructor: measures generation without outlier points control
     * and without Earth distance constraint.
     * @param ruggedA Rugged instance corresponding to the viewing model A
     * @param sensorNameA sensor name A
     * @param dimensionA number of line for acquisition A
     * @param ruggedB Rugged instance corresponding to the viewing model B
     * @param sensorNameB sensor name B
     * @param dimensionB number of line for acquisition B
     * @throws RuggedException
     */
    public InterMeasureGenerator(final Rugged ruggedA, final String sensorNameA, final int dimensionA,
                                 final Rugged ruggedB, final String sensorNameB, final int dimensionB)
        throws RuggedException {

        // Initialize parameters
        initParams(ruggedA, sensorNameA, dimensionA, ruggedB, sensorNameB, dimensionB);

        // Generate reference mapping, without Earth distance constraint
        interMapping = new SensorToSensorMapping(sensorNameA, sensorNameB);

        // Create observables for two models
        observables = new Observables(2);
    }


    /** Constructor for measures generation taking into account outlier points control,
     * without Earth distance constraint.
     * @param ruggedA Rugged instance corresponding to the viewing model A
     * @param sensorNameA sensor name A
     * @param dimensionA dimension for acquisition A
     * @param ruggedB Rugged instance corresponding to the viewing model B
     * @param sensorNameB sensor name B
     * @param dimensionB dimension for acquisition B
     * @param outlier limit value for outlier points
     * @throws RuggedException
     */
    public InterMeasureGenerator(final Rugged ruggedA, final String sensorNameA, final int dimensionA,
                                 final Rugged ruggedB, final String sensorNameB, final int dimensionB,
                                 final double outlier)
        throws RuggedException {

        this(ruggedA, sensorNameA, dimensionA, ruggedB, sensorNameB, dimensionB);
        this.outlier = outlier;
    }

    /** Constructor for measures generation taking into account outlier points control,
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
     * @throws RuggedException
     */
    public InterMeasureGenerator(final Rugged ruggedA, final String sensorNameA, final int dimensionA,
                                 final Rugged ruggedB, final String sensorNameB, final int dimensionB,
                                 final double outlier, final double earthConstraintWeight)
        throws RuggedException {

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
    public int getMeasureCount() {
        return measureCount;
    }

    @Override
    public void createMeasure(final int lineSampling, final int pixelSampling) throws RuggedException {

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

                    final double GEOdistance = DistanceTools.computeDistanceInMeter(gpA.getLongitude(), gpA.getLatitude(),
                                                                                    gpB.getLongitude(), gpB.getLatitude());

                    if (GEOdistance < this.outlier) {
                    	
                        final SensorPixel RealPixelA = new SensorPixel(line, pixelA);
                        final SensorPixel RealPixelB = new SensorPixel(sensorPixelB.getLineNumber(), sensorPixelB.getPixelNumber());

                        final AbsoluteDate RealDateA = sensorA.getDate(RealPixelA.getLineNumber());
                        final AbsoluteDate RealDateB = sensorB.getDate(RealPixelB.getLineNumber());
                        final double[] distanceLOSB = ruggedB.distanceBetweenLOS(sensorA, RealDateA, RealPixelA.getPixelNumber(), scToBodyA,
                                                                                 sensorB, RealDateB, RealPixelB.getPixelNumber());

                        final double losDistance = 0.0;
                        final double earthDistance = distanceLOSB[1];

                        interMapping.addMapping(RealPixelA, RealPixelB, losDistance, earthDistance);

                        // increment the number of measures
                        this.measureCount++;
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
     * @throws RuggedException
     */
    public void createNoisyMeasure(final int lineSampling, final int pixelSampling, final Noise noise)
        throws RuggedException {

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

        // TODO GP explanation about seed ???
        final GaussianRandomGenerator rngA = new GaussianRandomGenerator(new Well19937a(0xefac03d9be4d24b9l));
        final UncorrelatedRandomVectorGenerator rvgA = new UncorrelatedRandomVectorGenerator(meanA, stdA, rngA);

        // TODO GP explanation about seed ???
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
                    final double GEOdistance = DistanceTools.computeDistanceInMeter(gpA.getLongitude(), gpA.getLatitude(),
                                                                                    gpB.getLongitude(), gpB.getLatitude());
                    // TODO GP explanation about computation here
                    if (GEOdistance < outlier) {

                        final double[] vecRandomA = rvgA.nextVector();
                        final double[] vecRandomB = rvgB.nextVector();

                        final SensorPixel RealPixelA = new SensorPixel(line + vecRandomA[0], pixelA + vecRandomA[1]);
                        final SensorPixel RealPixelB = new SensorPixel(sensorPixelB.getLineNumber() + vecRandomB[0],
                                                                       sensorPixelB.getPixelNumber() + vecRandomB[1]);
                        final AbsoluteDate RealDateA = sensorA.getDate(RealPixelA.getLineNumber());
                        final AbsoluteDate RealDateB = sensorB.getDate(RealPixelB.getLineNumber());
                        final double[] distanceLOSB = ruggedB.distanceBetweenLOS(sensorA, RealDateA, RealPixelA.getPixelNumber(), scToBodyA,
                                                                                 sensorB, RealDateB, RealPixelB.getPixelNumber());
                        final Double losDistance = 0.0;
                        final Double earthDistance = distanceLOSB[1];

                        interMapping.addMapping(RealPixelA, RealPixelB, losDistance, earthDistance);

                        // increment the number of measures
                        this.measureCount++;
                    }
                }
            }
        }
        
        this.observables.addInterMapping(interMapping);
    }

    /** Default constructor: measures generation without outlier points control
     * and Earth distances constraint.
     * @param rA Rugged instance A
     * @param sNameA sensor name A
     * @param dimA dimension for acquisition A
     * @param rB Rugged instance B
     * @param sNameB sensor name B
     * @param dimB dimension for acquisition B
     * @throws RuggedException
     */
    private void initParams(final Rugged rA, final String sNameA, final int dimA,
                            final Rugged rB, final String sNameB, final int dimB)
        throws RuggedException {

        this.sensorNameB = sNameB;
        // Check that sensors's name is different
        if (sNameA.contains(sNameB)) {
           throw new RuggedExceptionWrapper(new RuggedException(RuggedMessages.DUPLICATED_PARAMETER_NAME, sNameA));
        }

        this.ruggedA = rA;
        this.ruggedB = rB;

        this.sensorA = rA.getLineSensor(sNameA);
        this.sensorB = rB.getLineSensor(sNameB);

        this.dimensionA = dimA;
        this.dimensionB = dimB;

        this.measureCount = 0;
    }
}
