/*
 * Copyright 2013-2016 CS Systèmes d'Information Licensed to CS Systèmes
 * d'Information (CS) under one or more contributor license agreements. See the
 * NOTICE file distributed with this work for additional information regarding
 * copyright ownership. CS licenses this file to You under the Apache License,
 * Version 2.0 (the "License"); you may not use this file except in compliance
 * with the License. You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0 Unless required by applicable law
 * or agreed to in writing, software distributed under the License is
 * distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied. See the License for the specific language
 * governing permissions and limitations under the License.
 */
package org.orekit.rugged.refining.generators;

import org.orekit.bodies.GeodeticPoint;
import org.orekit.rugged.api.Rugged;
import org.orekit.rugged.errors.RuggedException;
import org.orekit.rugged.errors.RuggedExceptionWrapper;
import org.orekit.rugged.errors.RuggedMessages;
import org.orekit.rugged.linesensor.LineSensor;
import org.orekit.rugged.linesensor.SensorPixel;
import org.orekit.rugged.refining.measures.Noise;
import org.orekit.rugged.refining.measures.SensorToSensorMapping;
import org.orekit.rugged.refining.metrics.DistanceTools;
import org.orekit.rugged.refining.models.PleiadesViewingModel;
import org.orekit.rugged.utils.SpacecraftToObservedBody;

import org.orekit.time.AbsoluteDate;

import org.hipparchus.random.UncorrelatedRandomVectorGenerator;
import org.hipparchus.random.GaussianRandomGenerator;
import org.hipparchus.random.Well19937a;

/**
 * Inter-measures generator (sensor to sensor mapping)
 * @author Jonathan Guinet
 * @author Lucie Labatallee
 */
public class InterMeasureGenerator implements Measurable {

    /** mapping */
    private SensorToSensorMapping interMapping;

    private Rugged ruggedA;

    private Rugged ruggedB;

    private LineSensor sensorA;

    private LineSensor sensorB;

    private PleiadesViewingModel viewingModelA;

    private PleiadesViewingModel viewingModelB;

    private int measureCount;

    private String sensorNameA;

    private String sensorNameB;
    
    private double outlier=0.0;

    /** Default constructor: measures generation without outlier points control 
     * and without Earth distance constraint.
     * @param viewingModelA
     * @param ruggedA
     * @param viewingModelB
     * @param ruggedB
     * @throws RuggedException
     */
    public InterMeasureGenerator(PleiadesViewingModel viewingModelA,
                                          Rugged ruggedA,
                                          PleiadesViewingModel viewingModelB,
                                          Rugged ruggedB)
        throws RuggedException {

        // Initialize parameters
        this.initParams(viewingModelA, ruggedA, viewingModelB, ruggedB);

        // generate reference mapping, without Earth distance constraint
        interMapping = new SensorToSensorMapping(sensorNameA, sensorNameB);

    }
    
    
    /** Constructor for measures generation taking into account outlier points control,
     * without Earth distance constraint. 
     * @param viewingModelA viewing model A
     * @param ruggedA Rugged instance corresponding to the viewing model A
     * @param viewingModelB viewing model B
     * @param ruggedB Rugged instance corresponding to the viewing model B
     * @param outlier limit value of outlier points
     * @throws RuggedException
     */
    public InterMeasureGenerator(PleiadesViewingModel viewingModelA,
                                          Rugged ruggedA,
                                          PleiadesViewingModel viewingModelB,
                                          Rugged ruggedB, double outlier)
        throws RuggedException {

        this(viewingModelA, ruggedA, viewingModelB, ruggedB);
        this.outlier = outlier;

    }

    
    /** Constructor for measures generation taking into account outlier points control,
     * and Earth distance constraint. 
     * @param viewingModelA viewing model A
     * @param ruggedA Rugged instance corresponding to the viewing model A
     * @param viewingModelB viewing model B
     * @param ruggedB Rugged instance corresponding to the viewing model B
     * @param outlier limit value of outlier points
     * @param earthConstraintWeight weight given to the Earth distance constraint 
     * with respect to the LOS distance (between 0 and 1).
     * @throws RuggedException
     * @see Rugged#estimateFreeParams2Models
     */
    public InterMeasureGenerator(PleiadesViewingModel viewingModelA,
                                 Rugged ruggedA,
                                 PleiadesViewingModel viewingModelB,
                                 Rugged ruggedB, double outlier, 
                                 final double earthConstraintWeight)
        throws RuggedException {

     // Initialize parameters
        this.initParams(viewingModelA, ruggedA, viewingModelB, ruggedB);

        // generate reference mapping, with Earth distance constraints
        // Weighting will be applied as follow :
        // (1-earthConstraintWeight) for losDistance weighting
        // earthConstraintWeight for earthDistance weighting
        interMapping = new SensorToSensorMapping(sensorNameA, sensorNameB, earthConstraintWeight);

        // outlier points control
        this.outlier = outlier;

    }

    public SensorToSensorMapping getInterMapping() {
        return interMapping;
    }

    public int getMeasureCount() {
        return measureCount;
    }

    public void createMeasure(final int lineSampling, final int pixelSampling) throws RuggedException {
        // Get outlier
        double outlier = this.outlier;
                    
        // Search the sensor pixel seeing point
        final int minLine = 0;
        final int maxLine = viewingModelB.dimension - 1;
                    
        for (double line = 0; line < viewingModelA.dimension; line += lineSampling) {

            AbsoluteDate dateA = sensorA.getDate(line);
            
            for (double pixelA = 0; pixelA < sensorA.getNbPixels(); pixelA += pixelSampling) {

                GeodeticPoint gpA = ruggedA.directLocation(dateA, sensorA.getPosition(),
                                                           sensorA.getLOS(dateA, pixelA));

                SensorPixel sensorPixelB = ruggedB.inverseLocation(sensorNameB, gpA, minLine, maxLine);
                // we need to test if the sensor pixel is found in the
                // prescribed lines otherwise the sensor pixel is null
                if (sensorPixelB != null) {
                    final AbsoluteDate dateB = sensorB.getDate(sensorPixelB.getLineNumber());
                    double pixelB = sensorPixelB.getPixelNumber();
                    final SpacecraftToObservedBody scToBodyA = ruggedA.getScToBody();
                               
                    GeodeticPoint gpB = ruggedB.directLocation(dateB, sensorB.getPosition(),
                                                               sensorB.getLOS(dateB, pixelB));
                                
                    double GEOdistance = DistanceTools.computeDistanceRad(gpA.getLongitude(), gpA.getLatitude(),
                                                                          gpB.getLongitude(), gpB.getLatitude());

                    if(GEOdistance < outlier)
                    {
                    /*System.out.format("A line %2.3f pixel %2.3f %n", line, pixelA);*/

                    SensorPixel RealPixelA = new SensorPixel(line, pixelA);

                    SensorPixel RealPixelB = new SensorPixel(sensorPixelB.getLineNumber(), 
                                                             sensorPixelB.getPixelNumber());

                    AbsoluteDate RealDateA = sensorA.getDate(RealPixelA.getLineNumber());
                    AbsoluteDate RealDateB = sensorB.getDate(RealPixelB.getLineNumber());
                    double[] distanceLOSB = ruggedB.distanceBetweenLOS(sensorA, RealDateA, RealPixelA.getPixelNumber(), scToBodyA,
                                                                       sensorB, RealDateB, RealPixelB.getPixelNumber());
                    
                    final double losDistance = distanceLOSB[1];
                    final double earthDistance = 0.0;
                    
                    interMapping.addMapping(RealPixelA, RealPixelB, losDistance, earthDistance);
                    measureCount++;
                    }
                    else
                    {
                        /*System.out.format("A line %2.3f pixel %2.3f %n rejected ", line, pixelA);*/
                    }
                } 
            }
        }
    }


    /**
     * tie point creation from directLocation with RuggedA and inverse location
     * with RuggedB
     * @param lineSampling sampling along lines
     * @param pixelSampling sampling along columns
     * @param noise error tabulation
     * @throws RuggedException
     */
    public void createNoisyMeasure(final int lineSampling, final int pixelSampling, final Noise noise)
        throws RuggedException {
        
        // Get outlier
        double outlier = this.outlier;
        
        /* Get noise features (errors) */
        final double[] mean = noise.getMean(); /* [pixelA, pixelB] mean */
        final double[] std = noise.getStandardDeviation(); /* [pixelA, pixelB] standard deviation */
        
        
        // Search the sensor pixel seeing point
        final int minLine = 0;
        final int maxLine = viewingModelB.dimension - 1;
        
        double meanA[] =  {mean[0],mean[0]};
        double stdA[] = {std[0],std[0]};
        double meanB[] =  {mean[1],mean[1]};
        double stdB[] = {std[1],std[1]};  

        GaussianRandomGenerator rngA = new GaussianRandomGenerator(new Well19937a(0xefac03d9be4d24b9l));
        UncorrelatedRandomVectorGenerator rvgA = new UncorrelatedRandomVectorGenerator(meanA, stdA, rngA);

        GaussianRandomGenerator rngB = new GaussianRandomGenerator(new Well19937a(0xdf1c03d9be0b34b9l));
        UncorrelatedRandomVectorGenerator rvgB = new UncorrelatedRandomVectorGenerator(meanB, stdB, rngB);

        
        
        
        
        for (double line = 0; line < viewingModelA.dimension; line += lineSampling) {

            AbsoluteDate dateA = sensorA.getDate(line);
            for (double pixelA = 0; pixelA < sensorA
                .getNbPixels(); pixelA += pixelSampling) {

                GeodeticPoint gpA = ruggedA
                    .directLocation(dateA, sensorA.getPosition(),
                                    sensorA.getLOS(dateA, pixelA));

                SensorPixel sensorPixelB = ruggedB
                    .inverseLocation(sensorNameB, gpA, minLine, maxLine);
                // we need to test if the sensor pixel is found in the
                // prescribed lines otherwise the sensor pixel is null
                if (sensorPixelB != null) {
                    //System.out.format("  %n");
                    final AbsoluteDate dateB = sensorB
                        .getDate(sensorPixelB.getLineNumber());
                    double pixelB = sensorPixelB.getPixelNumber();
                    final SpacecraftToObservedBody scToBodyA = ruggedA
                                    .getScToBody();
                   
                    
                    //System.out.format("%n distance %1.8e dist %f %n",distanceLOSB[0],distanceLOSB[1]);
                    // Get spacecraft to body transform of rugged instance A
                    
                    /*final SpacecraftToObservedBody scToBodyA = ruggedA
                        .getScToBody();
                    final SpacecraftToObservedBody scToBodyB = ruggedB
                        .getScToBody();

                    double distanceLOSB = ruggedB
                        .distanceBetweenLOS(sensorA, dateA, pixelA, scToBodyA,
                                            sensorB, dateB, pixelB);
                    double distanceLOSA = ruggedA
                                    .distanceBetweenLOS(sensorB, dateB, pixelB, scToBodyB,
                                                        sensorA, dateA, pixelA);*/
                    
                    //System.out.format("distance LOS %1.8e %1.8e %n",distanceLOSB,distanceLOSA);
                    
                    
                    
                    GeodeticPoint gpB = ruggedB
                                    .directLocation(dateB, sensorB.getPosition(),
                                                    sensorB.getLOS(dateB, pixelB));
                    
                    double GEOdistance = DistanceTools.computeDistanceRad(gpA.getLongitude(), gpA.getLatitude(),
                                                                gpB.getLongitude(), gpB.getLatitude());

                    if(GEOdistance < outlier)
                    {
                        /*System.out.format("A line %2.3f pixel %2.3f %n", line,
                                          pixelA);*/

                        final double[] vecRandomA = rvgA.nextVector();
                        final double[] vecRandomB = rvgB.nextVector();

                        SensorPixel RealPixelA = new SensorPixel(line +
                                                                 vecRandomA[0],
                                                                 pixelA + vecRandomA[1]);
                        SensorPixel RealPixelB = new SensorPixel(sensorPixelB
                            .getLineNumber() + vecRandomB[0], sensorPixelB
                                .getPixelNumber() + vecRandomB[1]);
                         /*System.out.format("pix A %f %f Real %f %f %n", line,
                         pixelA, RealPixelA.getLineNumber(),
                         RealPixelA.getPixelNumber());
                         System.out.format("pix B %f %f Real %f %f %n",
                         sensorPixelB.getLineNumber(),
                         sensorPixelB.getPixelNumber(),
                         RealPixelB.getLineNumber(),
                        RealPixelB.getPixelNumber());*/
                        AbsoluteDate RealDateA = sensorA.getDate(RealPixelA.getLineNumber());
                        AbsoluteDate RealDateB = sensorB.getDate(RealPixelB.getLineNumber());
                        double[] distanceLOSB = ruggedB
                                        .distanceBetweenLOS(sensorA, RealDateA, RealPixelA.getPixelNumber(), scToBodyA,
                                                            sensorB, RealDateB, RealPixelB.getPixelNumber());
                        
                        final Double losDistance = 0.0;
                        final Double earthDistance = distanceLOSB[1];
                        
                        interMapping.addMapping(RealPixelA, RealPixelB, losDistance, earthDistance);
                        measureCount++;
                    }
                    else
                    {
                        /*System.out.format("A line %2.3f pixel %2.3f %n rejected ", line,
                                          pixelA);*/
                    }
                } 

            }
        }
    }
    
    /** Default constructor: measures generation without outlier points control 
     * and Earth distances constraint.
     * @param viewingModelA
     * @param ruggedA
     * @param viewingModelB
     * @param ruggedB
     * @throws RuggedException
     */
    private void initParams(PleiadesViewingModel viewingModelA,
                            Rugged ruggedA,
                            PleiadesViewingModel viewingModelB,
                            Rugged ruggedB) throws RuggedException {
        
        // generate reference mapping
        sensorNameA = viewingModelA.getSensorName();
        sensorNameB = viewingModelB.getSensorName();
        // check that sensors's name is different
        if (sensorNameA.contains(sensorNameB)) {
            throw new RuggedExceptionWrapper(new RuggedException(RuggedMessages.DUPLICATED_PARAMETER_NAME,
                                                        sensorNameA));
        }

        this.ruggedA = ruggedA;
        this.ruggedB = ruggedB;

        this.viewingModelA = viewingModelA;
        this.viewingModelB = viewingModelB;

        sensorA = ruggedA.getLineSensor(sensorNameA);
        sensorB = ruggedB.getLineSensor(sensorNameB);
        measureCount = 0;

    }

}