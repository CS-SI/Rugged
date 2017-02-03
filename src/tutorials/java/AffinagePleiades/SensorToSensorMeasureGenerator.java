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
package AffinagePleiades;

import org.orekit.rugged.api.SensorToSensorMapping;

import org.orekit.rugged.api.Rugged;
import org.orekit.rugged.linesensor.LineSensor;
import org.orekit.rugged.linesensor.SensorPixel;

import org.orekit.rugged.errors.RuggedException;
import org.orekit.rugged.errors.RuggedExceptionWrapper;
import org.orekit.rugged.errors.RuggedMessages;
import org.orekit.time.AbsoluteDate;

import org.hipparchus.random.UncorrelatedRandomVectorGenerator;
import org.hipparchus.random.GaussianRandomGenerator;
import org.hipparchus.random.Well19937a;
import org.hipparchus.geometry.euclidean.threed.Vector3D;

import org.orekit.bodies.GeodeticPoint;
import org.orekit.rugged.utils.SpacecraftToObservedBody;

/**
 * class for measure generation
 * 
 * @author Jonathan Guinet
 * @author Lucie Labatallee
 */
public class SensorToSensorMeasureGenerator {

    /** mapping */
    private SensorToSensorMapping mapping;

    private Rugged ruggedA;

    private Rugged ruggedB;

    private LineSensor sensorA;

    private LineSensor sensorB;

    private PleiadesViewingModel viewingModelA;

    private PleiadesViewingModel viewingModelB;

    private int measureCount;

    private String sensorNameA;

    private String sensorNameB;

    /**
     * SensorToSensorMeasureGenerator
     * 
     * @param viewingModelA
     * @param ruggedA
     * @param viewingModelB
     * @param ruggedB
     * @throws RuggedException
     */
    public SensorToSensorMeasureGenerator(PleiadesViewingModel viewingModelA,
                                          Rugged ruggedA,
                                          PleiadesViewingModel viewingModelB,
                                          Rugged ruggedB)
        throws RuggedException {

        // generate reference mapping
        sensorNameA = viewingModelA.getSensorName();
        sensorNameB = viewingModelB.getSensorName();
        // check that sensors's name is different
        if (sensorNameA.contains(sensorNameB)) {
            throw new RuggedExceptionWrapper(new RuggedException(RuggedMessages.DUPLICATED_PARAMETER_NAME,
                                                                 sensorNameA));
        }

        mapping = new SensorToSensorMapping(sensorNameA, sensorNameB);
        this.ruggedA = ruggedA;
        this.ruggedB = ruggedB;

        this.viewingModelA = viewingModelA;
        this.viewingModelB = viewingModelB;

        sensorA = ruggedA.getLineSensor(sensorNameA);
        sensorB = ruggedB.getLineSensor(sensorNameB);
        measureCount = 0;

    }

    public SensorToSensorMapping getMapping() {
        return mapping;
    }

    public int getMeasureCount() {
        return measureCount;
    }

    /**
     * tie point creation from directLocation with RuugedA and inverse location
     * with RuggedB
     * 
     * @param lineSampling sampling along lines
     * @param pixelSampling sampling along columns
     * @param err error tab [meanA,stdA,meanB,stdB]
     * @param outlier outlier value 
     * @throws RuggedException
     */
    public void CreateMeasure(final int lineSampling, final int pixelSampling, double[]err, double outlier)
        throws RuggedException {
        // Search the sensor pixel seeing point
        final int minLine = 0;
        final int maxLine = viewingModelB.dimension - 1;

        
        double meanA[] =  {err[0],err[0]};
        double stdA[] = {err[1],err[1]};
        double meanB[] =  {err[2],err[2]};
        double stdB[] = {err[3],err[3]};  

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

                        Vector3D los = sensorA.getLOS(dateA, pixelA);
                        /*System.out.format("A los %2.3f %2.3f %2.3f %n",
                                          los.getX(), los.getY(), los.getZ());
                        System.out.format("B line %2.3f pixel %2.3f %n",
                                          sensorPixelB.getLineNumber(),
                                          sensorPixelB.getPixelNumber());

                        System.out.format("distance %1.8e meters %n",
                                          GEOdistance);*/

                        // System.out.format("A los %2.3f %2.3f %2.3f
                        // %n",los.getX(),los.getY(),los.getZ());
                        

                        //
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
                        final Double[] distance = {0.0,distanceLOSB[1]};
                        //final Double[] distance = {0.0, 6.36807530e+06};
                        //final Double[] distance = {0.0, 6.36807532e+06};
                        mapping.addMapping(RealPixelA, RealPixelB, distance);
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

}
