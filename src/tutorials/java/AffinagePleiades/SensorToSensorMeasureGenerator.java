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
import org.hipparchus.analysis.differentiation.DerivativeStructure;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.random.UncorrelatedRandomVectorGenerator;
import org.hipparchus.random.GaussianRandomGenerator;
import org.hipparchus.random.Well19937a;
import org.hipparchus.util.FastMath;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.rugged.utils.DSGenerator;
import org.orekit.rugged.utils.SpacecraftToObservedBody;

import java.util.Locale;

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
     * @throws RuggedException
     */
    public void CreateMeasure(final int lineSampling, final int pixelSampling)
        throws RuggedException {
        // Search the sensor pixel seeing point
        final int minLine = 0;
        final int maxLine = viewingModelB.dimension - 1;

        for (double line = 0; line < viewingModelA.dimension; line += lineSampling) {

            AbsoluteDate dateA = sensorA.getDate(line);
            for (int pixelA = 0; pixelA < sensorA
                .getNbPixels(); pixelA += pixelSampling) {

                GeodeticPoint gpA = ruggedA
                    .directLocation(dateA, sensorA.getPosition(),
                                    sensorA.getLOS(dateA, pixelA));

                SensorPixel sensorPixelB = ruggedB
                    .inverseLocation(sensorNameB, gpA, minLine, maxLine);
                // we need to test if the sensor pixel is found in the
                // prescribed lines otherwise the sensor pixel is null
                if (sensorPixelB != null) {

                    /*final AbsoluteDate dateB = sensorB
                        .getDate(sensorPixelB.getLineNumber());
                    double pixelB = sensorPixelB.getPixelNumber();*/

                    // Get spacecraft to body transform of rugged instance A
                    //final SpacecraftToObservedBody scToBodyA = ruggedA
                    //    .getScToBody();
                    //final SpacecraftToObservedBody scToBodyB = ruggedB
                    //    .getScToBody();

                    /*double distanceB = ruggedB
                        .distanceBetweenLOS(sensorA, dateA, pixelA, scToBodyA,
                                            sensorB, dateB, pixelB);*/

                    final double distance = 0.0;
                    mapping.addMapping(new SensorPixel(line, pixelA),
                                       sensorPixelB, distance);
                    measureCount++;
                } 

            }
        }
    }

}
