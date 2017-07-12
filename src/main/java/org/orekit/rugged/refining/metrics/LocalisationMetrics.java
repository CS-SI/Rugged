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
package org.orekit.rugged.refining.metrics;

import java.util.Iterator;
import java.util.Map;
import java.util.Set;

import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.util.FastMath;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.rugged.api.Rugged;
import org.orekit.rugged.errors.RuggedException;
import org.orekit.rugged.linesensor.LineSensor;
import org.orekit.rugged.linesensor.SensorPixel;
import org.orekit.rugged.refining.measures.SensorToGroundMapping;
import org.orekit.rugged.refining.measures.SensorToSensorMapping;
import org.orekit.rugged.utils.SpacecraftToObservedBody;
import org.orekit.time.AbsoluteDate;

/**
 * class for testing geometric performances in absolute location.
 * Metrics are computed for two scenarios: ground points and liaison points
 * @author Jonathan Guinet
 * @author Lucie Labat-Allee
 * @see SensorToSensorMapping
 * @see SensorToGroundMapping
 */
public class LocalisationMetrics {

    /** Maximum residue distance. */
    private double resMax;

    /** Mean residue distance. */
    private double resMean;

    /** LOS distance max. */
    private double losDistanceMax;

    /** LOS distance mean. */
    private double losDistanceMean;

    /** Earth distance max. */
    private double earthDistanceMax;

    /** Earth distance mean.*/
    private double earthDistanceMean;


    /** Compute metrics corresponding to the ground points study.
     * <p>
     * @param measMapping Mapping of observations/measures = the ground truth
     * @param rugged Rugged instance
     * @param computeAngular Flag to known if distance is computed in meters (false) or with angular (true)
     * </p>
     */
    public LocalisationMetrics(final SensorToGroundMapping measMapping, final Rugged rugged, final boolean computeAngular)
                    throws RuggedException {
        // Initialization
        this.resMax = 0.0;
        this.resMean = 0.0;

        // Compute metrics - Case of Sensor to Ground mapping (fulcrum points study)
        computeMetrics(measMapping, rugged, computeAngular);

    }


    /** Compute metrics corresponding to the liaison points study.
     * <p>
     * @param measMapping Mapping of observations/measures = the ground truth
     * @param ruggedA Rugged instance corresponding to viewing model A
     * @param ruggedB Rugged instance corresponding to viewing model B
     * @param computeAngular Flag to known if distance is computed in meters (false) or with angular (true)
     * </p>
     */
    public LocalisationMetrics(final SensorToSensorMapping measMapping, final Rugged ruggedA, final Rugged ruggedB,
                               final boolean computeAngular)
                                               throws RuggedException {

        // Initialization
        this.resMax = 0.0;
        this.resMean = 0.0;
        this.losDistanceMax = 0.0;
        this.losDistanceMean = 0.0;
        this.earthDistanceMax = 0.0;
        this.earthDistanceMean = 0.0;

        // Compute metrics - Case of Sensor to Sensor mapping (liaison points study)
        computeLiaisonMetrics(measMapping, ruggedA, ruggedB, computeAngular);
    }


    /**
     * Compute metrics: case of ground control points.
     * <p>
     * @param measMapping Mapping of observations/measures = the ground truth
     * @param rugged Rugged instance
     * @param computeAngular Flag to known if distance is computed in meters (false) or with angular (true)
     * @exception RuggedException if directLocation fails
     * </p>
     */
    public void computeMetrics(final SensorToGroundMapping measMapping, final Rugged rugged, final boolean computeAngular)
                    throws RuggedException {

        /* Mapping of observations/measures = the ground truth */
        final Set<Map.Entry<SensorPixel, GeodeticPoint>> measuresMapping;
        final LineSensor lineSensor;
        double count = 0;   /* counter for compute mean distance */
        int nbMeas = 0;     /* number of measures */
        double distance = 0;

        /* Initialization */
        measuresMapping = measMapping.getMapping();
        lineSensor = rugged.getLineSensor(measMapping.getSensorName());
        nbMeas = measuresMapping.size();
        final Iterator<Map.Entry<SensorPixel, GeodeticPoint>> gtIt = measuresMapping.iterator();

        /* Browse map of measures */
        while (gtIt.hasNext()) {

            distance = 0;

            final Map.Entry<SensorPixel, GeodeticPoint> gtMeas = gtIt.next();

            final SensorPixel gtSP = gtMeas.getKey();
            final GeodeticPoint gtGP = gtMeas.getValue();

            final AbsoluteDate date = lineSensor.getDate(gtSP.getLineNumber());

            final GeodeticPoint esGP = rugged.directLocation(date, lineSensor.getPosition(),
                                                             lineSensor.getLOS(date,
                                                                               (int) gtSP.getPixelNumber()));
            // Compute distance
            distance = DistanceTools.computeDistance(esGP.getLongitude(), esGP.getLatitude(),
                                                     gtGP.getLongitude(), gtGP.getLatitude(),
                                                     computeAngular);
            count += distance;

            if (distance > resMax) {
                resMax = distance;
            }
        }
        // Mean of residues
        resMean = count / nbMeas;

    }


    /**
     * Compute metrics: case of liaison points.
     * <p>
     * @param measMapping Mapping of observations/measures = the ground truth
     * @param ruggedA Rugged instance corresponding to viewing model A
     * @param ruggedB Rugged instance corresponding to viewing model B
     * @param computeAngular Flag to known if distance is computed in meters (false) or with angular (true)
     * @exception RuggedException if directLocation fails
     * </p>
     */
    public void computeLiaisonMetrics(final SensorToSensorMapping measMapping, final Rugged ruggedA, final Rugged ruggedB,
                                      final boolean computeAngular)
                                                      throws RuggedException {

        /* Mapping of observations/measures = the ground truth */
        final Set<Map.Entry<SensorPixel, SensorPixel>> measuresMapping;

        final LineSensor lineSensorA;   /* line sensor corresponding to rugged A */
        final LineSensor lineSensorB;   /* line sensor corresponding to rugged B */
        double count = 0;               /* counter for computing remaining mean distance */
        double losDistanceCount = 0;    /* counter for computing LOS distance mean */
        double earthDistanceCount = 0;  /* counter for computing Earth distance mean */
        int nbMeas = 0;                 /* number of measures */
        double distance = 0;            /* remaining distance */
        int i = 0;                      /* increment of measures */


        /* Initialization */
        measuresMapping = measMapping.getMapping();
        lineSensorA = ruggedA.getLineSensor(measMapping.getSensorNameA());
        lineSensorB = ruggedB.getLineSensor(measMapping.getSensorNameB());
        nbMeas = measuresMapping.size();

        /* Browse map of measures */
        for (Iterator<Map.Entry<SensorPixel, SensorPixel>> gtIt = measuresMapping.iterator();
                        gtIt.hasNext();
                        i++) {

            if (i == measuresMapping.size()) {
                break;
            }

            distance = 0;

            final Map.Entry<SensorPixel, SensorPixel> gtMapping = gtIt.next();

            final SensorPixel spA = gtMapping.getKey();
            final SensorPixel spB = gtMapping.getValue();

            final AbsoluteDate dateA = lineSensorA.getDate(spA.getLineNumber());
            final AbsoluteDate dateB = lineSensorB.getDate(spB.getLineNumber());

            final double pixelA = spA.getPixelNumber();
            final double pixelB = spB.getPixelNumber();

            final Vector3D losA = lineSensorA.getLOS(dateA, pixelA);
            final Vector3D losB = lineSensorB.getLOS(dateB, pixelB);


            final GeodeticPoint gpA = ruggedA.directLocation(dateA, lineSensorA.getPosition(), losA);
            final GeodeticPoint gpB = ruggedB.directLocation(dateB, lineSensorB.getPosition(), losB);

            final SpacecraftToObservedBody scToBodyA = ruggedA.getScToBody();

            // Estimated distances (LOS / Earth)
            final double[] distances = ruggedB.distanceBetweenLOS(lineSensorA, dateA, pixelA, scToBodyA, lineSensorB, dateB, pixelB);

            // LOS distance control
            final double estLosDistance =  distances[0]; // LOS distance estimation
            if (estLosDistance > losDistanceMax) {
                losDistanceMax = estLosDistance;
            }
            losDistanceCount += estLosDistance;

            // Earth distance control
            final double estEarthDistance = distances[1]; // Earth distance estimation
            final double measEarthDistance = measMapping.getEarthDistance(i).doubleValue(); // Earth measured distance
            final double earthDistance =  FastMath.abs(estEarthDistance - measEarthDistance);

            if (earthDistance > earthDistanceMax) {
                earthDistanceMax = earthDistance;

            }
            earthDistanceCount += earthDistance;

            // Compute remaining distance
            distance = DistanceTools.computeDistance(gpB.getLongitude(), gpB.getLatitude(),
                                                     gpA.getLongitude(), gpA.getLatitude(),
                                                     computeAngular);
            count += distance;
            if (distance > resMax) {
                resMax = distance;
            }
        }

        resMean = count / nbMeas;
        losDistanceMean = losDistanceCount / nbMeas;
        earthDistanceMean = earthDistanceCount / nbMeas;
    }


    /** Get the Max residue.
     * @return maximum of residues
     */
    public double getMaxResidual() {
        return resMax;
    }


    /** Get the mean of residues.
     * @return mean of residues
     */
    public double getMeanResidual() {
        return resMean;
    }


    /** Get the LOS maximum residue.
     * @return max residue
     */
    public double getLosMaxDistance() {
        return losDistanceMax;
    }


    /** Get the LOS mean residue.
     * @return mean residue
     */
    public double getLosMeanDistance() {
        return losDistanceMean;
    }


    /** Get the Earth distance maximum residue.
     * @return max residue
     */
    public double getEarthMaxDistance() {
        return earthDistanceMax;
    }


    /** Get the earth distance mean residue.
     * @return mean residue
     */
    public double getEarthMeanDistance() {
        return earthDistanceMean;
    }


}
