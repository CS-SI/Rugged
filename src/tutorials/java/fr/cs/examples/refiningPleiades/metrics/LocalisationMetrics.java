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
package fr.cs.examples.refiningPleiades.metrics;

import java.util.Iterator;
import java.util.Map;
import java.util.Set;

import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.util.FastMath;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.rugged.adjustment.measurements.SensorToGroundMapping;
import org.orekit.rugged.adjustment.measurements.SensorToSensorMapping;
import org.orekit.rugged.api.Rugged;
import org.orekit.rugged.linesensor.LineSensor;
import org.orekit.rugged.linesensor.SensorPixel;
import org.orekit.rugged.utils.SpacecraftToObservedBody;
import org.orekit.time.AbsoluteDate;

/**
 * Class for testing geometric performances in absolute location.
 * Metrics are computed for two scenarios: ground control points and tie points.
 * @see SensorToSensorMapping
 * @see SensorToGroundMapping
 * @author Jonathan Guinet
 * @author Lucie Labat-Allee
 * @author Guylaine Prat
 * @since 2.0
 */
public class LocalisationMetrics {

    /** Maximum residual distance. */
    private double resMax;

    /** Mean residual distance. */
    private double resMean;

    /** LOS distance max. */
    private double losDistanceMax;

    /** LOS distance mean. */
    private double losDistanceMean;

    /** Earth distance max. */
    private double earthDistanceMax;

    /** Earth distance mean.*/
    private double earthDistanceMean;


    /** Compute metrics corresponding to the Ground Control Points (GCP) study.
     * @param measMapping Mapping of observations/measurements = the ground truth
     * @param rugged Rugged instance
     * @param computeAngular flag to know if distance is computed in meters (false) or with angular (true)
     */
    public LocalisationMetrics(final SensorToGroundMapping measMapping, final Rugged rugged, final boolean computeAngular) {
    	
        // Initialization
        this.resMax = 0.0;
        this.resMean = 0.0;

        // Compute metrics - Case of Sensor to Ground mapping (Ground Control Points GCP study)
        computeGCPmetrics(measMapping, rugged, computeAngular);
    }

    /** Compute metrics corresponding to the tie points study.
     * @param measMapping Mapping of observations/measurements = the ground truth
     * @param ruggedA Rugged instance corresponding to viewing model A
     * @param ruggedB Rugged instance corresponding to viewing model B
     * @param computeAngular flag to know if distance is computed in meters (false) or with angular (true)
     */
    public LocalisationMetrics(final SensorToSensorMapping measMapping, final Rugged ruggedA, final Rugged ruggedB,
                               final boolean computeAngular) {

        // Initialization
        this.resMax = 0.0;
        this.resMean = 0.0;
        this.losDistanceMax = 0.0;
        this.losDistanceMean = 0.0;
        this.earthDistanceMax = 0.0;
        this.earthDistanceMean = 0.0;

        // Compute metrics - Case of Sensor to Sensor mapping (tie points study)
        computeTiePointsMetrics(measMapping, ruggedA, ruggedB, computeAngular);
    }

    /**
     * Compute metrics: case of ground control points (GCP).
     * @param measMapping Mapping of observations/measurements = the ground truth
     * @param rugged Rugged instance
     * @param computeAngular flag to know if distance is computed in meters (false) or with angular (true)
     */
    public void computeGCPmetrics(final SensorToGroundMapping measMapping, final Rugged rugged, final boolean computeAngular) {

        // Mapping of observations/measurements = the ground truth
        final Set<Map.Entry<SensorPixel, GeodeticPoint>> measurementsMapping;
        final LineSensor lineSensor;

        // counter for compute mean distance
        double count = 0;

        // Initialization
        measurementsMapping = measMapping.getMapping();
        lineSensor = rugged.getLineSensor(measMapping.getSensorName());

        // number of measurements
        int nbMeas = measurementsMapping.size();

        final Iterator<Map.Entry<SensorPixel, GeodeticPoint>> gtIt = measurementsMapping.iterator();

        // Browse map of measurements
        while (gtIt.hasNext()) {

            final Map.Entry<SensorPixel, GeodeticPoint> gtMeas = gtIt.next();

            final SensorPixel gtSP = gtMeas.getKey();
            final GeodeticPoint gtGP = gtMeas.getValue();

            final AbsoluteDate date = lineSensor.getDate(gtSP.getLineNumber());

            final GeodeticPoint esGP = rugged.directLocation(date, lineSensor.getPosition(),
                                                             lineSensor.getLOS(date,
                                                             (int) gtSP.getPixelNumber()));
            // Compute distance
            double distance = DistanceTools.computeDistance(esGP, gtGP, computeAngular);
            count += distance;
            if (distance > resMax) {
                resMax = distance;
            }
        }
        // Mean of residuals
        resMean = count / nbMeas;
    }

    /**
     * Compute metrics: case of tie points.
     * @param measMapping Mapping of observations/measurements = the ground truth
     * @param ruggedA Rugged instance corresponding to viewing model A
     * @param ruggedB Rugged instance corresponding to viewing model B
     * @param computeAngular Flag to know if distance is computed in meters (false) or with angular (true)
     */
    public void computeTiePointsMetrics(final SensorToSensorMapping measMapping, final Rugged ruggedA, final Rugged ruggedB,
                                      final boolean computeAngular) {

        // Mapping of observations/measurements = the ground truth
        final Set<Map.Entry<SensorPixel, SensorPixel>> measurementsMapping;

        final LineSensor lineSensorA;   // line sensor corresponding to rugged A
        final LineSensor lineSensorB;   // line sensor corresponding to rugged B
        double count = 0;               // counter for computing remaining mean distance
        double losDistanceCount = 0;    // counter for computing LOS distance mean
        double earthDistanceCount = 0;  // counter for computing Earth distance mean
        int i = 0;                      // increment of measurements

        // Initialization
        measurementsMapping = measMapping.getMapping();
        lineSensorA = ruggedA.getLineSensor(measMapping.getSensorNameA());
        lineSensorB = ruggedB.getLineSensor(measMapping.getSensorNameB());
        int nbMeas = measurementsMapping.size(); // number of measurements

        // Browse map of measurements
        for (Iterator<Map.Entry<SensorPixel, SensorPixel>> gtIt = measurementsMapping.iterator();
                        gtIt.hasNext();
                        i++) {

            if (i == measurementsMapping.size()) {
                break;
            }

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
            final double measEarthDistance = measMapping.getBodyDistance(i).doubleValue(); // Earth measurement distance
            final double earthDistance =  FastMath.abs(estEarthDistance - measEarthDistance);

            if (earthDistance > earthDistanceMax) {
                earthDistanceMax = earthDistance;

            }
            earthDistanceCount += earthDistance;

            // Compute remaining distance
            double distance = DistanceTools.computeDistance(gpB, gpA, computeAngular);
            count += distance;
            if (distance > resMax) {
                resMax = distance;
            }
        }

        resMean = count / nbMeas;
        losDistanceMean = losDistanceCount / nbMeas;
        earthDistanceMean = earthDistanceCount / nbMeas;
    }

    /** Get the max residual.
     * @return maximum of residuals
     */
    public double getMaxResidual() {
        return resMax;
    }

    /** Get the mean of residuals.
     * @return mean of residuals
     */
    public double getMeanResidual() {
        return resMean;
    }

    /** Get the LOS maximum distance.
     * @return LOS maximum distance
     */
    public double getLosMaxDistance() {
        return losDistanceMax;
    }

    /** Get the LOS mean distance.
     * @return LOS mean distance
     */
    public double getLosMeanDistance() {
        return losDistanceMean;
    }

    /** Get the Earth distance maximum residual.
     * @return Earth distance max residual
     */
    public double getEarthMaxDistance() {
        return earthDistanceMax;
    }

    /** Get the Earth distance mean residual.
     * @return Earth distance mean residual
     */
    public double getEarthMeanDistance() {
        return earthDistanceMean;
    }
}
