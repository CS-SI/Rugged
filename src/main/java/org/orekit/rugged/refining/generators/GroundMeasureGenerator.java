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
package org.orekit.rugged.refining.generators;

import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.random.GaussianRandomGenerator;
import org.hipparchus.random.UncorrelatedRandomVectorGenerator;
import org.hipparchus.random.Well19937a;
import org.hipparchus.util.FastMath;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.rugged.api.Rugged;
import org.orekit.rugged.errors.RuggedException;
import org.orekit.rugged.linesensor.LineSensor;
import org.orekit.rugged.linesensor.SensorPixel;
import org.orekit.rugged.refining.measures.Noise;
import org.orekit.rugged.refining.measures.Observables;
import org.orekit.rugged.refining.measures.SensorToGroundMapping;
import org.orekit.rugged.refining.metrics.DistanceTools;
import org.orekit.time.AbsoluteDate;



/** Ground measures generator (sensor to ground mapping).
 * @author Jonathan Guinet
 * @autor Lucie Labat-Allee
 */
public class GroundMeasureGenerator implements Measurable {


    /** mapping */
    private SensorToGroundMapping groundMapping;

    /** observables which contains ground mapping.*/
    private Observables observables;

    private Rugged rugged;

    private LineSensor sensor;

    private int measureCount;

    private int dimension;

    /** Simple constructor.
     * <p>
     *
     * </p>
     */
    public GroundMeasureGenerator(final Rugged rugged, final String sensorName, final int dimension) throws RuggedException
    {

        // generate reference mapping
        groundMapping = new SensorToGroundMapping(rugged.getName(), sensorName);

        //create observables for one model
        this.observables = new Observables(1);

        this.rugged = rugged;
        sensor = rugged.getLineSensor(groundMapping.getSensorName());
        measureCount = 0;
        this.dimension = dimension;

    }

    public SensorToGroundMapping getGroundMapping() {
        return groundMapping;
    }


    /**  getter for observables.
     * @return the observables
     */
    public Observables getObservables() {
        return observables;
    }

    @Override
    public int  getMeasureCount() {
        return measureCount;
    }

    @Override
    public void createMeasure(final int lineSampling,final int pixelSampling)  throws RuggedException
    {
        for (double line = 0; line < dimension; line += lineSampling) {

            final AbsoluteDate date = sensor.getDate(line);
            for (int pixel = 0; pixel < sensor.getNbPixels(); pixel += pixelSampling) {

                final GeodeticPoint gp2 = rugged.directLocation(date, sensor.getPosition(),
                                                                sensor.getLOS(date, pixel));

                groundMapping.addMapping(new SensorPixel(line, pixel), gp2);
                measureCount++;
            }
        }
        this.observables.addGroundMapping(groundMapping);
    }
    @Override
    public void createNoisyMeasure(final int lineSampling,final int pixelSampling, final Noise noise)  throws RuggedException
    {
        /* Estimate latitude and longitude errors estimation */
        final Vector3D latLongError = estimateLatLongError();

        /* Get noise features */
        final double[] mean = noise.getMean(); /* [latitude, longitude, altitude] mean */
        final double[] std = noise.getStandardDeviation(); /* [latitude, longitude, altitude] standard deviation */

        final double latErrorMean = mean[0] * latLongError.getX(); // in line: -0.000002 deg
        final double lonErrorMean = mean[1] * latLongError.getY(); // in line: 0.000012 deg
        final double latErrorStd = std[0] * latLongError.getX(); // in line: -0.000002 deg
        final double lonErrorStd = std[1] * latLongError.getY(); // in line: 0.000012 deg

        // Gaussian random generator
        // Build a null mean random uncorrelated vector generator with standard deviation corresponding to the estimated error on ground
        final double meanGenerator[] =  {latErrorMean, lonErrorMean, mean[2]};
        final double stdGenerator[] = {latErrorStd, lonErrorStd, std[2]};
        System.out.format("Corresponding error estimation on ground {Latitude, Longitude, Altitude}:%n");
        System.out.format("\tMean: {%1.10f rad, %1.10f rad, %1.10f m} %n", meanGenerator[0], meanGenerator[1], meanGenerator[2]);
        System.out.format("\tStd : {%1.10f rad, %1.10f rad, %1.10f m} %n", stdGenerator[0], stdGenerator[1], stdGenerator[2]);

        final GaussianRandomGenerator rng = new GaussianRandomGenerator(new Well19937a(0xefac03d9be4d24b9l));
        final UncorrelatedRandomVectorGenerator rvg = new UncorrelatedRandomVectorGenerator(meanGenerator, stdGenerator, rng);

        System.out.format("Add a gaussian noise to measures without biais (null mean) and standard deviation%n corresponding to the estimated error on ground.%n");
        for (double line = 0; line < dimension; line += lineSampling) {

            final AbsoluteDate date = sensor.getDate(line);
            for (int pixel = 0; pixel < sensor.getNbPixels(); pixel += pixelSampling) {

                // Components of generated vector follow (independent) Gaussian distribution
                final Vector3D vecRandom = new Vector3D(rvg.nextVector());

                final GeodeticPoint gp2 = rugged.directLocation(date, sensor.getPosition(),
                                                                sensor.getLOS(date, pixel));


                final GeodeticPoint gpNoisy = new GeodeticPoint(gp2.getLatitude()+vecRandom.getX(),
                                                                gp2.getLongitude()+vecRandom.getY(),
                                                                gp2.getAltitude()+vecRandom.getZ());

                //if(line == 0) {
                //	System.out.format("Init  gp: (%f,%d): %s %n",line,pixel,gp2.toString());
                //	System.out.format("Random:   (%f,%d): %s %n",line,pixel,vecRandom.toString());
                //	System.out.format("Final gp: (%f,%d): %s %n",line,pixel,gpNoisy.toString());
                //}

                groundMapping.addMapping(new SensorPixel(line, pixel), gpNoisy);
                measureCount++;
            }
        }
        this.observables.addGroundMapping(groundMapping);
    }
    private Vector3D estimateLatLongError() throws RuggedException {

        System.out.format("Uncertainty in pixel (in line) for a real geometric refining: 1 pixel (assumption)%n");
        final int pix =sensor.getNbPixels()/2;
        final int line= (int) FastMath.floor(pix); // assumption : same number of line and pixels;
        System.out.format("Pixel size estimated at position  pix: %d line: %d %n", pix, line);
        final AbsoluteDate date = sensor.getDate(line);
        final GeodeticPoint gp_pix0 = rugged.directLocation(date, sensor.getPosition(), sensor.getLOS(date, pix));
        final AbsoluteDate date1 = sensor.getDate(line+1);
        final GeodeticPoint gp_pix1 = rugged.directLocation(date1, sensor.getPosition(), sensor.getLOS(date1, pix+1));
        final double latErr = FastMath.abs(gp_pix0.getLatitude() - gp_pix1.getLatitude());
        final double lonErr = FastMath.abs(gp_pix0.getLongitude() - gp_pix1.getLongitude());
        //double dist = FastMath.sqrt(lonErr*lonErr + latErr*latErr)/FastMath.sqrt(2);
        final double distanceX =  DistanceTools.computeDistanceInMeter(gp_pix0.getLongitude(), gp_pix0.getLatitude(),gp_pix1.getLongitude(), gp_pix0.getLatitude());
        final double distanceY =  DistanceTools.computeDistanceInMeter(gp_pix0.getLongitude(), gp_pix0.getLatitude(),gp_pix0.getLongitude(), gp_pix1.getLatitude());

        System.out.format("Estimated distance: X %3.3f Y %3.3f %n",distanceX, distanceY);

        //System.out.format(" lat  : %1.10f %1.10f %n",  latErr, lonErr);
        return new Vector3D(latErr, lonErr, 0.0);
    }
}

