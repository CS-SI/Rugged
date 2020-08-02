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

import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.random.GaussianRandomGenerator;
import org.hipparchus.random.UncorrelatedRandomVectorGenerator;
import org.hipparchus.random.Well19937a;
import org.hipparchus.util.FastMath;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.rugged.adjustment.measurements.Observables;
import org.orekit.rugged.adjustment.measurements.SensorToGroundMapping;
import org.orekit.rugged.api.Rugged;
import org.orekit.rugged.linesensor.LineSensor;
import org.orekit.rugged.linesensor.SensorPixel;
import org.orekit.time.AbsoluteDate;

/** Ground measurements generator (sensor to ground mapping).
 * @author Jonathan Guinet
 * @author Lucie Labat-Allee
 * @author Guylaine Prat
 * @since 2.0
 */
public class GroundMeasurementGenerator implements Measurable {

    /** Sensor to ground mapping. */
    private SensorToGroundMapping groundMapping;

    /** Observables which contains ground mapping. */
    private Observables observables;

    /** Rugged instance */
    private Rugged rugged;

    /** Line sensor */
    private LineSensor sensor;

    /** Number of lines of the sensor */
    private int dimension;

    /** Number of measurements */
    private int measurementCount;

    /** Simple constructor.
     * @param rugged Rugged instance
     * @param sensorName sensor name
     * @param dimension number of line of the sensor
     */
    public GroundMeasurementGenerator(final Rugged rugged, final String sensorName, final int dimension) {
    	
        // Generate reference mapping
        this.groundMapping = new SensorToGroundMapping(rugged.getName(), sensorName);

        // Create observables for one model
        this.observables = new Observables(1);

        this.rugged = rugged;
        this.sensor = rugged.getLineSensor(groundMapping.getSensorName());
        this.dimension = dimension;
        this.measurementCount = 0;
    }

    /** Get the sensor to ground mapping
     * @return the sensor to ground mapping
     */
    public SensorToGroundMapping getGroundMapping() {
        return groundMapping;
    }

    /** Get the observables which contains the ground mapping
     * @return the observables which contains the ground mapping
     */
    public Observables getObservables() {
        return observables;
    }

    @Override
    public int  getMeasurementCount() {
        return measurementCount;
    }

    @Override
    public void createMeasurement(final int lineSampling, final int pixelSampling) {
    	
        for (double line = 0; line < dimension; line += lineSampling) {

            final AbsoluteDate date = sensor.getDate(line);

            for (int pixel = 0; pixel < sensor.getNbPixels(); pixel += pixelSampling) {

                final GeodeticPoint gp2 = rugged.directLocation(date, sensor.getPosition(),
                                                                sensor.getLOS(date, pixel));

                groundMapping.addMapping(new SensorPixel(line, pixel), gp2);

                // increment the number of measurements
                measurementCount++;
            }
        }

        observables.addGroundMapping(groundMapping);
    }

    @Override
    public void createNoisyMeasurement(final int lineSampling, final int pixelSampling, final Noise noise) {
    	
        // Estimate latitude and longitude errors (rad)
        final Vector3D latLongError = estimateLatLongError();

        // Get noise features
        final double[] mean = noise.getMean(); // [latitude, longitude, altitude] mean
        final double[] std = noise.getStandardDeviation(); // [latitude, longitude, altitude] standard deviation

        final double latErrorMean = mean[0] * latLongError.getX();
        final double lonErrorMean = mean[1] * latLongError.getY();
        final double latErrorStd = std[0] * latLongError.getX();
        final double lonErrorStd = std[1] * latLongError.getY();

        // Gaussian random generator
        // Build a null mean random uncorrelated vector generator with standard deviation corresponding to the estimated error on ground
        final double meanGenerator[] =  {latErrorMean, lonErrorMean, mean[2]};
        final double stdGenerator[] = {latErrorStd, lonErrorStd, std[2]};

        // seed has been fixed for tests purpose
        final GaussianRandomGenerator rng = new GaussianRandomGenerator(new Well19937a(0xefac03d9be4d24b9l));
        final UncorrelatedRandomVectorGenerator rvg = new UncorrelatedRandomVectorGenerator(meanGenerator, stdGenerator, rng);

        for (double line = 0; line < dimension; line += lineSampling) {

            final AbsoluteDate date = sensor.getDate(line);
            for (int pixel = 0; pixel < sensor.getNbPixels(); pixel += pixelSampling) {

                // Components of generated vector follow (independent) Gaussian distribution
                final Vector3D vecRandom = new Vector3D(rvg.nextVector());

                final GeodeticPoint gp2 = rugged.directLocation(date, sensor.getPosition(),
                                                                sensor.getLOS(date, pixel));


                final GeodeticPoint gpNoisy = new GeodeticPoint(gp2.getLatitude() + vecRandom.getX(),
                                                                gp2.getLongitude() + vecRandom.getY(),
                                                                gp2.getAltitude() + vecRandom.getZ());

                groundMapping.addMapping(new SensorPixel(line, pixel), gpNoisy);

                // increment the number of measurements
                measurementCount++;
            }
        }

        this.observables.addGroundMapping(groundMapping);
    }

    /** Compute latitude and longitude errors
     * @return the latitude and longitude errors (rad)
     */
    private Vector3D estimateLatLongError() {
        final int pix = sensor.getNbPixels() / 2;
        final int line = (int) FastMath.floor(pix); // assumption : same number of line and pixels;

        final AbsoluteDate date = sensor.getDate(line);
        final GeodeticPoint gp_pix0 = rugged.directLocation(date, sensor.getPosition(), sensor.getLOS(date, pix));

        final AbsoluteDate date1 = sensor.getDate(line + 1);
        final GeodeticPoint gp_pix1 = rugged.directLocation(date1, sensor.getPosition(), sensor.getLOS(date1, pix + 1));

        final double latErr = FastMath.abs(gp_pix0.getLatitude() - gp_pix1.getLatitude());
        final double lonErr = FastMath.abs(gp_pix0.getLongitude() - gp_pix1.getLongitude());
        return new Vector3D(latErr, lonErr, 0.0);
    }
}

