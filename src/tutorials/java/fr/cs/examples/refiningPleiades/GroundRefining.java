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
package fr.cs.examples.refiningPleiades;

import java.io.File;
import java.util.List;
import java.util.Locale;

import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.util.FastMath;
import org.orekit.bodies.BodyShape;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.data.DataContext;
import org.orekit.data.DirectoryCrawler;
import org.orekit.errors.OrekitException;
import org.orekit.forces.gravity.potential.NormalizedSphericalHarmonicsProvider;
import org.orekit.orbits.Orbit;
import org.orekit.rugged.adjustment.measurements.SensorToGroundMapping;
import org.orekit.rugged.api.AlgorithmId;
import org.orekit.rugged.api.BodyRotatingFrameId;
import org.orekit.rugged.api.EllipsoidId;
import org.orekit.rugged.api.InertialFrameId;
import org.orekit.rugged.api.Rugged;
import org.orekit.rugged.api.RuggedBuilder;
import org.orekit.rugged.errors.RuggedException;
import org.orekit.rugged.linesensor.LineSensor;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.AngularDerivativesFilter;
import org.orekit.utils.CartesianDerivativesFilter;
import org.orekit.utils.PVCoordinates;
import org.orekit.utils.TimeStampedAngularCoordinates;
import org.orekit.utils.TimeStampedPVCoordinates;

import fr.cs.examples.refiningPleiades.generators.GroundMeasurementGenerator;
import fr.cs.examples.refiningPleiades.generators.Noise;
import fr.cs.examples.refiningPleiades.metrics.DistanceTools;
import fr.cs.examples.refiningPleiades.models.OrbitModel;
import fr.cs.examples.refiningPleiades.models.PleiadesViewingModel;

/**
 * Class for testing refining (Ground Control Points GCP study)
 * with or without noisy measurements
 * @author Jonathan Guinet
 * @author Lucie Labat-Allee
 * @author Guylaine Prat
 * @see SensorToGroundMapping
 * @see GroundMeasurementGenerator
 * @since 2.0
 */
public class GroundRefining extends Refining {

    /** Pleiades viewing model */
    private static PleiadesViewingModel pleiadesViewingModel;

    /** Sensor name */
    private static String sensorName;


    /** Main function
     */
    public static void main(String[] args) {
    	
    	try {
            // Initialize Orekit, assuming an orekit-data folder is in user home directory
            // ---------------------------------------------------------------------------
            File home       = new File(System.getProperty("user.home"));
            File orekitData = new File(home, "orekit-data");
            DataContext.getDefault().getDataProvidersManager().addProvider(new DirectoryCrawler(orekitData));

            // Initialize refining context
            // ---------------------------
            GroundRefining refining = new GroundRefining();

            // Sensor's definition: create Pleiades viewing model
            // --------------------------------------------------
            System.out.format("**** Build Pleiades viewing model and orbit definition **** %n");
            AbsoluteDate minDate =  pleiadesViewingModel.getMinDate();
            AbsoluteDate maxDate =  pleiadesViewingModel.getMaxDate();
            AbsoluteDate refDate = pleiadesViewingModel.getDatationReference();
            LineSensor lineSensor =  pleiadesViewingModel.getLineSensor();

            // Satellite position, velocity and attitude: create an orbit model
            // ----------------------------------------------------------------
            OrbitModel orbitmodel = new OrbitModel();
            BodyShape earth = orbitmodel.createEarth();
            NormalizedSphericalHarmonicsProvider gravityField = orbitmodel.createGravityField();
            Orbit orbit = orbitmodel.createOrbit(gravityField.getMu(), refDate);

            // Nadir's pointing
            final double [] rollPoly = {0.0,0.0,0.0};
            double[] pitchPoly = {0.0,0.0};
            double[] yawPoly = {0.0,0.0,0.0};
            orbitmodel.setLOFTransform(rollPoly, pitchPoly, yawPoly, minDate);

            // Satellite attitude
            List<TimeStampedAngularCoordinates> satelliteQList =
            		orbitmodel.orbitToQ(orbit, earth, minDate.shiftedBy(-0.0), maxDate.shiftedBy(+0.0), 0.25);
            int nbQPoints = 2;

            // Position and velocities
            PVCoordinates PV = orbit.getPVCoordinates(earth.getBodyFrame());
            List<TimeStampedPVCoordinates> satellitePVList =
            		orbitmodel.orbitToPV(orbit, earth, minDate.shiftedBy(-0.0), maxDate.shiftedBy(+0.0), 0.25);
            int nbPVPoints = 8;

            // Convert frame and coordinates type in one call
            GeodeticPoint gp = earth.transform(PV.getPosition(), earth.getBodyFrame(), orbit.getDate());

            System.out.format(Locale.US, "Geodetic Point at date %s : φ = %8.10f °, λ = %8.10f %n",
                              orbit.getDate().toString(),
                              FastMath.toDegrees(gp.getLatitude()),
                              FastMath.toDegrees(gp.getLongitude()));

            // Rugged initialization
            // ---------------------
            System.out.format("\n**** Rugged initialization **** %n");
            RuggedBuilder ruggedBuilder = new RuggedBuilder();

            ruggedBuilder.addLineSensor(lineSensor);
            ruggedBuilder.setAlgorithm(AlgorithmId.IGNORE_DEM_USE_ELLIPSOID);
            ruggedBuilder.setEllipsoid(EllipsoidId.WGS84, BodyRotatingFrameId.ITRF);
            ruggedBuilder.setTimeSpan(minDate,maxDate, 0.001, 5.0);
            ruggedBuilder.setTrajectory(InertialFrameId.EME2000, satellitePVList,nbPVPoints,
            		                    CartesianDerivativesFilter.USE_PV, satelliteQList,
            		                    nbQPoints, AngularDerivativesFilter.USE_R);

            ruggedBuilder.setName("Rugged_refining");

            Rugged rugged = ruggedBuilder.build();

            
            // Compute ground sample distance (GSD)
            // ------------------------------------
            double [] gsd = refining.computeGSD(rugged, lineSensor);
            System.out.format("GSD - X: %2.2f Y: %2.2f **** %n", gsd[0], gsd[1]);

            // Initialize disruptions:
            // -----------------------
            // Introduce rotations around instrument axes (roll and pitch translations, scale factor)
            System.out.format("\n**** Add disruptions: roll and pitch rotations, scale factor **** %n");
            double rollValue =  FastMath.toRadians(-0.01);
            double pitchValue = FastMath.toRadians(0.02);
            double factorValue = 1.05;
            System.out.format("roll: %3.5f \tpitch: %3.5f \tfactor: %3.5f \n",rollValue, pitchValue, factorValue);

            // Apply disruptions on physical model
            refining.applyDisruptions(rugged, sensorName, rollValue, pitchValue, factorValue);

            // Generate measurements (observations) from physical model disrupted
            // ------------------------------------------------------------------
            int lineSampling = 1000;
            int pixelSampling = 1000;

            // Noise definition
            // distribution: gaussian(0), vector dimension:3
            final Noise noise = new Noise(0,3);
            
            // lat mean, long mean, alt mean
            final double[] mean = {0.0,0.0,0.0};
            // lat std, long std, alt std}
            final double[] standardDeviation = {0.0,0.0,0.0};
            
            noise.setMean(mean);
            noise.setStandardDeviation(standardDeviation);

            // Ground measurements
            GroundMeasurementGenerator measurements = refining.generateNoisyPoints(lineSampling, pixelSampling,
                                                                                   rugged, sensorName,
                                                                                   pleiadesViewingModel.getDimension(),
                                                                                   noise);

            // Compute ground truth residuals
            // ------------------------------
            System.out.format("\n**** Ground truth residuals **** %n");
            refining.computeMetrics(measurements.getGroundMapping(), rugged, false);

            // Initialize physical model without disruptions
            // ---------------------------------------------
            System.out.format("\n**** Initialize physical model without disruptions: reset Roll/Pitch/Factor **** %n");
            refining.resetModel(rugged, sensorName, true);

            // Compute initial residuals
            // -------------------------
            System.out.format("\n**** Initial Residuals  **** %n");
            refining.computeMetrics(measurements.getGroundMapping(), rugged, false);

            // Start optimization
            // ------------------
            System.out.format("\n**** Start optimization  **** %n");

            int maxIterations = 100;
            double convergenceThreshold =  1.e-11;

            refining.optimization(maxIterations, convergenceThreshold, measurements.getObservables(), rugged);

            // Check estimated values
            // ----------------------
            System.out.format("\n**** Check parameters ajustement **** %n");
            refining.paramsEstimation(rugged, sensorName, rollValue, pitchValue, factorValue);

            // Compute statistics
            // ------------------
            System.out.format("\n**** Compute Statistics **** %n");

            // Residuals computed in meters
            refining.computeMetrics(measurements.getGroundMapping(), rugged, false);

            // Residuals computed in degrees
            refining.computeMetrics(measurements.getGroundMapping(), rugged, true);

        } catch (OrekitException oe) {
            System.err.println(oe.getLocalizedMessage());
            System.exit(1);
        } catch (RuggedException re) {
            System.err.println(re.getLocalizedMessage());
            System.exit(1);
        }
    }

    /** Constructor */
    private GroundRefining() {

        sensorName = "line";
        pleiadesViewingModel = new PleiadesViewingModel(sensorName);

    }

     /** Estimate ground sample distance (GSD)
     * @param LineSensor line sensor
     * @return the GSD
     */
    private double[] computeGSD(final Rugged rugged, final LineSensor lineSensor) {

    	// Get number of line
    	int dimension = pleiadesViewingModel.getDimension();
    	
        // Get position
        Vector3D position = lineSensor.getPosition(); // This returns a zero vector since we set the relative position of the sensor w.r.T the satellite to 0.

        // Get upper left geodetic point
        AbsoluteDate firstLineDate = lineSensor.getDate(0);
        Vector3D los = lineSensor.getLOS(firstLineDate,0);
        GeodeticPoint upLeftPoint = rugged.directLocation(firstLineDate, position, los);
        los = lineSensor.getLOS(firstLineDate,dimension-1);

        // Get center geodetic point
        AbsoluteDate lineDate = lineSensor.getDate(dimension/2);
        los = lineSensor.getLOS(lineDate,dimension/2);

        // Get upper right geodetic point
        int pixelPosition = dimension-1;
        los = lineSensor.getLOS(firstLineDate,pixelPosition);
        GeodeticPoint upperRight = rugged.directLocation(firstLineDate, position, los);

        // Get lower left geodetic point
        AbsoluteDate lineDate_y = lineSensor.getDate(dimension-1);
        los = lineSensor.getLOS(lineDate_y,0);
        GeodeticPoint lowerLeft = rugged.directLocation(lineDate_y, position, los);

        double gsdX = DistanceTools.computeDistanceInMeter(upLeftPoint, upperRight)/dimension;
        double gsdY = DistanceTools.computeDistanceInMeter(upLeftPoint, lowerLeft)/dimension;

        double [] gsd = {gsdX, gsdY};
        return gsd;
    }

//    /**
//     * Get the Pleiades viewing model
//     * @return the Pleiades viewing model
//     */
//    public PleiadesViewingModel getPleiadesViewingModel() {
//        return pleiadesViewingModel;
//    }
//
//    /**
//     * Set the Pleiades viewing model
//     * @param pleiadesViewingModel Pleiades viewing model to set
//     */
//    public void setPleiadesViewingModel(final PleiadesViewingModel pleiadesViewingModel) {
//        this.pleiadesViewingModel = pleiadesViewingModel;
//    }
//
//    /**
//     * Get the orbit model
//     * @return the orbit model
//     */
//    public OrbitModel getOrbitmodel() {
//        return orbitmodel;
//    }
//
//    /**
//     * Set the orbit model
//     * @param orbitmodel the orbit model to set
//     */
//    public void setOrbitmodel(final OrbitModel orbitmodel) {
//        this.orbitmodel = orbitmodel;
//    }
//
//    /**
//     * Get the sensor name
//     * @return the sensor name
//     */
//    public String getSensorName() {
//        return sensorName;
//    }
//
//    /**
//     * Get the Rugged instance
//     * @return the rugged instance
//     */
//    public Rugged getRugged() {
//        return rugged;
//    }
//
//    /**
//     * Set the Rugged instance
//     * @param rugged the Rugged instance to set
//     */
//    public void setRugged(final Rugged rugged) {
//        this.rugged = rugged;
//    }
}

