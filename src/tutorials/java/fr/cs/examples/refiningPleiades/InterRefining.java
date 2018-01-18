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
package fr.cs.examples.refiningPleiades;

import java.io.File;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;

import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.util.FastMath;
import org.orekit.bodies.BodyShape;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.data.DataProvidersManager;
import org.orekit.data.DirectoryCrawler;
import org.orekit.errors.OrekitException;
import org.orekit.orbits.Orbit;
import org.orekit.rugged.api.AlgorithmId;
import org.orekit.rugged.api.BodyRotatingFrameId;
import org.orekit.rugged.api.EllipsoidId;
import org.orekit.rugged.api.InertialFrameId;
import org.orekit.rugged.api.Rugged;
import org.orekit.rugged.api.RuggedBuilder;
import org.orekit.rugged.errors.RuggedException;
import org.orekit.rugged.linesensor.LineSensor;
import org.orekit.rugged.adjustment.measurements.SensorToGroundMapping;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.AngularDerivativesFilter;
import org.orekit.utils.CartesianDerivativesFilter;
import org.orekit.utils.Constants;
import org.orekit.utils.PVCoordinates;
import org.orekit.utils.TimeStampedAngularCoordinates;
import org.orekit.utils.TimeStampedPVCoordinates;

import fr.cs.examples.refiningPleiades.generators.GroundMeasurementGenerator;
import fr.cs.examples.refiningPleiades.generators.InterMeasurementGenerator;
import fr.cs.examples.refiningPleiades.generators.Noise;
import fr.cs.examples.refiningPleiades.metrics.DistanceTools;
import fr.cs.examples.refiningPleiades.models.OrbitModel;
import fr.cs.examples.refiningPleiades.models.PleiadesViewingModel;

/**
 * Class for testing refining (liaison points study)
 * with or without noisy measurements
 * @author Jonathan Guinet
 * @author Lucie Labat-Allee
 * @author Guylaine Prat
 * @see SensorToGroundMapping
 * @see GroundMeasurementGenerator
 * @since 2.0
 */
public class InterRefining extends Refining {

	/** Pleiades viewing model A */
	private static PleiadesViewingModel pleiadesViewingModelA;

	/** Pleiades viewing model B */
	private static PleiadesViewingModel pleiadesViewingModelB;

	/** Sensor name A corresponding to viewing model A */
	private static String sensorNameA;

	/** Sensor name A corresponding to viewing model B */
	private static String sensorNameB;


	/** Main function
	 */
	public static void main(String[] args) {
		
		try {
			// Initialize Orekit, assuming an orekit-data folder is in user home directory
			// ---------------------------------------------------------------------------
			File home       = new File(System.getProperty("user.home"));
			File orekitData = new File(home, "orekit-data");
			DataProvidersManager.getInstance().addProvider(new DirectoryCrawler(orekitData));

			// Initialize refining context
			// ---------------------------
			InterRefining refining = new InterRefining();

			// Sensors's definition: creation of 2 Pleiades viewing models
            // -----------------------------------------------------------
			
            System.out.format("**** Build Pleiades viewing model A and its orbit definition **** %n");

            // 1/- Create First Pleiades Viewing Model
            final AbsoluteDate minDateA =  pleiadesViewingModelA.getMinDate();
            final AbsoluteDate maxDateA =  pleiadesViewingModelA.getMaxDate();
            final AbsoluteDate refDateA = pleiadesViewingModelA.getDatationReference();
            LineSensor lineSensorA =  pleiadesViewingModelA.getLineSensor();
            System.out.format(Locale.US, "Viewing model A - date min: %s max: %s ref: %s %n", minDateA, maxDateA, refDateA);

            // ----Satellite position, velocity and attitude: create orbit model A
            OrbitModel orbitmodelA = new OrbitModel();
            BodyShape earthA = orbitmodelA.createEarth();
            Orbit orbitA = orbitmodelA.createOrbit(Constants.EIGEN5C_EARTH_MU, refDateA);

            // ----If no LOF Transform Attitude Provider is Nadir Pointing Yaw Compensation
            final double [] rollPoly = {0.0,0.0,0.0};
            double[] pitchPoly = {0.025,0.0};
            double[] yawPoly = {0.0,0.0,0.0};
            orbitmodelA.setLOFTransform(rollPoly, pitchPoly, yawPoly, minDateA);

            // ----Satellite attitude
            List<TimeStampedAngularCoordinates> satelliteQListA = orbitmodelA.orbitToQ(orbitA, earthA, minDateA.shiftedBy(-0.0), maxDateA.shiftedBy(+0.0), 0.25);
            int nbQPoints = 2;

            // ----Position and velocities
            PVCoordinates PVA = orbitA.getPVCoordinates(earthA.getBodyFrame());
            List<TimeStampedPVCoordinates> satellitePVListA = orbitmodelA.orbitToPV(orbitA, earthA, minDateA.shiftedBy(-0.0), maxDateA.shiftedBy(+0.0), 0.25);
            int nbPVPoints = 8;

            // ----Convert frame and coordinates type in one call
            GeodeticPoint gpA = earthA.transform(PVA.getPosition(), earthA.getBodyFrame(), orbitA.getDate());

            System.out.format(Locale.US, "(A) Geodetic Point at date %s : φ = %8.10f °, λ = %8.10f %n",
                              orbitA.getDate().toString(),
                              FastMath.toDegrees(gpA.getLatitude()),
                              FastMath.toDegrees(gpA.getLongitude()));

            // Rugged A initialization
            // ---------------------
            System.out.format("**** Rugged A initialization **** %n");
            RuggedBuilder ruggedBuilderA = new RuggedBuilder();

            ruggedBuilderA.addLineSensor(lineSensorA);
            ruggedBuilderA.setAlgorithm(AlgorithmId.IGNORE_DEM_USE_ELLIPSOID);
            ruggedBuilderA.setEllipsoid(EllipsoidId.WGS84, BodyRotatingFrameId.ITRF);
            ruggedBuilderA.setTimeSpan(minDateA,maxDateA, 0.001, 5.0);
            ruggedBuilderA.setTrajectory(InertialFrameId.EME2000, satellitePVListA, nbPVPoints,
            		                     CartesianDerivativesFilter.USE_PV, satelliteQListA,
            		                     nbQPoints, AngularDerivativesFilter.USE_R);
            ruggedBuilderA.setLightTimeCorrection(false);
            ruggedBuilderA.setAberrationOfLightCorrection(false);

            ruggedBuilderA.setName("RuggedA");

            Rugged ruggedA = ruggedBuilderA.build();


            System.out.format("\n**** Build Pleiades viewing model B and its orbit definition **** %n");

            // 2/- Create Second Pleiades Viewing Model
            final AbsoluteDate minDateB =  pleiadesViewingModelB.getMinDate();
            final AbsoluteDate maxDateB =  pleiadesViewingModelB.getMaxDate();
            final AbsoluteDate refDateB = pleiadesViewingModelB.getDatationReference();
            LineSensor lineSensorB =  pleiadesViewingModelB.getLineSensor();
            System.out.format(Locale.US, "Viewing model B - date min: %s max: %s ref: %s  %n", minDateB, maxDateB, refDateB);

            // ----Satellite position, velocity and attitude: create orbit model B
            OrbitModel orbitmodelB =  new OrbitModel();
            BodyShape earthB = orbitmodelB.createEarth();
            Orbit orbitB = orbitmodelB.createOrbit(Constants.EIGEN5C_EARTH_MU, refDateB);

            // ----Satellite attitude
            List<TimeStampedAngularCoordinates> satelliteQListB = orbitmodelB.orbitToQ(orbitB, earthB, minDateB.shiftedBy(-0.0), maxDateB.shiftedBy(+0.0), 0.25);

            // ----Position and velocities
            PVCoordinates PVB = orbitB.getPVCoordinates(earthB.getBodyFrame());
            List<TimeStampedPVCoordinates> satellitePVListB = orbitmodelB.orbitToPV(orbitB, earthB, minDateB.shiftedBy(-0.0), maxDateB.shiftedBy(+0.0), 0.25);
            GeodeticPoint gpB = earthB.transform(PVB.getPosition(), earthB.getBodyFrame(), orbitB.getDate());

            System.out.format(Locale.US, "(B) Geodetic Point at date %s : φ = %8.10f °, λ = %8.10f %n",orbitA.getDate().toString(),
                              FastMath.toDegrees(gpB.getLatitude()),
                              FastMath.toDegrees(gpB.getLongitude()));


            // Rugged B initialization
            // ---------------------
            System.out.format("**** Rugged B initialization **** %n");
            RuggedBuilder ruggedBuilderB = new RuggedBuilder();

            ruggedBuilderB.addLineSensor(lineSensorB);
            ruggedBuilderB.setAlgorithm(AlgorithmId.IGNORE_DEM_USE_ELLIPSOID);
            ruggedBuilderB.setEllipsoid(EllipsoidId.WGS84, BodyRotatingFrameId.ITRF);
            ruggedBuilderB.setTimeSpan(minDateB,maxDateB, 0.001, 5.0);
            ruggedBuilderB.setTrajectory(InertialFrameId.EME2000, satellitePVListB, nbPVPoints,
            		                     CartesianDerivativesFilter.USE_PV, satelliteQListB,
            		                     nbQPoints, AngularDerivativesFilter.USE_R);
            ruggedBuilderB.setLightTimeCorrection(false);
            ruggedBuilderB.setAberrationOfLightCorrection(false);

            ruggedBuilderB.setName("RuggedB");

            Rugged ruggedB = ruggedBuilderB.build();

            // Compute distance between LOS
            // -----------------------------
            double distance = refining.computeDistanceBetweenLOS(ruggedA, lineSensorA, ruggedB, lineSensorB);
            System.out.format("distance %f meters %n",distance);


            // Initialize disruptions:
            // -----------------------

            // Introduce rotations around instrument axes (roll and pitch translations, scale factor)
            System.out.format("\n**** Add disruptions on both acquisitions (A,B): roll and pitch rotations, scale factor **** %n");
            double rollValueA =  FastMath.toRadians(0.004);
            double pitchValueA = FastMath.toRadians(0.0008);
            double rollValueB =  FastMath.toRadians(-0.004);
            double pitchValueB = FastMath.toRadians(0.0008);
            double factorValue = 1.0;
            System.out.format("Acquisition A roll: %3.5f \tpitch: %3.5f \tfactor: %3.5f \n",rollValueA,pitchValueA,factorValue);
            System.out.format("Acquisition B roll: %3.5f \tpitch: %3.5f \tfactor: %3.5f \n",rollValueB,pitchValueB,factorValue);

            // Apply disruptions on physical model (acquisition A)
            refining.applyDisruptions(ruggedA, sensorNameA, rollValueA, pitchValueA, factorValue);

            // Apply disruptions on physical model (acquisition B)
            refining.applyDisruptions(ruggedB, sensorNameB, rollValueB, pitchValueB, factorValue);


            // Generate measurements (observations) from physical model disrupted
            // ------------------------------------------------------------------
            int lineSampling = 1000;
            int pixelSampling = 1000;

            // Noise definition
            // distribution: gaussian(0), vector dimension: 2
            final Noise noise = new Noise(0,2);
            // {pixelA mean, pixelB mean}
            final double[] mean = {5.0,0.0};
            // {pixelB std, pixelB std}
            final double[] standardDeviation = {0.1,0.1};

            noise.setMean(mean);
            noise.setStandardDeviation(standardDeviation);

            System.out.format("\tSensor A mean: %f std %f %n",mean[0],standardDeviation[0]);
            System.out.format("\tSensor B mean: %f std %f %n",mean[1],standardDeviation[1]);

            // Inter-measurements (between both viewing models)
            InterMeasurementGenerator measurements = refining.generateNoisyPoints(lineSampling, pixelSampling,
                                                                          ruggedA, sensorNameA,
                                                                          pleiadesViewingModelA.getDimension(),
                                                                          ruggedB, sensorNameB,
                                                                          pleiadesViewingModelB.getDimension(),
                                                                          noise);

            // Compute ground truth residuals
            // ------------------------------
            System.out.format("\n**** Ground truth residuals **** %n");
            refining.computeMetrics(measurements.getInterMapping(), ruggedA, ruggedB, false);

            // Initialize physical models without disruptions
            // -----------------------------------------------
            System.out.format("\n**** Initialize physical models (A,B) without disruptions: reset Roll/Pitch/Factor **** %n");
            refining.resetModel(ruggedA, sensorNameA, false);
            refining.resetModel(ruggedB, sensorNameB, false);

            // Compute initial residuals
            // -------------------------
            System.out.format("\n**** Initial Residuals  **** %n");
            refining.computeMetrics(measurements.getInterMapping(), ruggedA, ruggedB, false);

            // Start optimization
            // ------------------
            System.out.format("\n**** Start optimization  **** %n");
            final int maxIterations = 100;
            final double convergenceThreshold = 1.e-7;
            List<Rugged> ruggedList = Arrays.asList(ruggedA, ruggedB);
            
            refining.optimization(maxIterations, convergenceThreshold,
                                  measurements.getObservables(), ruggedList);

            // Check estimated values
            // ----------------------
            System.out.format("\n**** Check parameters ajustement **** %n");
            System.out.format("Acquisition A:%n");
            refining.paramsEstimation(ruggedA, sensorNameA, rollValueA, pitchValueA, factorValue);
            System.out.format("Acquisition B:%n");
            refining.paramsEstimation(ruggedB, sensorNameB, rollValueB, pitchValueB, factorValue);

            // Compute statistics
            // ------------------
            System.out.format("\n**** Compute Statistics **** %n");
            refining.computeMetrics(measurements.getInterMapping(), ruggedA, ruggedB, false);
            

        } catch (OrekitException oe) {
            System.err.println(oe.getLocalizedMessage());
            System.exit(1);
        } catch (RuggedException re) {
            System.err.println(re.getLocalizedMessage());
            System.exit(1);
        }
    }

	/** Constructor */
	public InterRefining() throws RuggedException, OrekitException {

		sensorNameA = "SensorA";
		final double incidenceAngleA = -5.0;
		final String dateA = "2016-01-01T11:59:50.0";

		pleiadesViewingModelA = new PleiadesViewingModel(sensorNameA, incidenceAngleA, dateA);

		sensorNameB = "SensorB";
		final double incidenceAngleB = 0.0;
		final String dateB = "2016-01-01T12:02:50.0";

		pleiadesViewingModelB = new PleiadesViewingModel(sensorNameB, incidenceAngleB, dateB);

	}

    /** Estimate distance between LOS
     * @param lineSensorA line sensor A
     * @param lineSensorB line sensor B
     * @return GSD
     */
    private double computeDistanceBetweenLOS(final Rugged ruggedA, final LineSensor lineSensorA, 
                                             final Rugged ruggedB, final LineSensor lineSensorB) throws RuggedException {

    	// Get number of line of sensors
    	int dimensionA = pleiadesViewingModelA.getDimension();
    	int dimensionB = pleiadesViewingModelB.getDimension();

    	Vector3D positionA = lineSensorA.getPosition();
    	// This returns a zero vector since we set the relative position of the sensor w.r.T the satellite to 0.

        AbsoluteDate lineDateA = lineSensorA.getDate(dimensionA/2);
        Vector3D losA = lineSensorA.getLOS(lineDateA,dimensionA/2);
        GeodeticPoint centerPointA = ruggedA.directLocation(lineDateA, positionA, losA);
        System.out.format(Locale.US, "\ncenter geodetic position A : φ = %8.10f °, λ = %8.10f °, h = %8.3f m%n",
                          FastMath.toDegrees(centerPointA.getLatitude()),
                          FastMath.toDegrees(centerPointA.getLongitude()),centerPointA.getAltitude());

        Vector3D positionB = lineSensorB.getPosition();
        // This returns a zero vector since we set the relative position of the sensor w.r.T the satellite to 0.

        AbsoluteDate lineDateB = lineSensorB.getDate(dimensionB/2);
        Vector3D losB = lineSensorB.getLOS(lineDateB,dimensionB/2);
        GeodeticPoint centerPointB = ruggedB.directLocation(lineDateB, positionB, losB);
        System.out.format(Locale.US, "center geodetic position B : φ = %8.10f °, λ = %8.10f °, h = %8.3f m%n",
                          FastMath.toDegrees(centerPointB.getLatitude()),
                          FastMath.toDegrees(centerPointB.getLongitude()),centerPointB.getAltitude());

        lineDateB = lineSensorB.getDate(0);
        losB = lineSensorB.getLOS(lineDateB,0);
        GeodeticPoint firstPointB = ruggedB.directLocation(lineDateB, positionB, losB);
        System.out.format(Locale.US, "first geodetic position B : φ = %8.10f °, λ = %8.10f °, h = %8.3f m%n",
                          FastMath.toDegrees(firstPointB.getLatitude()),
                          FastMath.toDegrees(firstPointB.getLongitude()),firstPointB.getAltitude());

        lineDateB = lineSensorB.getDate(dimensionB-1);
        losB = lineSensorB.getLOS(lineDateB,dimensionB-1);
        GeodeticPoint lastPointB = ruggedB.directLocation(lineDateB, positionB, losB);
        System.out.format(Locale.US, "last geodetic position  B : φ = %8.10f °, λ = %8.10f °, h = %8.3f m%n",
                          FastMath.toDegrees(lastPointB.getLatitude()),
                          FastMath.toDegrees(lastPointB.getLongitude()),lastPointB.getAltitude());

        double distance = DistanceTools.computeDistanceInMeter(centerPointA, centerPointB);

        return distance;
    }
}
