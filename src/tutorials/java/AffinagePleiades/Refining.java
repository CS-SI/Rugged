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
package AffinagePleiades;

import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.optim.nonlinear.vector.leastsquares.LeastSquaresOptimizer.Optimum;
import org.hipparchus.util.FastMath;

import java.io.File;
import java.util.Locale;
import java.util.Collections;
import java.util.List;

import org.orekit.bodies.BodyShape;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.data.DataProvidersManager;
import org.orekit.data.DirectoryCrawler;
import org.orekit.errors.OrekitException;
import org.orekit.errors.OrekitExceptionWrapper;
import org.orekit.forces.gravity.potential.NormalizedSphericalHarmonicsProvider;
import org.orekit.orbits.Orbit;
import org.orekit.rugged.api.AlgorithmId;
import org.orekit.rugged.api.BodyRotatingFrameId;
import org.orekit.rugged.api.EllipsoidId;
import org.orekit.rugged.api.InertialFrameId;
import org.orekit.rugged.api.Rugged;
import org.orekit.rugged.api.RuggedBuilder;
import org.orekit.rugged.errors.RuggedException;
import org.orekit.rugged.linesensor.LineSensor;
import org.orekit.rugged.refining.generators.GroundMeasureGenerator;
import org.orekit.rugged.refining.measures.Noise;
import org.orekit.rugged.refining.measures.SensorToGroundMapping;
import org.orekit.rugged.refining.metrics.DistanceTools;
import org.orekit.rugged.refining.metrics.LocalisationMetrics;
import org.orekit.rugged.refining.models.OrbitModel;
import org.orekit.rugged.refining.models.PleiadesViewingModel;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.AngularDerivativesFilter;
import org.orekit.utils.CartesianDerivativesFilter;
import org.orekit.utils.PVCoordinates;
import org.orekit.utils.TimeStampedAngularCoordinates;
import org.orekit.utils.TimeStampedPVCoordinates;



/**
 * Class for testing refining (fulcrum points study)
 * with or without noisy measurements 
 * @author Jonathan Guinet
 * @author Lucie Labat-Allee
 * @see SensorToGroundMapping
 * @see GroundMeasureGenerator
 */
public class Refining {
    
    /** Pleiades viewing model */
    PleiadesViewingModel pleiadesViewingModel;
    
    /** Orbit model */
    OrbitModel orbitmodel;
    
    /** Rugged instance */
    Rugged rugged;
    
    /** Measurements */
    GroundMeasureGenerator measures;
        
    
    /**
     * Constructor
     */
    public Refining() throws RuggedException, OrekitException {
    
        pleiadesViewingModel = new PleiadesViewingModel();
        orbitmodel =  new OrbitModel();
    }
    
    /** Main function
     * @param args
     */
    public static void main(String[] args) {

        try {

            // Initialize Orekit, assuming an orekit-data folder is in user home directory
            // ---------------------------------------------------------------------------         
            File home       = new File(System.getProperty("user.home"));
            File orekitData = new File(home, "COTS/orekit-data");
            DataProvidersManager.getInstance().addProvider(new DirectoryCrawler(orekitData));
            
            
            // Initialize refining context
            // ---------------------------
            Refining refining = new Refining();
            
            
            // Sensor's definition: create Pleiades viewing model
            // --------------------------------------------------
            System.out.format("**** Build Pleiades viewing model and orbit definition **** %n");
            PleiadesViewingModel pleiadesViewingModel = refining.getPleiadesViewingModel();
            AbsoluteDate minDate =  pleiadesViewingModel.getMinDate();
            AbsoluteDate maxDate =  pleiadesViewingModel.getMaxDate();
            AbsoluteDate refDate = pleiadesViewingModel.getDatationReference();
            LineSensor lineSensor =  pleiadesViewingModel.getLineSensor(); 

            
            // Satellite position, velocity and attitude: create an orbit model
            // ----------------------------------------------------------------
            OrbitModel orbitmodel = refining.getOrbitmodel();
            BodyShape earth = orbitmodel.createEarth();
            NormalizedSphericalHarmonicsProvider gravityField = orbitmodel.createGravityField();
            Orbit orbit = orbitmodel.createOrbit(gravityField.getMu(), refDate);
            
            // Nadir's pointing
            final double [] rollPoly = {0.0,0.0,0.0};
            double[] pitchPoly = {0.0,0.0};
            double[] yawPoly = {0.0,0.0,0.0};
            orbitmodel.setLOFTransform(rollPoly, pitchPoly, yawPoly, minDate);
            
            // Satellite attitude
            List<TimeStampedAngularCoordinates> satelliteQList = orbitmodel.orbitToQ(orbit, earth, minDate.shiftedBy(-0.0), maxDate.shiftedBy(+0.0), 0.25);
            int nbQPoints = 2;
            
            // Position and velocities
            PVCoordinates PV = orbit.getPVCoordinates(earth.getBodyFrame());
            List<TimeStampedPVCoordinates> satellitePVList = orbitmodel.orbitToPV(orbit, earth, minDate.shiftedBy(-0.0), maxDate.shiftedBy(+0.0), 0.25);
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
            ruggedBuilder.setTimeSpan(minDate,maxDate, 0.001, 5.0).
            		      setTrajectory(InertialFrameId.EME2000,
            		                    satellitePVList,nbPVPoints, CartesianDerivativesFilter.USE_PV,
            		                    satelliteQList,nbQPoints, AngularDerivativesFilter.USE_R);                  
            
            refining.setRugged(ruggedBuilder.build());
            

            // Compute ground sample distance (GSD)
            // ------------------------------------
            double [] gsd = refining.computeGSD(lineSensor);
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
            refining.applyDisruptions(rollValue, pitchValue, factorValue);
                        

            // Generate measures (observations) from physical model disrupted
            // --------------------------------------------------------------
            GroundMeasureGenerator measures = refining.generatePoints(true);
            refining.setMeasures(measures);
            
            
            // Compute ground truth residues
            // -----------------------------
            System.out.format("\n**** Ground truth residuals **** %n");
            refining.computeMetrics(false);
            
             
            // Initialize physical model without disruptions
            // ---------------------------------------------
            System.out.format("\n**** Initialize physical model without disruptions: reset Roll/Pitch/Factor **** %n");
            refining.resetModel();
            
            
            // Compute initial residues
            // ------------------------
            System.out.format("\n**** Initial Residuals  **** %n");
            refining.computeMetrics(false);
            
             
            // Start optimization
            // ------------------
            System.out.format("\n**** Start optimization  **** %n");
            refining.optimization();
            
            
            // Check estimated values
            // ----------------------
            refining.paramsEstimation(rollValue, pitchValue, factorValue);
                        
            
            // Compute statistics
            // ------------------
            System.out.format("\n**** Compute Statistics **** %n");
            
            // Residues computed in meters
            refining.computeMetrics(false);
            
            // Residues computed in degrees
            refining.computeMetrics(true);
            
            
        } catch (OrekitException oe) {
            System.err.println(oe.getLocalizedMessage());
            System.exit(1);
        } catch (RuggedException re) {
            System.err.println(re.getLocalizedMessage());
            System.exit(1);
        }

    }

    /**
     * @return the pleiadesViewingModel
     */
    public PleiadesViewingModel getPleiadesViewingModel() {
        return pleiadesViewingModel;
    }

    
    /**
     * @param pleiadesViewingModel the pleiadesViewingModel to set
     */
    public void setPleiadesViewingModel(PleiadesViewingModel pleiadesViewingModel) {
        this.pleiadesViewingModel = pleiadesViewingModel;
    }

    
    /**
     * @return the orbitmodel
     */
    public OrbitModel getOrbitmodel() {
        return orbitmodel;
    }

    
    /**
     * @param orbitmodel the orbitmodel to set
     */
    public void setOrbitmodel(OrbitModel orbitmodel) {
        this.orbitmodel = orbitmodel;
    }

    /**
     * @return the rugged
     */
    public Rugged getRugged() {
        return rugged;
    }

    
    /**
     * @param rugged the rugged to set
     */
    public void setRugged(Rugged rugged) {
        this.rugged = rugged;
    }
    
    
    /**
     * @param measure the measure to set
     */
    public void setMeasures(GroundMeasureGenerator measures) {
        this.measures = measures;
    }

   
    /** Estimate ground sample distance (GSD)
     * @param LineSensor line sensor
     * @return GSD
     */
    public double[] computeGSD(LineSensor lineSensor) throws RuggedException {
        
        // Get position
        Vector3D position = lineSensor.getPosition(); // This returns a zero vector since we set the relative position of the sensor w.r.T the satellite to 0.
        
        // Get upper left geodetic point
        AbsoluteDate firstLineDate = lineSensor.getDate(0);
        Vector3D los = lineSensor.getLOS(firstLineDate,0);
        GeodeticPoint upLeftPoint = rugged.directLocation(firstLineDate, position, los);
        los = lineSensor.getLOS(firstLineDate,pleiadesViewingModel.dimension-1);
        
        // Get center geodetic point
        AbsoluteDate lineDate = lineSensor.getDate(pleiadesViewingModel.dimension/2);
        los = lineSensor.getLOS(lineDate,pleiadesViewingModel.dimension/2);
//        GeodeticPoint centerPoint = rugged.directLocation(lineDate, position, los);
//        System.out.format(Locale.US, "center geodetic position : φ = %8.10f °, λ = %8.10f °, h = %8.3f m%n",
//                          FastMath.toDegrees(centerPoint.getLatitude()),
//                          FastMath.toDegrees(centerPoint.getLongitude()),centerPoint.getAltitude());                   
        
        // Get upper right geodetic point    
        int pixelPosition = pleiadesViewingModel.dimension-1;
        los = lineSensor.getLOS(firstLineDate,pixelPosition);
        GeodeticPoint upperRight = rugged.directLocation(firstLineDate, position, los);
       
     // Get lower left geodetic point
        AbsoluteDate lineDate_y = lineSensor.getDate(pleiadesViewingModel.dimension-1);
        los = lineSensor.getLOS(lineDate_y,0);
        GeodeticPoint lowerLeft = rugged.directLocation(lineDate_y, position, los);
 
        double gsdX = DistanceTools.computeDistanceInMeter(upLeftPoint.getLongitude(), upLeftPoint.getLatitude(),upperRight.getLongitude() , upperRight.getLatitude())/pleiadesViewingModel.dimension;
        double gsdY = DistanceTools.computeDistanceInMeter(upLeftPoint.getLongitude(), upLeftPoint.getLatitude(),lowerLeft.getLongitude() , lowerLeft.getLatitude())/pleiadesViewingModel.dimension;
 
        double [] gsd = {gsdX, gsdY};
        return gsd;
    }
    
    
    /** Apply disruptions
     * @param  rollValue rotation on roll value
     * @param  pitchValue rotation on pitch value
     * @param  factorValue scale factor
     * @throws OrekitException 
     * @throws RuggedException 
     */
    public void applyDisruptions(double rollValue, double pitchValue, double factorValue) throws OrekitException, RuggedException {
    
        rugged.
        getLineSensor("line").
        getParametersDrivers().
        filter(driver -> driver.getName().equals("line_roll")).
        findFirst().get().setValue(rollValue);

        rugged.
        getLineSensor("line").
        getParametersDrivers().
        filter(driver -> driver.getName().equals("line_pitch")).
        findFirst().get().setValue(pitchValue);
        
        rugged.
        getLineSensor("line").
        getParametersDrivers().
        filter(driver -> driver.getName().equals("factor")).
        findFirst().get().setValue(factorValue);
    }
    
    
    /** Generate points of measures
     * @param  isNoise flag to known if measures are noisy or not
     * @throws OrekitException 
     * @throws RuggedException 
     */
    public GroundMeasureGenerator generatePoints(boolean isNoise) throws OrekitException, RuggedException {
        
        int lineSampling = 1000;
        int pixelSampling = 1000;       
        
        GroundMeasureGenerator measures = new GroundMeasureGenerator(pleiadesViewingModel,rugged);
        
        if(isNoise) {
        
            System.out.format("\n**** Generate noisy measures **** %n");
            
            // Noise definition
            final Noise noise = new Noise(0,3); /* distribution: gaussian(0), vector dimension:3 */
            final double[] mean = {0.0,0.0,0.0};                /* {lat mean, long mean, alt mean} */
            final double[] standardDeviation = {0.0,0.0,0.0};   /* {lat std, long std, alt std} */
            noise.setMean(mean);
            noise.setStandardDeviation(standardDeviation);

            // Generation noisy measures
            measures.createNoisyMeasure(lineSampling, pixelSampling, noise); 
    
        } else {
            
            System.out.format("\n**** Generate measures (without noise) **** %n");
            // Generation measures without noise
            measures.createMeasure(lineSampling, pixelSampling);
        }
 
        System.out.format("Number of tie points generated: %d %n", measures.getMeasureCount());

        return measures;
        
    }
    
    
    /** Compute metrics
     * @param unit 
     * @throws OrekitException 
     * @throws RuggedException 
     */
    public void computeMetrics(boolean unit) throws RuggedException {
        
        String stUnit = null;
        if(unit) stUnit="degrees";
        else stUnit="meters";
        
        LocalisationMetrics residues = new LocalisationMetrics(measures.getGroundMapping(),rugged, unit);
        System.out.format("Max: %3.4e Mean: %3.4e %s%n",residues.getMaxResidual(),residues.getMeanResidual(), stUnit);
        
        
        
        
    }
    
    
    /** Reset models
     * @throws OrekitException 
     * @throws RuggedException 
     */
    public void resetModel() throws OrekitException, RuggedException {
    
        rugged.
        getLineSensor(pleiadesViewingModel.getSensorName()).
        getParametersDrivers().
        filter(driver -> driver.getName().equals("line_roll") || driver.getName().equals("line_pitch")).
        forEach(driver -> {
            try {
                driver.setSelected(true);
                driver.setValue(0.0);
            } catch (OrekitException e) {
                throw new OrekitExceptionWrapper(e);
            }
        });
        rugged.
        getLineSensor(pleiadesViewingModel.getSensorName()).
        getParametersDrivers().
        filter(driver -> driver.getName().equals("factor")).
        forEach(driver -> {
            try {
                driver.setSelected(true);
                driver.setValue(1.0);       // default value: no Z scale factor applied
            } catch (OrekitException e) {
                throw new OrekitExceptionWrapper(e);
            }
        });
        
    }
    
    
    /** Start optimization
     * @throws OrekitException 
     * @throws RuggedException 
     */
    public void optimization() throws OrekitException, RuggedException {
    
        // perform estimation parameters 
        int maxIterations = 100;
        double convergenceThreshold =  1e-14;
        
        System.out.format("Iterations max: %d\tconvergence threshold: %3.6e \n",maxIterations, convergenceThreshold);

        // Adapt parameters
        Optimum optimum = rugged.estimateFreeParameters(Collections.singletonList(measures.getGroundMapping()), maxIterations,convergenceThreshold);
        
        // Print statistics 
        System.out.format("Max value: %3.6e %n",optimum.getResiduals().getMaxValue());
        System.out.format("Optimization performed in %d iterations \n",optimum.getEvaluations());
        System.out.format("RMSE: %f \n",optimum.getRMS());

    }

    
    /** Check parameters estimation
     * @throws OrekitException 
     * @throws RuggedException 
     */
    public void paramsEstimation(double rollValue, double pitchValue, double factorValue) throws OrekitException, RuggedException {
        
        System.out.format("\n**** Check parameters ajustement **** %n");
        // Estimate Roll
        double estimatedRoll = rugged.getLineSensor(pleiadesViewingModel.getSensorName()).
                                      getParametersDrivers().
                                      filter(driver -> driver.getName().equals("line_roll")).
                                      findFirst().get().getValue();
        
        double rollError = (estimatedRoll - rollValue);
        System.out.format("Estimated roll: %3.5f\troll error: %3.6e %n", estimatedRoll, rollError);

        // Estimate pitch
        double estimatedPitch = rugged.getLineSensor(pleiadesViewingModel.getSensorName()).
                                       getParametersDrivers().
                                       filter(driver -> driver.getName().equals("line_pitch")).
                                       findFirst().get().getValue();
        
        double pitchError = (estimatedPitch - pitchValue);
        System.out.format("Estimated pitch: %3.5f\tpitch error: %3.6e %n", estimatedPitch, pitchError);

        // Estimate factor
        double estimatedFactor = rugged.getLineSensor(pleiadesViewingModel.getSensorName()).
                                        getParametersDrivers().
                                        filter(driver -> driver.getName().equals("factor")).
                                        findFirst().get().getValue();
        
        double factorError = (estimatedFactor - factorValue);
        System.out.format("Estimated factor: %3.5f\tfactor error: %3.6e %n", estimatedFactor, factorError);
    }
}
