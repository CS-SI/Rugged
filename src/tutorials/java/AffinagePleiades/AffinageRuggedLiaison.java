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


import org.hipparchus.util.FastMath;
import org.hipparchus.optim.nonlinear.vector.leastsquares.LeastSquaresOptimizer.Optimum;
import java.io.File;
import java.util.Locale;

import java.util.Collections;


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
import org.orekit.rugged.refining.generators.InterMeasureGenerator;
import org.orekit.rugged.refining.measures.Noise;
import org.orekit.rugged.refining.metrics.SensorToSensorLocalisationMetrics;
import org.orekit.rugged.refining.models.OrbitModel;
import org.orekit.rugged.refining.models.PleiadesViewingModel;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.AngularDerivativesFilter;
import org.orekit.utils.CartesianDerivativesFilter;
import org.orekit.utils.PVCoordinates;



/**
 * Parameter refining context 
 * @author Jonathan Guinet
 * @author Lucie LabatAllee
 */
public class AffinageRuggedLiaison {

    /**
     * @param args
     */
    public static void main(String[] args) {

        try {

            // Initialize Orekit, assuming an orekit-data folder is in user home directory
            File home       = new File(System.getProperty("user.home"));
            
            File orekitData = new File(home, "COTS/orekit-data");
            DataProvidersManager.getInstance().addProvider(new DirectoryCrawler(orekitData));

            

            //create 2 Pleiades Viewing Model
            final String SensorNameA = "SensorA";
            final double incidenceAngleA = -5.0;
            //final String dateA = "2016-01-01T11:59:50.0";
            final String dateA = "2016-01-01T11:59:50.0";
            PleiadesViewingModel pleiadesViewingModelA = new PleiadesViewingModel(SensorNameA,incidenceAngleA,dateA);
            final AbsoluteDate minDateA =  pleiadesViewingModelA.getMinDate();
            final AbsoluteDate maxDateA =  pleiadesViewingModelA.getMaxDate();
            final AbsoluteDate refDateA = pleiadesViewingModelA.getDatationReference();
            System.out.format(Locale.US, "Viewing model A - date min: %s max: %s ref: %s  %n", minDateA, maxDateA, refDateA);        

            //CreateOrbitA
            OrbitModel orbitmodelA =  new OrbitModel();
            BodyShape earthA = orbitmodelA.createEarth();
            NormalizedSphericalHarmonicsProvider gravityFieldA = orbitmodelA.createGravityField();
            Orbit      orbitA  = orbitmodelA.createOrbit(gravityFieldA.getMu(), refDateA);
            
            
            // if no LOF Transform Attitude Provider is Nadir Pointing Yaw Compensation
            final double [] rollPoly = {0.0,0.0,0.0};
            double[] pitchPoly = {0.025,0.0};
            double[] yawPoly = {0.0,0.0,0.0};
            orbitmodelA.setLOFTransform(rollPoly, pitchPoly, yawPoly, minDateA);
            
            PVCoordinates PVA = orbitA.getPVCoordinates(earthA.getBodyFrame());
            GeodeticPoint gpA = earthA.transform(PVA.getPosition(), earthA.getBodyFrame(), orbitA.getDate());
            
            System.out.format(Locale.US, "Geodetic Point at date %s : φ = %8.10f °, λ = %8.10f %n",orbitA.getDate().toString(),
                              FastMath.toDegrees(gpA.getLatitude()),
                              FastMath.toDegrees(gpA.getLongitude()));                       
            
            // Build Rugged A and B
            RuggedBuilder ruggedBuilderA = new RuggedBuilder();
    
            LineSensor lineSensorA =  pleiadesViewingModelA.getLineSensor(); 
            ruggedBuilderA.addLineSensor(lineSensorA);
            
      
            ruggedBuilderA.setAlgorithm(AlgorithmId.IGNORE_DEM_USE_ELLIPSOID);
            ruggedBuilderA.setEllipsoid(EllipsoidId.WGS84, BodyRotatingFrameId.ITRF);
            ruggedBuilderA.setTimeSpan(minDateA,maxDateA, 0.001, 5.0).
                    setTrajectory(InertialFrameId.EME2000,
                    orbitmodelA.orbitToPV(orbitA, earthA, minDateA.shiftedBy(-0.0), maxDateA.shiftedBy(+0.0), 0.25),
                    8, CartesianDerivativesFilter.USE_PV,
                    orbitmodelA.orbitToQ(orbitA, earthA, minDateA.shiftedBy(-0.0), maxDateA.shiftedBy(+0.0), 0.25),
                    2, AngularDerivativesFilter.USE_R)
                    .setLightTimeCorrection(false)
                    .setAberrationOfLightCorrection(false);                  
            Rugged ruggedA = ruggedBuilderA.build();
            

            
            
            //create 2 Pleiades Viewing Model
            final String SensorNameB = "SensorB";
            final double incidenceAngleB = 0.0;
            //final String dateB = "2016-01-01T12:00:10.0";
            final String dateB = "2016-01-01T12:02:50.0";
            PleiadesViewingModel pleiadesViewingModelB = new PleiadesViewingModel(SensorNameB,incidenceAngleB,dateB);
            final AbsoluteDate minDateB =  pleiadesViewingModelB.getMinDate();
            final AbsoluteDate maxDateB =  pleiadesViewingModelB.getMaxDate();
            final AbsoluteDate refDateB = pleiadesViewingModelB.getDatationReference();
            System.out.format(Locale.US, "Viewing model B - date min: %s max: %s ref: %s  %n", minDateB, maxDateB, refDateB);        

            //CreateOrbitA
            OrbitModel orbitmodelB =  new OrbitModel();
            BodyShape earthB = orbitmodelB.createEarth();
            NormalizedSphericalHarmonicsProvider gravityFieldB = orbitmodelB.createGravityField();
            Orbit      orbitB  = orbitmodelB.createOrbit(gravityFieldB.getMu(), refDateB);
            

            //orbitmodelB.setLOFTransform(rollPoly, pitchPoly, yawPoly, minDateB);
            
            PVCoordinates PVB = orbitB.getPVCoordinates(earthB.getBodyFrame());
            GeodeticPoint gpB = earthB.transform(PVB.getPosition(), earthB.getBodyFrame(), orbitB.getDate());
            
            System.out.format(Locale.US, "Geodetic Point at date %s : φ = %8.10f °, λ = %8.10f %n",orbitA.getDate().toString(),
                              FastMath.toDegrees(gpB.getLatitude()),
                              FastMath.toDegrees(gpB.getLongitude()));                       
            

                       
            RuggedBuilder ruggedBuilderB = new RuggedBuilder();
            
            LineSensor lineSensorB =  pleiadesViewingModelB.getLineSensor(); 
            ruggedBuilderB.addLineSensor(lineSensorB);
            
      
            ruggedBuilderB.setAlgorithm(AlgorithmId.IGNORE_DEM_USE_ELLIPSOID);
            ruggedBuilderB.setEllipsoid(EllipsoidId.WGS84, BodyRotatingFrameId.ITRF);
            ruggedBuilderB.setTimeSpan(minDateB,maxDateB, 0.001, 5.0).
                    setTrajectory(InertialFrameId.EME2000,
                    orbitmodelB.orbitToPV(orbitB, earthB, minDateB.shiftedBy(-0.0), maxDateB.shiftedBy(+0.0), 0.25),
                    8, CartesianDerivativesFilter.USE_PV,
                    orbitmodelB.orbitToQ(orbitB, earthB, minDateB.shiftedBy(-0.0), maxDateB.shiftedBy(+0.0), 0.25),
                    2, AngularDerivativesFilter.USE_R)
                    .setLightTimeCorrection(false)
                    .setAberrationOfLightCorrection(false);                  
                  
            Rugged ruggedB = ruggedBuilderB.build();
   

            
            
          
            
            /*
            
            Vector3D positionA = lineSensorA.getPosition(); // This returns a zero vector since we set the relative position of the sensor w.r.T the satellite to 0.
                 
            AbsoluteDate lineDateA = lineSensorA.getDate(pleiadesViewingModelA.dimension/2);
            Vector3D losA = lineSensorA.getLOS(lineDateA,pleiadesViewingModelA.dimension/2);
            GeodeticPoint centerPointA = ruggedA.directLocation(lineDateA, positionA, losA);
            System.out.format(Locale.US, "center geodetic position A : φ = %8.10f °, λ = %8.10f °, h = %8.3f m%n",
                              FastMath.toDegrees(centerPointA.getLatitude()),
                              FastMath.toDegrees(centerPointA.getLongitude()),centerPointA.getAltitude());      
            
            
            Vector3D positionB = lineSensorB.getPosition(); // This returns a zero vector since we set the relative position of the sensor w.r.T the satellite to 0.
            
            AbsoluteDate lineDateB = lineSensorB.getDate(pleiadesViewingModelB.dimension/2);
            Vector3D losB = lineSensorB.getLOS(lineDateB,pleiadesViewingModelB.dimension/2);
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
            
            lineDateB = lineSensorB.getDate(pleiadesViewingModelB.dimension-1);
            losB = lineSensorB.getLOS(lineDateB,pleiadesViewingModelB.dimension-1);
            GeodeticPoint lastPointB = ruggedB.directLocation(lineDateB, positionB, losB);
            System.out.format(Locale.US, "last geodetic position  B : φ = %8.10f °, λ = %8.10f °, h = %8.3f m%n",
                              FastMath.toDegrees(lastPointB.getLatitude()),
                              FastMath.toDegrees(lastPointB.getLongitude()),lastPointB.getAltitude());      
            
            
            
            
            
                     
            
            
            double distance = DistanceTools.computeDistanceRad(centerPointA.getLongitude(), centerPointA.getLatitude(),
                                                               centerPointB.getLongitude(), centerPointB.getLatitude());
            
            System.out.format("distance %f meters %n",distance);*/
            
            
            System.out.format(" **** Add roll / pitch / factor values **** %n"); 
            double rollValueA =  FastMath.toRadians(0.004);//0.6
            double pitchValueA = FastMath.toRadians(0.0008);//0.02
            double factorValue = 1.0;
            final String SensorNameAFactor = "factor";
            final String SensorNameBFactor = "factor";
            
            System.out.format("Acquisition A roll : %3.5f pitch : %3.5f factor : %3.5f \n",rollValueA,pitchValueA,factorValue);
                        
            ruggedA.
            getLineSensor(SensorNameA).
            getParametersDrivers().
            filter(driver -> driver.getName().equals(SensorNameA+"_roll")).
            findFirst().get().setValue(rollValueA);

            ruggedA.
            getLineSensor(SensorNameA).
            getParametersDrivers().
            filter(driver -> driver.getName().equals(SensorNameA+"_pitch")).
            findFirst().get().setValue(pitchValueA);
            
            ruggedA.
            getLineSensor(SensorNameA).
            getParametersDrivers().
            filter(driver -> driver.getName().equals(SensorNameAFactor)).
            findFirst().get().setValue(factorValue);
            
            /*
            lineDateA = lineSensorA.getDate(pleiadesViewingModelA.dimension/2);
            losA = lineSensorA.getLOS(lineDateA,(int) pleiadesViewingModelA.dimension/2);*/
            
            //Vector3D losADouble = lineSensorA.getLOS(lineDateA,(double) pleiadesViewingModelA.dimension/2);
            //final Vector3D vDistance = losA.subtract(losADouble);
            //final double d = vDistance.getNorm();
            //System.out.format("distance double/int %f ",d);
            double rollValueB =  FastMath.toRadians(-0.004);//0.6
            double pitchValueB = FastMath.toRadians(0.0008);//0.02
            //double pitchValueB = FastMath.toRadians(0.0);//0.02
                        
            System.out.format(" Acquisition B roll : %3.5f pitch : %3.5f factor : %3.5f \n",rollValueB,pitchValueB,factorValue);
                        
            ruggedB.
            getLineSensor(SensorNameB).
            getParametersDrivers().
            filter(driver -> driver.getName().equals(SensorNameB+"_roll")).
            findFirst().get().setValue(rollValueB);

            ruggedB.
            getLineSensor(SensorNameB).
            getParametersDrivers().
            filter(driver -> driver.getName().equals(SensorNameB+"_pitch")).
            findFirst().get().setValue(pitchValueB);
            
            ruggedB.
            getLineSensor(SensorNameB).
            getParametersDrivers().
            filter(driver -> driver.getName().equals(SensorNameBFactor)).
            findFirst().get().setValue(factorValue);
            
            
            /*centerPointA = ruggedA.directLocation(lineDateA, positionA, losA);
            System.out.format(Locale.US, "center geodetic position : φ = %8.10f °, λ = %8.10f °, h = %8.3f m%n",
                              FastMath.toDegrees(centerPointA.getLatitude()),
                              FastMath.toDegrees(centerPointA.getLongitude()),centerPointA.getAltitude());      
            
            distance = DistanceTools.computeDistanceRad(centerPointA.getLongitude(), centerPointA.getLatitude(),
                                                               centerPointB.getLongitude(), centerPointB.getLatitude());
            
            */
            
            //add mapping
            final Noise noise = new Noise(0,2); /* distribution: gaussian(0), vector dimension:3 */
            final double[] mean = {5.0,0.0}; // {pixelA mean, pixelB mean}
            final double[] standardDeviation = {0.1,0.1};  // {pixelB std, pixelB std}
            noise.setMean(mean);
            noise.setStandardDeviation(standardDeviation);

            System.out.format(" **** Generate Measures  **** %n");
            System.out.format(" **** Sensor A mean %f std %f   **** %n",mean[0],standardDeviation[0]);
            System.out.format(" **** Sensor B mean %f std %f   **** %n",mean[1],standardDeviation[1]);
            
            final double outlierValue = 1e+2;
            final double earthConstraintWeight = 0.1;

            InterMeasureGenerator measure = new InterMeasureGenerator(pleiadesViewingModelA,ruggedA,
                                                                      pleiadesViewingModelB,ruggedB,
                                                                      outlierValue,
                                                                      earthConstraintWeight);
            final int lineSampling = 1000;
            final int pixelSampling = 1000;  

            
            measure.createNoisyMeasure(lineSampling, pixelSampling,noise); 
            System.out.format("nb TiePoints %d %n", measure.getMeasureCount());
            
            SensorToSensorLocalisationMetrics gtResiduals = new SensorToSensorLocalisationMetrics(measure.getInterMapping(),ruggedA,ruggedB, false);
            System.out.format("gt residuals max :  %1.4e mean %1.4e meters  %n",gtResiduals.getMaxResidual(),gtResiduals.getMeanResidual());
            System.out.format("gt los distance max :  %1.4e mean %1.4e meters  %n",gtResiduals.getLosMaxDistance(),gtResiduals.getLosMeanDistance());
            System.out.format("gt earth distance max :  %1.4e mean %1.4e meters  %n",gtResiduals.getEarthMaxDistance(),gtResiduals.getEarthMeanDistance());
                                           
            
            // initialize ruggedA without perturbation
            System.out.format(" **** Reset Roll/Pitch/Factor (ruggedA) **** %n");
            ruggedA.
            getLineSensor(pleiadesViewingModelA.getSensorName()).
            getParametersDrivers().
            filter(driver -> driver.getName().equals(SensorNameA+"_roll")).
            forEach(driver -> {
                try {
                    driver.setSelected(true);
                    driver.setValue(0.0);
                } catch (OrekitException e) {
                    throw new OrekitExceptionWrapper(e);
                }
            });
            ruggedA.
            getLineSensor(pleiadesViewingModelA.getSensorName()).
            getParametersDrivers().
            filter(driver -> driver.getName().equals(SensorNameA+"_pitch")).
            forEach(driver -> {
                try {
                    driver.setSelected(true);
                    driver.setValue(0.0);
                } catch (OrekitException e) {
                    throw new OrekitExceptionWrapper(e);
                }
            });           

            
            
            ruggedA.
            getLineSensor(pleiadesViewingModelA.getSensorName()).
            getParametersDrivers().
            filter(driver -> driver.getName().equals(SensorNameAFactor)).
            forEach(driver -> {
                try {
                    driver.setSelected(false);
                    driver.setValue(1.0);       // default value: no Z scale factor applied
                } catch (OrekitException e) {
                    throw new OrekitExceptionWrapper(e);
                }
            });
            
         // initialize ruggedB with selected flag set to true
            System.out.format(" **** Select parameters drivers (ruggedB) **** %n");
            /*ruggedB.
            getLineSensor(pleiadesViewingModelB.getSensorName()).
            getParametersDrivers().
            filter(driver -> driver.getName().equals(SensorNameB+"_roll") || driver.getName().equals(SensorNameB+"_pitch")).
            forEach(driver -> {
                try {
                    driver.setSelected(true);
                    driver.setValue(0.0);
                } catch (OrekitException e) {
                    throw new OrekitExceptionWrapper(e);
                }
            });
            ruggedB.
            getLineSensor(pleiadesViewingModelB.getSensorName()).
            getParametersDrivers().
            filter(driver -> driver.getName().equals(SensorNameBFactor)).
            forEach(driver -> {
                try {
                    driver.setSelected(true);
                    driver.setValue(1.0); 
                } catch (OrekitException e) {
                    throw new OrekitExceptionWrapper(e);
                }
            });*/
            
            
            ruggedB.
            getLineSensor(SensorNameB).
            getParametersDrivers().
            filter(driver -> driver.getName().equals(SensorNameB+"_roll")).
            findFirst().get().setValue(0.0);
            ruggedB.
            getLineSensor(SensorNameB).
            getParametersDrivers().
            filter(driver -> driver.getName().equals(SensorNameB+"_roll")).
            findFirst().get().setSelected(true);
            
            ruggedB.
            getLineSensor(SensorNameB).
            getParametersDrivers().
            filter(driver -> driver.getName().equals(SensorNameB+"_pitch")).
            findFirst().get().setValue(0.0);
            ruggedB.
            getLineSensor(SensorNameB).
            getParametersDrivers().
            filter(driver -> driver.getName().equals(SensorNameB+"_pitch")).
            findFirst().get().setSelected(true);
            
            ruggedB.
            getLineSensor(SensorNameB).
            getParametersDrivers().
            filter(driver -> driver.getName().equals(SensorNameBFactor)).
            findFirst().get().setValue(1.0);
            ruggedB.
            getLineSensor(SensorNameB).
            getParametersDrivers().
            filter(driver -> driver.getName().equals(SensorNameBFactor)).
            findFirst().get().setSelected(false);            
           
            
            SensorToSensorLocalisationMetrics initialResiduals = new SensorToSensorLocalisationMetrics(measure.getInterMapping(),ruggedA,ruggedB, false);
            System.out.format("initial residuals max :  %3.4f mean %3.4f meters  %n",initialResiduals.getMaxResidual(),initialResiduals.getMeanResidual());
            System.out.format("initial los distance max :  %1.4e mean %1.4e meters  %n",initialResiduals.getLosMaxDistance(),initialResiduals.getLosMeanDistance());
            System.out.format("initial earth distance max :  %1.4e mean %1.4e meters  %n",initialResiduals.getEarthMaxDistance(),initialResiduals.getEarthMeanDistance());
               
            
            System.out.format(" **** Start optimization  **** %n");
            // perform parameters estimation
            int maxIterations = 100;
            double convergenceThreshold =  1e-10;//1e-14;
            
            System.out.format("iterations max %d convergence threshold  %3.6e \n",maxIterations, convergenceThreshold);
            
            // Note: estimateFreeParams2Models() is supposed to be applying on ruggedB, not on ruggedA (to be compliant with notations)
            
            Optimum optimum = ruggedB.estimateFreeParams2Models(Collections.singletonList(measure.getInterMapping()), maxIterations,convergenceThreshold, ruggedA);
            System.out.format("max value  %3.6e %n",optimum.getResiduals().getMaxValue());
   
            System.out.format(" Optimization performed in %d iterations \n",optimum.getEvaluations());
            System.out.format(" RMSE: %f \n",optimum.getRMS());
            
            
            System.out.format(" **** Check estimated values  **** %n");
            // check estimated values
            // Viewing model A
            double estRollA = ruggedA.getLineSensor(pleiadesViewingModelA.getSensorName()).
                                          getParametersDrivers().
                                          filter(driver -> driver.getName().equals(SensorNameA+"_roll")).
                                          findFirst().get().getValue();
            double rollErrorA = (estRollA - rollValueA);
            System.out.format("Estimated roll A %3.5f, roll error %3.6e  %n", estRollA, rollErrorA);

            double estPitchA = ruggedA.getLineSensor(pleiadesViewingModelA.getSensorName()).
                                           getParametersDrivers().
                                           filter(driver -> driver.getName().equals(SensorNameA+"_pitch")).
                                           findFirst().get().getValue();
            double pitchErrorA = (estPitchA - pitchValueA);
            System.out.format("Estimated pitch A %3.5f, pitch error %3.6e  %n ", estPitchA, pitchErrorA);
            
            double estFactorA = ruggedA.getLineSensor(pleiadesViewingModelA.getSensorName()).
                    getParametersDrivers().
                    //filter(driver -> driver.getName().equals(SensorNameAFactor)).
                    filter(driver -> driver.getName().equals(SensorNameAFactor)).
                    findFirst().get().getValue();
            double factorErrorA = (estFactorA - factorValue);
            System.out.format("Estimated factor A %3.5f, factor error %3.6e  %n ", estFactorA, factorErrorA);

            // Viewing model B
            double estRollB = ruggedB.getLineSensor(pleiadesViewingModelB.getSensorName()).
                                          getParametersDrivers().
                                          filter(driver -> driver.getName().equals(SensorNameB+"_roll")).
                                          findFirst().get().getValue();
            double rollErrorB = (estRollB - rollValueB);
            System.out.format("Estimated roll B %3.5f, roll error %3.6e  %n", estRollB, rollErrorB);

            double estPitchB = ruggedB.getLineSensor(pleiadesViewingModelB.getSensorName()).
                                           getParametersDrivers().
                                           filter(driver -> driver.getName().equals(SensorNameB+"_pitch")).
                                           findFirst().get().getValue();
            double pitchErrorB = (estPitchB - pitchValueB);
            System.out.format("Estimated pitch B %3.5f, pitch error %3.6e  %n ", estPitchB, pitchErrorB);
            
            double estFactorB = ruggedB.getLineSensor(pleiadesViewingModelB.getSensorName()).
                    getParametersDrivers().
                    filter(driver -> driver.getName().equals(SensorNameBFactor)).
                    findFirst().get().getValue();
            double factorErrorB = (estFactorB - factorValue);
            System.out.format("Estimated factor B %3.5f, factor error %3.6e  %n ", estFactorB, factorErrorB);
            
            
            
            
            SensorToSensorLocalisationMetrics finalResiduals = new SensorToSensorLocalisationMetrics(measure.getInterMapping(),ruggedA,ruggedB, false);
            System.out.format("final residuals max :  %1.4e mean %1.4e meters  %n",finalResiduals.getMaxResidual(),finalResiduals.getMeanResidual());
            System.out.format("final los distance max :  %1.4e mean %1.4e meters %n",finalResiduals.getLosMaxDistance(),finalResiduals.getLosMeanDistance());
            System.out.format("final earth distance max :  %1.4e mean %1.4e meters  %n",finalResiduals.getEarthMaxDistance(),finalResiduals.getEarthMeanDistance());
                         
                 
        } catch (OrekitException oe) {
            System.err.println(oe.getLocalizedMessage());
            System.exit(1);
        } catch (RuggedException re) {
            System.err.println(re.getLocalizedMessage());
            System.exit(1);
        }

    }



}
