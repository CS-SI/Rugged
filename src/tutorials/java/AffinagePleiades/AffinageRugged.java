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

import org.orekit.time.AbsoluteDate;
import org.orekit.utils.AngularDerivativesFilter;
import org.orekit.utils.CartesianDerivativesFilter;
import org.orekit.utils.PVCoordinates;



/**
 * Parameter estimation context 
 * @author Jonathan Guinet
 *
 */
public class AffinageRugged {

    public static void main(String[] args) {

        try {

            // Initialize Orekit, assuming an orekit-data folder is in user home directory
            File home       = new File(System.getProperty("user.home"));
            //File orekitData = new File(home, "workspace/data/orekit-data");
            File orekitData = new File(home, "COTS/orekit-data");
            DataProvidersManager.getInstance().addProvider(new DirectoryCrawler(orekitData));

            
            //CreateOrbit
            OrbitModel orbitmodel =  new OrbitModel();
            BodyShape earth = orbitmodel.createEarth();
            NormalizedSphericalHarmonicsProvider gravityField = orbitmodel.createGravityField();

            //create Pleiades Viewing Model
            PleiadesViewingModel pleiadesViewingModel = new PleiadesViewingModel();
            AbsoluteDate minDate =  pleiadesViewingModel.getMinDate();
            AbsoluteDate maxDate =  pleiadesViewingModel.getMaxDate();
            AbsoluteDate refDate = pleiadesViewingModel.getDatationReference();

            Orbit      orbit                                  = orbitmodel.createOrbit(gravityField.getMu(), refDate);
            
            PVCoordinates PV = orbit.getPVCoordinates(earth.getBodyFrame());
            GeodeticPoint gp = earth.transform(PV.getPosition(), earth.getBodyFrame(), orbit.getDate());
            
            
            
            System.out.format(" **** Orbit Definition **** %n");
            
            System.out.format(Locale.US, "Geodetic Point at date %s : φ = %8.10f °, λ = %8.10f %n",orbit.getDate().toString(),
                    FastMath.toDegrees(gp.getLatitude()),
                    FastMath.toDegrees(gp.getLongitude()));
            

            System.out.format(" **** Build Viewing Model **** %n");       
            
           
            // Build Rugged ...
            RuggedBuilder ruggedBuilder = new RuggedBuilder();
    
            LineSensor lineSensor =  pleiadesViewingModel.getLineSensor(); 
            ruggedBuilder.addLineSensor(lineSensor);
            
      
            ruggedBuilder.setAlgorithm(AlgorithmId.IGNORE_DEM_USE_ELLIPSOID);
            ruggedBuilder.setEllipsoid(EllipsoidId.WGS84, BodyRotatingFrameId.ITRF);
            ruggedBuilder.setTimeSpan(minDate,maxDate, 0.001, 5.0).
            		setTrajectory(InertialFrameId.EME2000,
                    orbitmodel.orbitToPV(orbit, earth, minDate.shiftedBy(-0.0), maxDate.shiftedBy(+0.0), 0.25),
                    8, CartesianDerivativesFilter.USE_PV,
                    orbitmodel.orbitToQ(orbit, earth, minDate.shiftedBy(-0.0), maxDate.shiftedBy(+0.0), 0.25),
                    2, AngularDerivativesFilter.USE_R);                  
            Rugged rugged = ruggedBuilder.build();
            

           
            
            Vector3D position = lineSensor.getPosition(); // This returns a zero vector since we set the relative position of the sensor w.r.T the satellite to 0.
            AbsoluteDate firstLineDate = lineSensor.getDate(pleiadesViewingModel.dimension/2);
            System.out.format("date %s %n",firstLineDate.toString());
            Vector3D los = lineSensor.getLOS(firstLineDate,0);
            GeodeticPoint upLeftPoint = rugged.directLocation(firstLineDate, position, los);
            System.out.format(Locale.US, "upper left point: φ = %8.10f °, λ = %8.10f °, h = %8.3f m%n",
                              FastMath.toDegrees(upLeftPoint.getLatitude()),
                              FastMath.toDegrees(upLeftPoint.getLongitude()),
                              upLeftPoint.getAltitude());
            los = lineSensor.getLOS(firstLineDate,pleiadesViewingModel.dimension-1);
            GeodeticPoint upRightPoint = rugged.directLocation(firstLineDate, position, los);
            System.out.format(Locale.US, "upper right point: φ = %8.10f °, λ = %8.10f °, h = %8.3f m%n",
                              FastMath.toDegrees(upRightPoint.getLatitude()),
                              FastMath.toDegrees(upRightPoint.getLongitude()),upRightPoint.getAltitude());                   
            los = lineSensor.getLOS(firstLineDate,pleiadesViewingModel.dimension/2);
            GeodeticPoint centerPoint = rugged.directLocation(firstLineDate, position, los);
            System.out.format(Locale.US, "center point: φ = %8.10f °, λ = %8.10f °, h = %8.3f m%n",
                              FastMath.toDegrees(centerPoint.getLatitude()),
                              FastMath.toDegrees(centerPoint.getLongitude()),centerPoint.getAltitude());                   
            
            
            System.out.format(" **** Add roll and pitch values **** %n");       
            
            
            double rollValue =  FastMath.toRadians( 0.1);
            double pitchValue = FastMath.toRadians(-0.3);
            double factorValue = 0.5;
                        
            System.out.format("roll : %3.5f pitch : %3.5f factor : %3.5f \n",rollValue,pitchValue,factorValue);
                        
            rugged.
            getLineSensor("line").
            getParametersDrivers().
            filter(driver -> driver.getName().equals("roll")).
            findFirst().get().setValue(rollValue);

            rugged.
            getLineSensor("line").
            getParametersDrivers().
            filter(driver -> driver.getName().equals("pitch")).
            findFirst().get().setValue(pitchValue);
            
            rugged.
            getLineSensor("line").
            getParametersDrivers().
            filter(driver -> driver.getName().equals("factor")).
            findFirst().get().setValue(factorValue);
            

            
            System.out.format(" **** Generate Measures **** %n");
            
            MeasureGenerator measure = new MeasureGenerator(pleiadesViewingModel,rugged);
            int lineSampling = 1000;
            int pixelSampling = 1000;		
            measure.CreateMeasure(lineSampling, pixelSampling);
            System.out.format("nb TiePoints %d %n", measure.getMeasureCount());

            System.out.format(" **** Reset Roll/Pitch/Factor **** %n");
            rugged.
            getLineSensor(pleiadesViewingModel.getSensorName()).
            getParametersDrivers().
            filter(driver -> driver.getName().equals("roll") || driver.getName().equals("pitch")).
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
                    driver.setValue(1.0);		// default value: no Z scale factor applied
                } catch (OrekitException e) {
                    throw new OrekitExceptionWrapper(e);
                }
            });
            
            System.out.format(" **** Initial Residuals  **** %n");
            
            LocalisationMetrics initlialResiduals = new LocalisationMetrics(measure.getMapping(),rugged, false);
            System.out.format("residuals max :  %3.4e mean %3.4e meters  %n",initlialResiduals.getMaxResidual(),initlialResiduals.getMeanResidual());
             
           
            
            System.out.format(" **** Start optimization  **** %n");
            // perform parameters estimation
            int maxIterations = 15;
            double convergenceThreshold =  1e-12;
            
            System.out.format("iterations max %d convergence threshold  %3.6e \n",maxIterations, convergenceThreshold);

            
            Optimum optimum = rugged.estimateFreeParameters(Collections.singletonList(measure.getMapping()), maxIterations,convergenceThreshold);
            
   
            System.out.format(" Optimization performed in %d iterations \n",optimum.getEvaluations());

            // check estimated values
            double estimatedRoll = rugged.getLineSensor(pleiadesViewingModel.getSensorName()).
                                          getParametersDrivers().
                                          filter(driver -> driver.getName().equals("roll")).
                                          findFirst().get().getValue();
            double rollError = (estimatedRoll - rollValue);
            System.out.format("Estimated roll %3.5f roll error %3.6e  %n", estimatedRoll, rollError);

            double estimatedPitch = rugged.getLineSensor(pleiadesViewingModel.getSensorName()).
                                           getParametersDrivers().
                                           filter(driver -> driver.getName().equals("pitch")).
                                           findFirst().get().getValue();
            double pitchError = (estimatedPitch - pitchValue);
            System.out.format("Estimated pitch %3.5f pitch error %3.6e  %n ", estimatedPitch, pitchError);
            
            double estimatedFactor = rugged.getLineSensor(pleiadesViewingModel.getSensorName()).
                    getParametersDrivers().
                    filter(driver -> driver.getName().equals("factor")).
                    findFirst().get().getValue();
            double factorError = (estimatedFactor - factorValue);
            System.out.format("Estimated factor %3.5f factor error %3.6e  %n ", estimatedFactor, factorError);
            
            System.out.format(" **** Compute Statistics **** %n");

            LocalisationMetrics localisationResiduals = new LocalisationMetrics(measure.getMapping(),rugged, false);
            System.out.format("residuals max :  %3.4e mean %3.4e  meters %n",localisationResiduals.getMaxResidual(),localisationResiduals.getMeanResidual());
            //RealVector residuals =  optimum.getResiduals();


            
            

        } catch (OrekitException oe) {
            System.err.println(oe.getLocalizedMessage());
            System.exit(1);
        } catch (RuggedException re) {
            System.err.println(re.getLocalizedMessage());
            System.exit(1);
        }

    }



}
