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
import java.io.FileWriter;
import java.io.StringWriter;
import java.io.PrintWriter;
import java.io.IOException;

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
 * Parameter refining context 
 * @author Jonathan Guinet
 * @author Lucie LabatAllee
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
            
            final double [] rollPoly = {0.0,0.0,0.0};
            double[] pitchPoly = {0.0,0.0};
            double[] yawPoly = {0.0,0.0,0.0};
            orbitmodel.setLOFTransform(rollPoly, pitchPoly, yawPoly, minDate);
            
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
            AbsoluteDate firstLineDate = lineSensor.getDate(0);
            Vector3D los = lineSensor.getLOS(firstLineDate,0);
            GeodeticPoint upLeftPoint = rugged.directLocation(firstLineDate, position, los);
          
            los = lineSensor.getLOS(firstLineDate,pleiadesViewingModel.dimension-1);
            //GeodeticPoint upRightPoint = rugged.directLocation(firstLineDate, position, los);               
            
            
            AbsoluteDate lineDate = lineSensor.getDate(pleiadesViewingModel.dimension/2);
            los = lineSensor.getLOS(lineDate,pleiadesViewingModel.dimension/2);
            GeodeticPoint centerPoint = rugged.directLocation(lineDate, position, los);
            System.out.format(Locale.US, "center geodetic position : φ = %8.10f °, λ = %8.10f °, h = %8.3f m%n",
                              FastMath.toDegrees(centerPoint.getLatitude()),
                              FastMath.toDegrees(centerPoint.getLongitude()),centerPoint.getAltitude());                   
        	
            /*try {
            FileWriter out = new FileWriter("geodetic_position.txt");
            
            out.write("LATITUDE, LONGITUDE, LINE, PIX \n");

            for (double line = 0; line < pleiadesViewingModel.dimension; line += 1000) {
            	
            	AbsoluteDate date = lineSensor.getDate(line);
            	for (int pixel = 0; pixel < lineSensor.getNbPixels(); pixel += 1000) {
            		GeodeticPoint gp2 = rugged.directLocation(date, lineSensor.getPosition(),
            				lineSensor.getLOS(date, pixel));
            		String result = String.format("%8.10f, %8.10f, %2.1f, %2.1f \n ",FastMath.toDegrees(gp2.getLatitude()),
            				FastMath.toDegrees(gp2.getLongitude()), line,(double) pixel);
            			//System.out.println(result);
                    out.write(result);
            	}
        	}
            out.close();
            }
            catch (IOException e) {
                System.err.println(e.getLocalizedMessage());
                System.exit(1);
            }*/
                
            int pixelPosition = pleiadesViewingModel.dimension-1;
            los = lineSensor.getLOS(firstLineDate,pixelPosition);
            GeodeticPoint upperRight = rugged.directLocation(firstLineDate, position, los);
           
            AbsoluteDate lineDate_y = lineSensor.getDate(pleiadesViewingModel.dimension-1);
            los = lineSensor.getLOS(lineDate_y,0);
            GeodeticPoint lowerLeft = rugged.directLocation(lineDate_y, position, los);
     
            
            double GSD_X = DistanceTools.computeDistanceRad(upLeftPoint.getLongitude(), upLeftPoint.getLatitude(),upperRight.getLongitude() , upperRight.getLatitude())/pleiadesViewingModel.dimension;
            double GSD_Y = DistanceTools.computeDistanceRad(upLeftPoint.getLongitude(), upLeftPoint.getLatitude(),lowerLeft.getLongitude() , lowerLeft.getLatitude())/pleiadesViewingModel.dimension;
            
            
            System.out.format(" GSD X %2.2f Y %2.2f **** %n", GSD_X, GSD_Y);    
            
            System.out.format(" **** Add roll and pitch values **** %n");       
            
            double rollValue =  FastMath.toRadians(-0.01);
            double pitchValue = FastMath.toRadians(0.02);
            double factorValue = 1.05;
                        
            System.out.format("roll : %3.5f pitch : %3.5f factor : %3.5f \n",rollValue,pitchValue,factorValue);
                        
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
            filter(driver -> driver.getName().equals("line_factor")).
            findFirst().get().setValue(factorValue);
            

            
            System.out.format(" **** Generate Measures **** %n");
            
            MeasureGenerator measure = new MeasureGenerator(pleiadesViewingModel,rugged);
            int lineSampling = 1000;
            int pixelSampling = 1000;		
            //measure.CreateMeasure(lineSampling, pixelSampling); 
            
            //double pixErr = 0.0;
            //double altErr = 0.0;
            final double[] pixErr = new double[4];
            pixErr[0] = 0; // lat mean
            pixErr[1] = 0.0; // lat std
            pixErr[2] = 0; // lon mean
            pixErr[3] = 0.0; // lon std
            
            final double[] altErr = new double[2];
            altErr[0] = 0.0; //mean
            altErr[1] = 0.0; //std  
            measure.CreateNoisyMeasure(lineSampling, pixelSampling,pixErr,altErr); // Test with noisy measures
            System.out.format("nb TiePoints %d %n", measure.getMeasureCount());

            System.out.format(" **** Initial Residuals  **** %n");
            
            LocalisationMetrics gtResiduals = new LocalisationMetrics(measure.getMapping(),rugged, false);
            System.out.format("gt residuals max :  %3.4f mean %3.4f meters  %n",gtResiduals.getMaxResidual(),gtResiduals.getMeanResidual());
             
           
            
            
            System.out.format(" **** Reset Roll/Pitch/Factor **** %n");
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
            filter(driver -> driver.getName().equals("line_factor")).
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
            System.out.format("residuals max :  %3.4f mean %3.4f meters  %n",initlialResiduals.getMaxResidual(),initlialResiduals.getMeanResidual());
             
           
            
            System.out.format(" **** Start optimization  **** %n");
            // perform parameters estimation
            int maxIterations = 100;
            double convergenceThreshold =  1e-14;
            
            System.out.format("iterations max %d convergence threshold  %3.6e \n",maxIterations, convergenceThreshold);

            
            Optimum optimum = rugged.estimateFreeParameters(Collections.singletonList(measure.getMapping()), maxIterations,convergenceThreshold);
            System.out.format("max value  %3.6e %n",optimum.getResiduals().getMaxValue());
   
            System.out.format(" Optimization performed in %d iterations \n",optimum.getEvaluations());
            System.out.format(" RMSE: %f \n",optimum.getRMS());

            // check estimated values
            double estimatedRoll = rugged.getLineSensor(pleiadesViewingModel.getSensorName()).
                                          getParametersDrivers().
                                          filter(driver -> driver.getName().equals("line_roll")).
                                          findFirst().get().getValue();
            double rollError = (estimatedRoll - rollValue);
            System.out.format("Estimated roll %3.5f roll error %3.6e  %n", estimatedRoll, rollError);

            double estimatedPitch = rugged.getLineSensor(pleiadesViewingModel.getSensorName()).
                                           getParametersDrivers().
                                           filter(driver -> driver.getName().equals("line_pitch")).
                                           findFirst().get().getValue();
            double pitchError = (estimatedPitch - pitchValue);
            System.out.format("Estimated pitch %3.5f pitch error %3.6e  %n ", estimatedPitch, pitchError);
            
            double estimatedFactor = rugged.getLineSensor(pleiadesViewingModel.getSensorName()).
                    getParametersDrivers().
                    filter(driver -> driver.getName().equals("line_factor")).
                    findFirst().get().getValue();
            double factorError = (estimatedFactor - factorValue);
            System.out.format("Estimated factor %3.5f factor error %3.6e  %n ", estimatedFactor, factorError);
            
            System.out.format(" **** Compute Statistics **** %n");

            LocalisationMetrics localisationResiduals = new LocalisationMetrics(measure.getMapping(),rugged, false);
            System.out.format("residuals max :  %3.4e mean %3.4e  meters %n",localisationResiduals.getMaxResidual(),localisationResiduals.getMeanResidual());
            LocalisationMetrics localisationResidualsDeg = new LocalisationMetrics(measure.getMapping(),rugged, true);
            System.out.format("residuals max  :  %3.4e deg mean %3.4e  deg %n",localisationResidualsDeg.getMaxResidual(),localisationResidualsDeg.getMeanResidual());
            
            
        } catch (OrekitException oe) {
            System.err.println(oe.getLocalizedMessage());
            System.exit(1);
        } catch (RuggedException re) {
            System.err.println(re.getLocalizedMessage());
            System.exit(1);
        }

    }



}
