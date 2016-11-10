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
import org.orekit.rugged.linesensor.SensorPixel;
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
            File orekitData = new File(home, "workspace/data/orekit-data");
            //File orekitData = new File(home, "COTS/orekit-data");
            DataProvidersManager.getInstance().addProvider(new DirectoryCrawler(orekitData));

            

            //create 2 Pleiades Viewing Model
            final String SensorNameA = "SensorA";
            final double incidenceAngleA = -0.0;
            final String dateA = "2016-01-01T11:59:50.0";
            PleiadesViewingModel pleiadesViewingModelA = new PleiadesViewingModel(SensorNameA,incidenceAngleA,dateA);
            final AbsoluteDate minDateA =  pleiadesViewingModelA.getMinDate();
            final AbsoluteDate maxDateA =  pleiadesViewingModelA.getMaxDate();
            final AbsoluteDate refDateA = pleiadesViewingModelA.getDatationReference();

            //CreateOrbitA
            OrbitModel orbitmodelA =  new OrbitModel();
            BodyShape earthA = orbitmodelA.createEarth();
            NormalizedSphericalHarmonicsProvider gravityFieldA = orbitmodelA.createGravityField();
            Orbit      orbitA  = orbitmodelA.createOrbit(gravityFieldA.getMu(), refDateA);
            
            final double [] rollPoly = {0.0,0.0,0.0};
            double[] pitchPoly = {0.0,0.0};
            double[] yawPoly = {0.0,0.0,0.0};
            orbitmodelA.setLOFTransform(rollPoly, pitchPoly, yawPoly, minDateA);
            
            PVCoordinates PVA = orbitA.getPVCoordinates(earthA.getBodyFrame());
            GeodeticPoint gpA = earthA.transform(PVA.getPosition(), earthA.getBodyFrame(), orbitA.getDate());
            
            System.out.format(Locale.US, "Geodetic Point at date %s : φ = %8.10f °, λ = %8.10f %n",orbitA.getDate().toString(),
                              FastMath.toDegrees(gpA.getLatitude()),
                              FastMath.toDegrees(gpA.getLongitude()));                       
            
            
            //create 2 Pleiades Viewing Model
            final String SensorNameB = "SensorB";
            final double incidenceAngleB = +0.0;
            final String dateB = "2016-01-01T12:00:10.0";
            PleiadesViewingModel pleiadesViewingModelB = new PleiadesViewingModel(SensorNameB,incidenceAngleB,dateB);
            final AbsoluteDate minDateB =  pleiadesViewingModelB.getMinDate();
            final AbsoluteDate maxDateB =  pleiadesViewingModelB.getMaxDate();
            final AbsoluteDate refDateB = pleiadesViewingModelB.getDatationReference();

            //CreateOrbitA
            OrbitModel orbitmodelB =  new OrbitModel();
            BodyShape earthB = orbitmodelB.createEarth();
            NormalizedSphericalHarmonicsProvider gravityFieldB = orbitmodelB.createGravityField();
            Orbit      orbitB  = orbitmodelB.createOrbit(gravityFieldB.getMu(), refDateB);
            

            orbitmodelB.setLOFTransform(rollPoly, pitchPoly, yawPoly, minDateB);
            
            PVCoordinates PVB = orbitB.getPVCoordinates(earthB.getBodyFrame());
            GeodeticPoint gpB = earthB.transform(PVB.getPosition(), earthB.getBodyFrame(), orbitB.getDate());
            
            System.out.format(Locale.US, "Geodetic Point at date %s : φ = %8.10f °, λ = %8.10f %n",orbitA.getDate().toString(),
                              FastMath.toDegrees(gpB.getLatitude()),
                              FastMath.toDegrees(gpB.getLongitude()));                       
            
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
                    2, AngularDerivativesFilter.USE_R);                  
            Rugged ruggedA = ruggedBuilderA.build();
                       
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
                    2, AngularDerivativesFilter.USE_R);                  
            Rugged ruggedB = ruggedBuilderB.build();
   

            
            
            
            
            
            
            
            Vector3D positionA = lineSensorA.getPosition(); // This returns a zero vector since we set the relative position of the sensor w.r.T the satellite to 0.
                 
            AbsoluteDate lineDateA = lineSensorA.getDate(pleiadesViewingModelA.dimension/2);
            Vector3D losA = lineSensorA.getLOS(lineDateA,pleiadesViewingModelA.dimension/2);
            GeodeticPoint centerPointA = ruggedA.directLocation(lineDateA, positionA, losA);
            System.out.format(Locale.US, "center geodetic position : φ = %8.10f °, λ = %8.10f °, h = %8.3f m%n",
                              FastMath.toDegrees(centerPointA.getLatitude()),
                              FastMath.toDegrees(centerPointA.getLongitude()),centerPointA.getAltitude());      
            
            
            Vector3D positionB = lineSensorB.getPosition(); // This returns a zero vector since we set the relative position of the sensor w.r.T the satellite to 0.
            
            AbsoluteDate lineDateB = lineSensorB.getDate(pleiadesViewingModelB.dimension/2);
            Vector3D losB = lineSensorB.getLOS(lineDateB,pleiadesViewingModelB.dimension/2);
            GeodeticPoint centerPointB = ruggedB.directLocation(lineDateB, positionB, losB);
            System.out.format(Locale.US, "center geodetic position : φ = %8.10f °, λ = %8.10f °, h = %8.3f m%n",
                              FastMath.toDegrees(centerPointB.getLatitude()),
                              FastMath.toDegrees(centerPointB.getLongitude()),centerPointB.getAltitude());      
            

            lineDateB = lineSensorB.getDate(0);
            losB = lineSensorB.getLOS(lineDateB,0);
            GeodeticPoint firstPointB = ruggedB.directLocation(lineDateB, positionB, losB);
            System.out.format(Locale.US, "first geodetic position : φ = %8.10f °, λ = %8.10f °, h = %8.3f m%n",
                              FastMath.toDegrees(firstPointB.getLatitude()),
                              FastMath.toDegrees(firstPointB.getLongitude()),firstPointB.getAltitude());      
            
            lineDateB = lineSensorB.getDate(pleiadesViewingModelB.dimension-1);
            losB = lineSensorB.getLOS(lineDateB,pleiadesViewingModelB.dimension-1);
            GeodeticPoint lastPointB = ruggedB.directLocation(lineDateB, positionB, losB);
            System.out.format(Locale.US, "last geodetic position : φ = %8.10f °, λ = %8.10f °, h = %8.3f m%n",
                              FastMath.toDegrees(lastPointB.getLatitude()),
                              FastMath.toDegrees(lastPointB.getLongitude()),lastPointB.getAltitude());      
            
            
            
            
            
            
            
            
            
            double distance = DistanceTools.computeDistanceRad(centerPointA.getLongitude(), centerPointA.getLatitude(),
                                                               centerPointB.getLongitude(), centerPointB.getLatitude());
            
            System.out.format("distance %f meters %n",distance);
            
            
            double rollValueA =  FastMath.toRadians(0.6);
            double pitchValueA = FastMath.toRadians(0.0);
            double factorValue = 1.00;
                        
            System.out.format("roll : %3.5f pitch : %3.5f factor : %3.5f \n",rollValueA,pitchValueA,factorValue);
                        
            ruggedA.
            getLineSensor(SensorNameA).
            getParametersDrivers().
            filter(driver -> driver.getName().equals("roll")).
            findFirst().get().setValue(rollValueA);

            ruggedA.
            getLineSensor(SensorNameA).
            getParametersDrivers().
            filter(driver -> driver.getName().equals("pitch")).
            findFirst().get().setValue(pitchValueA);
            
            ruggedA.
            getLineSensor(SensorNameA).
            getParametersDrivers().
            filter(driver -> driver.getName().equals("factor")).
            findFirst().get().setValue(factorValue);
            
            
            lineDateA = lineSensorA.getDate(pleiadesViewingModelA.dimension/2);
            losA = lineSensorA.getLOS(lineDateA,pleiadesViewingModelA.dimension/2);
            centerPointA = ruggedA.directLocation(lineDateA, positionA, losA);
            System.out.format(Locale.US, "center geodetic position : φ = %8.10f °, λ = %8.10f °, h = %8.3f m%n",
                              FastMath.toDegrees(centerPointA.getLatitude()),
                              FastMath.toDegrees(centerPointA.getLongitude()),centerPointA.getAltitude());      
            
            
            distance = DistanceTools.computeDistanceRad(centerPointA.getLongitude(), centerPointA.getLatitude(),
                                                               centerPointB.getLongitude(), centerPointB.getLatitude());
            
            System.out.format("distance %f meters %n",distance);
            
            // Search the sensor pixel seeing point
            int minLine = 0;
            int maxLine = pleiadesViewingModelB.dimension-1;
            SensorPixel sensorPixelB = ruggedB.inverseLocation(SensorNameB, centerPointA, minLine, maxLine);
            // we need to test if the sensor pixel is found in the prescribed lines otherwise the sensor pixel is null
            if (sensorPixelB != null){
                System.out.format(Locale.US, "Sensor Pixel found : line = %5.3f, pixel = %5.3f %n", sensorPixelB.getLineNumber(), sensorPixelB.getPixelNumber());
            } else {
                System.out.println("Sensor Pixel is null: point cannot be seen between the prescribed line numbers\n");
            }

            
            //add mapping
            
            SensorToSensorMeasureGenerator measure = new SensorToSensorMeasureGenerator(pleiadesViewingModelA,ruggedA,pleiadesViewingModelB,ruggedB);
            int lineSampling = 1000;
            int pixelSampling = 1000;       
            measure.CreateMeasure(lineSampling, pixelSampling); 
            System.out.format("nb TiePoints %d %n", measure.getMeasureCount());
            
            

        } catch (OrekitException oe) {
            System.err.println(oe.getLocalizedMessage());
            System.exit(1);
        } catch (RuggedException re) {
            System.err.println(re.getLocalizedMessage());
            System.exit(1);
        }

    }



}
