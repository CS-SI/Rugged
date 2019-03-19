<!--- Copyright 2013-2019 CS Systèmes d'Information
  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at
  
    http://www.apache.org/licenses/LICENSE-2.0
  
  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
-->

<a name="top"></a>

# Inverse Location

The aim of this tutorial is to compute the inverse location of a point on Earth 
in order to give the sensor pixel, with the associated line, seeing this point.

We will also explain how to find the date at which sensor sees a ground point, 
which is a kind of inverse location only focusing on date.

## Inverse location of a point on Earth
The initialization of Rugged is similar as in the [Direct location](./direct-location.html) tutorial up to rugged initialization..

### Point defined by its latitude, longitude and altitude
Once Rugged initialized, one can compute the line number and the pixel number of a point defined by its Geodetic coordinates:

    import org.orekit.bodies.GeodeticPoint;
    import org.orekit.rugged.linesensor.SensorPixel;
    GeodeticPoint gp = new GeodeticPoint(latitude, longitude, altitude);
    SensorPixel sensorPixel = rugged.inverseLocation(sensorName, gp, minLine, maxLine);

where minLine (maxLine, respectively) is the minimum line number for the search interval (maximum line number, respectively). 

The inverse location will give the sensor pixel number and the associated line number 
seeing the point on ground. 
**In case the point cannot be seen between the prescribed line numbers, the return result is null. 
No exception will be thrown in this particular case**.
   
### Point defined by its latitude and longitude (no altitude)
Similarly, one can compute the line number and the pixel number of a point defined solely 
by its latitude en longitude. The altitude will be determined automatically with the DEM.

     SensorPixel sensorPixel = rugged.inverseLocation(sensorName, latitude, longitude, minLine, maxLine);

## Date location 
Once Rugged initialized, one can compute the date at which sensor sees a point on Earth.

### Point defined by its latitude, longitude and altitude
For a point defined by its Geodetic coordinates:

     AbsoluteDate dateLine = rugged.dateLocation(sensorName, gp, minLine, maxLine);

### Point defined by its latitude and longitude (no altitude)
Similarly, for a point defined solely by its latitude en longitude (altitude determined automatically with the DEM): 

     AbsoluteDate dateLine = rugged.dateLocation(sensorName, latitude, longitude, minLine, maxLine);

## Determine the min/max lines interval
Rugged provides a way to determine a **very** rough estimation of the line using only 
the position-velocities of the satellite. It assumes the position-velocities are regular enough and without holes.

     OneAxisEllipsoid oneAxisEllipsoid = ruggedBuilder.getEllipsoid();
     Frame pvFrame = ruggedBuilder.getInertialFrame();
     RoughVisibilityEstimator roughVisibilityEstimator= new RoughVisibilityEstimator(oneAxisEllipsoid, pvFrame, satellitePVList);

One can compute the approximated line with the rough visibility estimator:

     AbsoluteDate roughLineDate = roughVisibilityEstimator.estimateVisibility(gp);
     double roughLine = lineSensor.getLine(roughLineDate);

The result will never be null, but may be really far from reality if ground point is away from trajectory.
With this rough line, taken some margin around (for instance 100), one can initialize 
the min/max lines as search boundaries for inverse location, taken into account sensor min and max lines:

     int minLineRough = (int) FastMath.max(FastMath.floor(roughLine - margin), sensorMinLine);
     int maxLineRough = (int) FastMath.min(FastMath.floor(roughLine + margin), sensorMaxLine);

then one can compute the inverse location:

     SensorPixel sensorPixel = rugged.inverseLocation(sensorName, gp, minLineRough, maxLineRough);

## Source code
The source code is available in InverseLocation.java (package fr.cs.examples under src/tutorials)

[Top of the page](#top)
