<!--- Copyright 2013-2014 CS Systèmes d'Information
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

# Rugged initialization and direct location

This tutorial explains how to initialize Rugged and use it to geolocate a satellite image.
Let's imagine the sensor is a single line imager with 2000 pixels and 20° field of view,
oriented 10° off nadir. GPS and AOCS auxiliary data are available and provide us with a
list of positions, velocities and attitude quaternions recorded during the acquisition. By
passing all this information to Rugged, we will be able to precisely locate each point of
the image on the Earth. Well, not exactly precise, as this first tutorial does not use a
Digital Elevation Model, but considers the Earth as an ellipsoid. The DEM will be added in
a second tutorial: [Direct location with a DEM](direct-location-with-DEM.html). The objective
here is limited to explain how to initialize everything Rugged needs to know about the sensor
and the acquisition.   


## Sensor's definition


Let's start by defining the imager. The sensor model is described by its viewing geometry
(i.e. the line of sight of each physical pixel) and by its datation model.

### Line of sight


We need to define the line of sight (LOS) vector coordinates of each sensor pixel in the
satellite frame; X axis is parallel to the velocity vector in the absence of steering, Z is
pointing towards the Earth and Y is such that X,Y,Z forms a right-handed coordinate system. 

For this we need the following packages

    import java.util.ArrayList;
    import java.util.List;
    import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
    import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
    import org.apache.commons.math3.util.FastMath;
    import org.orekit.rugged.los.LOSBuilder;
    import org.orekit.rugged.los.FixedRotation;
    import org.orekit.rugged.los.TimeDependentLOS;
 

The raw viewing direction of pixel i with respect to the instrument is defined by the vector:

    List<Vector3D> rawDirs = new ArrayList<Vector3D>();
    for (int i = 0; i < 2000; i++) {
        //20° field of view, 2000 pixels
        rawDirs.add(new Vector3D(0d, i*FastMath.toRadians(20)/2000d, 1d));
    }


The instrument is oriented 10° off nadir around the X-axis, we need to rotate the viewing
direction to obtain the line of sight in the satellite frame

    LOSBuilder losBuilder = new LOSBuilder(rawDirs);
    losBuilder.addTransform(new FixedRotation(new Rotation(Vector3D.PLUS_I, FastMath.toRadians(10))));

Here we have considered that the viewing directions are constant with time, it is also possible to
have time-dependent lines-of-sight by using other transforms. It is also possible to append several
transforms between the raw directions and the final lines-of-sight.

    TimeDependentLOS lineOfSight = losBuilder.build();

### Datation model

The datation model gives the line number in the image as a function of time, and vice-versa the
time as a function of the line number. Here we are using a linear datation model (line sampling
rate is constant)


We use Orekit for handling time and dates, and Rugged for defining the datation model:

    import org.orekit.time.AbsoluteDate;
    import org.orekit.time.TimeScalesFactory;
    import org.orekit.rugged.linesensor.LinearLineDatation;
    AbsoluteDate absDate = new AbsoluteDate("2009-12-11T10:49:55.899994", TimeScalesFactory.getGPS());
    LinearLineDatation lineDatation = new LinearLineDatation(absDate, 1d, 20); 

The LinearLineDataion class is instanciated with 3 arguments: the first is the reference date,
the second is the line number at the reference date, and the last argument is the line rate (20
lines per second in this example) . 

Note that Orekit can handle many different time scales through TimeScalesFactory, so we do not
need to worry about conversion.

### Line sensor

With the LOS and the datation now defined , we can initialize a line sensor object in Rugged:

    import org.orekit.rugged.linesensor.LineSensor;
    LineSensor lineSensor = new LineSensor("mySensor", lineDatation, Vector3D.ZERO, lineOfSight);

The first parameter is the nickname of the sensor. It is necessary because we can define multiple
sensors (useful for an imager with several spectral channels or detector arrays). The third argument
is the sensor position relative to the centre of the satellite reference frame. Here it is set to 0. 


## Satellite position, velocity and attitude


As input to Rugged, we will need to pass sufficient information to describe the position and
orientation of the platform during the acquisition. In our example, the list of positions, velocities
and quaternions are hard-coded. In real life, we would extract GPS and AOCS data from the satellite
auxiliary telemetry.

Note that for simulation purpose, we could also use Orekit to simulate the orbit. It offers very
convenient functions to propagate sun-synchronous orbits with yaw steering compensation (typical
orbits for Earth Observation satellites).

### Reference frames


Rugged expects the positions (unit: m), velocities (unit: m/s) and quaternions to be expressed in an inertial frame.
All standard inertial and terrestrial frames are implemented in Orekit, so we just need to specify
which frames we are using and convert if necessary. 

Conversion from inertial to Earth-rotating frame is transparent to the user and is based on the most
recent precession/nutation model on top of which corrections published by the IERS are applied. IERS
bulletins and other physical data are provided within the orekit-data folder. There are several ways
to configure Orekit to use this data. More information is given
[here](https://www.orekit.org/forge/projects/orekit/wiki/Configuration).

In our application, we simply need to know the name of the frames we are working with. Positions and
velocities are given in the ITRF terrestrial frame, while the quaternions are given in EME2000
inertial frame.  

    import org.orekit.frames.Frame;
    import org.orekit.frames.FramesFactory;
    import org.orekit.utils.IERSConventions;
    Frame eme2000 = FramesFactory.getEME2000();
    boolean simpleEOP = true; // we don't want to compute tiny tidal effects at millimeter level
    Frame itrf = FramesFactory.getITRF(IERSConventions.IERS_2010, simpleEOP);

### Satellite attitude


The attitude quaternions are grouped in a list of TimeStampedAngularCoordinates objects: 

    import org.orekit.utils.TimeStampedAngularCoordinates;
    List<TimeStampedAngularCoordinates> satelliteQList = new ArrayList<TimeStampedAngularCoordinates>();

Each attitude sample (quaternion, time) is added to the list, 

    AbsoluteDate attitudeDate = new AbsoluteDate(gpsDateAsString, TimeScalesFactory.getGPS());
    Rotation rotation = new Rotation(q0, q1, q2, q3, true); // q0 is the scalar term
    Vector3D rotationRate = Vector3D.ZERO;
    Vector3D rotationAcceleration = Vector3D.ZERO;
    TimeStampedAngularCoordinates pair = new TimeStampedAngularCoordinates(attitudeDate, rotation, rotationRate, rotationAcceleration); 
    satelliteQList.add(pair);

where, for instance, gpsDateAsString is set to "2009-12-11T10:49:55.899994"

### Position and velocities


Similarly the positions and velocities will be set in a list of `TimeStampedPVCoordinates`. Before being
added to the list, they must be transformed to EME2000: 

    import org.orekit.utils.TimeStampedPVCoordinates;
    import org.orekit.utils.PVCoordinates;
    import org.orekit.frames.Transform;
    List<TimeStampedPVCoordinates> satellitePVList = new ArrayList<TimeStampedPVCoordinates>();
    AbsoluteDate ephemerisDate = new AbsoluteDate(gpsDateAsString, TimeScalesFactory.getGPS());
    Vector3D position = new Vector3D(px, py, pz); // in ITRF, unit: m 
    Vector3D velocity = new Vector3D(vx, vy, vz); // in ITRF, unit: m/s
    PVCoordinates pvITRF = new PVCoordinates(position, velocity);
    Transform transform = itrf.getTransformTo(eme2000, ephemerisDate);
    PVCoordinates pvEME2000 = transform.transformPVCoordinates(pvITRF); 
    satellitePVList.add(new TimeStampedPVCoordinates(ephemerisDate, pvEME2000.getPosition(), pvEME2000.getVelocity(), Vector3D.ZERO)));

## Rugged initialization


Finally we can initialize Rugged. It looks like this:

    import org.orekit.rugged.api.AlgorithmId;
    import org.orekit.rugged.api.BodyRotatingFrameId;
    import org.orekit.rugged.api.EllipsoidId;
    import org.orekit.rugged.api.InertialFrameId;
    import org.orekit.rugged.api.Rugged;
    import org.orekit.rugged.errors.RuggedException;
    import org.orekit.utils.AngularDerivativesFilter;
    import org.orekit.utils.CartesianDerivativesFilter;
    import org.orekit.utils.IERSConventions;
    Rugged rugged = new RuggedBuilder().
                    setAlgorithm(demAlgoId). 
                    setDigitalElevationModel(demTileUpdater, nbTiles).
                    setEllipsoid(EllipsoidId.WGS84, BodyRotatingFrameId.ITRF).
                    setTimeSpan(acquisitionStartDate, acquisitionStopDate, tStep, timeTolerance). 
                    setTrajectory(InertialFrameId.EME2000,
                                  satellitePVList, nbPVPoints, CartesianDerivativesFilter.USE_P,
                                  satelliteQList,  nbQPoints,  AngularDerivativesFilter.USE_R).
                    addLineSensor(lineSensor).
                    build();

Argh, that sounds complicated. It is not so difficult since we have already defined most of what is
needed. Let's describe the Rugged instance building process line by line:

As the Rugged top level object that will be used for all user interaction is quite involved and can be
built in several different ways, a [builder pattern](https://en.wikipedia.org/wiki/Builder_pattern)
approach has been adopted. The builder itself is configured by calling dedicated setters for each
concept (Digital Elevation Model, intersection algorithm, trajectory, ...).
As all these concepts can be chained together, the setters themselves implement the
[fluent interface](https://en.wikipedia.org/wiki/Fluent_interface) pattern, which implies each setter
returns the builder instance, and therefore another setter can be called directly.

The *setAlgorithm* setter specifies the intersection algorithm to use. As this tutorial is intended to be very
simple for a beginning, we choose to use directly the ellipsoid and not a real Digital Elevation Model,
so we can use `AlgorithmId.IGNORE_DEM_USE_ELLIPSOID` as the single parameter of this setter.

The *setDigitalElevationModel* setter specifies the Digital Elevation Model. In fact, as we decided to ignore the Digital
Elevation Model in this tutorial, we could have omitted this call and it would have worked correctly.
We preferred to let it in so users do not forget to set the Digital Elevation Model for intersection
algorithms that really use them. As the model will be ignored, we can put the parameters for this
setter to `null` and `0`. Of course if another algorithm had been chosen,  null parameters would clearly
not work, this is explained in another tutorial: [[DirectLocationWithDEM|Direct location with a DEM]].

The *setEllipsoid* setter defines the shape and orientation of the ellipsoid. We use simple predefined enumerates:
`EllipsoidId.WGS84`, `InertialFrameId.EME2000`, but could also use a custom ellipsoid if needed.

The *setTimeSpan* setter is used to define the global time span of the search algorithms (direct and inverse
location). Four parameters are used for this: acquisitionStartDate, acquisitionStopDate,
tStep (step at which the pre-computed frames transforms cache will be filled), timeTolerance (margin
allowed for extrapolation during inverse location, in seconds. The tStep parameter is a key parameter
to achieve good performances as frames transforms will be precomputed throughout the time span using
this time step. These computation are costly because they involve Earth precession/nutation models to
be evaluated. So the transformed are precomputed and cached instead of being recomputed for each pixel.
However, if direct and inverse location are expected to be performed over a reduced time span (say a few
tens of seconds), precomputing the transforms over half an orbit at one millisecond rate would be a
waste of computing power. Typical values are therefore to restrict the time span as much as possible
to properly cover the expected direct and inverse location calls, and to use a step between one millisecond
and one second, depending on the required accuracy. The exact value to use is mission-dependent. The
final timeTolerance parameter is simply a margin used before and after the final precomputed transforms to
allow a slight extrapolation if during a search the interval is slightly overshoot. A typical value is
to allow a few images lines so for example a 5 lines tolerance would imply computing the tolerance as:
timeTolerance = 5 / lineSensor.getRate(0)).
`BodyRotatingFrameId.ITRF`

The *setTrajectory* setter defines the spacecraft evolution. The arguments are the list of time-stamped positions and
velocities as well as the inertial frame with respect to which they are defined and options for interpolation:
number of points to use and type of filter for derivatives. The interpolation polynomials for nbPVPoints
without any derivatives (case of CartesianDerivativesFilter.USE_P: only positions are used, without velocities)
have a degree nbPVPoints - 1. In case of computation with velocities included (case of
CartesianDerivativesFilter.USE_PV), the interpolation polynomials have a degree 2*nbPVPoints - 1. If the
positions/velocities data are of good quality and separated by a few seconds, one may choose only a few points
but interpolate with both positions and velocities; in other cases, one may choose more points but interpolate
only with positions. We find similar arguments for the attitude quaternions. 

The last setter used, *addLineSensor*, registers a line sensor. As can be deduced from its prefix (`add` instead of `set`), it
can be called several time to register several sensors that will all be available in the built Rugged instance.
We have called the method only once here, so we will use only one sensor.

After the last setter has been called, we call the `build()` method which really build the Rugged instance
(and not a RuggedBuilder instance has the setter did).

Rugged takes into account by default some corrections for more accurate locations: 

* light time correction (compensates or not light time between ground and spacecraft). 
* aberration of light correction (compensates or not aberration of light, which is velocity composition between light and spacecraft when the light from ground points reaches the sensor).

Not compensating the delay or the velocity composition are mainly useful for validation purposes against system
that do not compensate it. When the pixels line of sight already includes the aberration of light correction,
one must obviously deactivate the correction.

If those corrections should be ignored, some other setters must be inserted before the call to `build()`:

    setXxxx().
    setLightTimeCorrection(false).
    setAberrationOfLightCorrection(false).
    build();

The various setters can be called in any order. The only important thing is that once a Rugged instance
has been created by calling the `build()` method, it is independent from the builder so later calls
to setters will not change the build instance. In fact, it is possible to create a builder, then
call its `build()` method to create a first Rugged instance, and then to modify the builder configuration
by calling again some of the setters and building a second Rugged instance from the new configuration.
This allows to perform comparisons between two different configurations in the same program and without
having to recreate everything. For instance, one can procede in three steps like this:

    RuggedBuilder ruggedBuilder = new RuggedBuilder().
                                  setAlgorithm(xxxx). 
                                  [ ... some setters ...]
                                  setTrajectory(xxxx);

Then add the desired sensor(s):

    ruggedBuilder.addLineSensor(lineSensor);

And create the Rugged instance:

    Rugged rugged = ruggedBuilder.build();
   

## Direct location

Finally everything is set to do some real work. Let's try to locate a point on Earth 
for upper left point (first line, first pixel): 

    import org.orekit.bodies.GeodeticPoint;
    Vector3D position = lineSensor.getPosition(); // This returns a zero vector since we set the relative position of the sensor w.r.T the satellite to 0.
    AbsoluteDate firstLineDate = lineSensor.getDate(0);
    Vector3D los = lineSensor.getLos(firstLineDate, 0);
    GeodeticPoint upLeftPoint = rugged.directLocation(firstLineDate, position, los);

## Source code 

The source code is available in DirectLocation.java
