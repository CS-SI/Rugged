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
a second tutorial [Direct location with DEM](direct-location-with-DEM.html). The objective here is limited to explain how to initialize everything
Rugged needs to know about the sensor and the acquisition.   


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
    import org.orekit.rugged.api.TimeDependentLOS;
    import org.orekit.rugged.api.FixedLOS;


The viewing direction of pixel i with respect to the instrument is defined by the vector:

    Vector3D viewDir = new Vector3D(0d, i*FastMath.toRadians(20)/2000d, 1d).normalize(); //20° field of view, 2000 pixels


The instrument is oriented 10° off nadir around the X-axis, we need to rotate the viewing
direction to obtain the line of sight in the satellite frame

    Rotation rotation = new Rotation(Vector3D.PLUS_I, FastMath.toRadians(10));
    Vector3D los = rotation.applyTo(viewDir);

The viewing direction is the line of sight of one pixel. In the case of a single line sensor,
Rugged expects to receive an array of LOS. The LOS can be time dependent (useful when applying
time-dependent corrections to the viewing angles). Here we will suppose that the viewing
directions are constant with time. In consequence we can use the object `FixedLOS` which is
instantiated from the viewing direction vector.  

    List<TimeDependentLOS> lineOfSight = new ArrayList<TimeDependentLOS>();
    lineOfSight.add(new FixedLOS(los));

### Datation model

The datation model gives the line number in the image as a function of time, and vice-versa the
time as a function of the line number. Here we are using a linear datation model (line sampling
rate is constant)


We use Orekit for handling time and dates, and Rugged for defining the datation model:

    import org.orekit.time.AbsoluteDate;
    import org.orekit.time.TimeScalesFactory;
    import org.orekit.rugged.api.LinearLineDatation;
    AbsoluteDate absDate = new AbsoluteDate("2009-12-11T10:49:55.899994", TimeScalesFactory.getGPS());
    LinearLineDatation lineDatation = new LinearLineDatation(absDate, 1d, 20); 

The LinearLineDataion class is instanciated with 3 arguments: the first is the reference date,
the second is the line number at the reference date, and the last argument is the line rate (20
lines per second in this example) . 

Note that Orekit can handle many different time scales through TimeScalesFactory, so we do not
need to worry about conversion.

### Line sensor

With the LOS and the datation now defined , we can initialize a line sensor object in Rugged:

    import org.orekit.rugged.api.LineSensor;
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
    import org.orekit.rugged.api.RuggedException;
    import org.orekit.utils.AngularDerivativesFilter;
    import org.orekit.utils.CartesianDerivativesFilter;
    import org.orekit.utils.IERSConventions;
    Rugged rugged = new Rugged (demTileUpdater, nbTiles,  demAlgoId, 
                                EllipsoidId.WGS84, InertialFrameId.EME2000, BodyRotatingFrameId.ITRF,
                                acquisitionStartDate, acquisitionStopDate, tStep, timeTolerance,
                                satellitePVList, nbPVPoints, CartesianDerivativesFilter.USE_P,
                                satelliteQList, nbQPoints, AngularDerivativesFilter.USE_R);

Argh, that sounds complicated. It is not so difficult since we have already defined most of what is
needed. Let's describe the arguments line by line:
 
The first 3 are related to the DEM. Since we are ignoring the DEM, they can be set respectively to
`null`, `0` and `AlgorithmId.IGNORE_DEM_USE_ELLIPSOID`.

The next 3 arguments define the ellipsoid and the reference frames: `EllipsoidId.WGS84`,
`InertialFrameId.EME2000`, `BodyRotatingFrameId.ITRF`

On the third line, we find the time interval of the acquisition: acquisitionStartDate, acquisitionStopDate,
tStep (step at which the pre-computed frames transforms cache will be filled), timeTolerance (margin
allowed for extrapolation during inverse location, in seconds. A few lines must be taken into account like: timeTolerance = 5/lineSensor.getRate(0)). This is an important information as Rugged
will pre-compute a lot of stuff at initialization in order to optimize further calls to the direct and
inverse location functions. 

On the fourth line, the arguments are the list of time-stamped positions and velocities as well as options
for interpolation: number of points to use and type of filter for derivatives. The interpolation polynomials for nbPVPoints without any derivatives (case of CartesianDerivativesFilter.USE_P: only positions are used, without velocities) have a degree nbPVPoints - 1. In case of computation with velocities included (case of CartesianDerivativesFilter.USE_PV), the interpolation polynomials have a degree 2*nbPVPoints - 1. If the positions/velocities data are of good quality and spaced by a few seconds, one may choose only a few points but interpolate with both positions and velocities; in other cases, one may choose more points but interpolate only with positions.
We find the same arguments on the last line for the attitude quaternions. 

Rugged takes into account by default some corrections for more accurate locations: 

* light time correction (compensates or not light time between ground and spacecraft). 
* aberration of light correction (compensates or not aberration of light, which is velocity composition between light and spacecraft when the light from ground points reaches the sensor).

Not compensating the delay or the velocity composition are mainly useful for validation purposes against system that do not compensate it. When the pixels line of sight already includes the aberration of light correction, one must obviously deactivate the correction.
In order not to take into account those corrections, one must finalize Rugged initialization by setting the related flags to false:

    rugged.setLightTimeCorrection(false);

or

    rugged.setAberrationOfLightCorrection(false);


The sensor models are added after initialization. We can add as many as we want. 

    rugged.addLineSensor(lineSensor);

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
