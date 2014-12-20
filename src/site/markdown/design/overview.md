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

Global architecture
-------------------

Rugged is an intermediate level mission-independent library. It relies on
the Orekit library and on the Apache Commons Math library. It is itself
intended to be used from a mission-specific interface by one or more
image processing applications.

![architecture](../images/rugged-architecture.png)

The Java platform provides the runtime environment, the Apache Commons
Math library provides the mathematical algorithms (3D geometry, root
solvers ...), the Orekit library provides the space flight dynamics
computation (frames transforms, orbits and attitude propagation and
interpolation ...). The Rugged library itself provides the algorithms
dealing with line-of-sight intersection with Digital Elevation Models
in a mission-independent way. Rugged does not parse the DEM models itself,
nor does it performs image processing. Mission-dependent parts (including
Digital Elevation Model parsing or instrument viewing model creation) remain
under the responsibility of Rugged caller, typically using a mission-specific
library used by several image processing applications.

This architecture allows both the image processing application and the mission
specific interface to be as independent as possible from space flight dynamics and
geometry. These parts can therefore be implemented by image processing specialists.
The application itself can even be written in a programming language as C++, there is
no need for the Java language at this level. It is expected that the mission specific
interface is written using the Java language to simplify data exchanges with the lower
layers and avoid complex data conversion. Data conversion is performed only between the
image processing application and the interface layer, and is limited to very few high
level functions with few primitive types (raw arrays for pixels or ground coordinates).

The Rugged library is developed in the Java language and has full access to the Orekit and
Apache Commons Math libraries. It is designed and developed by space flight dynamics and
geometry specialists, with support from the image processing specialists for the API definition.

Functional Breakdown
--------------------

The following table sorts out the various topics between the various layers.

|          Topic                   |           Layer         |                                                      Comment
|----------------------------------|-------------------------|-----------------------------------------------------------------------------
|  Sensor to ground mapping        |           Rugged        |                          Direct location is the base feature provided
|  Ground to sensor mapping        |           Rugged        |                       Inverse location is another base feature provided
|     Individual pixels            |           Rugged        |The API supports any number of pixels, defined by their individual line of sight provided by caller
|        Optical path              |         Interface       |The folded optical path inside the spacecraft is taken into account by computing an overall transform combining all inside reflections, so each pixel position and line of sight can be computed later on by a single translation and rotation with respect to spacecraft center of mass
|    Line time-stamping            |     Interface/Rugged    |The caller must provide a simple time-stamping model (typically linear) that will be applied
|Orbit and attitude interpolation  |          Orekit         |Both simple interpolation from timestamped position samples and full orbit propagation are available, thanks to Orekit streamlined propagator architecture
|CCSDS Orbit/Attitude file parsing |          Orekit         |This is supported as long as standard CCSDS Orbit Data Message (CCSDS 502.0-B-2) and CCSDS Attitude Data Messages (CCSDS 504.0-B-1) are used
|Custom Orbit/Attitude file parsing|        Interface        |Custom files can be loaded by mission specific readers, and the list or orbit/attitude states can be provided to Orekit which is able to handle interpolation from these sample data
|       Frames transforms          |          Orekit         |Full support to all classical reference inertial and Earth frames is already provided by Orekit (including the legacy EME2000, MOD, TOD, but also the more modern GCRF, ICRF, TIRF or exotic frames like TEME or Veis1950, as well as several versions of ITRF)
|      IERS data correction        |          Orekit         |All frame transforms support the full set of IERS Earth Orientation Parameters corrections, including of course the large DUT1 time correction, but also the smaller corrections to older IAU-76/80 or newer IAU-2000/2006 precession nutation models as well as the polar wander. The frames level accuracy is at sub-millimeter level
|     Grid-post elevation model    |          Rugged         |Only raster elevation models are supported
|Triangulated Irregular Network elevation model | Not supported |If vector elevation models are needed, they must be converted to raster form in order to be used
|         Geoid computation        |     Not in version 1    |The first version only supports Digital Elevation Models computed with respect to a reference ellipsoid. If needed, this feature could be added after version 1, either at Rugged or Orekit level, using Orekit gravity fields
|  Time-dependent deformations     |     Interface/Rugged    |Simple line-of-sight models (typically polynomial) can be used
|           Calibration            |Image processing or interface|The calibration phase remains at the mission-specific caller level (pixels geometry, clock synchronization …), the caller is required to provide the already calibrated line of sights
|         DEM file parsing         |         Interface       |The elevation models are dedicated to the mission and there are several formats (DTED, GeoTIFF, raw data …).Rugged only deals with raw elevation on small latitude/longitude cells
