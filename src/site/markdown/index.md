<!--- Copyright 2013-2017 CS Systèmes d'Information
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

Overview
========

  Rugged is  a sensor-to-terrain mapping tool which takes into account Digital Elevation Models (DEM)
  in its line of sight computation. It is a free software
  intermediate-level library written in Java and implemented as an add-on for Orekit.

  ![Earth_FlatVsRugged.gif](src/site/resources/images/Earth_FlatVsRugged.gif)

  It mainly provides direct and inverse location, i.e. it allows
  to compute accurately which ground point is looked at from a specific
  pixel in a spacecraft instrument, and conversely which pixel will
  see a specified ground point. This mapping between ground and sensor
  is computed with a viewing model taking into account ground Digital
  Elevation Model, Earth rotation will all its tiny irregularities,
  on-board sensor pixels individual line-of-sights, spacecraft motion and
  attitude and several physical effects.

![RuggedExplained.png](src/site/resources/images/RuggedExplained.png)
 *Effects of taking into account the DEM in the computation of latitude, longitude and altitude*

  Direct and inverse location can be used to perform full ortho-rectification
  of images and correlation between sensors observing the same area.

Features
--------

  * Direct/inverse location

  * Refinement

  * can support several types of Digital Elevation Models, including user-provided models

  * several intersection models algorithms available

  * can propagate orbit by itself for preliminary mission analysis or data generation

  * can propagate attitude by itself for preliminary mission analysis or data generation

  * *very* fast

  * Both modern and legacy models for Earth rotation
    * Lieske (1976), Wahr (1980), 
    * Mathews, Herring, Buffett (2002)
    * Capitaine (2006)

  * complete set of corrections applied for greater accuracy
    * δΔψ, δΔε on precession nutation (about 3m correction since 2013, steadily increasing)
    * ΔUT₁, lod on proper rotation (can theoretically reach up to 400m)
    * u, v pole wander (polhody), (about 15m correction)
    * light time correction (about 1.2m)
    * aberration of light correction (about 20m)
    * line-of-sight curvature in geodetic coordinates,
      (0m at nadir, 10m at 30° dive angle, hundreds of meters for skimming los)
    * atmospheric refraction

  * not limited to Earth

  * highly portable (Linux, Windows, MacOSX, ...)

  * Localized in several languages

    * Danish
    * English
    * French
    * Galician
    * German
    * Italian
    * Norwegian
    * Romanian
    * Spanish

Free software
-------------

Rugged is freely available both in source and binary formats, with all related
documentation and tests.

It is distributed under the [Apache License Version 2.0](LICENSE.txt). This
is a well known business-friendly license. This means anybody can use it to build
any application, free or not. There are no strings attached to your own code.

Everybody is encouraged to use Rugged as a common intermediate level layer to improve
interoperability in space systems.

Maintained library
------------------

Rugged has been in development since 2014 inside [CS Systèmes
d'Information](http://www.c-s.fr/) and is still used and maintained by its dual teams
of space dynamics and image processing experts.

Rugged is used for image processing of the Sentinel 2 mission at European Space
Agency (ESA).
