<!--- Copyright 2013-2018 CS Systèmes d'Information
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
# FAQ

## References

### Has Rugged already been used?

Yes. Rugged is used operationally in the Image Processing Facility
of the Sentinel 2 ESA mission, which was launched in June 2015.

### Is Rugged validated?

Yes, Rugged has been validated, by comparison with other image
processing systems.

Note that as Rugged takes into account some effects that may not be considered
by other systems, validation implies careful configuration and analysis of
discrepancies. Some differences come from missing correction in other systems,
like the Δδψ and Δδε precession/nutation correction parameters, or the light time
correction, or the aberration of light correction, or the non-straight line nature
of line-of-sight in geodetic space ... These differences are mostly dealt with
by disabling Rugged correction to match the other systems (only for test purposes,
of course, the corrections should be enabled for operational use!). Some differences
come from different models like precession and nutation compliant to latest IERS
conventions, or different Earth frames. These differences are mostly dealt with by
configuring Rugged to use the same legacy models (these legacy models like for example
TOD and MOD frames are available for consistency with existing systems).


## Installation

### What are the dependencies for Rugged?

Rugged relies on the Orekit space flight dynamics library and on
Hipparchus mathematical libraries. Both libraries are free
software libraries distributed under the terms of the Apache Software
License version 2.0.

## Runtime errors

### I get an error "no IERS UTC-TAI history data loaded" (or something similar in another language). What does it mean?

This error is probably the most frequent one, or at least it's the first one new users encounter.

Rugged relies on the Orekit library to perform all space flight related computation (typically
frames transforms). This library in turn needs some external data to be loaded in order to run.
This includes UTC-TAI history for leap seconds handling, Earth Orientation Parameters for
transforms to and from Earth fixed frames, or planetar ephemerides for Sun direction, for example.

The error message "no IERS UTC-TAI history data loaded" means the UTC-TAI history file which is used for leap
seconds management was not found. As leap seconds are used each time a UTC date is used, this message is
often seen very early and is the first one unsuspecting users experience. It often means the user forgot
to configure Orekit to load data.

Configuring data loading is explained in the configuration page For a start, the simplest configuration
is to download the orekit-data.zip file available in the [Orekit project download page](https://www.orekit.org/download.html) (Physical Data)., to unzip it anywhere you
want, note the path of the orekit-data folder that will be created and add the following lines at the start
of your program:

    File orekitData = new File("/path/to/the/folder/orekit-data");
    DataProvidersManager manager = DataProvidersManager.getInstance();
    manager.addProvider(new DirectoryCrawler(orekitData));

Using a folder allows one to change the data in it after the initial download, e.g., adding new EOP files as they
are published by IERS.
