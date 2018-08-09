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

# Configuration

As Rugged relied on Orekit for the frames computation, Orekit
must be properly initialized for Rugged to run.

The simplest way to configure is to first retrieve the example orekit-data.zip
file from Orekit, available in the [Orekit project download page](https://www.orekit.org/download.html) (Physical Data)
and to unzip it in a known fixed location on your computer (let's assume it is on
your home folder, and it creates an orekit-data subfolder there).

Then near the start of your main program, and before Orekit is called for the
first time, you will add the following code snippet:

    File home       = new File(System.getProperty("user.home"));
    File orekitData = new File(home, "orekit-data");
    DataProvidersManager.getInstance().addProvider(new DirectoryCrawler(orekitData));

This is sufficient to start working.

Note that some of the data in the orekit-data folder needs to be updated,
typically the UTC-TAI.history file, which is updated about once every 18 months
by IERS, and the files in the Earth-Orientation-Parameters folder which are updated
regularly by IERS. The update frequency depends on which file you use.

The data provided in the example archive from Orekit site are example only and are
not kept up to date. The real operational data are live, and remain under the
responsibility of the user.