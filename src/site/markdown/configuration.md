<!--- Copyright 2013-2022 CS GROUP
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

# Configuration

As Rugged relied on [Orekit](https://www.orekit.org/ "Orekit homepage") for the frames computation, Orekit
must be properly initialized for Rugged to run.

The simplest way to configure is to first retrieve the example `orekit-data-master.zip`
file from Rugged download page, available in the [Rugged project download page](https://www.orekit.org/rugged/download.html) 
(Get the physical data)
and to unzip it anywhere you want, rename the `orekit-data-master` folder that will be created
into `orekit-data` and add the following lines at the start of your program (before Orekit is called for the
first time):

    File orekitData = new File("/path/to/the/folder/orekit-data");
    DataContext.getDefault().getDataProvidersManager().addProvider(new DirectoryCrawler(orekitData));

This is sufficient to start working.

Note that some of the data in the orekit-data-master folder needs to be updated,
typically the UTC-TAI history file, which is updated about once every 18 months
by IERS, and the files in the Earth-Orientation-Parameters folder which are updated
regularly by IERS. The update frequency depends on which file you use.

The data provided in the example archive from Rugged site are example only and are
not kept up to date. The real operational data are live, and remain under the
responsibility of the user.

[Top of the page](#top)
