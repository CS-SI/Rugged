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

# Downloads

## Development Library version

The development version of the Rugged library is always available to
download from our version control system. We use [Git](http://git-scm.com/ "Git homepage")
as our SCM. The anonymous read access to our Git repository  allows users who
need the latest features and the latest bug fixes to get them even before an
official release.

The Git repository can be cloned using the following command, which can
be easily adapted if you are using one of the numerous Git graphical
user interface available or if Git is supported by you integrated
development environment:

    git clone -b develop https://gitlab.orekit.org/orekit/rugged.git

## Released Library versions

Rugged is provided both in several packaging systems. You can pick up
the one that better suits your needs. Source packages are the most complete
ones since the other packages (binary jars, javadoc jars and even the bulk
of this site) are created from these sources.

Rugged is also available in maven central repository,
with groupID org.orekit and artifactId rugged so maven
internal mechanism will download automatically all artifacts and dependencies
as required.


|  package |                                              link                                                         |
|----------|-----------------------------------------------------------------------------------------------------------|
|  source  | [`rugged-2.1-sources.zip`](https://gitlab.orekit.org/orekit/rugged/uploads/a8ed096b6e9b5d1088ad135ac29fce9d/rugged-2.1-sources.zip)    |
|  binary  | [`rugged-2.1.jar`](https://gitlab.orekit.org/orekit/rugged/uploads/4b9041f962ed8cd3b55164873bb5c861/rugged-2.1.jar)                    |
version 2.1 downloads (release date: 2019-03-14)

|  package |                                              link                                                         |
|----------|-----------------------------------------------------------------------------------------------------------|
|  source  | [`rugged-2.0-sources.zip`](https://gitlab.orekit.org/orekit/rugged/uploads/f7f30111d4d3cef19636cb7c504530dd/rugged-2.0-sources.zip)    |
|  binary  | [`rugged-2.0.jar`](https://gitlab.orekit.org/orekit/rugged/uploads/8393279152c0cad15659e145018fa834/rugged-2.0.jar)                    |
|  javadoc | [`rugged-2.0-javadoc.jar`](https://gitlab.orekit.org/orekit/rugged/uploads/b42c3ef2fcff36aa44570d114102a439/rugged-2.0-javadoc.jar)    |
version 2.0 downloads (release date: 2017-12-19)

|  package |                                              link                                                         |
|----------|-----------------------------------------------------------------------------------------------------------|
|  source  | [`rugged-1.0-sources.zip`](https://gitlab.orekit.org/orekit/rugged/uploads/0a5e5a39e72dfa94f54c3193170d5ee2/rugged-1.0-sources.zip)    |
|  binary  | [`rugged-1.0.jar`](https://gitlab.orekit.org/orekit/rugged/uploads/55df1454320b8f625c05d9bee5c9abcd/rugged-1.0.jar)                    |
|  javadoc | [`rugged-1.0-javadoc.jar`](https://gitlab.orekit.org/orekit/rugged/uploads/8f7f399b1dd6ebf55b17f9a49fc88782/rugged-1.0-javadoc.jar)    |
version 1.0 downloads (release date: 2016-02-10)


## Data

For convenience, a zip archive containing some configuration data is available
for download. Similar files can be custom made by users with updated data.
Configuring data loading is explained in the configuration page. For a start,
the simplest configuration is to download the
[orekit-data-master.zip](https://gitlab.orekit.org/orekit/orekit-data/-/archive/master/orekit-data-master.zip)
file from the forge, to unzip it anywhere you want, rename the `orekit-data-master` folder that will be created
into `orekit-data` and add the following lines at the start of your program:

    File orekitData = new File("/path/to/the/folder/orekit-data");
    DataProvidersManager manager = DataProvidersManager.getInstance();
    manager.addProvider(new DirectoryCrawler(orekitData));

This file contains the following data sets. Note that the data is updated only
from time to time, so users must check by themselves they cover the time range
needed for their computation.

  * leap seconds data,

  * IERS Earth orientation parameters from 1973 (both IAU-1980 and IAU-2000),

  * Marshall Solar Activity Future Estimation from 1999,

  * JPL DE 430 planetary ephemerides from 1990 to 2069,

  * Eigen 06S gravity field,

  * FES 2004 ocean tides model.


There are no guarantees that this file will be available indefinitely or that its
content will be updated. It should be considered as a simple configuration example.
Users are encouraged to set up their own configuration data.

The file is available by following the
[download](https://gitlab.orekit.org/orekit/orekit-data/-/archive/master/orekit-data-master.zip)
link in the project dedicated to Orekit Data in the forge.

[Top of the page](#top)
