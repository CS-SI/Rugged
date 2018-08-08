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

Downloads
=========

Development Library version
---------------------------

The development version of the Rugged library is always available to
download from our version control system. We use [ Git](http://git-scm.com/)
as our SCM. The anonymous read access to our Git repository  allows users who
need the latest features and the latest bug fixes to get them even before an
official release.

The Git repository can be cloned using the following command, which can
be easily adapted if you are using one of the numerous Git graphical
user interface available or if Git is supported by you integrated
development environment:

    git clone https://gitlab.orekit.org/orekit/rugged.git

 :interrobang: EXPLANATIONS TO ADD about branches : master, develop, ... :interrobang:

Released Library versions
-------------------------

Rugged is provided both in several packaging systems. You can pick up
the one that better suits your needs. Source packages are the most complete
ones since the other packages (binary jars, javadoc jars and even the bulk
of this site) are created from these sources.

Rugged is also available in maven central repository,
with groupID org.orekit and artifactId rugged so maven
internal mechanism will download automatically all artifacts and dependencies
as required.

`Below URLs to be CHANGED ...`

|  package |                                              link                                                         |
|----------|-----------------------------------------------------------------------------------------------------------|
|  source  | [`rugged-2.0-sources.zip`](https://www.orekit.org/forge/attachments/download/719/rugged-2.0-sources.zip)    |
|  binary  | [`rugged-2.0.jar`](https://www.orekit.org/forge/attachments/download/720/rugged-2.0.jar)                    |
|  javadoc | [`rugged-2.0-javadoc.jar`](https://www.orekit.org/forge/attachments/download/721/rugged-2.0-javadoc.jar)    |
version 2.0 downloads (release date: 2017-12-19)

|  package |                                              link                                                         |
|----------|-----------------------------------------------------------------------------------------------------------|
|  source  | [`rugged-1.0-sources.zip`](https://www.orekit.org/forge/attachments/download/592/rugged-1.0-sources.zip)    |
|  binary  | [`rugged-1.0.jar`](https://www.orekit.org/forge/attachments/download/593/rugged-1.0.jar)                    |
|  javadoc | [`rugged-1.0-javadoc.jar`](https://www.orekit.org/forge/attachments/download/594/rugged-1.0-javadoc.jar)    |
version 1.0 downloads (release date: 2016-02-10)


## Data

For convenience, a zip archive containing some configuration data is
available for download. Similar files can be custom made by users with updated data.
Configuring data loading is explained in the configuration page For a start, the simplest configuration
is to download the orekit-data.zip file from the download page, to unzip it anywhere you want, note the
path of the orekit-data folder that will be created and add the following lines at the start of
your program:

    File orekitData = new File("/path/to/the/folder/orekit-data");
    DataProvidersManager manager = DataProvidersManager.getInstance();
    manager.addProvider(new DirectoryCrawler(orekitData));

This file contents is:

  * leap seconds data up to end of 2017,

  * IERS Earth orientation parameters from 1973 to end 2017
    with predicted date to mid 2018 for some parameters (both IAU-1980 and IAU-2000),

  * Marshall Solar Activity Futur Estimation from 1999 to end 2017,

  * DE 430 planetary ephemerides from 1990 to 2069,

  * Eigen 06S gravity field,

  * FES 2004 ocean tides model.

There are no guarantees that this file will be available indefinitely or that its
content will be updated. It should be considered as a simple configuration example.
Users are encouraged to set up their own configuration data.

`Below URLs to be CHANGED ...`

The file is available in the [`orekit files page`](https://www.orekit.org/forge/projects/orekit/files).

