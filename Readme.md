![Rugged logo](https://www.orekit.org/rugged/img/rugged-logo-small.jpg)

# Rugged

> A sensor-to-terrain mapping tool

[Rugged](https://www.orekit.org/rugged/  "Rugged homepage") is a free java
library for geolocation and used for satellite imagery.

Rugged is an add-on for [Orekit](https://www.orekit.org/ "Orekit homepage")
handling Digital Elevation Models contribution to line of sight computation. It
is a free software intermediate-level library written in Java.

It mainly provides direct and inverse location, i.e. it allows to compute
accurately which ground point is looked at from a specific pixel in a spacecraft
instrument, and conversely which pixel will see a specified ground point. This
mapping between ground and sensor is computed with a viewing model taking into
account:
* ground Digital Elevation Model (DEM),
* Earth rotation will all its tiny irregularities,
* on-board sensor pixels individual line-of-sights,
* spacecraft motion and attitude,
* several physical effects.

Direct and inverse location can be used to perform full ortho-rectification of
images and correlation between sensors observing the same area.

[![](https://img.shields.io/:license-apache-blue.svg)](https://www.apache.org/licenses/LICENSE-2.0.html)
[![](https://sonar.orekit.org/api/project_badges/measure?project=orekit%3Arugged&metric=alert_status)](https://sonar.orekit.org/dashboard?id=orekit%3Arugged)

[![](https://sonar.orekit.org/api/project_badges/measure?project=orekit%3Arugged&metric=coverage)](https://sonar.orekit.org/component_measures?id=orekit%3Arugged&metric=coverage&view=treemap)

## Download

### Official releases

[Official Rugged releases](https://gitlab.orekit.org/orekit/rugged/-/releases)
are available on our [Gitlab instance](https://gitlab.orekit.org/orekit/rugged).
They are also available in the
[Maven repository](https://mvnrepository.com/artifact/org.orekit/rugged).

### Development version

To get the latest development version, please clone our official repository
and checkout the `develop` branch:

```bash
git clone -b develop https://gitlab.orekit.org/orekit/rugged.git
```
__Note:__ Our official repository is
[mirrored on Github](https://github.com/CS-SI/Rugged).

## Documentation

The following documentation is available:

* [Latest API documentation](https://www.orekit.org/site-rugged-development/apidocs/index.html)
* [Latest Maven site](https://www.orekit.org/site-rugged-development/) for the project overview, architecture and development,
  detailed features list, Javadoc and a lot of other information

## Getting help

The main communication channel is our [forum](https://forum.orekit.org/). You
can report bugs and suggest new features in our
[issues tracking system](https://gitlab.orekit.org/orekit/rugged/-/issues). When
reporting security issues check the "This issue is confidential" box.

## Contributing

Please take a look at our
[contributing guidelines](https://www.orekit.org/site-rugged-development/contributing.html)
if you're interested in helping!

## Building

Detailed information on how to build Rugged from source either using Maven or
Eclipse is provided in
[building](https://www.orekit.org/site-rugged-development/building.html) explanations.

## Dependencies

Rugged relies on the following
[Free and Open-Source Software (FOSS)](https://en.wikipedia.org/wiki/Free_and_open-source_software) libraries,
all released under business friendly FOSS licenses.

### Compile-time/run-time dependencies

* [Orekit](https://www.orekit.org/), a low level space dynamics library released
  under the Apache License, version 2.0.

* [Hipparchus](https://hipparchus.org/), a mathematics library released under
  the Apache License, version 2.0.

### Test-time dependencies

* [JUnit 4](https://www.junit.org/), a widely used unit test framework
  released under the Eclipse Public License, version 1.0.

More detailed information is available in the
[Maven site](https://www.orekit.org/site-rugged-development/dependencies.html).

## License

Rugged is licensed by [CS GROUP](https://www.cs-soprasteria.com/en/) under
the [Apache License, version 2.0](https://www.apache.org/licenses/LICENSE-2.0.html).
A copy of this license is provided in the [LICENSE.txt](LICENSE.txt) file.
