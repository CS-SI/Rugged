# Rugged 

_**A sensor-to-terrain mapping tool**_

**Rugged is a free java library for geolocation and used for satellite imagery.**

![rugged-logo.png](src/site/resources/images/rugged-logo.png)


Rugged is an add-on for [Orekit](https://www.orekit.org/) handling Digital Elevation Models contribution to 
line of sight computation. It is a free software intermediate-level library written in Java.

It mainly provides direct and inverse location, i.e. it allows to compute accurately 
which ground point is looked at from a specific pixel in a spacecraft instrument, 
and conversely which pixel will see a specified ground point. This mapping between 
ground and sensor is computed with a viewing model taking into account:
* ground Digital Elevation Model (DEM), 
* Earth rotation will all its tiny irregularities, 
* on-board sensor pixels individual line-of-sights, 
* spacecraft motion and attitude,
* several physical effects.

Direct and inverse location can be used to perform full ortho-rectification of 
images and correlation between sensors observing the same area.

Homepage: [www.orekit.org/rugged/](https://www.orekit.org/rugged/)




* [Overview](src/site/markdown/index.md)  
* [Getting the sources](src/site/markdown/sources.md)
* [Building](src/site/markdown/building.md)
* [Configuration](src/site/markdown/configuration.md)
* [FAQ](src/site/markdown/faq.md)
* [License](LICENSE.txt)
* [Downloads](src/site/markdown/downloads.md)
* [`Changes`](src/site/xdoc/changes.xml)
* [Contact](src/site/markdown/contact.md)

## Design


* [Overview](src/site/markdown/design/overview.md)
* [Technical choices](src/site/markdown/design/technical-choices.md)
* [Digital Elevation Model](src/site/markdown/design/digital-elevation-model.md)
* [Design of major functions](src/site/markdown/design/design.md)

## Tutorial

* [Direct location (with Rugged initialization example)](src/site/markdown/tutorials/direct-location.md)
* [Direct location with DEM](src/site/markdown/tutorials/direct-location-with-DEM.md)
* [Inverse location](src/site/markdown/tutorials/inverse-location.md)
* [Example in Matlab](src/site/markdown/tutorials/matlab-example.md)

## Development

* [Contributing](src/site/markdown/contributing.md)
* [Guidelines](src/site/markdown/guidelines.md)
* [`Javadoc, development`](https://www.orekit.org/site-rugged-development/apidocs/index.html)
* [`SOCIS|Summer Of Code In Space (SOCIS)`]()
