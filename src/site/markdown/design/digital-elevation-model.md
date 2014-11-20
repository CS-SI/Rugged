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

DEM intersection
------------
The page [technical choices](./technical-choices.html) explain how Rugged goes from an on-board pixel
line-of-sight to a ground-based line-of-sight arrival in the vicinity of the ellipsoid entry point. At
this step, we have a 3D line defined near the surface and want to compute where it exactly traverses the
Digital Elevation Model surface. There is no support for this computation at Orekit library level,
everything is done at Rugged library level.

As this part of the algorithm represents an inner loop, it is one that must use fast algorithms. Depending
on the conditions (line-of-sight skimming over the terrain near field of view edges or diving directly in
a nadir view), some algorithms are more suitable than others. This computation is isolated in the smallest
programming unit possible in the Rugged library and an interface is defined with several different
implementations among which user can select.

Three different algorithms are predefined in Rugged:

 * a recursive algorithm based on Bernardt Duvenhage's 2009 paper
   [Using An Implicit Min/Max KD-Tree for Doing Efficient Terrain Line of Sight Calculations](http://researchspace.csir.co.za/dspace/bitstream/10204/3041/1/Duvenhage_2009.pdf)
 * an alternate version of the Duvenhage algorithm using flat-body hypothesis,
 * a basic scan algorithm sequentially checking all pixels in the rectangular array defined by Digital Elevation Model entry and exit points,
 * a no-operation algorithm that ignores the Digital Elevation Model and uses only the ellipsoid.

It is expected that other algorithms like line-stepping (perhaps using Bresenham line algorithm) will be added afterwards.

The Duvenhage algorithm with full consideration of the ellipsoid shape is the baseline approach for operational
computation. The alternate version of Duvenhage algorithm with flat-body hypothesis does not really save anything
meaningful in terms of computation, so it should only be used for testing purposes. The basic scan algorithm is only
intended as a basic reference that can be used for validation and tests. The no-operation algorithm can be used for
low accuracy fast computation needs without changing the complete data product work-flow.

DEM loading
-----------

As the min/max KD-tree structure is specific to the Duvenhage algorithm, and as the algorithm is hidden behind
a generic interface, the tree remains an implementation detail the user should not see. The min/max KD-tree structure is
therefore built directly at Rugged level, only when the Duvenhage algorithm has been selected to perform location computation.

On the other hand, Rugged is not expected to parsed DEM files, so the algorithm relies on the raw data being passed by the upper
layer. In order to pass these data, a specific callback function is implemented in the mission specific interface layer and
registered to Rugged, which can call it to retrieve parts of the DEM, in the form of small cells. The implicit KD-tree is then
built from leafs to root and cached.
