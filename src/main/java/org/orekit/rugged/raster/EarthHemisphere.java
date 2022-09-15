package org.orekit.rugged.raster;

/**
 * Enumerate for Earth hemispheres for tiles definition.
 * <p>
 * For Latitude: NORTH / SOUTH
 * <p>
 * For Longitude: WESTEXTREME / WEST / EAST / EASTEXTREME
 * @author Guylaine Prat
 * @since X.x
 */
public enum EarthHemisphere {

    /** South hemisphere. */
   SOUTH,
   /** North hemisphere. */
   NORTH,
   /** Extreme West. */
   WESTEXTREME,
   /** West hemisphere. */
   WEST,
   /** East hemisphere. */
   EAST,
   /** Extreme East. */
   EASTEXTREME
}
