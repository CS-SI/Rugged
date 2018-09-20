package org.orekit.rugged.los;

import java.io.Serializable;

import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.orekit.rugged.linesensor.SensorPixel;


/** Container for pixel line-of-sight.
 * <p>
 * Instances of this class are guaranteed to be immutable.
 * </p>
 * @author Guylaine Prat
 * @since 3.0
 */
public class PixelLOS implements Serializable {

    /** Serializable UID. */
    private static final long serialVersionUID = -6674056279573271367L;

    /** Sensor pixel. */
    private final SensorPixel sensorPixel;

    /** Pixel line-of-sight in spacecraft frame. */
    private final Vector3D los;

    /**
     * Build a new instance.
     * @param sensorPixel the sensor pixel cell
     * @param los the pixel line-of-sight in spacecraft frame
     */
    public PixelLOS(final SensorPixel sensorPixel, final Vector3D los) {
        this.sensorPixel = sensorPixel;
        this.los = los;
    }

    /**
     * @return the sensorPixel
     */
    public SensorPixel getSensorPixel() {
        return sensorPixel;
    }

    /**
     * @return the lOS in spacecraft frame
     */
    public Vector3D getLOS() {
        return los;
    }

    @Override
    public String toString() {
        return "{Sensor pixel: " +  sensorPixel.toString() +
               ", LOS: " + los.toString() + "}";
    }
}
