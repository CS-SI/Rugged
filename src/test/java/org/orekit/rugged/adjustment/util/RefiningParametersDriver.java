package org.orekit.rugged.adjustment.util;

import org.orekit.rugged.api.Rugged;
import org.orekit.utils.ParameterDriver;


/** Apply disruptions or select/unselect parameter to adjust for Refining JUnit tests.
 * @author Guylaine Prat
 */
public class RefiningParametersDriver {
    
    // Part of the name of parameter drivers
    static final String rollSuffix = "_roll";
    static final String pitchSuffix = "_pitch";
    static final String factorName = "factor";

    /** Apply disruptions on acquisition for roll angle
     * @param rugged Rugged instance
     * @param sensorName line sensor name
     * @param rollValue rotation on roll value
     */
    public static void applyDisruptionsRoll(final Rugged rugged, final String sensorName, final double rollValue) {

        rugged.
        getLineSensor(sensorName).
        getParametersDrivers().
        filter(driver -> driver.getName().equals(sensorName + rollSuffix)).
        findFirst().get().setValue(rollValue);
    }
    
    /** Apply disruptions on acquisition for pitch angle
     * @param rugged Rugged instance
     * @param sensorName line sensor name
     * @param pitchValue rotation on pitch value
     */
    public static void applyDisruptionsPitch(final Rugged rugged, final String sensorName, final double pitchValue) {

        rugged.
        getLineSensor(sensorName).
        getParametersDrivers().
        filter(driver -> driver.getName().equals(sensorName + pitchSuffix)).
        findFirst().get().setValue(pitchValue);
    }
    
    /** Apply disruptions on acquisition for scale factor
     * @param rugged Rugged instance
     * @param sensorName line sensor name
     * @param factorValue scale factor
     */
    public static void applyDisruptionsFactor(final Rugged rugged, final String sensorName, final double factorValue) {

        rugged.
        getLineSensor(sensorName).
        getParametersDrivers().
        filter(driver -> driver.getName().equals(factorName)).
        findFirst().get().setValue(factorValue);
    }
    
    /** Select roll angle to adjust
     * @param rugged Rugged instance
     * @param sensorName line sensor name
     */
    public static void setSelectedRoll(final Rugged rugged, final String sensorName) {

        ParameterDriver rollDriver =
                rugged.getLineSensor(sensorName).getParametersDrivers().
                filter(driver -> driver.getName().equals(sensorName + rollSuffix)).findFirst().get();
        rollDriver.setSelected(true);
    }
    
    /** Select pitch angle to adjust
     * @param rugged Rugged instance
     * @param sensorName line sensor name
     */
    public static void setSelectedPitch(final Rugged rugged, final String sensorName) {
        
        ParameterDriver pitchDriver =
                rugged.getLineSensor(sensorName).getParametersDrivers().
                filter(driver -> driver.getName().equals(sensorName + pitchSuffix)).findFirst().get();
        pitchDriver.setSelected(true);
    }

    /** Select scale factor to adjust
     * @param rugged Rugged instance
     * @param sensorName line sensor name
     */
    public static void setSelectedFactor(final Rugged rugged, final String sensorName) {

        ParameterDriver factorDriver =
                rugged.getLineSensor(sensorName).getParametersDrivers().
                filter(driver -> driver.getName().equals(factorName)).findFirst().get();
        factorDriver.setSelected(true);
    }  
    
    /** Unselect roll angle to adjust (for test coverage purpose)
     * @param rugged Rugged instance
     * @param sensorName line sensor name
     */
    public static void unselectRoll(final Rugged rugged, final String sensorName) {

        ParameterDriver rollDriver =
                rugged.getLineSensor(sensorName).getParametersDrivers().
                filter(driver -> driver.getName().equals(sensorName + rollSuffix)).findFirst().get();
        rollDriver.setSelected(false);
    }
    
    /** Unselect pitch angle to adjust (for test coverage purpose)
     * @param rugged Rugged instance
     * @param sensorName line sensor name
     */
    public static void unselectPitch(final Rugged rugged, final String sensorName) {
        
        ParameterDriver pitchDriver =
                rugged.getLineSensor(sensorName).getParametersDrivers().
                filter(driver -> driver.getName().equals(sensorName + pitchSuffix)).findFirst().get();
        pitchDriver.setSelected(false);
    }

    /** Unselect factor angle to adjust (for test coverage purpose)
     * @param rugged Rugged instance
     * @param sensorName line sensor name
     */
    public static void unselectFactor(final Rugged rugged, final String sensorName) {

        ParameterDriver factorDriver =
                rugged.getLineSensor(sensorName).getParametersDrivers().
                filter(driver -> driver.getName().equals(factorName)).findFirst().get();
        factorDriver.setSelected(false);
    }  

}
