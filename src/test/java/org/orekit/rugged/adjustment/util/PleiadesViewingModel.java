package org.orekit.rugged.adjustment.util;

import java.util.ArrayList;
import java.util.List;

import org.hipparchus.geometry.euclidean.threed.Rotation;
import org.hipparchus.geometry.euclidean.threed.RotationConvention;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.util.FastMath;
import org.orekit.rugged.linesensor.LineDatation;
import org.orekit.rugged.linesensor.LineSensor;
import org.orekit.rugged.linesensor.LinearLineDatation;
import org.orekit.rugged.los.FixedRotation;
import org.orekit.rugged.los.FixedZHomothety;
import org.orekit.rugged.los.LOSBuilder;
import org.orekit.rugged.los.TimeDependentLOS;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeScale;
import org.orekit.time.TimeScalesFactory;

/**
 * Pleiades viewing model for refining Junit tests.
 */
public class PleiadesViewingModel {

    /** Pleiades parameters. */
    private static final double FOV = 1.65; // 20km - alt 694km
    private static final int DIMENSION = 40000;
    private static final double LINE_PERIOD =  1.e-4; 

    private double rollAngle;
    private LineSensor lineSensor;
    private String referenceDate;
    private String sensorName;

    /** PleiadesViewingModel constructor.
     * @param sensorName sensor name
     * @param rollAngle roll angle
     * @param referenceDate reference date
     */
    public PleiadesViewingModel(final String sensorName, final double rollAngle, final String referenceDate) {

        this.sensorName = sensorName;
        this.referenceDate = referenceDate;
        this.rollAngle = rollAngle;
        this.createLineSensor();
    }

    /** Create raw fixed Line Of sight
     */
    public LOSBuilder rawLOS(final Vector3D center, final Vector3D normal, final double halfAperture, final int n) {

        final List<Vector3D> list = new ArrayList<Vector3D>(n);
        for (int i = 0; i < n; ++i) {
            final double alpha = (halfAperture * (2 * i + 1 - n)) / (n - 1);
            list.add(new Rotation(normal, alpha, RotationConvention.VECTOR_OPERATOR).applyTo(center));
        }

        return new LOSBuilder(list);
    }

    /** Build a LOS provider
     */
    public TimeDependentLOS buildLOS() {

        // Roll angle applied to the LOS
        // Compute the transformation of vector K (Z axis) through the rotation around I (X axis) with the roll angle
        // If roll angle = 0: vector center = vector K (Z axis)
        final LOSBuilder losBuilder = rawLOS(new Rotation(Vector3D.PLUS_I, FastMath.toRadians(rollAngle),
                                                          RotationConvention.VECTOR_OPERATOR).applyTo(Vector3D.PLUS_K),
                                             Vector3D.PLUS_I, FastMath.toRadians(FOV / 2), DIMENSION);

        losBuilder.addTransform(new FixedRotation(sensorName + InitInterRefiningTest.rollSuffix,  Vector3D.MINUS_I, 0.00));
        losBuilder.addTransform(new FixedRotation(sensorName + InitInterRefiningTest.pitchSuffix, Vector3D.MINUS_J, 0.00));

        // factor is a common parameters shared between all Pleiades models
        losBuilder.addTransform(new FixedZHomothety(InitInterRefiningTest.factorName, 1.0));

        return losBuilder.build();
    }

    /** Get the reference date.
     */
    public AbsoluteDate getDatationReference() {

        // We use Orekit for handling time and dates, and Rugged for defining the datation model:
        final TimeScale utc = TimeScalesFactory.getUTC();

        return new AbsoluteDate(referenceDate, utc);
    }

    /** Get the min date.
     */
    public AbsoluteDate getMinDate() {
        return lineSensor.getDate(0);
    }

    /** Get the max date.
     */
    public AbsoluteDate getMaxDate() {
        return lineSensor.getDate(DIMENSION);
    }

    /** Get the line sensor.
     */
    public LineSensor getLineSensor() {
        return lineSensor;
    }

    /** Get the sensor name.
     */
    public String getSensorName() {
        return sensorName;
    }

    /** Get the number of lines of the sensor.
     * @return the number of lines of the sensor
     */
    public int getDimension() {
        return DIMENSION;
    }

    /** Create the line sensor.
     */
    private void createLineSensor() {

        // Offset of the MSI from center of mass of satellite
        // one line sensor
        // los: swath in the (YZ) plane, looking at 50° roll, 2.6" per pixel
        final Vector3D msiOffset = new Vector3D(0, 0, 0);

        final TimeDependentLOS lineOfSight = buildLOS();

        final double rate =  1. / LINE_PERIOD;
        // linear datation model: at reference time we get the middle line, and the rate is one line every 1.5ms

        final LineDatation lineDatation = new LinearLineDatation(getDatationReference(), DIMENSION / 2, rate);

        lineSensor = new LineSensor(sensorName, lineDatation, msiOffset, lineOfSight);
    }

}

