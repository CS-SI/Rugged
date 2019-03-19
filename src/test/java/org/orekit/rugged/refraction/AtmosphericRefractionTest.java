package org.orekit.rugged.refraction;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.io.File;
import java.net.URISyntaxException;
import java.util.ArrayList;

import org.hipparchus.geometry.euclidean.threed.Rotation;
import org.hipparchus.geometry.euclidean.threed.RotationConvention;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.util.FastMath;
import org.junit.Assert;
import org.junit.Test;
import org.orekit.bodies.BodyShape;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.data.DataProvidersManager;
import org.orekit.data.DirectoryCrawler;
import org.orekit.orbits.Orbit;
import org.orekit.rugged.TestUtils;
import org.orekit.rugged.api.AlgorithmId;
import org.orekit.rugged.api.BodyRotatingFrameId;
import org.orekit.rugged.api.EllipsoidId;
import org.orekit.rugged.api.InertialFrameId;
import org.orekit.rugged.api.Rugged;
import org.orekit.rugged.api.RuggedBuilder;
import org.orekit.rugged.errors.RuggedException;
import org.orekit.rugged.errors.RuggedMessages;
import org.orekit.rugged.linesensor.LineDatation;
import org.orekit.rugged.linesensor.LineSensor;
import org.orekit.rugged.linesensor.LinearLineDatation;
import org.orekit.rugged.linesensor.SensorPixel;
import org.orekit.rugged.los.LOSBuilder;
import org.orekit.rugged.los.TimeDependentLOS;
import org.orekit.rugged.raster.RandomLandscapeUpdater;
import org.orekit.rugged.raster.TileUpdater;
import org.orekit.rugged.utils.GeodeticUtilities;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeScalesFactory;
import org.orekit.utils.AngularDerivativesFilter;
import org.orekit.utils.CartesianDerivativesFilter;
import org.orekit.utils.Constants;

public class AtmosphericRefractionTest {
    
    
    @Test
    public void testAtmosphericRefractionCorrection() throws URISyntaxException  {

        String sensorName = "line";
        int dimension = 4000;

        RuggedBuilder builder = initRuggedForAtmosphericTests(dimension, sensorName);
        // Build Rugged without atmospheric refraction model
        Rugged ruggedWithout = builder.build();
        
        // Defines atmospheric refraction model (with the default multi layers model)
        AtmosphericRefraction atmosphericRefraction = new MultiLayerModel(ruggedWithout.getEllipsoid());
        int pixelStep = 100;
        int lineStep = 100;
        atmosphericRefraction.setGridSteps(pixelStep, lineStep);

        // Build Rugged with atmospheric refraction model
        builder.setRefractionCorrection(atmosphericRefraction);
        Rugged ruggedWith = builder.build();
        
        // For test coverage
        assertTrue(ruggedWith.getRefractionCorrection().getClass().isInstance(new MultiLayerModel(ruggedWith.getEllipsoid())));

        
        LineSensor lineSensor = ruggedWithout.getLineSensor(sensorName);
        int minLine = (int) FastMath.floor(lineSensor.getLine(ruggedWithout.getMinDate()));
        int maxLine = (int) FastMath.ceil(lineSensor.getLine(ruggedWithout.getMaxDate()));

        final double pixelThreshold = 1.e-3;
        final double lineThreshold = 1.e-2;

        final double epsilonPixel = pixelThreshold;
        final double epsilonLine = lineThreshold;
        double earthRadius = ruggedWithout.getEllipsoid().getEquatorialRadius();

        // Direct loc on a line WITHOUT and WITH atmospheric correction
        // ============================================================
        double chosenLine = 200.;
        GeodeticPoint[] gpWithoutAtmosphericRefractionCorrection = ruggedWithout.directLocation(sensorName, chosenLine);
        GeodeticPoint[] gpWithAtmosphericRefractionCorrection = ruggedWith.directLocation(sensorName, chosenLine);
        
        // Check the shift on the ground due to atmospheric correction
        for (int i = 0; i < gpWithAtmosphericRefractionCorrection.length; i++) {
            double currentRadius = earthRadius + (gpWithAtmosphericRefractionCorrection[i].getAltitude()+ gpWithoutAtmosphericRefractionCorrection[i].getAltitude())/2.;
            double distance = GeodeticUtilities.computeDistanceInMeter(currentRadius, gpWithAtmosphericRefractionCorrection[i], gpWithoutAtmosphericRefractionCorrection[i]);
            // Check if the distance is not 0 and < 2m
            Assert.assertTrue(distance > 0.0);
            Assert.assertTrue(distance < 2.);
        }
        
        // Inverse loc WITH atmospheric correction
        // ==========================================================================
        for (int i = 0; i < gpWithAtmosphericRefractionCorrection.length; i++) {
            
            // to check if we go back to the initial point when taking the geodetic point with atmospheric correction
            GeodeticPoint gpWith = gpWithAtmosphericRefractionCorrection[i];
            SensorPixel sensorPixelReverseWith = ruggedWith.inverseLocation(sensorName, gpWith, minLine, maxLine);
                        
            if (sensorPixelReverseWith != null) {
                assertEquals(i, sensorPixelReverseWith.getPixelNumber(), epsilonPixel);
                assertEquals(chosenLine, sensorPixelReverseWith.getLineNumber(), epsilonLine);
            } else {
                fail("Inverse location failed for pixel " + i + " with atmospheric refraction correction for geodetic point computed with" );
            }
        } // end loop on pixel i 
        
        
        // For test coverage 
        double dummyLat = gpWithAtmosphericRefractionCorrection[0].getLatitude() + FastMath.PI/4.;
        double dummyLon = gpWithAtmosphericRefractionCorrection[0].getLongitude() - FastMath.PI/4.;
        GeodeticPoint dummyGP = new GeodeticPoint(dummyLat, dummyLon, 0.);
        try {
            ruggedWith.inverseLocation(sensorName, dummyGP, minLine, maxLine);
        } catch (RuggedException re) {
            Assert.assertEquals(RuggedMessages.INVALID_RANGE_FOR_LINES, re.getSpecifier());
        }
    }

    private RuggedBuilder initRuggedForAtmosphericTests(final int dimension, final String sensorName) throws URISyntaxException {
        
        String path = getClass().getClassLoader().getResource("orekit-data").toURI().getPath();
        DataProvidersManager.getInstance().addProvider(new DirectoryCrawler(new File(path)));
        final BodyShape  earth = TestUtils.createEarth();
        final Orbit      orbit = TestUtils.createOrbit(Constants.EIGEN5C_EARTH_MU);

        // one line sensor
        // position: 1.5m in front (+X) and 20 cm above (-Z) of the S/C center of mass
        // los: swath in the (YZ) plane, looking at 5° roll, 2.6" per pixel
        Vector3D position = new Vector3D(1.5, 0, -0.2);
        TimeDependentLOS los = TestUtils.createLOSPerfectLine(new Rotation(Vector3D.PLUS_I,
                               FastMath.toRadians(5.0),
                               RotationConvention.VECTOR_OPERATOR).applyTo(Vector3D.PLUS_K),
                               Vector3D.PLUS_I,
                               FastMath.toRadians((dimension/2.) * 2.6 / 3600.0), dimension).build();

        // With the orbit (795km), the size of the pixel on the ground is around : 10m

        // linear datation model: at reference time we get the middle line, and the rate is one line every 1.5ms
        AbsoluteDate crossing = new AbsoluteDate("2012-01-01T12:30:00.000", TimeScalesFactory.getUTC());
        LineDatation lineDatation = new LinearLineDatation(crossing, dimension / 2, 1.0 / 1.5e-3);
        int firstLine = 0;
        int lastLine  = dimension;
        LineSensor lineSensor = new LineSensor(sensorName, lineDatation, position, los);
        AbsoluteDate minDate = lineSensor.getDate(firstLine).shiftedBy(-1.0);
        AbsoluteDate maxDate = lineSensor.getDate(lastLine).shiftedBy(+1.0);

        TileUpdater updater = new RandomLandscapeUpdater(800.0, 9000.0, 0.1, 0xf0a401650191f9f6l, FastMath.toRadians(2.0), 257);


        RuggedBuilder builder = new RuggedBuilder().
                    setDigitalElevationModel(updater, 8).
                    setAlgorithm(AlgorithmId.DUVENHAGE).
                    setEllipsoid(EllipsoidId.WGS84, BodyRotatingFrameId.ITRF).
                    setTimeSpan(minDate, maxDate, 0.001, 5.0).
                    setTrajectory(InertialFrameId.EME2000,
                                  TestUtils.orbitToPV(orbit, earth, minDate.shiftedBy(-1.0), maxDate.shiftedBy(+1.0), 0.25),
                                  8, CartesianDerivativesFilter.USE_PV,
                                  TestUtils.orbitToQ(orbit, earth, minDate.shiftedBy(-1.0), maxDate.shiftedBy(+1.0), 0.25),
                                  2, AngularDerivativesFilter.USE_R).
                    setLightTimeCorrection(false).
                    setAberrationOfLightCorrection(false).
                    addLineSensor(lineSensor);
            
        return builder;
    }


    @Test
    public void testBadConfig() {

        int dimension = 400;
        
        TimeDependentLOS los = new LOSBuilder(new ArrayList<Vector3D>(dimension)).build();
        LineSensor lineSensor = new LineSensor("line", null, Vector3D.ZERO, los);
        
        // Defines atmospheric refraction model (with the default multi layers model)
        AtmosphericRefraction atmosphericRefraction = new MultiLayerModel(null);

        // Check the context
        atmosphericRefraction.setGridSteps(100, 100);
        atmosphericRefraction.configureCorrectionGrid(lineSensor, 0, 300);
        assertFalse(atmosphericRefraction.isSameContext("otherSensor", 0, 300));
        assertFalse(atmosphericRefraction.isSameContext("line", 42, 300));
        assertFalse(atmosphericRefraction.isSameContext("line", 0, 42));

        // Check the test of validity of min / max line vs line step
        try {
            atmosphericRefraction.setGridSteps(100, 100);
            atmosphericRefraction.configureCorrectionGrid(lineSensor, 0, 100);
            Assert.fail("An exception should have been thrown");
    
        } catch (RuggedException re) {
            Assert.assertEquals(RuggedMessages.INVALID_RANGE_FOR_LINES,re.getSpecifier());
        }
        
        // Bad pixel step
        try {
            atmosphericRefraction.setGridSteps(-5, 100);
            atmosphericRefraction.configureCorrectionGrid(lineSensor, 0, 100);
            Assert.fail("An exception should have been thrown");

        } catch (RuggedException re) {
            Assert.assertEquals(RuggedMessages.INVALID_STEP,re.getSpecifier());
        }
     
        // Bad line step
        try {
            atmosphericRefraction.setGridSteps(10, -42);
            atmosphericRefraction.configureCorrectionGrid(lineSensor, 0, 100);
            Assert.fail("An exception should have been thrown");

        } catch (RuggedException re) {
            Assert.assertEquals(RuggedMessages.INVALID_STEP,re.getSpecifier());
        }
    }
}
