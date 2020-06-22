package fr.cs.examples;

import java.io.File;
import java.net.URISyntaxException;
import java.text.DecimalFormat;
import java.text.DecimalFormatSymbols;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import org.hipparchus.geometry.euclidean.threed.Rotation;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.stat.descriptive.DescriptiveStatistics;
import org.hipparchus.util.FastMath;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.data.DataContext;
import org.orekit.data.DirectoryCrawler;
import org.orekit.frames.Frame;
import org.orekit.frames.FramesFactory;
import org.orekit.frames.Transform;
import org.orekit.rugged.api.AlgorithmId;
import org.orekit.rugged.api.BodyRotatingFrameId;
import org.orekit.rugged.api.EllipsoidId;
import org.orekit.rugged.api.InertialFrameId;
import org.orekit.rugged.api.Rugged;
import org.orekit.rugged.api.RuggedBuilder;
import org.orekit.rugged.linesensor.LineSensor;
import org.orekit.rugged.linesensor.LinearLineDatation;
import org.orekit.rugged.linesensor.SensorPixel;
import org.orekit.rugged.los.FixedRotation;
import org.orekit.rugged.los.LOSBuilder;
import org.orekit.rugged.los.TimeDependentLOS;
import org.orekit.rugged.refraction.AtmosphericRefraction;
import org.orekit.rugged.refraction.MultiLayerModel;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeScale;
import org.orekit.time.TimeScalesFactory;
import org.orekit.utils.AngularDerivativesFilter;
import org.orekit.utils.CartesianDerivativesFilter;
import org.orekit.utils.IERSConventions;
import org.orekit.utils.PVCoordinates;
import org.orekit.utils.TimeStampedAngularCoordinates;
import org.orekit.utils.TimeStampedPVCoordinates;

/** A simple example of how to use the atmospheric refraction correction.
 */
public class AtmosphericRefractionExamples {
    
    
    public static void main(String[] args) throws URISyntaxException {

        // Initialize Orekit, assuming an orekit-data folder is in user home directory
        File home       = new File(System.getProperty("user.home"));
        File orekitData = new File(home, "orekit-data");
        DataContext.getDefault().getDataProvidersManager().addProvider(new DirectoryCrawler(orekitData));

        // Sensor's definition
        // ===================
        // Line of sight
        // -------------
        // The raw viewing direction of pixel i with respect to the instrument is defined by the vector:
        List<Vector3D> rawDirs = new ArrayList<Vector3D>();
        for (int i = 0; i < 2000; i++) {
            // 20° field of view, 2000 pixels
            rawDirs.add(new Vector3D(0., i*FastMath.toRadians(20.)/2000., 1.));
        }

        // The instrument is oriented 10° off nadir around the X-axis, we need to rotate the viewing
        // direction to obtain the line of sight in the satellite frame
        LOSBuilder losBuilder = new LOSBuilder(rawDirs);
        losBuilder.addTransform(new FixedRotation("10-degrees-rotation", Vector3D.PLUS_I, FastMath.toRadians(10.)));

        TimeDependentLOS lineOfSight = losBuilder.build();

        // Datation model
        // --------------
        // We use Orekit for handling time and dates, and Rugged for defining the datation model:
        TimeScale gps = TimeScalesFactory.getGPS();
        AbsoluteDate absDate = new AbsoluteDate("2009-12-11T16:59:30.0", gps);
        LinearLineDatation lineDatation = new LinearLineDatation(absDate, 1., 20.);

        // Line sensor
        // -----------
        // With the LOS and the datation now define, we can initialize a line sensor object in Rugged:
        LineSensor lineSensor = new LineSensor("mySensor", lineDatation, Vector3D.ZERO, lineOfSight);

        // Satellite position, velocity and attitude
        // =========================================
        // Reference frames
        // ----------------
        // In our application, we simply need to know the name of the frames we are working with. Positions and
        // velocities are given in the ITRF terrestrial frame, while the quaternions are given in EME2000
        // inertial frame.
        Frame eme2000 = FramesFactory.getEME2000();
        boolean simpleEOP = true; // we don't want to compute tiny tidal effects at millimeter level
        Frame itrf = FramesFactory.getITRF(IERSConventions.IERS_2010, simpleEOP);

        // Satellite attitude
        // ------------------
        ArrayList<TimeStampedAngularCoordinates> satelliteQList = new ArrayList<TimeStampedAngularCoordinates>();

        addSatelliteQ(gps, satelliteQList, "2009-12-11T16:58:42.592937", -0.340236, 0.333952, -0.844012, -0.245684);
        addSatelliteQ(gps, satelliteQList, "2009-12-11T16:59:06.592937", -0.354773, 0.329336, -0.837871, -0.252281);
        addSatelliteQ(gps, satelliteQList, "2009-12-11T16:59:30.592937", -0.369237, 0.324612, -0.831445, -0.258824);
        addSatelliteQ(gps, satelliteQList, "2009-12-11T16:59:54.592937", -0.3836,   0.319792, -0.824743, -0.265299);
        addSatelliteQ(gps, satelliteQList, "2009-12-11T17:00:18.592937", -0.397834, 0.314883, -0.817777, -0.271695);
        addSatelliteQ(gps, satelliteQList, "2009-12-11T17:00:42.592937", -0.411912, 0.309895, -0.810561, -0.278001);
        addSatelliteQ(gps, satelliteQList, "2009-12-11T17:01:06.592937", -0.42581,  0.304838, -0.803111, -0.284206);
        addSatelliteQ(gps, satelliteQList, "2009-12-11T17:01:30.592937", -0.439505, 0.299722, -0.795442, -0.290301);
        addSatelliteQ(gps, satelliteQList, "2009-12-11T17:01:54.592937", -0.452976, 0.294556, -0.787571, -0.296279);
        addSatelliteQ(gps, satelliteQList, "2009-12-11T17:02:18.592937", -0.466207, 0.28935,  -0.779516, -0.302131);

        // Positions and velocities
        // ------------------------
        ArrayList<TimeStampedPVCoordinates> satellitePVList = new ArrayList<TimeStampedPVCoordinates>();

        addSatellitePV(gps, eme2000, itrf, satellitePVList, "2009-12-11T16:58:42.592937",  -726361.466, -5411878.485, 4637549.599, -2463.635, -4447.634, -5576.736);
        addSatellitePV(gps, eme2000, itrf, satellitePVList, "2009-12-11T16:59:04.192937",  -779538.267, -5506500.533, 4515934.894, -2459.848, -4312.676, -5683.906);
        addSatellitePV(gps, eme2000, itrf, satellitePVList, "2009-12-11T16:59:25.792937",  -832615.368, -5598184.195,  4392036.13, -2454.395, -4175.564, -5788.201);
        addSatellitePV(gps, eme2000, itrf, satellitePVList, "2009-12-11T16:59:47.392937",  -885556.748, -5686883.696, 4265915.971, -2447.273, -4036.368, -5889.568);
        addSatellitePV(gps, eme2000, itrf, satellitePVList, "2009-12-11T17:00:08.992937",   -938326.32, -5772554.875, 4137638.207, -2438.478, -3895.166, -5987.957);
        addSatellitePV(gps, eme2000, itrf, satellitePVList, "2009-12-11T17:00:30.592937",  -990887.942,  -5855155.21, 4007267.717, -2428.011, -3752.034, -6083.317);
        addSatellitePV(gps, eme2000, itrf, satellitePVList, "2009-12-11T17:00:52.192937", -1043205.448, -5934643.836, 3874870.441, -2415.868,  -3607.05, -6175.600);
        addSatellitePV(gps, eme2000, itrf, satellitePVList, "2009-12-11T17:01:13.792937", -1095242.669, -6010981.571,  3740513.34, -2402.051, -3460.291, -6264.760);
        addSatellitePV(gps, eme2000, itrf, satellitePVList, "2009-12-11T17:01:35.392937", -1146963.457,  -6084130.93, 3604264.372, -2386.561, -3311.835, -6350.751);
        addSatellitePV(gps, eme2000, itrf, satellitePVList, "2009-12-11T17:01:56.992937", -1198331.706, -6154056.146, 3466192.446, -2369.401, -3161.764, -6433.531);
        addSatellitePV(gps, eme2000, itrf, satellitePVList, "2009-12-11T17:02:18.592937", -1249311.381, -6220723.191, 3326367.397, -2350.574, -3010.159, -6513.056);

        // Rugged initialization
        // ---------------------
        RuggedBuilder builder = new RuggedBuilder().
                setAlgorithm(AlgorithmId.IGNORE_DEM_USE_ELLIPSOID).
                setEllipsoid(EllipsoidId.WGS84, BodyRotatingFrameId.ITRF).
                setTimeSpan(absDate, absDate.shiftedBy(60.0), 0.01, 5. / lineSensor.getRate(0.)).
                setTrajectory(InertialFrameId.EME2000,
                              satellitePVList, 4, CartesianDerivativesFilter.USE_P,
                              satelliteQList,  4, AngularDerivativesFilter.USE_R).
                addLineSensor(lineSensor);

        // Build Rugged without atmospheric refraction model
        Rugged ruggedWithout = builder.build();

        // Defines atmospheric refraction model (with the default multi layers model)
        AtmosphericRefraction atmosphericRefraction = new MultiLayerModel(ruggedWithout.getEllipsoid());
        // One can use its own atmospheric model that must extend the abstract refraction.AtmosphericRefraction class.

        // Set the optional grid steps, for the inverse location computation. Useless for direct location computation.
        // This setting is optional as default values exist. But can be useful for instance to improve results by given smaller steps 
        // or to speed up the computation bigger steps. 
        int pixelStep = 100;
        int lineStep = 100;
        atmosphericRefraction.setGridSteps(pixelStep, lineStep);

        // Build Rugged with atmospheric refraction model
        builder.setRefractionCorrection(atmosphericRefraction);
        Rugged ruggedWith = builder.build();

        // Direct location on a line WITHOUT and WITH atmospheric correction
        // -----------------------------------------------------------------
        double chosenLine = 200.;
        // Without atmosphere
        GeodeticPoint[] gpWithoutAtmosphericRefractionCorrection = ruggedWithout.directLocation("mySensor", chosenLine);
        // With atmosphere
        GeodeticPoint[] gpWithAtmosphericRefractionCorrection = ruggedWith.directLocation("mySensor", chosenLine);

        double earthRadius = ruggedWithout.getEllipsoid().getEquatorialRadius();
        DescriptiveStatistics statDistance  = new DescriptiveStatistics();

        for (int i = 0; i < gpWithAtmosphericRefractionCorrection.length; i++) {
            double currentRadius = earthRadius + (gpWithAtmosphericRefractionCorrection[i].getAltitude()+ gpWithoutAtmosphericRefractionCorrection[i].getAltitude())/2.;
            double distance = computeDistanceInMeter(currentRadius, gpWithAtmosphericRefractionCorrection[i], gpWithoutAtmosphericRefractionCorrection[i]);
            statDistance.addValue(distance);
        }

        System.out.format("Distance must be > 0 and < 2m:" +
                          " Max = " + DFS.format(statDistance.getMax()) +
                          " Min = " + DFS.format(statDistance.getMin()) +
                          " Median = " + DFS.format(statDistance.getPercentile(50.)) +
                          " Mean = " + DFS.format(statDistance.getMean()) +
                          " Std deviation= " + DFS.format(statDistance.getStandardDeviation()) +
                          "\n");

        // Inverse loc WITH atmospheric correction
        // ==========================================================================
        final double epsilonPixel = 1.e-3;
        final double epsilonLine = 1.1e-2;

        int minLine = (int) FastMath.floor(lineSensor.getLine(ruggedWithout.getMinDate()));
        int maxLine = (int) FastMath.ceil(lineSensor.getLine(ruggedWithout.getMaxDate()));

        for (int i = 0; i < gpWithAtmosphericRefractionCorrection.length; i++) {

            // to check if we go back to the initial point when taking the geodetic point with atmospheric correction
            GeodeticPoint gpWith = gpWithAtmosphericRefractionCorrection[i];
            SensorPixel sensorPixelReverseWith = ruggedWith.inverseLocation("mySensor", gpWith, minLine, maxLine);

            if (sensorPixelReverseWith != null) {
                // Compare the computed pixel vs expected
                if (FastMath.abs(i - sensorPixelReverseWith.getPixelNumber()) > epsilonPixel) {
                    System.out.format("Problem with pixel " + i + " . Delta pixel= " + DFS.format((i - sensorPixelReverseWith.getPixelNumber())) + "\n");
                }
                // Compare the computed line vs the expected
                if (FastMath.abs(chosenLine - sensorPixelReverseWith.getLineNumber())> epsilonLine) {
                    System.out.format("Probem with line, for pixel " + i + ". Delta line= " + DFS.format((chosenLine - sensorPixelReverseWith.getLineNumber())) + "\n");
                }
            } else {
                System.out.println("Inverse location failed for pixel " + i + " with atmospheric refraction correction for geodetic point computed with" );
            }
        } // end loop on pixel i 
        System.out.format("Inverse location gave same pixel and line as direct location input for" +
                          " epsilon pixel= " + DFS.format(epsilonPixel) +
                          " and epsilon line= " + DFS.format(epsilonLine) +
                          "\n");
    }

    
    private static void addSatellitePV(TimeScale gps, Frame eme2000, Frame itrf,
                                  ArrayList<TimeStampedPVCoordinates> satellitePVList,
                                  String absDate,
                                  double px, double py, double pz, double vx, double vy, double vz) {
        
        AbsoluteDate ephemerisDate = new AbsoluteDate(absDate, gps);
        Vector3D position = new Vector3D(px, py, pz); // in ITRF, unit: m
        Vector3D velocity = new Vector3D(vx, vy, vz); // in ITRF, unit: m/s
        PVCoordinates pvITRF = new PVCoordinates(position, velocity);
        Transform transform = itrf.getTransformTo(eme2000, ephemerisDate);
        PVCoordinates pvEME2000 = transform.transformPVCoordinates(pvITRF);
        satellitePVList.add(new TimeStampedPVCoordinates(ephemerisDate, pvEME2000.getPosition(), pvEME2000.getVelocity(), Vector3D.ZERO));
    }

    private static void addSatelliteQ(TimeScale gps, ArrayList<TimeStampedAngularCoordinates> satelliteQList, String absDate,
                                      double q0, double q1, double q2, double q3) {
        AbsoluteDate attitudeDate = new AbsoluteDate(absDate, gps);
        Rotation rotation = new Rotation(q0, q1, q2, q3, true);  // q0 is the scalar term
        TimeStampedAngularCoordinates pair =
                new TimeStampedAngularCoordinates(attitudeDate, rotation, Vector3D.ZERO, Vector3D.ZERO);
        satelliteQList.add(pair);
    }

    private static double computeDistanceInMeter(double earthRadius, final GeodeticPoint gp1, final GeodeticPoint gp2) {

        // get vectors on unit sphere from angular coordinates
        final Vector3D p1 = new Vector3D(gp1.getLongitude(), gp1.getLatitude());
        final Vector3D p2 = new Vector3D(gp2.getLongitude(), gp2.getLatitude());
        return earthRadius * Vector3D.angle(p1, p2);
    }
    
    /** Definition of print format. */
    private static final DecimalFormatSymbols SYMBOLS = new DecimalFormatSymbols(Locale.US);
    /** Defined of number of digit after the point. */
    private static final DecimalFormat DFS = new DecimalFormat("#.#####", SYMBOLS);
}
