package org.orekit.rugged.adjustment.util;

import java.io.File;
import java.lang.reflect.InvocationTargetException;
import java.net.URISyntaxException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.hipparchus.analysis.differentiation.DerivativeStructure;
import org.hipparchus.analysis.differentiation.Gradient;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.random.GaussianRandomGenerator;
import org.hipparchus.random.UncorrelatedRandomVectorGenerator;
import org.hipparchus.random.Well19937a;
import org.hipparchus.util.FastMath;
import org.junit.After;
import org.junit.Assert;
import org.orekit.bodies.BodyShape;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.data.DataContext;
import org.orekit.data.DirectoryCrawler;
import org.orekit.errors.OrekitException;
import org.orekit.orbits.Orbit;
import org.orekit.rugged.TestUtils;
import org.orekit.rugged.adjustment.InterSensorsOptimizationProblemBuilder;
import org.orekit.rugged.adjustment.measurements.Observables;
import org.orekit.rugged.adjustment.measurements.SensorToSensorMapping;
import org.orekit.rugged.api.AlgorithmId;
import org.orekit.rugged.api.BodyRotatingFrameId;
import org.orekit.rugged.api.EllipsoidId;
import org.orekit.rugged.api.InertialFrameId;
import org.orekit.rugged.api.Rugged;
import org.orekit.rugged.api.RuggedBuilder;
import org.orekit.rugged.linesensor.LineSensor;
import org.orekit.rugged.linesensor.SensorPixel;
import org.orekit.rugged.utils.DerivativeGenerator;
import org.orekit.rugged.utils.SpacecraftToObservedBody;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.AngularDerivativesFilter;
import org.orekit.utils.CartesianDerivativesFilter;
import org.orekit.utils.Constants;
import org.orekit.utils.TimeStampedAngularCoordinates;
import org.orekit.utils.TimeStampedPVCoordinates;




/** Initialization for inter sensor refining Junit tests.
 * @author Guylaine Prat
 */
public class InitInterRefiningTest {

    /** Pleiades viewing model A */
    private PleiadesViewingModel pleiadesViewingModelA;

    /** Pleiades viewing model B */
    private PleiadesViewingModel pleiadesViewingModelB;
    
    /** Line sensor A */
    private LineSensor lineSensorA;
    
    /** Line sensor B */
    private LineSensor lineSensorB;

    /** RuggedA's instance */
    private Rugged ruggedA;

    /** RuggedB's instance */
    private Rugged ruggedB;
    
    /** Number of parameters to adjust */
    private int parameterToAdjust;
    
    // Part of the name of parameter drivers
    static final String rollSuffix = "_roll";
    static final String pitchSuffix = "_pitch";
    static final String factorName = "factor";

    // Default values for disruption to apply to roll (deg), pitch (deg) and factor 
    static final double defaultRollDisruptionA =  0.004;
    static final double defaultPitchDisruptionA = 0.0008;
    static final double defaultFactorDisruptionA = 1.000000001;
    static final double defaultRollDisruptionB =  -0.004;
    static final double defaultPitchDisruptionB = 0.0008;
    
    /**
     * Initialize refining tests with default values for disruptions on sensors characteristics
     */
    public void initRefiningTest() {
        
        initRefiningTest(defaultRollDisruptionA, defaultPitchDisruptionA, defaultFactorDisruptionA, 
                         defaultRollDisruptionB, defaultPitchDisruptionB);
    }

    /** Initialize refining tests with disruption on sensors characteristics
     * @param rollDisruptionA disruption to apply to roll angle for sensor A (deg)
     * @param pitchDisruptionA disruption to apply to pitch angle for sensor A (deg)
     * @param factorDisruptionA disruption to apply to homothety factor for sensor A
     * @param rollDisruptionB disruption to apply to roll angle for sensor B (deg)
     * @param pitchDisruptionB disruption to apply to pitch angle for sensor B (deg)
     */
    public void initRefiningTest(double rollDisruptionA, double pitchDisruptionA, double factorDisruptionA, 
                                 double rollDisruptionB, double pitchDisruptionB) {
        try {
            
            String path = getClass().getClassLoader().getResource("orekit-data").toURI().getPath();
            DataContext.getDefault().getDataProvidersManager().addProvider(new DirectoryCrawler(new File(path)));
            
            // Initialize refining context
            // ---------------------------
            final String sensorNameA = "SensorA";
            final double incidenceAngleA = -5.0;
            final String dateA = "2016-01-01T11:59:50.0";
            this.pleiadesViewingModelA = new PleiadesViewingModel(sensorNameA, incidenceAngleA, dateA);

            final String sensorNameB = "SensorB";
            final double incidenceAngleB = 0.0;
            final String dateB = "2016-01-01T12:02:50.0";
            this.pleiadesViewingModelB = new PleiadesViewingModel(sensorNameB, incidenceAngleB, dateB);

            PleiadesOrbitModel orbitmodelA =  new PleiadesOrbitModel();
            PleiadesOrbitModel orbitmodelB =  new PleiadesOrbitModel();

            // Sensors's definition: creation of 2 Pleiades viewing models
            // -----------------------------------------------------------
            
            // 1/- Create First Pleiades Viewing Model A
            final AbsoluteDate minDateA =  pleiadesViewingModelA.getMinDate();
            final AbsoluteDate maxDateA =  pleiadesViewingModelA.getMaxDate();
            final AbsoluteDate refDateA = pleiadesViewingModelA.getDatationReference();
            this.lineSensorA =  pleiadesViewingModelA.getLineSensor();

            // ----Satellite position, velocity and attitude: create orbit model A
            BodyShape earthA = TestUtils.createEarth();
            Orbit orbitA  = orbitmodelA.createOrbit(Constants.EIGEN5C_EARTH_MU, refDateA);

            // ----If no LOF Transform Attitude Provider is Nadir Pointing Yaw Compensation
            final double [] rollPoly = {0.0,0.0,0.0};
            final double[] pitchPoly = {0.025,0.0};
            final double[] yawPoly = {0.0,0.0,0.0};
            orbitmodelA.setLOFTransform(rollPoly, pitchPoly, yawPoly, minDateA);

            // ----Satellite attitude
            List<TimeStampedAngularCoordinates> satelliteQListA = orbitmodelA.orbitToQ(orbitA, earthA, minDateA.shiftedBy(-0.0), maxDateA.shiftedBy(+0.0), 0.25);
            final int nbQPoints = 2;

            // ----Position and velocities
            List<TimeStampedPVCoordinates> satellitePVListA = orbitmodelA.orbitToPV(orbitA, earthA, minDateA.shiftedBy(-0.0), maxDateA.shiftedBy(+0.0), 0.25);
            final int nbPVPoints = 8;

            // Rugged A initialization
            // ---------------------
            RuggedBuilder ruggedBuilderA = new RuggedBuilder();

            ruggedBuilderA.addLineSensor(lineSensorA);
            ruggedBuilderA.setAlgorithm(AlgorithmId.IGNORE_DEM_USE_ELLIPSOID);
            ruggedBuilderA.setEllipsoid(EllipsoidId.WGS84, BodyRotatingFrameId.ITRF);
            ruggedBuilderA.setTimeSpan(minDateA,maxDateA, 0.001, 5.0);
            ruggedBuilderA.setTrajectory(InertialFrameId.EME2000, satellitePVListA, nbPVPoints,
                                         CartesianDerivativesFilter.USE_PV, satelliteQListA,
                                         nbQPoints, AngularDerivativesFilter.USE_R);
            ruggedBuilderA.setLightTimeCorrection(false);
            ruggedBuilderA.setAberrationOfLightCorrection(false);

            ruggedBuilderA.setName("RuggedA");

            this.ruggedA = ruggedBuilderA.build();


            // 2/- Create Second Pleiades Viewing Model
            final AbsoluteDate minDateB =  pleiadesViewingModelB.getMinDate();
            final AbsoluteDate maxDateB =  pleiadesViewingModelB.getMaxDate();
            final AbsoluteDate refDateB = pleiadesViewingModelB.getDatationReference();
            this.lineSensorB =  pleiadesViewingModelB.getLineSensor();

            // ----Satellite position, velocity and attitude: create orbit model B
            BodyShape earthB = TestUtils.createEarth();
            Orbit orbitB = orbitmodelB.createOrbit(Constants.EIGEN5C_EARTH_MU, refDateB);

            // ----Satellite attitude
            List<TimeStampedAngularCoordinates> satelliteQListB = orbitmodelB.orbitToQ(orbitB, earthB, minDateB.shiftedBy(-0.0), maxDateB.shiftedBy(+0.0), 0.25);

            // ----Position and velocities
            List<TimeStampedPVCoordinates> satellitePVListB = orbitmodelB.orbitToPV(orbitB, earthB, minDateB.shiftedBy(-0.0), maxDateB.shiftedBy(+0.0), 0.25);

            // Rugged B initialization
            // ---------------------
            RuggedBuilder ruggedBuilderB = new RuggedBuilder();

            ruggedBuilderB.addLineSensor(lineSensorB);
            ruggedBuilderB.setAlgorithm(AlgorithmId.IGNORE_DEM_USE_ELLIPSOID);
            ruggedBuilderB.setEllipsoid(EllipsoidId.WGS84, BodyRotatingFrameId.ITRF);
            ruggedBuilderB.setTimeSpan(minDateB,maxDateB, 0.001, 5.0);
            ruggedBuilderB.setTrajectory(InertialFrameId.EME2000, satellitePVListB, nbPVPoints,
                                         CartesianDerivativesFilter.USE_PV, satelliteQListB,
                                         nbQPoints, AngularDerivativesFilter.USE_R);
            ruggedBuilderB.setLightTimeCorrection(false);
            ruggedBuilderB.setAberrationOfLightCorrection(false);

            ruggedBuilderB.setName("RuggedB");

            this.ruggedB = ruggedBuilderB.build();

            
            // Select parameters to adjust
            // ---------------------------
            RefiningParametersDriver.setSelectedRoll(ruggedA, sensorNameA);
            RefiningParametersDriver.setSelectedPitch(ruggedA, sensorNameA);

            RefiningParametersDriver.setSelectedRoll(ruggedB, sensorNameB);
            RefiningParametersDriver.setSelectedPitch(ruggedB, sensorNameB);
            
            this.parameterToAdjust = 4;

            // Initialize disruptions:
            // -----------------------
            // introduce rotations around instrument axes (roll and pitch angles, scale factor)

            // apply disruptions on physical model (acquisition A)
            RefiningParametersDriver.applyDisruptionsRoll(ruggedA, sensorNameA, FastMath.toRadians(rollDisruptionA));
            RefiningParametersDriver.applyDisruptionsPitch(ruggedA, sensorNameA, FastMath.toRadians(pitchDisruptionA));
            RefiningParametersDriver.applyDisruptionsFactor(ruggedA, sensorNameA, factorDisruptionA);
            
            // apply disruptions on physical model (acquisition B)
            RefiningParametersDriver.applyDisruptionsRoll(ruggedB, sensorNameB, FastMath.toRadians(rollDisruptionB));
            RefiningParametersDriver.applyDisruptionsPitch(ruggedB, sensorNameB, FastMath.toRadians(pitchDisruptionB));
            
            
        } catch (OrekitException oe) {
            Assert.fail(oe.getLocalizedMessage());
        } catch (URISyntaxException use) {
            Assert.fail(use.getLocalizedMessage());
        }
    }
    
    /**
     * Get the list of Rugged instances
     * @return rugged instances as list
     */
    public List<Rugged> getRuggedList(){
        
        List<Rugged> ruggedList = Arrays.asList(ruggedA, ruggedB);
        return ruggedList;
    }
    

    /** Compute the distances between LOS of two real pixels (one from sensor A and one from sensor B)
     * @param realPixelA real pixel from sensor A
     * @param realPixelB real pixel from sensor B
     * @return the distances of two real pixels computed between LOS and to the ground
     */
    public double[] computeDistancesBetweenLOS(final SensorPixel realPixelA, final SensorPixel realPixelB) {
        
        final SpacecraftToObservedBody scToBodyA = ruggedA.getScToBody();

        final AbsoluteDate realDateA = lineSensorA.getDate(realPixelA.getLineNumber());
        final AbsoluteDate realDateB = lineSensorB.getDate(realPixelB.getLineNumber());
        
        final double[] distanceLOSB = ruggedB.distanceBetweenLOS(
                                           lineSensorA, realDateA, realPixelA.getPixelNumber(), 
                                           scToBodyA,
                                           lineSensorB, realDateB, realPixelB.getPixelNumber());
        
        return distanceLOSB;
    }
    
    /** Compute the distances with derivatives between LOS of two real pixels (one from sensor A and one from sensor B)
     * @param realPixelA real pixel from sensor A
     * @param realPixelB real pixel from sensor B
     * @return the distances of two real pixels computed between LOS and to the ground
     * @deprecated as of 2.2, replaced by {@link #computeDistancesBetweenLOSGradient(SensorPixel, SensorPixel, double, double)}
     */
    public DerivativeStructure[] computeDistancesBetweenLOSDerivatives(final SensorPixel realPixelA, final SensorPixel realPixelB,
                                                                       final double losDistance, final double earthDistance) 
        throws NoSuchMethodException, SecurityException, IllegalAccessException, IllegalArgumentException, InvocationTargetException {
        final Gradient[] gradient = computeDistancesBetweenLOSGradient(realPixelA, realPixelB, losDistance, earthDistance);
        final DerivativeStructure[] ds = new DerivativeStructure[gradient.length];
        for (int i = 0; i < gradient.length; ++i) {
            ds[i] = gradient[i].toDerivativeStructure();
        }
        return ds;
    }

    /** Compute the distances with derivatives between LOS of two real pixels (one from sensor A and one from sensor B)
     * @param realPixelA real pixel from sensor A
     * @param realPixelB real pixel from sensor B
     * @return the distances of two real pixels computed between LOS and to the ground
     * @since 2.2
     */
    public Gradient[] computeDistancesBetweenLOSGradient(final SensorPixel realPixelA, final SensorPixel realPixelB,
                                                         final double losDistance, final double earthDistance) 
        throws NoSuchMethodException, SecurityException, IllegalAccessException, IllegalArgumentException, InvocationTargetException {
            
        final SpacecraftToObservedBody scToBodyA = ruggedA.getScToBody();

        final AbsoluteDate realDateA = lineSensorA.getDate(realPixelA.getLineNumber());
        final AbsoluteDate realDateB = lineSensorB.getDate(realPixelB.getLineNumber());
        
        final List<LineSensor> sensors = new ArrayList<LineSensor>();
        sensors.addAll(ruggedA.getLineSensors());
        sensors.addAll(ruggedB.getLineSensors());
        
        final List<Rugged> ruggedList = new ArrayList<Rugged>();
        ruggedList.add(ruggedA);
        ruggedList.add(ruggedB);
        
        // prepare generator
        final Observables measurements = new Observables(2);
        SensorToSensorMapping interMapping = new SensorToSensorMapping(lineSensorA.getName(), ruggedA.getName(), lineSensorB.getName(), ruggedB.getName());
        interMapping.addMapping(realPixelA, realPixelB, losDistance, earthDistance);
        measurements.addInterMapping(interMapping);
        
        
        InterSensorsOptimizationProblemBuilder optimizationPbBuilder = new InterSensorsOptimizationProblemBuilder(sensors, measurements, ruggedList);
        java.lang.reflect.Method createGenerator = InterSensorsOptimizationProblemBuilder.class.getSuperclass().getDeclaredMethod("createGenerator", List.class);
        createGenerator.setAccessible(true);
        
        List<LineSensor> listLineSensor = new ArrayList<LineSensor>();
        listLineSensor.addAll(ruggedA.getLineSensors());
        listLineSensor.addAll(ruggedB.getLineSensors());

        @SuppressWarnings("unchecked")
        DerivativeGenerator<Gradient> generator = (DerivativeGenerator<Gradient>) createGenerator.invoke(optimizationPbBuilder, listLineSensor);

        return ruggedB.distanceBetweenLOSderivatives(lineSensorA, realDateA, realPixelA.getPixelNumber(), 
                                                     scToBodyA,
                                                     lineSensorB, realDateB, realPixelB.getPixelNumber(),
                                                     generator);
        
    }
    
    /** Generate noisy measurements (sensor to sensor mapping)
     * @param lineSampling line sampling
     * @param pixelSampling pixel sampling
     * @param earthConstraintWeight the earth constrint weight
     * @param earthConstraintPostponed flag to tell if the earth constraint weight is set at construction (false) or after (true) - For JUnit coverage purpose
     */
    public Observables generateNoisyPoints(final int lineSampling, final int pixelSampling, final double earthConstraintWeight, final boolean earthConstraintPostponed) {

        // Outliers control
        final double outlierValue = 1.e+2;
        
        // Generate measurements with constraints on Earth distance and outliers control
        
        // Generate reference mapping, with Earth distance constraints.
        // Weighting will be applied as follow :
        //     (1-earthConstraintWeight) for losDistance weighting
        //     earthConstraintWeight for earthDistance weighting
        SensorToSensorMapping interMapping;
        if (! earthConstraintPostponed) {
            interMapping = new SensorToSensorMapping(lineSensorA.getName(), ruggedA.getName(), 
                    lineSensorB.getName(), ruggedB.getName(), 
                    earthConstraintWeight);
        } else { // used for JUnit coverage purpose
            interMapping = new SensorToSensorMapping(lineSensorA.getName(), ruggedA.getName(), 
                    lineSensorB.getName(), ruggedB.getName());
            // set the earthConstraintWeight after construction
            interMapping.setBodyConstraintWeight(earthConstraintWeight);
        }

        // Observables which contains sensor to sensor mapping
        Observables observables = new Observables(2);
        
        // Generation noisy measurements
        // distribution: gaussian(0), vector dimension: 2
        final double meanA[] = { 5.0, 5.0 };
        final double stdA[]  = { 0.1, 0.1 };
        final double meanB[] = { 0.0, 0.0 };
        final double stdB[]  = { 0.1, 0.1 };

        // Seed has been fixed for tests purpose
        final GaussianRandomGenerator rngA = new GaussianRandomGenerator(new Well19937a(0xefac03d9be4d24b9l));
        final UncorrelatedRandomVectorGenerator rvgA = new UncorrelatedRandomVectorGenerator(meanA, stdA, rngA);

        // Seed has been fixed for tests purpose
        final GaussianRandomGenerator rngB = new GaussianRandomGenerator(new Well19937a(0xdf1c03d9be0b34b9l));
        final UncorrelatedRandomVectorGenerator rvgB = new UncorrelatedRandomVectorGenerator(meanB, stdB, rngB);

        
        // Search the sensor pixel seeing point
        final int minLine = 0;
        final int maxLine = pleiadesViewingModelB.getDimension() - 1;
        
        final String sensorNameB = lineSensorB.getName();

        for (double line = 0; line < pleiadesViewingModelA.getDimension(); line += lineSampling) {

            final AbsoluteDate dateA = lineSensorA.getDate(line);
            
            for (double pixelA = 0; pixelA < lineSensorA.getNbPixels(); pixelA += pixelSampling) {
                
                final GeodeticPoint gpA = ruggedA.directLocation(dateA, lineSensorA.getPosition(),
                                                                 lineSensorA.getLOS(dateA, pixelA));
                final SensorPixel sensorPixelB = ruggedB.inverseLocation(sensorNameB, gpA, minLine, maxLine);

                // We need to test if the sensor pixel is found in the prescribed lines
                // otherwise the sensor pixel is null
                if (sensorPixelB != null) {
                    
                    final AbsoluteDate dateB = lineSensorB.getDate(sensorPixelB.getLineNumber());
                    final double pixelB = sensorPixelB.getPixelNumber();

                    // Get spacecraft to body transform of Rugged instance A
                    final SpacecraftToObservedBody scToBodyA = ruggedA.getScToBody();

                    final GeodeticPoint gpB = ruggedB.directLocation(dateB, lineSensorB.getPosition(),
                                                                     lineSensorB.getLOS(dateB, pixelB));
                    final double geoDistance = computeDistanceInMeter(gpA, gpB);
                    
                    // Create the inter mapping if distance is below outlier value
                    if (geoDistance < outlierValue) {

                        final double[] vecRandomA = rvgA.nextVector();
                        final double[] vecRandomB = rvgB.nextVector();

                        final SensorPixel realPixelA = new SensorPixel(line + vecRandomA[0], pixelA + vecRandomA[1]);
                        final SensorPixel realPixelB = new SensorPixel(sensorPixelB.getLineNumber() + vecRandomB[0],
                                                                       sensorPixelB.getPixelNumber() + vecRandomB[1]);
                        
                        final AbsoluteDate realDateA = lineSensorA.getDate(realPixelA.getLineNumber());
                        final AbsoluteDate realDateB = lineSensorB.getDate(realPixelB.getLineNumber());
                        
                        final double[] distanceLOSB = ruggedB.distanceBetweenLOS(lineSensorA, realDateA, realPixelA.getPixelNumber(), scToBodyA,
                                                                                 lineSensorB, realDateB, realPixelB.getPixelNumber());
                        final Double losDistance = 0.0;
                        final Double earthDistance = distanceLOSB[1];

                        interMapping.addMapping(realPixelA, realPixelB, losDistance, earthDistance);

                    } // end test if geoDistance < outlierValue
                } // end test if sensorPixelB != null
                
            } // end loop on pixel of sensorA
        } // end loop on line of sensorA

        observables.addInterMapping(interMapping);
        return observables;
    }
    
    
    /** Generate simple noisy measurements (sensor to sensor mapping)
     * @param lineSampling line sampling
     * @param pixelSampling pixel sampling
     * @param earthConstraintWeight the earth constrint weight
     * @param earthConstraintPostponed flag to tell if the earth constraint weight is set at construction (false) or after (true) - For JUnit coverage purpose
     */
    public Observables generateSimpleInterMapping(final int lineSampling, final int pixelSampling, final double earthConstraintWeight, final boolean earthConstraintPostponed) {

        // Outliers control
        final double outlierValue = 1.e+2;
        
        // Generate measurements with constraints on Earth distance and outliers control
        
        // Generate reference mapping, with Earth distance constraints.
        // Weighting will be applied as follow :
        //     (1-earthConstraintWeight) for losDistance weighting
        //     earthConstraintWeight for earthDistance weighting
        SensorToSensorMapping interMapping;
        if (! earthConstraintPostponed) {
            interMapping = new SensorToSensorMapping(lineSensorA.getName(), lineSensorB.getName(), earthConstraintWeight);
        } else { // used for JUnit coverage purpose
            interMapping = new SensorToSensorMapping(lineSensorA.getName(),  lineSensorB.getName());
            // set the earthConstraintWeight after construction
            interMapping.setBodyConstraintWeight(earthConstraintWeight);
        }

        // Observables which contains sensor to sensor mapping
        Observables observables = new Observables(2);
        
        // Generation noisy measurements
        // distribution: gaussian(0), vector dimension: 2
        final double meanA[] = { 5.0, 5.0 };
        final double stdA[]  = { 0.1, 0.1 };
        final double meanB[] = { 0.0, 0.0 };
        final double stdB[]  = { 0.1, 0.1 };

        // Seed has been fixed for tests purpose
        final GaussianRandomGenerator rngA = new GaussianRandomGenerator(new Well19937a(0xefac03d9be4d24b9l));
        final UncorrelatedRandomVectorGenerator rvgA = new UncorrelatedRandomVectorGenerator(meanA, stdA, rngA);

        // Seed has been fixed for tests purpose
        final GaussianRandomGenerator rngB = new GaussianRandomGenerator(new Well19937a(0xdf1c03d9be0b34b9l));
        final UncorrelatedRandomVectorGenerator rvgB = new UncorrelatedRandomVectorGenerator(meanB, stdB, rngB);

        
        // Search the sensor pixel seeing point
        final int minLine = 0;
        final int maxLine = pleiadesViewingModelB.getDimension() - 1;
        
        final String sensorNameB = lineSensorB.getName();

        for (double line = 0; line < pleiadesViewingModelA.getDimension(); line += lineSampling) {

            final AbsoluteDate dateA = lineSensorA.getDate(line);
            
            for (double pixelA = 0; pixelA < lineSensorA.getNbPixels(); pixelA += pixelSampling) {
                
                final GeodeticPoint gpA = ruggedA.directLocation(dateA, lineSensorA.getPosition(),
                                                                 lineSensorA.getLOS(dateA, pixelA));
                final SensorPixel sensorPixelB = ruggedB.inverseLocation(sensorNameB, gpA, minLine, maxLine);

                // We need to test if the sensor pixel is found in the prescribed lines
                // otherwise the sensor pixel is null
                if (sensorPixelB != null) {
                    
                    final AbsoluteDate dateB = lineSensorB.getDate(sensorPixelB.getLineNumber());
                    final double pixelB = sensorPixelB.getPixelNumber();

                    final GeodeticPoint gpB = ruggedB.directLocation(dateB, lineSensorB.getPosition(),
                                                                     lineSensorB.getLOS(dateB, pixelB));
                    final double geoDistance = computeDistanceInMeter(gpA, gpB);
                    
                    // Create the inter mapping if distance is below outlier value
                    if (geoDistance < outlierValue) {

                        final double[] vecRandomA = rvgA.nextVector();
                        final double[] vecRandomB = rvgB.nextVector();

                        final SensorPixel realPixelA = new SensorPixel(line + vecRandomA[0], pixelA + vecRandomA[1]);
                        final SensorPixel realPixelB = new SensorPixel(sensorPixelB.getLineNumber() + vecRandomB[0],
                                                                       sensorPixelB.getPixelNumber() + vecRandomB[1]);
                        // dummy value for JUnit test purpose
                        final Double losDistance = FastMath.abs(vecRandomA[0]);

                        interMapping.addMapping(realPixelA, realPixelB, losDistance);

                    } // end test if geoDistance < outlierValue
                } // end test if sensorPixelB != null
                
            } // end loop on pixel of sensorA
        } // end loop on line of sensorA

        observables.addInterMapping(interMapping);
        return observables;
    }
    
    /** Compute a geodetic distance in meters between two geodetic points.
     * @param geoPoint1 first geodetic point (rad)
     * @param geoPoint2 second geodetic point (rad)
     * @return distance in meters
     */
    private static double computeDistanceInMeter(final GeodeticPoint geoPoint1, final GeodeticPoint geoPoint2) {

        // get vectors on unit sphere from angular coordinates
        final Vector3D p1 = new Vector3D(geoPoint1.getLatitude(), geoPoint1.getLongitude());
        final Vector3D p2 = new Vector3D(geoPoint2.getLatitude(), geoPoint2.getLongitude());
        return Constants.WGS84_EARTH_EQUATORIAL_RADIUS * Vector3D.angle(p1, p2);
    }

    /** Get the number of parameters to adjust
     * @return number of parameters to adjust
     */
    public int getParameterToAdjust() {
        return parameterToAdjust;
    }
    
    @After
    public void tearDown() {
    }
}
