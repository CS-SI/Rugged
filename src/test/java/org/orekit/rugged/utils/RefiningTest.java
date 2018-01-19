package org.orekit.rugged.utils;

import java.io.File;
import java.lang.reflect.InvocationTargetException;
import java.net.URISyntaxException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.junit.After;
import org.junit.Assert;

import org.hipparchus.analysis.differentiation.DerivativeStructure;
import org.hipparchus.geometry.euclidean.threed.Rotation;
import org.hipparchus.geometry.euclidean.threed.RotationConvention;
import org.hipparchus.geometry.euclidean.threed.RotationOrder;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.ode.nonstiff.DormandPrince853Integrator;
import org.hipparchus.random.GaussianRandomGenerator;
import org.hipparchus.random.UncorrelatedRandomVectorGenerator;
import org.hipparchus.random.Well19937a;
import org.hipparchus.util.FastMath;

import org.orekit.attitudes.AttitudeProvider;
import org.orekit.attitudes.LofOffset;
import org.orekit.attitudes.NadirPointing;
import org.orekit.attitudes.TabulatedLofOffset;
import org.orekit.attitudes.YawCompensation;
import org.orekit.bodies.BodyShape;
import org.orekit.bodies.CelestialBodyFactory;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.data.DataProvidersManager;
import org.orekit.data.DirectoryCrawler;
import org.orekit.errors.OrekitException;
import org.orekit.forces.gravity.HolmesFeatherstoneAttractionModel;
import org.orekit.forces.gravity.ThirdBodyAttraction;
import org.orekit.forces.gravity.potential.NormalizedSphericalHarmonicsProvider;
import org.orekit.frames.Frame;
import org.orekit.frames.FramesFactory;
import org.orekit.frames.LOFType;
import org.orekit.orbits.CircularOrbit;
import org.orekit.orbits.Orbit;
import org.orekit.orbits.OrbitType;
import org.orekit.orbits.PositionAngle;
import org.orekit.propagation.Propagator;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.analytical.KeplerianPropagator;
import org.orekit.propagation.numerical.NumericalPropagator;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeScale;
import org.orekit.time.TimeScalesFactory;
import org.orekit.utils.AngularDerivativesFilter;
import org.orekit.utils.CartesianDerivativesFilter;
import org.orekit.utils.Constants;
import org.orekit.utils.ParameterDriver;
import org.orekit.utils.TimeStampedAngularCoordinates;
import org.orekit.utils.TimeStampedPVCoordinates;

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
import org.orekit.rugged.errors.RuggedException;
import org.orekit.rugged.linesensor.LineDatation;
import org.orekit.rugged.linesensor.LineSensor;
import org.orekit.rugged.linesensor.LinearLineDatation;
import org.orekit.rugged.linesensor.SensorPixel;
import org.orekit.rugged.los.FixedRotation;
import org.orekit.rugged.los.FixedZHomothety;
import org.orekit.rugged.los.LOSBuilder;
import org.orekit.rugged.los.TimeDependentLOS;


/** Initialization for refining Junit tests.
 * @author Guylaine Prat
 */
public class RefiningTest {

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
    static final double defaultPitchDisruptionB = -0.0008;
    
    /**
     * Initialize refining tests with default values for disruptions on sensors characteristics
     * @throws RuggedException
     */
    public void initRefiningTest() throws RuggedException {
        
        initRefiningTest(defaultRollDisruptionA, defaultPitchDisruptionA, defaultFactorDisruptionA, defaultPitchDisruptionB);
    }

    /** Initialize refining tests with disruption on sensors characteristics
     * @param rollDisruptionA disruption to apply to roll angle for sensor A (deg)
     * @param pitchDisruptionA disruption to apply to pitch angle for sensor A (deg)
     * @param factorDisruptionA disruption to apply to homothety factor for sensor A
     * @param pitchDisruptionB disruption to apply to pitch angle for sensor B (deg)
     * @throws RuggedException
     */
    public void initRefiningTest(double rollDisruptionA, double pitchDisruptionA, double factorDisruptionA, double pitchDisruptionB) throws RuggedException {
        try {
            
            String path = getClass().getClassLoader().getResource("orekit-data").toURI().getPath();
            DataProvidersManager.getInstance().addProvider(new DirectoryCrawler(new File(path)));
            
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

            OrbitModel orbitmodelA =  new OrbitModel();
            OrbitModel orbitmodelB =  new OrbitModel();

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
            setSelectedRoll(ruggedA, sensorNameA);
            setSelectedPitch(ruggedA, sensorNameA);

            setSelectedRoll(ruggedB, sensorNameB);
            setSelectedPitch(ruggedB, sensorNameB);
            
            this.parameterToAdjust = 4;

            // Initialize disruptions:
            // -----------------------
            // introduce rotations around instrument axes (roll and pitch angles, scale factor)

            // apply disruptions on physical model (acquisition A)
            applyDisruptionsRoll(ruggedA, sensorNameA, FastMath.toRadians(rollDisruptionA));
            applyDisruptionsPitch(ruggedA, sensorNameA, FastMath.toRadians(pitchDisruptionA));
            applyDisruptionsFactor(ruggedA, sensorNameA, factorDisruptionA);
            
            // apply disruptions on physical model (acquisition B)
            applyDisruptionsPitch(ruggedB, sensorNameB, FastMath.toRadians(pitchDisruptionB));
            
            
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
    
    /** Apply disruptions on acquisition for roll angle
     * @param rugged Rugged instance
     * @param sensorName line sensor name
     * @param rollValue rotation on roll value
     */
    private void applyDisruptionsRoll(final Rugged rugged, final String sensorName, final double rollValue) 
        throws OrekitException, RuggedException {

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
    private void applyDisruptionsPitch(final Rugged rugged, final String sensorName, final double pitchValue) 
        throws OrekitException, RuggedException {

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
    private void applyDisruptionsFactor(final Rugged rugged, final String sensorName, final double factorValue)
        throws OrekitException, RuggedException {

        rugged.
        getLineSensor(sensorName).
        getParametersDrivers().
        filter(driver -> driver.getName().equals(factorName)).
        findFirst().get().setValue(factorValue);
    }
    
    /** Select roll angle to adjust
     * @param rugged Rugged instance
     * @param sensorName line sensor name
     * @throws OrekitException, RuggedException
     */
    private void setSelectedRoll(final Rugged rugged, final String sensorName) throws OrekitException, RuggedException {

        ParameterDriver rollDriver =
                rugged.getLineSensor(sensorName).getParametersDrivers().
                filter(driver -> driver.getName().equals(sensorName + rollSuffix)).findFirst().get();
        rollDriver.setSelected(true);
    }
    
    /** Select pitch angle to adjust
     * @param rugged Rugged instance
     * @param sensorName line sensor name
     * @throws OrekitException, RuggedException
     */
    private void setSelectedPitch(final Rugged rugged, final String sensorName) throws OrekitException, RuggedException {
        
        ParameterDriver pitchDriver =
                rugged.getLineSensor(sensorName).getParametersDrivers().
                filter(driver -> driver.getName().equals(sensorName + pitchSuffix)).findFirst().get();
        pitchDriver.setSelected(true);
    }

    /** Select scale factor to adjust
     * @param rugged Rugged instance
     * @param sensorName line sensor name
     * @throws OrekitException, RuggedException
     */
    private void setSelectedFactor(final Rugged rugged, final String sensorName) throws OrekitException, RuggedException {

        ParameterDriver factorDriver =
                rugged.getLineSensor(sensorName).getParametersDrivers().
                filter(driver -> driver.getName().equals(factorName)).findFirst().get();
        factorDriver.setSelected(true);
    }

    /** Compute the distances between LOS of two real pixels (one from sensor A and one from sensor B)
     * @param realPixelA real pixel from sensor A
     * @param realPixelB real pixel from sensor B
     * @return the distances of two real pixels computed between LOS and to the ground
     * @throws RuggedException
     */
    public double[] computeDistancesBetweenLOS(final SensorPixel realPixelA, final SensorPixel realPixelB) throws RuggedException {
        
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
     * @throws RuggedException
     * @throws SecurityException 
     * @throws NoSuchMethodException 
     * @throws InvocationTargetException 
     * @throws IllegalArgumentException 
     * @throws IllegalAccessException 
     */
    public DerivativeStructure[] computeDistancesBetweenLOSDerivatives(final SensorPixel realPixelA, final SensorPixel realPixelB,
                                                                       double losDistance, double earthDistance) 
        throws RuggedException, NoSuchMethodException, SecurityException, IllegalAccessException, IllegalArgumentException, InvocationTargetException {
        
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

        DSGenerator generator = (DSGenerator) createGenerator.invoke(optimizationPbBuilder, listLineSensor);

        final DerivativeStructure[] distanceLOSwithDS = ruggedB.distanceBetweenLOSderivatives(
                                           lineSensorA, realDateA, realPixelA.getPixelNumber(), 
                                           scToBodyA,
                                           lineSensorB, realDateB, realPixelB.getPixelNumber(),
                                           generator);
        
        return distanceLOSwithDS;
    }
    
    /** Generate noisy measurements (sensor to sensor mapping)
     * @param lineSampling line sampling
     * @param pixelSampling pixel sampling
     * @throws RuggedException
     */
    public Observables generateNoisyPoints(final int lineSampling, final int pixelSampling) throws RuggedException {

        // Outliers control
        final double outlierValue = 1.e+2;
        
        // Earth constraint weight
        final double earthConstraintWeight = 0.1;

        // Number of measurements
        int measurementCount = 0;

        // Generate measurements with constraints on Earth distance and outliers control
        
        // Generate reference mapping, with Earth distance constraints.
        // Weighting will be applied as follow :
        //     (1-earthConstraintWeight) for losDistance weighting
        //     earthConstraintWeight for earthDistance weighting
        SensorToSensorMapping interMapping = new SensorToSensorMapping(lineSensorA.getName(), ruggedA.getName(), 
                                                                       lineSensorB.getName(), ruggedB.getName(), 
                                                                       earthConstraintWeight);
        
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

                        // Increment the number of measurements
                        measurementCount++;
                        
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


    @After
    public void tearDown() {
    }
    
    /** Get the number of parameters to adjust
     * @return number of parameters to adjust
     */
    public int getParameterToAdjust() {
        return parameterToAdjust;
    }
}



/**
 * Class to compute orbit for refining tests
 */
class OrbitModel {

    /** Flag to change Attitude Law (by default Nadir Pointing Yaw Compensation). */
    private boolean userDefinedLOFTransform;

    /** User defined Roll law. */
    private double[] lofTransformRollPoly;

    /** User defined Pitch law. */
    private double[] lofTransformPitchPoly;

    /** User defined Yaw law. */
    private double[] lofTransformYawPoly;

    /** Reference date. */
    private AbsoluteDate refDate;

    
    
    /** Default constructor.
     */
    public OrbitModel() {
        userDefinedLOFTransform = false;
    }

    /** Create a circular orbit.
     */
    public Orbit createOrbit(final double mu, final AbsoluteDate date) throws OrekitException {
        
        // the following orbital parameters have been computed using
        // Orekit tutorial about phasing, using the following configuration:
        //
        // orbit.date = 2012-01-01T00:00:00.000
        // phasing.orbits.number = 143
        // phasing.days.number = 10
        // sun.synchronous.reference.latitude = 0
        // sun.synchronous.reference.ascending = false
        // sun.synchronous.mean.solar.time = 10:30:00
        // gravity.field.degree = 12
        // gravity.field.order = 12

        final Frame eme2000 = FramesFactory.getEME2000();
        return new CircularOrbit(694000.0 + Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                 -4.029194321683225E-4,
                                 0.0013530362644647786,
                                 FastMath.toRadians(98.2), // Pleiades inclination 98.2 deg
                                 FastMath.toRadians(-86.47 + 180),
                                 FastMath.toRadians(135.9 + 0.3),
                                 PositionAngle.TRUE,
                                 eme2000,
                                 date,
                                 mu);
    }

    /** Set the Local Orbital Frame characteristics.
     */
    public void setLOFTransform(final double[] rollPoly, final double[] pitchPoly,
                                final double[] yawPoly, final AbsoluteDate date) {

        this.userDefinedLOFTransform = true;
        lofTransformRollPoly = rollPoly.clone();
        lofTransformPitchPoly = pitchPoly.clone();
        lofTransformYawPoly = yawPoly.clone();
        this.refDate = date;
    }

    /** Recompute the polynom coefficients with shift.
     */
    private double getPoly(final double[] poly, final double shift) {
        
        double val = 0.0;
        for (int coef = 0; coef < poly.length; coef++) {
            val = val + poly[coef] * FastMath.pow(shift, coef);
        }
        return val;
    }

    /** Get the offset.
     */
   private Rotation getOffset(final BodyShape earth, final Orbit orbit, final double shift)
        throws OrekitException {
        
        final LOFType type = LOFType.VVLH;
        final double roll = getPoly(lofTransformRollPoly, shift);
        final double pitch = getPoly(lofTransformPitchPoly, shift);
        final double yaw = getPoly(lofTransformYawPoly, shift);

        final LofOffset law = new LofOffset(orbit.getFrame(), type, RotationOrder.XYZ,
                                      roll, pitch, yaw);
        final Rotation offsetAtt = law.
                                   getAttitude(orbit, orbit.getDate().shiftedBy(shift), orbit.getFrame()).
                                   getRotation();
        final NadirPointing nadirPointing = new NadirPointing(orbit.getFrame(), earth);
        final Rotation nadirAtt = nadirPointing.
                                  getAttitude(orbit, orbit.getDate().getDate().shiftedBy(shift), orbit.getFrame()).
                                  getRotation();
        final Rotation offsetProper = offsetAtt.compose(nadirAtt.revert(), RotationConvention.VECTOR_OPERATOR);

        return offsetProper;
    }

   /** Create the attitude provider.
    */
    public AttitudeProvider createAttitudeProvider(final BodyShape earth, final Orbit orbit)
        throws OrekitException {

        if (userDefinedLOFTransform) {
            final LOFType type = LOFType.VVLH;

            final List<TimeStampedAngularCoordinates> list = new ArrayList<TimeStampedAngularCoordinates>();

            for (double shift = -10.0; shift < +10.0; shift += 1e-2) {
                list.add(new TimeStampedAngularCoordinates(refDate.shiftedBy(shift), 
                                                           getOffset(earth, orbit, shift),
                                                           Vector3D.ZERO,
                                                           Vector3D.ZERO));
            }

            final TabulatedLofOffset tabulated = new TabulatedLofOffset(orbit.getFrame(), type, list,
                                                                        2, AngularDerivativesFilter.USE_R);

            return tabulated;
        } else {
            return new YawCompensation(orbit.getFrame(), new NadirPointing(orbit.getFrame(), earth));
        }
    }

    /** Create the orbit propagator.
     */
   public Propagator createPropagator(final BodyShape earth,
                                       final NormalizedSphericalHarmonicsProvider gravityField,
                                       final Orbit orbit)
        throws OrekitException {

        final AttitudeProvider attitudeProvider = createAttitudeProvider(earth, orbit);

        final SpacecraftState state =
                new SpacecraftState(orbit,
                                    attitudeProvider.getAttitude(orbit, orbit.getDate(), orbit.getFrame()), 1180.0);

        // numerical model for improving orbit
        final OrbitType type = OrbitType.CIRCULAR;
        final double[][] tolerances = NumericalPropagator.tolerances(0.1, orbit, type);
        final DormandPrince853Integrator integrator =
                     new DormandPrince853Integrator(1.0e-4 * orbit.getKeplerianPeriod(),
                                                    1.0e-1 * orbit.getKeplerianPeriod(),
                                                    tolerances[0],
                                                    tolerances[1]);
        integrator.setInitialStepSize(1.0e-2 * orbit.getKeplerianPeriod());

        final NumericalPropagator numericalPropagator = new NumericalPropagator(integrator);
        numericalPropagator.addForceModel(new HolmesFeatherstoneAttractionModel(earth.getBodyFrame(), gravityField));
        numericalPropagator .addForceModel(new ThirdBodyAttraction(CelestialBodyFactory.getSun()));
        numericalPropagator .addForceModel(new ThirdBodyAttraction(CelestialBodyFactory.getMoon()));
        numericalPropagator.setOrbitType(type);
        numericalPropagator.setInitialState(state);
        numericalPropagator.setAttitudeProvider(attitudeProvider);

        return numericalPropagator;
    }

   /** Generate the orbit.
    */
   public List<TimeStampedPVCoordinates> orbitToPV(final Orbit orbit, final BodyShape earth,
                                                    final AbsoluteDate minDate, final AbsoluteDate maxDate,
                                                    final double step)
        throws OrekitException {
        
        final Propagator propagator = new KeplerianPropagator(orbit);

        propagator.setAttitudeProvider(createAttitudeProvider(earth, orbit));

        propagator.propagate(minDate);
        final List<TimeStampedPVCoordinates> list = new ArrayList<TimeStampedPVCoordinates>();
        propagator.setMasterMode(step,
                                 (currentState, isLast) ->
                                 list.add(new TimeStampedPVCoordinates(currentState.getDate(),
                                                                       currentState.getPVCoordinates().getPosition(),
                                                                       currentState.getPVCoordinates().getVelocity(),
                                                                       Vector3D.ZERO)));
        propagator.propagate(maxDate);

        return list;
    }

   /** Generate the attitude.
    */
   public List<TimeStampedAngularCoordinates> orbitToQ(final Orbit orbit, final BodyShape earth,
                                                        final AbsoluteDate minDate, final AbsoluteDate maxDate,
                                                        final double step)
        throws OrekitException {
        
        final Propagator propagator = new KeplerianPropagator(orbit);
        propagator.setAttitudeProvider(createAttitudeProvider(earth, orbit));
        propagator.propagate(minDate);
        final List<TimeStampedAngularCoordinates> list = new ArrayList<>();
        propagator.setMasterMode(step,
                                 (currentState, isLast) ->
                                 list.add(new TimeStampedAngularCoordinates(currentState.getDate(),
                                                                            currentState.getAttitude().getRotation(),
                                                                            Vector3D.ZERO,
                                                                            Vector3D.ZERO)));
        propagator.propagate(maxDate);

        return list;
    }
}  // end class OrbitModel


/**
 * Pleiades viewing model class definition.
 */
class PleiadesViewingModel {

    /** Pleiades parameters. */
    private static final double FOV = 1.65; // 20km - alt 694km
    private static final int DIMENSION = 40000;
    private static final double LINE_PERIOD =  1.e-4; 

    private double incidenceAngle;
    private LineSensor lineSensor;
    private String referenceDate;
    private String sensorName;


    /** Simple constructor.
     * <p>
     *  initialize PleiadesViewingModel with
     *  sensorName="line", incidenceAngle = 0.0, date = "2016-01-01T12:00:00.0"
     * </p>
     */
    public PleiadesViewingModel(final String sensorName) throws RuggedException, OrekitException {

        this(sensorName, 0.0, "2016-01-01T12:00:00.0");
    }

    /** PleiadesViewingModel constructor.
     * @param sensorName sensor name
     * @param incidenceAngle incidence angle
     * @param referenceDate reference date
     */
    public PleiadesViewingModel(final String sensorName, final double incidenceAngle, final String referenceDate)
            throws RuggedException, OrekitException {

        this.sensorName = sensorName;
        this.referenceDate = referenceDate;
        this.incidenceAngle = incidenceAngle;
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

        final LOSBuilder losBuilder = rawLOS(new Rotation(Vector3D.PLUS_I,
                                                          FastMath.toRadians(incidenceAngle),
                                                          RotationConvention.VECTOR_OPERATOR).applyTo(Vector3D.PLUS_K),
                                             Vector3D.PLUS_I, FastMath.toRadians(FOV / 2), DIMENSION);

        losBuilder.addTransform(new FixedRotation(sensorName + RefiningTest.rollSuffix,  Vector3D.MINUS_I, 0.00));
        losBuilder.addTransform(new FixedRotation(sensorName + RefiningTest.pitchSuffix, Vector3D.MINUS_J, 0.00));

        // factor is a common parameters shared between all Pleiades models
        losBuilder.addTransform(new FixedZHomothety(RefiningTest.factorName, 1.0));

        return losBuilder.build();
    }


    /** Get the reference date.
     */
    public AbsoluteDate getDatationReference() throws OrekitException {

        // We use Orekit for handling time and dates, and Rugged for defining the datation model:
        final TimeScale utc = TimeScalesFactory.getUTC();

        return new AbsoluteDate(referenceDate, utc);
    }

    /** Get the min date.
     */
    public AbsoluteDate getMinDate() throws RuggedException {
        return lineSensor.getDate(0);
    }

    /** Get the max date.
     */
    public AbsoluteDate getMaxDate() throws RuggedException {
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
    private void createLineSensor() throws RuggedException, OrekitException {

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
    
} // end class PleiadesViewingModel
