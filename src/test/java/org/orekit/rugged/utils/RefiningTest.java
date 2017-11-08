package org.orekit.rugged.utils;

import java.io.File;
import java.net.URISyntaxException;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;

import org.hipparchus.geometry.euclidean.threed.Rotation;
import org.hipparchus.geometry.euclidean.threed.RotationConvention;
import org.hipparchus.geometry.euclidean.threed.RotationOrder;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.ode.nonstiff.DormandPrince853Integrator;
import org.hipparchus.random.GaussianRandomGenerator;
import org.hipparchus.random.UncorrelatedRandomVectorGenerator;
import org.hipparchus.random.Well19937a;
import org.hipparchus.util.FastMath;
import org.junit.After;
import org.junit.Assert;
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
import org.orekit.rugged.TestUtils;
import org.orekit.rugged.adjustment.measurements.Observables;
import org.orekit.rugged.adjustment.measurements.SensorToSensorMapping;
import org.orekit.rugged.api.AlgorithmId;
import org.orekit.rugged.api.BodyRotatingFrameId;
import org.orekit.rugged.api.EllipsoidId;
import org.orekit.rugged.api.InertialFrameId;
import org.orekit.rugged.api.Rugged;
import org.orekit.rugged.api.RuggedBuilder;
import org.orekit.rugged.errors.RuggedException;
import org.orekit.rugged.errors.RuggedExceptionWrapper;
import org.orekit.rugged.errors.RuggedMessages;
import org.orekit.rugged.linesensor.LineDatation;
import org.orekit.rugged.linesensor.LineSensor;
import org.orekit.rugged.linesensor.LinearLineDatation;
import org.orekit.rugged.linesensor.SensorPixel;
import org.orekit.rugged.los.FixedRotation;
import org.orekit.rugged.los.FixedZHomothety;
import org.orekit.rugged.los.LOSBuilder;
import org.orekit.rugged.los.TimeDependentLOS;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeScale;
import org.orekit.time.TimeScalesFactory;
import org.orekit.utils.AngularDerivativesFilter;
import org.orekit.utils.CartesianDerivativesFilter;
import org.orekit.utils.Constants;
import org.orekit.utils.TimeStampedAngularCoordinates;
import org.orekit.utils.TimeStampedPVCoordinates;


public class RefiningTest {

    /** Pleiades viewing model A */
    PleiadesViewingModel pleiadesViewingModelA;

    /** Pleiades viewing model B */
    PleiadesViewingModel pleiadesViewingModelB;
    
    /** Orbit model corresponding to viewing model A */
    OrbitModel orbitmodelA;

    /** Orbit model corresponding to viewing model B */
    OrbitModel orbitmodelB;

    /** Sensor name A corresponding to viewing model A */
    String sensorNameA;

    /** Sensor name A corresponding to viewing model B */
    String sensorNameB;

    /** RuggedA's instance */
    Rugged ruggedA;

    /** RuggedB's instance */
    Rugged ruggedB;

    /** Inter-measurements (between both viewing models) */
    InterMeasurementGenerator measurements;
    
    /** Mapping from sensor A to sensor B. */
    private SensorToSensorMapping interMapping;


    /** Initialize refining tests
     * @throws RuggedException
     */
    public void InitRefiningTest() throws RuggedException {
        try {
            
            String path = getClass().getClassLoader().getResource("orekit-data").toURI().getPath();
            DataProvidersManager.getInstance().addProvider(new DirectoryCrawler(new File(path)));
            
            // Initialize refining context
            // ---------------------------
            this.sensorNameA = "SensorA";
            final double incidenceAngleA = -5.0;
            final String dateA = "2016-01-01T11:59:50.0";
            this.pleiadesViewingModelA = new PleiadesViewingModel(sensorNameA, incidenceAngleA, dateA);

            this.sensorNameB = "SensorB";
            final double incidenceAngleB = 0.0;
            final String dateB = "2016-01-01T12:02:50.0";
            this.pleiadesViewingModelB = new PleiadesViewingModel(sensorNameB, incidenceAngleB, dateB);

            this.orbitmodelA =  new OrbitModel();
            this.orbitmodelB =  new OrbitModel();

            // Sensors's definition: creation of 2 Pleiades viewing models
            // -----------------------------------------------------------
            
            // 1/- Create First Pleiades Viewing Model A
//            PleiadesViewingModel pleiadesViewingModelA = refining.getPleiadesViewingModelA();
            final AbsoluteDate minDateA =  pleiadesViewingModelA.getMinDate();
            final AbsoluteDate maxDateA =  pleiadesViewingModelA.getMaxDate();
            final AbsoluteDate refDateA = pleiadesViewingModelA.getDatationReference();
            LineSensor lineSensorA =  pleiadesViewingModelA.getLineSensor();
//            System.out.format(Locale.US, "Viewing model A - date min: %s max: %s ref: %s %n", minDateA, maxDateA, refDateA);

            // ----Satellite position, velocity and attitude: create orbit model A
//            OrbitModel orbitmodelA = refining.getOrbitmodelA();
            BodyShape earthA = TestUtils.createEarth();
            NormalizedSphericalHarmonicsProvider gravityFieldA = TestUtils.createGravityField();
            Orbit orbitA  = orbitmodelA.createOrbit(gravityFieldA.getMu(), refDateA);

            // ----If no LOF Transform Attitude Provider is Nadir Pointing Yaw Compensation
            final double [] rollPoly = {0.0,0.0,0.0};
            double[] pitchPoly = {0.025,0.0};
            double[] yawPoly = {0.0,0.0,0.0};
            orbitmodelA.setLOFTransform(rollPoly, pitchPoly, yawPoly, minDateA);

            // ----Satellite attitude
            List<TimeStampedAngularCoordinates> satelliteQListA = orbitmodelA.orbitToQ(orbitA, earthA, minDateA.shiftedBy(-0.0), maxDateA.shiftedBy(+0.0), 0.25);
            int nbQPoints = 2;

            // ----Position and velocities
//            PVCoordinates PVA = orbitA.getPVCoordinates(earthA.getBodyFrame());
            List<TimeStampedPVCoordinates> satellitePVListA = orbitmodelA.orbitToPV(orbitA, earthA, minDateA.shiftedBy(-0.0), maxDateA.shiftedBy(+0.0), 0.25);
            int nbPVPoints = 8;

            // ----Convert frame and coordinates type in one call
//            GeodeticPoint gpA = earthA.transform(PVA.getPosition(), earthA.getBodyFrame(), orbitA.getDate());

//            System.out.format(Locale.US, "(A) Geodetic Point at date %s : φ = %8.10f °, λ = %8.10f %n",
//                              orbitA.getDate().toString(),
//                              FastMath.toDegrees(gpA.getLatitude()),
//                              FastMath.toDegrees(gpA.getLongitude()));

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
//            PleiadesViewingModel pleiadesViewingModelB = refining.getPleiadesViewingModelB();
            final AbsoluteDate minDateB =  pleiadesViewingModelB.getMinDate();
            final AbsoluteDate maxDateB =  pleiadesViewingModelB.getMaxDate();
            final AbsoluteDate refDateB = pleiadesViewingModelB.getDatationReference();
            LineSensor lineSensorB =  pleiadesViewingModelB.getLineSensor();
//            System.out.format(Locale.US, "Viewing model B - date min: %s max: %s ref: %s  %n", minDateB, maxDateB, refDateB);

            // ----Satellite position, velocity and attitude: create orbit model B
            BodyShape earthB = TestUtils.createEarth();
            NormalizedSphericalHarmonicsProvider gravityFieldB = TestUtils.createGravityField();
            Orbit orbitB = orbitmodelB.createOrbit(gravityFieldB.getMu(), refDateB);

            // ----Satellite attitude
            List<TimeStampedAngularCoordinates> satelliteQListB = orbitmodelB.orbitToQ(orbitB, earthB, minDateB.shiftedBy(-0.0), maxDateB.shiftedBy(+0.0), 0.25);

            // ----Position and velocities
//            PVCoordinates PVB = orbitB.getPVCoordinates(earthB.getBodyFrame());
            List<TimeStampedPVCoordinates> satellitePVListB = orbitmodelB.orbitToPV(orbitB, earthB, minDateB.shiftedBy(-0.0), maxDateB.shiftedBy(+0.0), 0.25);
//            GeodeticPoint gpB = earthB.transform(PVB.getPosition(), earthB.getBodyFrame(), orbitB.getDate());

//            System.out.format(Locale.US, "(B) Geodetic Point at date %s : φ = %8.10f °, λ = %8.10f %n",orbitA.getDate().toString(),
//                              FastMath.toDegrees(gpB.getLatitude()),
//                              FastMath.toDegrees(gpB.getLongitude()));
//

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

            // Compute distance between LOS
            // -----------------------------
            double distance = computeDistanceBetweenLOS(lineSensorA, lineSensorB);
            System.out.format("distance %f meters %n",distance);


            // Initialize disruptions:
            // -----------------------

            // Introduce rotations around instrument axes (roll and pitch translations, scale factor)
            System.out.format("\n**** Add disruptions on both acquisitions (A,B): roll and pitch rotations, scale factor **** %n");
            double rollValueA =  FastMath.toRadians(0.004);
            double pitchValueA = FastMath.toRadians(0.0008);
            double rollValueB =  FastMath.toRadians(-0.004);
            double pitchValueB = FastMath.toRadians(0.0008);
            double factorValue = 1.0;

            // Apply disruptions on physical model (acquisition A)
            applyDisruptions(this.ruggedA,this.sensorNameA, rollValueA, pitchValueA, factorValue);
            
            // Apply disruptions on physical model (acquisition B)
            applyDisruptions(this.ruggedB,this.sensorNameB, rollValueB, pitchValueB, factorValue);


            // Generate measurements (observations) from physical model disrupted
            // ------------------------------------------------------------------
            int lineSampling = 1000;
            int pixelSampling = 1000;

            // Noise definition
            // distribution: gaussian(0), vector dimension: 2
            final Noise noise = new Noise(0,2);
            // {pixelA mean, pixelB mean}
            final double[] mean = {5.0,0.0};
            // {pixelB std, pixelB std}
            final double[] standardDeviation = {0.1,0.1};

            noise.setMean(mean);
            noise.setStandardDeviation(standardDeviation);

//            System.out.format("\tSensor A mean: %f std %f %n",mean[0],standardDeviation[0]);
//            System.out.format("\tSensor B mean: %f std %f %n",mean[1],standardDeviation[1]);

            InterMeasurementGenerator measurements = generateNoisyPoints(lineSampling, pixelSampling,
                                                                          ruggedA, sensorNameA,
                                                                          pleiadesViewingModelA.getDimension(),
                                                                          ruggedB, sensorNameB,
                                                                          pleiadesViewingModelB.getDimension(),
                                                                          noise);
            this.interMapping = measurements.getInterMapping();

//            // Compute ground truth residuals
//            // ------------------------------
//            System.out.format("\n**** Ground truth residuals **** %n");
//            computeLiaisonMetrics(this.interMapping, ruggedA, ruggedB);
            

        } catch (OrekitException oe) {
            Assert.fail(oe.getLocalizedMessage());
        } catch (URISyntaxException use) {
            Assert.fail(use.getLocalizedMessage());
        }
    }
    
    
    /** Estimate distance between LOS
     * @param lineSensorA line sensor A
     * @param lineSensorB line sensor B
     * @return GSD
     */
    private double computeDistanceBetweenLOS(final LineSensor lineSensorA, final LineSensor lineSensorB) throws RuggedException {


        // Get number of line of sensors
        int dimensionA = pleiadesViewingModelA.getDimension();
        int dimensionB = pleiadesViewingModelB.getDimension();
        

        Vector3D positionA = lineSensorA.getPosition();
        // This returns a zero vector since we set the relative position of the sensor w.r.T the satellite to 0.

        
        AbsoluteDate lineDateA = lineSensorA.getDate(dimensionA/2);
        Vector3D losA = lineSensorA.getLOS(lineDateA,dimensionA/2);
        GeodeticPoint centerPointA = ruggedA.directLocation(lineDateA, positionA, losA);
        System.out.format(Locale.US, "\ncenter geodetic position A : φ = %8.10f °, λ = %8.10f °, h = %8.3f m%n",
                          FastMath.toDegrees(centerPointA.getLatitude()),
                          FastMath.toDegrees(centerPointA.getLongitude()),centerPointA.getAltitude());


        Vector3D positionB = lineSensorB.getPosition();
        // This returns a zero vector since we set the relative position of the sensor w.r.T the satellite to 0.

        AbsoluteDate lineDateB = lineSensorB.getDate(dimensionB/2);
        Vector3D losB = lineSensorB.getLOS(lineDateB,dimensionB/2);
        GeodeticPoint centerPointB = ruggedB.directLocation(lineDateB, positionB, losB);
        System.out.format(Locale.US, "center geodetic position B : φ = %8.10f °, λ = %8.10f °, h = %8.3f m%n",
                          FastMath.toDegrees(centerPointB.getLatitude()),
                          FastMath.toDegrees(centerPointB.getLongitude()),centerPointB.getAltitude());


        lineDateB = lineSensorB.getDate(0);
        losB = lineSensorB.getLOS(lineDateB,0);
        GeodeticPoint firstPointB = ruggedB.directLocation(lineDateB, positionB, losB);
        System.out.format(Locale.US, "first geodetic position B : φ = %8.10f °, λ = %8.10f °, h = %8.3f m%n",
                          FastMath.toDegrees(firstPointB.getLatitude()),
                          FastMath.toDegrees(firstPointB.getLongitude()),firstPointB.getAltitude());

        lineDateB = lineSensorB.getDate(dimensionB-1);
        losB = lineSensorB.getLOS(lineDateB,dimensionB-1);
        GeodeticPoint lastPointB = ruggedB.directLocation(lineDateB, positionB, losB);
        System.out.format(Locale.US, "last geodetic position  B : φ = %8.10f °, λ = %8.10f °, h = %8.3f m%n",
                          FastMath.toDegrees(lastPointB.getLatitude()),
                          FastMath.toDegrees(lastPointB.getLongitude()),lastPointB.getAltitude());

//        final Vector3D p1 = new Vector3D(centerPointA.getLatitude(), centerPointA.getLongitude()); //
//        final Vector3D p2 = new Vector3D(centerPointB.getLatitude(), centerPointB.getLongitude());
//        double distance =  Constants.WGS84_EARTH_EQUATORIAL_RADIUS * Vector3D.angle(p1, p2);

        return computeDistanceInMeter(centerPointA.getLongitude(), centerPointA.getLatitude(), centerPointB.getLongitude(), centerPointB.getLatitude());
    }
    
    /** Compute a geodetic distance in meters between 2 geodetic points (long1, lat1) and point (long2, lat2).
     * @param long1 Longitude of first geodetic point (rad)
     * @param lat1 Latitude of first geodetic point (rad)
     * @param long2 Longitude of second geodetic point (rad)
     * @param lat2 Latitude of second geodetic point (rad)
     * @return distance in meters
     */
    public static double computeDistanceInMeter(final double long1, final double lat1,
                                                final double long2, final double lat2) {

        // get vectors on unit sphere from angular coordinates
        final Vector3D p1 = new Vector3D(lat1, long1); //
        final Vector3D p2 = new Vector3D(lat2, long2);
        final double distance =  Constants.WGS84_EARTH_EQUATORIAL_RADIUS * Vector3D.angle(p1, p2);
        return distance;
    }

    /**
     * Compute metrics: case of liaison points.
     * @param measMapping Mapping of observations/measurements = the ground truth
     * @param ruggedA Rugged instance corresponding to viewing model A
     * @param ruggedB Rugged instance corresponding to viewing model B
     * @param computeAngular Flag to know if distance is computed in meters (false) or with angular (true)
     * @exception RuggedException if direct location fails
     */
    public RefiningLiaisonMetrics computeLiaisonMetrics(final SensorToSensorMapping measMapping, final Rugged ruggedA, final Rugged ruggedB)
        throws RuggedException {
        
        RefiningLiaisonMetrics liaisonMetrics = new RefiningLiaisonMetrics();
        double resMax = liaisonMetrics.getMaxResidual();
        double resMean = liaisonMetrics.getMeanResidual();
        double losDistanceMax = liaisonMetrics.getLosMaxDistance();
        double losDistanceMean = liaisonMetrics.getLosMeanDistance();
        double earthDistanceMax = liaisonMetrics.getEarthMaxDistance();
        double earthDistanceMean = liaisonMetrics.getEarthMeanDistance();
        

        // Mapping of observations/measurements = the ground truth
        final Set<Map.Entry<SensorPixel, SensorPixel>> measurementsMapping;

        final LineSensor lineSensorA;   // line sensor corresponding to rugged A
        final LineSensor lineSensorB;   // line sensor corresponding to rugged B
        double count = 0;               // counter for computing remaining mean distance
        double losDistanceCount = 0;    // counter for computing LOS distance mean
        double earthDistanceCount = 0;  // counter for computing Earth distance mean
        int i = 0;                      // increment of measurements

        // Initialization
        measurementsMapping = measMapping.getMapping();
        lineSensorA = ruggedA.getLineSensor(measMapping.getSensorNameA());
        lineSensorB = ruggedB.getLineSensor(measMapping.getSensorNameB());
        int nbMeas = measurementsMapping.size(); // number of measurements

        // Browse map of measurements
        for (Iterator<Map.Entry<SensorPixel, SensorPixel>> gtIt = measurementsMapping.iterator();
                        gtIt.hasNext();
                        i++) {

            if (i == measurementsMapping.size()) {
                break;
            }

            final Map.Entry<SensorPixel, SensorPixel> gtMapping = gtIt.next();

            final SensorPixel spA = gtMapping.getKey();
            final SensorPixel spB = gtMapping.getValue();

            final AbsoluteDate dateA = lineSensorA.getDate(spA.getLineNumber());
            final AbsoluteDate dateB = lineSensorB.getDate(spB.getLineNumber());

            final double pixelA = spA.getPixelNumber();
            final double pixelB = spB.getPixelNumber();

            final Vector3D losA = lineSensorA.getLOS(dateA, pixelA);
            final Vector3D losB = lineSensorB.getLOS(dateB, pixelB);

            final GeodeticPoint gpA = ruggedA.directLocation(dateA, lineSensorA.getPosition(), losA);
            final GeodeticPoint gpB = ruggedB.directLocation(dateB, lineSensorB.getPosition(), losB);

            final SpacecraftToObservedBody scToBodyA = ruggedA.getScToBody();

            // Estimated distances (LOS / Earth)
            final double[] distances = ruggedB.distanceBetweenLOS(lineSensorA, dateA, pixelA, scToBodyA, lineSensorB, dateB, pixelB);

            // LOS distance control
            final double estLosDistance =  distances[0]; // LOS distance estimation
            if (estLosDistance > losDistanceMax) {
                losDistanceMax = estLosDistance;
            }
            losDistanceCount += estLosDistance;

            // Earth distance control
            final double estEarthDistance = distances[1]; // Earth distance estimation
            final double measEarthDistance = measMapping.getBodyDistance(i).doubleValue(); // Earth measurement distance
            final double earthDistance =  FastMath.abs(estEarthDistance - measEarthDistance);

            if (earthDistance > earthDistanceMax) {
                earthDistanceMax = earthDistance;

            }
            earthDistanceCount += earthDistance;

            // Compute remaining distance
            double distance = RefiningTest.computeDistanceInMeter(gpB.getLongitude(), gpB.getLatitude(),
                                                                  gpA.getLongitude(), gpA.getLatitude());
            count += distance;
            if (distance > resMax) {
                resMax = distance;
            }
        }

        resMean = count / nbMeas;
        losDistanceMean = losDistanceCount / nbMeas;
        earthDistanceMean = earthDistanceCount / nbMeas;
        
        // Set the results
        liaisonMetrics.setMaxResidual(resMax);
        liaisonMetrics.setMeanResidual(resMean);
        liaisonMetrics.setLosMaxDistance(losDistanceMax);
        liaisonMetrics.setLosMeanDistance(losDistanceMean);
        liaisonMetrics.setEarthMaxDistance(earthDistanceMax);
        liaisonMetrics.setEarthMeanDistance(earthDistanceMean);
        
        return liaisonMetrics;
    }
    
    /** Apply disruptions on acquisition for roll, pitch and scale factor
     * @param rugged Rugged instance
     * @param sensorName line sensor name
     * @param rollValue rotation on roll value
     * @param pitchValue rotation on pitch value
     * @param factorValue scale factor
     * @throws RuggedException
     */
    private void applyDisruptions(final Rugged rugged, final String sensorName,
                                 final double rollValue, final double pitchValue, final double factorValue)
        throws OrekitException, RuggedException {

        final String commonFactorName = "factor";

        rugged.
        getLineSensor(sensorName).
        getParametersDrivers().
        filter(driver -> driver.getName().equals(sensorName+"_roll")).
        findFirst().get().setValue(rollValue);

        rugged.
        getLineSensor(sensorName).
        getParametersDrivers().
        filter(driver -> driver.getName().equals(sensorName+"_pitch")).
        findFirst().get().setValue(pitchValue);

        rugged.
        getLineSensor(sensorName).
        getParametersDrivers().
        filter(driver -> driver.getName().equals(commonFactorName)).
        findFirst().get().setValue(factorValue);
    }
    
    /** Generate noisy measurements (sensor to sensor mapping)
     * @param lineSampling line sampling
     * @param pixelSampling pixel sampling
     * @param ruggedA Rugged instance of acquisition A
     * @param sensorNameA line sensor name A
     * @param dimensionA dimension for acquisition A
     * @param ruggedB Rugged instance of acquisition B
     * @param sensorNameB line sensor name B
     * @param dimensionB dimension for acquisition B
     * @param noise noise structure to generate noisy measurements
     * @return inter-measurements generator (sensor to sensor mapping)
     * @throws RuggedException
     */
    public InterMeasurementGenerator generateNoisyPoints(final int lineSampling, final int pixelSampling,
                                                     final Rugged ruggedA, final String sensorNameA, final int dimensionA,
                                                     final Rugged ruggedB, final String sensorNameB, final int dimensionB,
                                                     final Noise noise) throws RuggedException {

        // Outliers control
        final double outlierValue = 1.e+2;
        
        // Earth constraint weight
        final double earthConstraintWeight = 0.1;

        // Generate measurements with constraints on Earth distance and outliers control
        InterMeasurementGenerator measurements = new InterMeasurementGenerator(ruggedA, sensorNameA, dimensionA,
                                                                   ruggedB, sensorNameB, dimensionB,
                                                                   outlierValue,
                                                                   earthConstraintWeight);
        System.out.format("\n**** Generate noisy measurements (sensor to sensor mapping) **** %n");

        // Generation noisy measurements
        measurements.createNoisyMeasurement(lineSampling, pixelSampling, noise);

        System.out.format("Number of tie points generated: %d %n", measurements.getMeasurementCount());

        return measurements;
    }

    
    public Rugged getRuggedA() {
        return ruggedA;
    }


    public Rugged getRuggedB() {
        return ruggedB;
    }
    
    public SensorToSensorMapping getInterMapping() {
        return interMapping;
    }


    @After
    public void tearDown() {
    }
}

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

    public OrbitModel() {
        userDefinedLOFTransform = false;
    }


    /** TODO GP add comments
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

    /** TODO GP add comments
     */
    public void setLOFTransform(final double[] rollPoly, final double[] pitchPoly,
                                final double[] yawPoly, final AbsoluteDate date) {

        this.userDefinedLOFTransform = true;
        lofTransformRollPoly = rollPoly.clone();
        lofTransformPitchPoly = pitchPoly.clone();
        lofTransformYawPoly = yawPoly.clone();
        this.refDate = date;
    }

    /** TODO GP add comments
     */
    private double getPoly(final double[] poly, final double shift) {
        
        double val = 0.0;
        for (int coef = 0; coef < poly.length; coef++) {
            val = val + poly[coef] * FastMath.pow(shift, coef);
        }
        return val;
    }

    /** TODO GP add comments
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

   /** TODO GP add comments
    */
    public AttitudeProvider createAttitudeProvider(final BodyShape earth, final Orbit orbit)
        throws OrekitException {

        if (userDefinedLOFTransform) {
            final LOFType type = LOFType.VVLH;

            final List<TimeStampedAngularCoordinates> list = new ArrayList<TimeStampedAngularCoordinates>();

            for (double shift = -10.0; shift < +10.0; shift += 1e-2) {
                list.add(new TimeStampedAngularCoordinates(refDate
                    .shiftedBy(shift), getOffset(earth, orbit, shift),
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

    /** TODO GP add comments
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

   /** TODO GP add comments
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

   /** TODO GP add comments
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
}  

/**
 * Inter-measurements generator (sensor to sensor mapping).
 */
class InterMeasurementGenerator {

    /** Mapping from sensor A to sensor B. */
    private SensorToSensorMapping interMappingGenerated;

    /** Observables which contains sensor to sensor mapping.*/
    private Observables observables;

    /** Rugged instance corresponding to the viewing model A */
    private Rugged ruggedA;

    /** Rugged instance corresponding to the viewing model B */
    private Rugged ruggedB;

    /** Sensor A */
    private LineSensor sensorA;

    /** Sensor B */
    private LineSensor sensorB;

    /** Number of measurements */
    private int measurementCount;


    /** Sensor name B */
    private String sensorNameB;

    /** Number of line for acquisition A */
    private int dimensionA;

    /** Number of line for acquisition B */
    private int dimensionB;

    /** Limit value for outlier points */
    private double outlier;


    /** Default constructor: measurements generation without outlier points control
     * and without Earth distance constraint.
     */
    public InterMeasurementGenerator(final Rugged ruggedA, final String sensorNameA, final int dimensionA,
                                 final Rugged ruggedB, final String sensorNameB, final int dimensionB)
        throws RuggedException {

        // Initialize parameters
        initParams(ruggedA, sensorNameA, dimensionA, ruggedB, sensorNameB, dimensionB);

        // Generate reference mapping, without Earth distance constraint
        interMappingGenerated = new SensorToSensorMapping(sensorNameA, sensorNameB);

        // Create observables for two models
        observables = new Observables(2);
    }


    /** Constructor for measurements generation taking into account outlier points control,
     * without Earth distance constraint.
     */
    public InterMeasurementGenerator(final Rugged ruggedA, final String sensorNameA, final int dimensionA,
                                 final Rugged ruggedB, final String sensorNameB, final int dimensionB,
                                 final double outlier)
        throws RuggedException {

        this(ruggedA, sensorNameA, dimensionA, ruggedB, sensorNameB, dimensionB);
        this.outlier = outlier;
    }

    /** Constructor for measurements generation taking into account outlier points control,
     * and Earth distance constraint.
     */
    public InterMeasurementGenerator(final Rugged ruggedA, final String sensorNameA, final int dimensionA,
                                 final Rugged ruggedB, final String sensorNameB, final int dimensionB,
                                 final double outlier, final double earthConstraintWeight)
        throws RuggedException {

        // Initialize parameters
        initParams(ruggedA, sensorNameA, dimensionA, ruggedB, sensorNameB, dimensionB);

        // Generate reference mapping, with Earth distance constraints.
        // Weighting will be applied as follow :
        //     (1-earthConstraintWeight) for losDistance weighting
        //     earthConstraintWeight for earthDistance weighting
        interMappingGenerated = new SensorToSensorMapping(sensorNameA, ruggedA.getName(), sensorNameB, ruggedB.getName(), earthConstraintWeight);

        // Outlier points control
        this.outlier = outlier;

        // Observables which contains sensor to sensor mapping
        this.observables = new Observables(2);
    }

    /** Get the mapping from sensor A to sensor B
     * @return the mapping from sensor A to sensor B
     */
    public SensorToSensorMapping getInterMapping() {
        return interMappingGenerated;
    }

    /**  Get the observables which contains sensor to sensor mapping
     * @return the observables which contains sensor to sensor mapping
     */
    public Observables getObservables() {
        return observables;
    }

    public int getMeasurementCount() {
        return measurementCount;
    }

    public void createMeasurement(final int lineSampling, final int pixelSampling) throws RuggedException {

        // Search the sensor pixel seeing point
        final int minLine = 0;
        final int maxLine = dimensionB - 1;

        for (double line = 0; line < dimensionA; line += lineSampling) {

            final AbsoluteDate dateA = sensorA.getDate(line);

            for (double pixelA = 0; pixelA < sensorA.getNbPixels(); pixelA += pixelSampling) {

                final GeodeticPoint gpA = ruggedA.directLocation(dateA, sensorA.getPosition(),
                                                                 sensorA.getLOS(dateA, pixelA));

                final SensorPixel sensorPixelB = ruggedB.inverseLocation(sensorNameB, gpA, minLine, maxLine);

                // We need to test if the sensor pixel is found in the prescribed lines
                // otherwise the sensor pixel is null
                if (sensorPixelB != null) {
                    
                    final AbsoluteDate dateB = sensorB.getDate(sensorPixelB.getLineNumber());
                    final double pixelB = sensorPixelB.getPixelNumber();
                    final SpacecraftToObservedBody scToBodyA = ruggedA.getScToBody();

                    final GeodeticPoint gpB = ruggedB.directLocation(dateB, sensorB.getPosition(),
                                                                     sensorB.getLOS(dateB, pixelB));

                    final double GEOdistance = RefiningTest.computeDistanceInMeter(gpA.getLongitude(), gpA.getLatitude(),
                                                                      gpB.getLongitude(), gpB.getLatitude());

                    if (GEOdistance < this.outlier) {
                        
                        final SensorPixel RealPixelA = new SensorPixel(line, pixelA);
                        final SensorPixel RealPixelB = new SensorPixel(sensorPixelB.getLineNumber(), sensorPixelB.getPixelNumber());

                        final AbsoluteDate RealDateA = sensorA.getDate(RealPixelA.getLineNumber());
                        final AbsoluteDate RealDateB = sensorB.getDate(RealPixelB.getLineNumber());
                        final double[] distanceLOSB = ruggedB.distanceBetweenLOS(sensorA, RealDateA, RealPixelA.getPixelNumber(), scToBodyA,
                                                                                 sensorB, RealDateB, RealPixelB.getPixelNumber());

                        final double losDistance = 0.0;
                        final double earthDistance = distanceLOSB[1];

                        interMappingGenerated.addMapping(RealPixelA, RealPixelB, losDistance, earthDistance);

                        // increment the number of measurements
                        this.measurementCount++;
                    }
                }
            }
        }

        this.observables.addInterMapping(interMappingGenerated);
    }


    /** Tie point creation from direct 1ocation with Rugged A and inverse location with Rugged B
     * @param lineSampling sampling along lines
     * @param pixelSampling sampling along columns
     * @param noise errors to apply to measure for pixel A and pixel B
     * @throws RuggedException
     */
    public void createNoisyMeasurement(final int lineSampling, final int pixelSampling, final Noise noise)
        throws RuggedException {

        // Get noise features (errors)
        // [pixelA, pixelB] mean
        final double[] mean = noise.getMean();
        // [pixelA, pixelB] standard deviation
        final double[] std = noise.getStandardDeviation();

        // Search the sensor pixel seeing point
        final int minLine = 0;
        final int maxLine = dimensionB - 1;

        final double meanA[] = { mean[0], mean[0] };
        final double stdA[]  = { std[0], std[0] };
        final double meanB[] = { mean[1], mean[1] };
        final double stdB[]  = { std[1], std[1] };

        // TODO GP explanation about seed ???
        final GaussianRandomGenerator rngA = new GaussianRandomGenerator(new Well19937a(0xefac03d9be4d24b9l));
        final UncorrelatedRandomVectorGenerator rvgA = new UncorrelatedRandomVectorGenerator(meanA, stdA, rngA);

        // TODO GP explanation about seed ???
        final GaussianRandomGenerator rngB = new GaussianRandomGenerator(new Well19937a(0xdf1c03d9be0b34b9l));
        final UncorrelatedRandomVectorGenerator rvgB = new UncorrelatedRandomVectorGenerator(meanB, stdB, rngB);

        for (double line = 0; line < dimensionA; line += lineSampling) {

            final AbsoluteDate dateA = sensorA.getDate(line);
            for (double pixelA = 0; pixelA < sensorA.getNbPixels(); pixelA += pixelSampling) {

                final GeodeticPoint gpA = ruggedA.directLocation(dateA, sensorA.getPosition(),
                                                                 sensorA.getLOS(dateA, pixelA));
                final SensorPixel sensorPixelB = ruggedB.inverseLocation(sensorNameB, gpA, minLine, maxLine);

                // We need to test if the sensor pixel is found in the prescribed lines
                // otherwise the sensor pixel is null
                if (sensorPixelB != null) {
                    
                    final AbsoluteDate dateB = sensorB.getDate(sensorPixelB.getLineNumber());
                    final double pixelB = sensorPixelB.getPixelNumber();

                    // Get spacecraft to body transform of Rugged instance A
                    final SpacecraftToObservedBody scToBodyA = ruggedA.getScToBody();

                    final GeodeticPoint gpB = ruggedB.directLocation(dateB, sensorB.getPosition(),
                                                                     sensorB.getLOS(dateB, pixelB));
                    final double GEOdistance = RefiningTest.computeDistanceInMeter(gpA.getLongitude(), gpA.getLatitude(),
                                                                                   gpB.getLongitude(), gpB.getLatitude());
                    // TODO GP explanation about computation here
                    if (GEOdistance < outlier) {

                        final double[] vecRandomA = rvgA.nextVector();
                        final double[] vecRandomB = rvgB.nextVector();

                        final SensorPixel RealPixelA = new SensorPixel(line + vecRandomA[0], pixelA + vecRandomA[1]);
                        final SensorPixel RealPixelB = new SensorPixel(sensorPixelB.getLineNumber() + vecRandomB[0],
                                                                       sensorPixelB.getPixelNumber() + vecRandomB[1]);
                        final AbsoluteDate RealDateA = sensorA.getDate(RealPixelA.getLineNumber());
                        final AbsoluteDate RealDateB = sensorB.getDate(RealPixelB.getLineNumber());
                        final double[] distanceLOSB = ruggedB.distanceBetweenLOS(sensorA, RealDateA, RealPixelA.getPixelNumber(), scToBodyA,
                                                                                 sensorB, RealDateB, RealPixelB.getPixelNumber());
                        final Double losDistance = 0.0;
                        final Double earthDistance = distanceLOSB[1];

                        interMappingGenerated.addMapping(RealPixelA, RealPixelB, losDistance, earthDistance);

                        // increment the number of measurements
                        this.measurementCount++;
                    }
                }
            }
        }

        this.observables.addInterMapping(interMappingGenerated);
    }

    /** Default constructor: measurements generation without outlier points control
     * and Earth distances constraint.
     * @param rA Rugged instance A
     * @param sNameA sensor name A
     * @param dimA dimension for acquisition A
     * @param rB Rugged instance B
     * @param sNameB sensor name B
     * @param dimB dimension for acquisition B
     * @throws RuggedException
     */
    private void initParams(final Rugged rA, final String sNameA, final int dimA,
                            final Rugged rB, final String sNameB, final int dimB)
        throws RuggedException {

        this.sensorNameB = sNameB;
        // Check that sensors's name is different
        if (sNameA.contains(sNameB)) {
           throw new RuggedExceptionWrapper(new RuggedException(RuggedMessages.DUPLICATED_PARAMETER_NAME, sNameA));
        }

        this.ruggedA = rA;
        this.ruggedB = rB;

        this.sensorA = rA.getLineSensor(sNameA);
        this.sensorB = rB.getLineSensor(sNameB);

        this.dimensionA = dimA;
        this.dimensionB = dimB;

        this.measurementCount = 0;
    }
}


/**
 * Pleiades viewing model class definition.
 */
class PleiadesViewingModel {

    /** intrinsic Pleiades parameters. */
    private double fov = 1.65; // 20km - alt 694km

    // Number of line of the sensor
    private int dimension = 40000;

    private double angle;
    private LineSensor lineSensor;
    private String date;

    private String sensorName;

    private final double linePeriod =  1e-4;


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
     * @throws RuggedException
     * @throws OrekitException
     */
    public PleiadesViewingModel(final String sensorName, final double incidenceAngle, final String referenceDate)
        throws RuggedException, OrekitException {
        
        this.sensorName = sensorName;
        this.date = referenceDate;
        this.angle = incidenceAngle;
        this.createLineSensor();
    }

    /** TODO GP add comments
     */
    public LOSBuilder rawLOS(final Vector3D center, final Vector3D normal, final double halfAperture, final int n) {

        final List<Vector3D> list = new ArrayList<Vector3D>(n);
        for (int i = 0; i < n; ++i) {
            final double alpha = (halfAperture * (2 * i + 1 - n)) / (n - 1);
            list.add(new Rotation(normal, alpha, RotationConvention.VECTOR_OPERATOR).applyTo(center));
        }

        return new LOSBuilder(list);
    }

    /** TODO GP add comments
     */
    public TimeDependentLOS buildLOS() {
        
        final LOSBuilder losBuilder = rawLOS(new Rotation(Vector3D.PLUS_I,
                                                          FastMath.toRadians(this.angle),
                                                          RotationConvention.VECTOR_OPERATOR).applyTo(Vector3D.PLUS_K),
                                             Vector3D.PLUS_I, FastMath.toRadians(fov / 2), dimension);

        losBuilder.addTransform(new FixedRotation(sensorName + "_roll",  Vector3D.MINUS_I, 0.00));
        losBuilder.addTransform(new FixedRotation(sensorName + "_pitch", Vector3D.MINUS_J, 0.00));

        // factor is a common parameters shared between all Pleiades models
        losBuilder.addTransform(new FixedZHomothety("factor", 1.0));

        return losBuilder.build();
    }


    /** TODO GP add comments
     */
    public AbsoluteDate getDatationReference() throws OrekitException {

        // We use Orekit for handling time and dates, and Rugged for defining the datation model:
        final TimeScale utc = TimeScalesFactory.getUTC();

        return new AbsoluteDate(date, utc);
    }

    /** TODO GP add comments
     */
   public  AbsoluteDate getMinDate() throws RuggedException {
        return lineSensor.getDate(0);
    }

   /** TODO GP add comments
    */
   public  AbsoluteDate  getMaxDate() throws RuggedException {
        return lineSensor.getDate(dimension);
    }

   /** TODO GP add comments
    */
   public  LineSensor  getLineSensor() {
        return lineSensor;
    }

   /** TODO GP add comments
    */
   public  String getSensorName() {
        return sensorName;
    }

   /** Get the number of lines of the sensor
    * @return the number of lines of the sensor
    */
public int getDimension() {
        return dimension;
    }

   /** TODO GP add comments
    */
   private void createLineSensor() throws RuggedException, OrekitException {

        // Offset of the MSI from center of mass of satellite
        // one line sensor
        // los: swath in the (YZ) plane, looking at 50° roll, 2.6" per pixel
        final Vector3D msiOffset = new Vector3D(0, 0, 0);

        // TODO build complex los
        final TimeDependentLOS lineOfSight = buildLOS();

        final double rate =  1 / linePeriod;
        // linear datation model: at reference time we get the middle line, and the rate is one line every 1.5ms

        final LineDatation lineDatation = new LinearLineDatation(getDatationReference(), dimension / 2, rate);

        lineSensor = new LineSensor(sensorName, lineDatation, msiOffset, lineOfSight);
    }
}

/** Class for adding a noise to measurements.
 */
class Noise {

    /** Type of distribution. */
    private static final int GAUSSIAN = 0;

    /** Dimension. */
    private int dimension;

    /** Mean. */
    private double[] mean;

    /** Standard deviation. */
    private double[] standardDeviation;

    /** Distribution. */
    private int distribution = GAUSSIAN;

    /** Build a new instance.
     * @param distribution noise type
     * @param dimension noise dimension
     */
    public Noise(final int distribution, final int dimension) {

        this.mean = new double[dimension];
        this.standardDeviation = new double[dimension];
        this.dimension = dimension;
        this.distribution = distribution;
    }

    /** Get the mean.
     * @return the mean
     */
    public double[] getMean() {
        return mean.clone();
    }

    /** Set the mean.
     * @param meanValue the mean to set
     */
    public void setMean(final double[] meanValue) {
        this.mean = meanValue.clone();
    }

    /** Get the standard deviation.
     * @return the standard deviation
     */
    public double[] getStandardDeviation() {
        return standardDeviation.clone();
    }

    /** Set the standard deviation.
     * @param standardDeviationValue the standard deviation to set
     */
    public void setStandardDeviation(final double[] standardDeviationValue) {
        this.standardDeviation = standardDeviationValue.clone();
    }

    /** Get the distribution.
     * @return the distribution
     */
    public int getDistribution() {
        return distribution;
    }

    /** Get the dimension.
     * @return the dimension
     */
    public int getDimension() {
        return dimension;
    }
}
