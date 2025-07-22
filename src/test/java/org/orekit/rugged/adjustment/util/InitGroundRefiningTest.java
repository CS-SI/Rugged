package org.orekit.rugged.adjustment.util;

import java.io.File;
import java.net.URISyntaxException;
import java.util.List;

import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.random.GaussianRandomGenerator;
import org.hipparchus.random.UncorrelatedRandomVectorGenerator;
import org.hipparchus.random.Well19937a;
import org.hipparchus.util.FastMath;
import org.junit.Assert;
import org.orekit.bodies.BodyShape;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.data.DataContext;
import org.orekit.data.DirectoryCrawler;
import org.orekit.errors.OrekitException;
import org.orekit.orbits.Orbit;
import org.orekit.rugged.TestUtils;
import org.orekit.rugged.adjustment.measurements.Observables;
import org.orekit.rugged.adjustment.measurements.SensorToGroundMapping;
import org.orekit.rugged.api.AlgorithmId;
import org.orekit.rugged.api.BodyRotatingFrameId;
import org.orekit.rugged.api.EllipsoidId;
import org.orekit.rugged.api.InertialFrameId;
import org.orekit.rugged.api.Rugged;
import org.orekit.rugged.api.RuggedBuilder;
import org.orekit.rugged.linesensor.LineSensor;
import org.orekit.rugged.linesensor.SensorPixel;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.AngularDerivativesFilter;
import org.orekit.utils.CartesianDerivativesFilter;
import org.orekit.utils.Constants;
import org.orekit.utils.TimeStampedAngularCoordinates;
import org.orekit.utils.TimeStampedPVCoordinates;


/** Initialization for ground refining (Ground Control Points GCP study) Junit tests.
 * @author Guylaine Prat
 */
public class InitGroundRefiningTest {

    /** Pleiades viewing model */
    private PleiadesViewingModel pleiadesViewingModel;

    /** Line sensor */
    private LineSensor lineSensor;

    /** Rugged's instance */
    private Rugged rugged;

    /** Number of parameters to adjust */
    private int parameterToAdjust;
    
    // Part of the name of parameter drivers
    static final String rollSuffix = "_roll";
    static final String pitchSuffix = "_pitch";
    static final String factorName = "factor";

    // Default values for disruption to apply to roll (deg), pitch (deg) and factor 
    static final double defaultRollDisruption =  0.004;
    static final double defaultPitchDisruption = 0.0008;
    static final double defaultFactorDisruption = 1.000000001;
    
    /**
     * Initialize ground refining tests with default values for disruptions on sensors characteristics
     */
    public void initGroundRefiningTest() {
        
        initGroundRefiningTest(defaultRollDisruption, defaultPitchDisruption, defaultFactorDisruption);
    }

    /** Initialize ground refining tests with disruption on sensors characteristics
     * @param rollDisruption disruption to apply to roll angle for sensor (deg)
     * @param pitchDisruption disruption to apply to pitch angle for sensor (deg)
     * @param factorDisruption disruption to apply to homothety factor for sensor
     */
    public void initGroundRefiningTest(double rollDisruption, double pitchDisruption, double factorDisruption) {
        try {
            
            String path = getClass().getClassLoader().getResource("orekit-data").toURI().getPath();
            DataContext.getDefault().getDataProvidersManager().addProvider(new DirectoryCrawler(new File(path)));
            
            // Initialize refining context
            // ---------------------------
            final String sensorName = "line";
            final double rollAngle = -5.0;
            final String date = "2016-01-01T11:59:50.0";
            this.pleiadesViewingModel = new PleiadesViewingModel(sensorName, rollAngle, date);


            PleiadesOrbitModel orbitmodel =  new PleiadesOrbitModel();

            // Sensor's definition: creation of Pleiades viewing model
            // --------------------------------------------------------
            
            // Create Pleiades Viewing Model
            final AbsoluteDate minDate =  pleiadesViewingModel.getMinDate();
            final AbsoluteDate maxDate =  pleiadesViewingModel.getMaxDate();
            final AbsoluteDate refDate = pleiadesViewingModel.getDatationReference();
            this.lineSensor =  pleiadesViewingModel.getLineSensor();

            // ----Satellite position, velocity and attitude: create orbit model
            BodyShape earth = TestUtils.createEarth();
            Orbit orbit  = orbitmodel.createOrbit(Constants.EIGEN5C_EARTH_MU, refDate);

            // ----If no LOF Transform Attitude Provider is Nadir Pointing Yaw Compensation
            final double [] rollPoly = {0.0,0.0,0.0};
            final double[] pitchPoly = {0.025,0.0};
            final double[] yawPoly = {0.0,0.0,0.0};
            orbitmodel.setLOFTransform(rollPoly, pitchPoly, yawPoly, minDate);

            // ----Satellite attitude
            List<TimeStampedAngularCoordinates> satelliteQList = orbitmodel.orbitToQ(orbit, earth, minDate.shiftedBy(-0.0), maxDate.shiftedBy(+0.0), 0.25);
            final int nbQPoints = 2;

            // ----Position and velocities
            List<TimeStampedPVCoordinates> satellitePVListA = orbitmodel.orbitToPV(orbit, earth, minDate.shiftedBy(-0.0), maxDate.shiftedBy(+0.0), 0.25);
            final int nbPVPoints = 8;

            // Rugged initialization
            // ---------------------
            RuggedBuilder ruggedBuilder = new RuggedBuilder();

            ruggedBuilder.addLineSensor(lineSensor);
            ruggedBuilder.setAlgorithm(AlgorithmId.IGNORE_DEM_USE_ELLIPSOID);
            ruggedBuilder.setEllipsoid(EllipsoidId.WGS84, BodyRotatingFrameId.ITRF);
            ruggedBuilder.setTimeSpan(minDate,maxDate, 0.001, 5.0);
            ruggedBuilder.setTrajectory(InertialFrameId.EME2000, satellitePVListA, nbPVPoints,
                                         CartesianDerivativesFilter.USE_PV, satelliteQList,
                                         nbQPoints, AngularDerivativesFilter.USE_R);
            ruggedBuilder.setLightTimeCorrection(false);
            ruggedBuilder.setAberrationOfLightCorrection(false);

            ruggedBuilder.setName("Rugged");

            this.rugged = ruggedBuilder.build();

            
            // Select parameters to adjust
            // ---------------------------
            RefiningParametersDriver.setSelectedRoll(rugged, sensorName);
            RefiningParametersDriver.setSelectedPitch(rugged, sensorName);

            this.parameterToAdjust = 2;

            // Initialize disruptions:
            // -----------------------
            // introduce rotations around instrument axes (roll and pitch angles, scale factor)

            // apply disruptions on physical model
            RefiningParametersDriver.applyDisruptionsRoll(rugged, sensorName, FastMath.toRadians(rollDisruption));
            RefiningParametersDriver.applyDisruptionsPitch(rugged, sensorName, FastMath.toRadians(pitchDisruption));
            RefiningParametersDriver.applyDisruptionsFactor(rugged, sensorName, factorDisruption);
            
            
        } catch (OrekitException oe) {
            Assert.fail(oe.getLocalizedMessage());
        } catch (URISyntaxException use) {
            Assert.fail(use.getLocalizedMessage());
        }
    }
    
    /**
     * Get the Rugged instance
     * @return rugged instance
     */
    public Rugged getRugged(){
        
        return this.rugged;
    }
       
    /** Generate noisy measurements (sensor to ground mapping)
     * @param lineSampling line sampling
     * @param pixelSampling pixel sampling
     * @param flag to tell if the Rugged name used is the default one (true) or not (false)
     */
    public Observables generateNoisyPoints(final int lineSampling, final int pixelSampling, boolean defaultRuggedName) {

        // Generate reference mapping
        SensorToGroundMapping groundMapping;
        if (!defaultRuggedName) {
            groundMapping = new SensorToGroundMapping(rugged.getName(), lineSensor.getName());
        } else {
            // The rugged name used in this test is the same as the dafult one in SensorToGroundMapping
            groundMapping = new SensorToGroundMapping(lineSensor.getName());
        }
        
        // Observable which contains sensor to ground mapping (one model)
        Observables observable = new Observables(1);
        
        // Estimate latitude and longitude errors (rad)
        final int pixelMiddle= lineSensor.getNbPixels() / 2;
        final int lineMiddle = (int) FastMath.floor(pixelMiddle); 
        
        final AbsoluteDate dateMiddle = lineSensor.getDate(lineMiddle);
        final GeodeticPoint gp_pix0 = rugged.directLocation(dateMiddle, lineSensor.getPosition(), lineSensor.getLOS(dateMiddle, pixelMiddle));

        final AbsoluteDate date1 = lineSensor.getDate(lineMiddle + 1);
        final GeodeticPoint gp_pix1 = rugged.directLocation(date1, lineSensor.getPosition(), lineSensor.getLOS(date1, pixelMiddle + 1));

        final double latitudeErr = FastMath.abs(gp_pix0.getLatitude() - gp_pix1.getLatitude());
        final double longitudeErr = FastMath.abs(gp_pix0.getLongitude() - gp_pix1.getLongitude());

        // Generation noisy measurements
        // distribution: gaussian(0), vector dimension: 3 (for latitude, longitude and altitude)
        // Mean latitude, longitude and altitude
        final double[] mean = {5.0,5.0,5.0};
        // Standard deviation latitude, longitude and altitude
        final double[] std = {0.1,0.1,0.1};

        final double latErrorMean = mean[0] * latitudeErr;
        final double lonErrorMean = mean[1] * longitudeErr;
        final double latErrorStd = std[0] * latitudeErr;
        final double lonErrorStd = std[1] * longitudeErr;

        // Gaussian random generator
        // Build a null mean random uncorrelated vector generator with standard deviation corresponding to the estimated error on ground
        final double meanGenerator[] =  {latErrorMean, lonErrorMean, mean[2]};
        final double stdGenerator[] = {latErrorStd, lonErrorStd, std[2]};

        // seed has been fixed for tests purpose
        final GaussianRandomGenerator rng = new GaussianRandomGenerator(new Well19937a(0xefac03d9be4d24b9l));
        final UncorrelatedRandomVectorGenerator rvg = new UncorrelatedRandomVectorGenerator(meanGenerator, stdGenerator, rng);

        for (double line = 0; line < pleiadesViewingModel.getDimension(); line += lineSampling) {
            
            final AbsoluteDate date = lineSensor.getDate(line);
            for (int pixel = 0; pixel < lineSensor.getNbPixels(); pixel += pixelSampling) {

                // Components of generated vector follow (independent) Gaussian distribution
                final Vector3D vecRandom = new Vector3D(rvg.nextVector());

                final GeodeticPoint gp2 = rugged.directLocation(date, lineSensor.getPosition(),
                                                                lineSensor.getLOS(date, pixel));

                final GeodeticPoint gpNoisy = new GeodeticPoint(gp2.getLatitude() + vecRandom.getX(),
                                                                gp2.getLongitude() + vecRandom.getY(),
                                                                gp2.getAltitude() + vecRandom.getZ());

                groundMapping.addMapping(new SensorPixel(line, pixel), gpNoisy);
            }
        }

        observable.addGroundMapping(groundMapping);
        return observable;
    }
    
    /** Get the number of parameters to adjust
     * @return number of parameters to adjust
     */
    public int getParameterToAdjust() {
        return parameterToAdjust;
    }

}
