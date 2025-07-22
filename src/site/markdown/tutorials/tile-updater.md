<!--- Copyright 2013-2025 CS GROUP
  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at
  
    http://www.apache.org/licenses/LICENSE-2.0
  
  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
-->

# Example with the real DEM SRTM

This tutorial shows how to use the tile updater with a real DEM (for instance SRTM here).

Same case as the tutorial [Direct location with a DEM](./direct-location-with-DEM.html).

**WARNING**

This tutorial will not be able to compile under [Rugged gitlab CI](https:gitlab.orekit.org/orekit/rugged/-/pipelines) as GDAL is not authorized in Rugged official releases.
Don't commit under official branches (master, develop, release, ...) of Rugged, a pom.xml with the dependency to GDAL.

## Needs

### Install GDAL

Search the web for how to install GDAL according to your OS.

For instance for Linux/Ubuntu:

The first time you want to install GDAL, you need to add a specific repository:
   
For GDAL 3.x, use the following repository:

    sudo add-apt-repository ppa:ubuntugis/ubuntugis-unstable
                     
For GDAL 2.x, use the following repository:

    sudo add-apt-repository ppa:ubuntugis/ppa

Then

    sudo apt update
    sudo apt install gdal-bin gdal-data libgdal-java

### Add GDAL in pom.xml file

In `<properties>` part:

    <!-- GDAL version -->
    <rugged.gdal.version>3.0.0</rugged.gdal.version>
    <!-- GDAL native library path -->
    <rugged.gdal.native.library.path>${env.GDAL_PATH}</rugged.gdal.native.library.path>
   
and in `<dependencies>` part:

    <dependency>
      <groupId>org.gdal</groupId>
      <artifactId>gdal</artifactId>
      <version>${rugged.gdal.version}</version>
      <type>jar</type>
      <optional>false</optional>
    </dependency>

### Configure the environment variable GDAL_PATH

To access GDAL libraries (.so files), one must configure the environment variable GDAL_PATH as defined in pom.xml (see above).

For instance (for Linux) according to the GDAL version and the linux distribution:

       export GDAL_PATH=/COTS/gdal-3.0.0/swig/java/
   or
   
       export GDAL_PATH=/usr/lib/jni

### Get the following SRTM tile

This example needs one tile under a specific directory: `17/05/srtm_17_05.tif`

For instance from : [SRTM Tile Grabber](http://dwtkns.com/srtm/)

    srtm_17_05.zip

and unzip it under a directory like : `mypath/dem-data/SRTM/17/05/`
or changed the method: `getRasterFilePath(latitude, longitude)`

### Get the GEOID data: egm96_15.gtx

This example needs the geoid file: `egm96_15.gtx`

For instance from the [GeographicLib](https:geographiclib.sourceforge.io/C++/doc/geoid.html)

### Update the path in variables demRootDir and geoidFilePath

Into the following code ...

## Code example

This is a whole example using the SRTM tiles.

    public class DirectLocationWithSRTMdem {
    
    // Root dir for the SRTM tiles
    final static String demRootDir = "/mypath/dem-data/SRTM";
    
    // File path for the GEOID
    final static String geoidFilePath = "/mypath/dem-data/GEOID/egm96_15.gtx";
    
    // Geoid elevations 
    static GEOIDelevationReader geoidElevationReader;
    static GdalTransformTools geoidGTTools;
    static double[][] geoidElevations;

    
    public static void main(String[] args) {

        try {

            // Initialize Orekit, assuming an orekit-data folder is in user home directory
            File home       = new File(System.getProperty("user.home"));
            File orekitData = new File(home, "orekit-data");
            DataContext.getDefault().getDataProvidersManager().addProvider(new DirectoryCrawler(orekitData));


            // Initialize the DEM datas
            // ========================
            // Initialize GDAL
            org.gdal.gdal.gdal.AllRegister();

            // Get the GEOID data once for all
            geoidElevationReader = new GEOIDelevationReader(geoidFilePath);
            geoidGTTools = new GdalTransformTools(geoidElevationReader.getGeoidGeoTransformInfo(), 
                                                  geoidElevationReader.getGeoidLonSize(), geoidElevationReader.getGeoidLatSize());
            geoidElevations = geoidElevationReader.getElevationsWholeEarth();

            // Initialize the SRTM tile updater
            
            // SRTM elevations are given over the GEOID.
            // Rugged needs elevations over the ellipsoid so the Geoid must be added to raw SRTM elevations.
            SRTMelevationUpdater srtmUpdater = new SRTMelevationUpdater(demRootDir);


            // Sensor's definition
            // ===================
            // Line of sight
            // -------------
            // The raw viewing direction of pixel i with respect to the instrument is defined by the vector:
            List<Vector3D> rawDirs = new ArrayList<Vector3D>();
            for (int i = 0; i < 2000; i++) {
                // 20° field of view, 2000 pixels
                rawDirs.add(new Vector3D(0d, i*FastMath.toRadians(20)/2000d, 1d));
            }

            // The instrument is oriented 10° off nadir around the X-axis, we need to rotate the viewing
            // direction to obtain the line of sight in the satellite frame
            LOSBuilder losBuilder = new LOSBuilder(rawDirs);
            losBuilder.addTransform(new FixedRotation("10-degrees-rotation", Vector3D.PLUS_I, FastMath.toRadians(10)));

            TimeDependentLOS lineOfSight = losBuilder.build();

            // Datation model
            // --------------
            // We use Orekit for handling time and dates, and Rugged for defining the datation model:
            TimeScale gps = TimeScalesFactory.getGPS();
            AbsoluteDate absDate = new AbsoluteDate("2009-12-11T16:59:30.0", gps);
            LinearLineDatation lineDatation = new LinearLineDatation(absDate, 1d, 20);

            // Line sensor
            // -----------
            // With the LOS and the datation now defined, we can initialize a line sensor object in Rugged:
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

            addSatelliteQ(gps, satelliteQList, "2009-12-11T16:58:42.592937", -0.340236d, 0.333952d, -0.844012d, -0.245684d);
            addSatelliteQ(gps, satelliteQList, "2009-12-11T16:59:06.592937", -0.354773d, 0.329336d, -0.837871d, -0.252281d);
            addSatelliteQ(gps, satelliteQList, "2009-12-11T16:59:30.592937", -0.369237d, 0.324612d, -0.831445d, -0.258824d);
            addSatelliteQ(gps, satelliteQList, "2009-12-11T16:59:54.592937", -0.3836d, 0.319792d, -0.824743d, -0.265299d);
            addSatelliteQ(gps, satelliteQList, "2009-12-11T17:00:18.592937", -0.397834d, 0.314883d, -0.817777d, -0.271695d);
            addSatelliteQ(gps, satelliteQList, "2009-12-11T17:00:42.592937", -0.411912d, 0.309895d, -0.810561d, -0.278001d);
            addSatelliteQ(gps, satelliteQList, "2009-12-11T17:01:06.592937", -0.42581d, 0.304838d, -0.803111d, -0.284206d);
            addSatelliteQ(gps, satelliteQList, "2009-12-11T17:01:30.592937", -0.439505d, 0.299722d, -0.795442d, -0.290301d);
            addSatelliteQ(gps, satelliteQList, "2009-12-11T17:01:54.592937", -0.452976d, 0.294556d, -0.787571d, -0.296279d);
            addSatelliteQ(gps, satelliteQList, "2009-12-11T17:02:18.592937", -0.466207d, 0.28935d, -0.779516d, -0.302131d);

            // Positions and velocities
            // ------------------------
            ArrayList<TimeStampedPVCoordinates> satellitePVList = new ArrayList<TimeStampedPVCoordinates>();
            addSatellitePV(gps, eme2000, itrf, satellitePVList, "2009-12-11T16:58:42.592937", -726361.466d, -5411878.485d, 4637549.599d, -2463.635d, -4447.634d, -5576.736d);
            addSatellitePV(gps, eme2000, itrf, satellitePVList, "2009-12-11T16:59:04.192937", -779538.267d, -5506500.533d, 4515934.894d, -2459.848d, -4312.676d, -5683.906d);
            addSatellitePV(gps, eme2000, itrf, satellitePVList, "2009-12-11T16:59:25.792937", -832615.368d, -5598184.195d, 4392036.13d, -2454.395d, -4175.564d, -5788.201d);
            addSatellitePV(gps, eme2000, itrf, satellitePVList, "2009-12-11T16:59:47.392937", -885556.748d, -5686883.696d, 4265915.971d, -2447.273d, -4036.368d, -5889.568d);
            addSatellitePV(gps, eme2000, itrf, satellitePVList, "2009-12-11T17:00:08.992937", -938326.32d, -5772554.875d, 4137638.207d, -2438.478d, -3895.166d, -5987.957d);
            addSatellitePV(gps, eme2000, itrf, satellitePVList, "2009-12-11T17:00:30.592937", -990887.942d, -5855155.21d, 4007267.717d, -2428.011d, -3752.034d, -6083.317d);
            addSatellitePV(gps, eme2000, itrf, satellitePVList, "2009-12-11T17:00:52.192937", -1043205.448d, -5934643.836d, 3874870.441d, -2415.868d, -3607.05d, -6175.6d);
            addSatellitePV(gps, eme2000, itrf, satellitePVList, "2009-12-11T17:01:13.792937", -1095242.669d, -6010981.571d, 3740513.34d, -2402.051d, -3460.291d, -6264.76d);
            addSatellitePV(gps, eme2000, itrf, satellitePVList, "2009-12-11T17:01:35.392937", -1146963.457d, -6084130.93d, 3604264.372d, -2386.561d, -3311.835d, -6350.751d);
            addSatellitePV(gps, eme2000, itrf, satellitePVList, "2009-12-11T17:01:56.992937", -1198331.706d, -6154056.146d, 3466192.446d, -2369.401d, -3161.764d, -6433.531d);
            addSatellitePV(gps, eme2000, itrf, satellitePVList, "2009-12-11T17:02:18.592937", -1249311.381d, -6220723.191d, 3326367.397d, -2350.574d, -3010.159d, -6513.056d);

    
            // Rugged initialization
            // ---------------------
            Rugged rugged = new RuggedBuilder().
                            setAlgorithm(AlgorithmId.DUVENHAGE).
                            setDigitalElevationModel(srtmUpdater, SRTMelevationUpdater.NUMBER_TILES_CACHED, false).
                            setEllipsoid(EllipsoidId.WGS84, BodyRotatingFrameId.ITRF).
                            setTimeSpan(absDate, absDate.shiftedBy(60.0), 0.01, 5 / lineSensor.getRate(0)).
                            setTrajectory(InertialFrameId.EME2000,
                                          satellitePVList, 4, CartesianDerivativesFilter.USE_P,
                                          satelliteQList,  4,  AngularDerivativesFilter.USE_R).
                            addLineSensor(lineSensor).
                            build();

            // Direct location of a single sensor pixel (first line, first pixel)
            // ------------------------------------------------------------------
            Vector3D position = lineSensor.getPosition(); // This returns a zero vector since we set the relative position of the sensor w.r.T the satellite to 0.
            AbsoluteDate firstLineDate = lineSensor.getDate(0);
            Vector3D los = lineSensor.getLOS(firstLineDate, 0);
            GeodeticPoint upLeftPoint = rugged.directLocation(firstLineDate, position, los);
            System.out.format(Locale.US, "upper left point: φ = %8.3f °, λ = %8.3f °, h = %8.3f m (elevation with SRTM + Geoid))%n",
                    FastMath.toDegrees(upLeftPoint.getLatitude()),
                    FastMath.toDegrees(upLeftPoint.getLongitude()),
                    upLeftPoint.getAltitude());

            // DEM SRTM with Geoid: (value of GEOID checked with QGIS around -30 meters) 
            // upper left point: φ =   37.585 °, λ =  -96.950 °, h =  344.024 m
            
            // DEM SRTM without Geoid (not to be used as elevation must be over ellipsoid for Rugged):
            // upper left point: φ =   37.585 °, λ =  -96.950 °, h =  373.675 m
            
           
            
        } catch (OrekitException oe) {
            System.err.println(oe.getLocalizedMessage());
            System.exit(1);
        } catch (RuggedException re) {
            System.err.println(re.getLocalizedMessage());
            System.exit(1);
        }

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

    
    /**
     * To read SRTM tiles (get from http://dwtkns.com/srtm/)
     */
    private static class SRTMelevationUpdater implements TileUpdater {

        /** Nb cached tiles send to init rugged. */
        public static final int NUMBER_TILES_CACHED = 8;

        /** Last lat value used to ask to update a tile. */
        private double lastLat = Double.NaN;

        /** Last lon value used to ask to update a tile. */
        private double lastLon = Double.NaN;

        /** Nb times the same lat/lon value is used to ask to update a tile (used to avoid infinite loop). */
        private int nbCall = 0;

        /** Nb time rugged ask exactly same DEM tile: means that infinite loop. */
        private static final int NB_TIME_ASK_SAME_TILE = 1000;

        /** Raster root directory, should contains raster files. */
        private String srtmRootDirectory;


        /**
         * Constructor.
         */
        public SRTMelevationUpdater(final String SRTMrootDirectory) {

            this.srtmRootDirectory = SRTMrootDirectory;
            checkRasterDirectory(SRTMrootDirectory);
        }

        /** Update given tile using SRTM elevation.
         * @param latitude latitude that must be covered by the tile (rad)
         * @param longitude longitude that must be covered by the tile (rad)
         * @param tile to update
         */
        @Override
        public void updateTile(final double latitude, final double longitude, final UpdatableTile tile) {

            // Check if latitude and longitude already known
            if (latitude == this.lastLat && longitude == this.lastLon) {
                this.nbCall++;
            } else {
                this.lastLat = latitude;
                this.lastLon = longitude;
                this.nbCall = 0;
            }
            if (this.nbCall > NB_TIME_ASK_SAME_TILE) {
                String str = String.format("infinite loop for %3.8f long %3.8f lat ", longitude, latitude);
                DummyLocalizable message = new DummyLocalizable(str);
                throw new RuggedException(message);
            }

            // Compute the SRTM tile name according to the latitude and longitude
            String rasterFileName = getRasterFilePath(latitude, longitude);

            File rasterFile = new File(rasterFileName);
            if (!rasterFile.exists()) {
                // No SRTM tile found. We may be on water.
                // For instance one can read the geoid ...
                System.out.format("WARNING: No DEM tile found for latitude (deg) = %3.8f  and longitude (deg) = %3.8f." +
                        " SRTM file %s wasn't found. %n",
                        FastMath.toDegrees(latitude), FastMath.toDegrees(longitude), rasterFileName);
                return;
            }

            // Open the SRTM tile
            org.gdal.gdal.Dataset srtmDataset = org.gdal.gdal.gdal.Open(rasterFileName, org.gdal.gdalconst.gdalconst.GA_ReadOnly);
            
            // Get information about the tile
            int rasterLonSize = srtmDataset.GetRasterXSize();
            int rasterLatSize = srtmDataset.GetRasterYSize();
            double[] srtmGeoTransformInfo = srtmDataset.GetGeoTransform();

            double minLat = Double.NaN;
            double minLon = Double.NaN;
            double lonStep = Double.NaN;
            double latStep = Double.NaN;

            if (srtmGeoTransformInfo.length < 6) { // Check that the geoTransformInfo is correct
                String str = "GDALGeoTransform has < 6 elements";
                DummyLocalizable message = new DummyLocalizable(str);
                throw new RuggedException(message);
            } else {
                // Get the latitudes and longitudes datas (in degrees)
                lonStep = FastMath.abs(srtmGeoTransformInfo[1]);
                latStep = FastMath.abs(srtmGeoTransformInfo[5]);

                minLon = srtmGeoTransformInfo[0] + 0.5 * lonStep;
                minLat = srtmGeoTransformInfo[3] - (rasterLatSize - 0.5) * latStep;
            }
            
            // Get the raster band from the tile
            org.gdal.gdal.Band band = srtmDataset.GetRasterBand(1);

            // Define Tile Geometry
            // TBN: angles needed in radians
            tile.setGeometry(FastMath.toRadians(minLat), FastMath.toRadians(minLon), FastMath.toRadians(latStep), FastMath.toRadians(lonStep), rasterLatSize, rasterLonSize);

            // Loop over raster values
            double[] data = new double[rasterLatSize * rasterLonSize];

            // To use geo transform information from GDAL
            GdalTransformTools srtmGTTools = new GdalTransformTools(srtmGeoTransformInfo, rasterLonSize, rasterLatSize);

            // We read all raster at once to limit the numbers of Band.ReadRaster calls
            band.ReadRaster(0, 0, rasterLonSize, rasterLatSize, data);

            // Get the no data value from the raster
            Double[] noDataValue = new Double[1];
            band.GetNoDataValue(noDataValue);

            // test if the no data value exists
            Boolean noDataValueExist = false;
            if (noDataValue[0] != null) noDataValueExist = true;

            // from bottom left to upper right corner
            for (int iLat = rasterLatSize - 1; iLat >= 0; iLat--) {
                for (int jLon = 0; jLon < rasterLonSize; jLon++) {

                    double elevationOverGeoid = 0.0;
                    
                    // Get elevation over GEOID (for SRTM)
                    elevationOverGeoid = data[iLat * rasterLonSize + jLon];

                    if (noDataValueExist && (elevationOverGeoid == noDataValue[0])) {
                        elevationOverGeoid = 0.0;
                    }

                    // The elevation value we send to rugged must be computed against ellipsoid
                    // => for SRTM , we must add geoid value
                    double lonDeg = srtmGTTools.getXFromPixelLine(jLon, iLat);
                    double latDeg = srtmGTTools.getYFromPixelLine(jLon, iLat);
                    
                    // Get the geoid elevation at latDeg and lonDeg
                    int geoidIndexLatitude = (int) FastMath.floor(geoidGTTools.getLineNumFromLatLon(latDeg, lonDeg));
                    int geoidIndexLongitude = (int) FastMath.floor(geoidGTTools.getPixelNumFromLatLon(latDeg, lonDeg));
                    
                    double elevationOverEllipsoid = elevationOverGeoid + geoidElevations[geoidIndexLatitude][geoidIndexLongitude];

                    // Set elevation over the ellipsoid
                    tile.setElevation(rasterLatSize - 1 - iLat, jLon, elevationOverEllipsoid);
                }
            }
            band.delete();
            band = null;
            srtmDataset.delete();
            srtmDataset = null;
        }

        /** Get the file path to the SRTM tile.
         * Need the SRTM tif file named, for instance: srtm_17_05.tif under the directory rootDirectory/17/05/
         * @param latitude latitude (rad)
         * @param longitude longitude (rad)
         * @return the SRTM tile path
         */
        private String getRasterFilePath(final double latitude, final double longitude) {

            double latDeg = FastMath.toDegrees(latitude);
            // Assure that the longitude belongs to [-180, + 180]
            double lonDeg = FastMath.toDegrees(MathUtils.normalizeAngle(longitude, 0.0));

            // Compute parent dir with longitude value according to SRTM naming convention
            int parentDirValue = (int) (1 + (lonDeg + 180.) / 5);
            String parentDir = String.format("%02d", parentDirValue);

            // Compute sub dir with latitude value according to SRTM naming convention
            int subDirValue = (int) (1 + (60. - latDeg) / 5);
            String subDir = String.format("%02d", subDirValue);

            String filePath = this.srtmRootDirectory + File.separator + parentDir + File.separator + subDir + File.separator +
                    "srtm_" + parentDir + "_" + subDir + ".tif";
            
            return filePath;
        }

        /** Chech the root directory for the SRTM files and the presence of .tif 
         * @param directory
         * @return true if everything OK
         */
        private boolean checkRasterDirectory(final String directory) {

            if (directory == null) {

                String str = "Directory not defined";
                DummyLocalizable message = new DummyLocalizable(str);
                throw new RuggedException(message);

            } else {
                try {
                    Path dir = FileSystems.getDefault().getPath(directory);
                    DirectoryStream<Path> stream = Files.newDirectoryStream(dir);
                    boolean found = false;
                    for (Path path : stream) {
                        if (!found) {
                            File currentFile = path.toFile();
                            if (currentFile.isDirectory()) {
                                found = checkRasterDirectory(currentFile.getAbsolutePath());
                            } else {
                                String filePath = currentFile.getAbsolutePath();
                                if (filePath.matches(".*.tif")) {
                                    found = true;
                                }
                            }
                            if (found) {
                                stream.close();
                                return true;
                            }
                        }
                    }
                    stream.close();

                    String str = "SRTM raster file (.tif) not found in " + directory;
                    DummyLocalizable message = new DummyLocalizable(str);
                    throw new RuggedException(message);

                } catch (IOException e) {
                    String str = "Directory for SRTM tiles not found " + directory;
                    DummyLocalizable message = new DummyLocalizable(str);
                    throw new RuggedException(message);
                }
            }
        }
    }

    /**
     * To read the GEOID
     */
    private static class GEOIDelevationReader {

        /** Geoid dataset */
        private org.gdal.gdal.Dataset geoidDataset = null;

        /** Geoid elevations for the whole Earth (read from Geoid dataset).
         * Filled in with GDAL convention : [0][0] = North West         */
        private double[][] elevationsWholeEarth = null;

        /** the Geo transform info */
        private double[] geoidGeoTransformInfo;

        /** Size in longitude of the Geoid (number of columns/pixels). */
        private int geoidLonSize;

        /** Size in latitude of the Geoid (number of lines). */
        private int geoidLatSize;

        /**
         * Constructor
         * @param geoidFilePath path to geoid file
         */
        public GEOIDelevationReader(String geoidFilePath) {

            // Open the Geoid dataset
            openGeoid(geoidFilePath);
            
            // Analyze and read the raster: initialize this.elevationsWholeEarth
            this.geoidGeoTransformInfo = readRaster();
        }

        /**
         * Analyse and read the Geoid raster. Initialize elevationsWholeEarth (fill in with GDAL convention : [0][0] = North West)
         * @return the geoTransformInfo (read from the geoidDataset)
         */
        private double[] readRaster() {

            // Analyse the Geoid dataset
            this.geoidLonSize = geoidDataset.GetRasterXSize();
            this.geoidLatSize = geoidDataset.GetRasterYSize();

            // Get the Geo transform info: lon origin (deg), lon step (deg), 0, lat origin (deg), 0, lat step (deg)
            double[] geoTransformInfo = geoidDataset.GetGeoTransform();
            if (geoTransformInfo.length < 6) { // should not occur; if a problem occurs : the default transform value is set to (0,1,0,0,0,1)
                String str = "GDALGeoTransform does not contains 6 elements";
                DummyLocalizable message = new DummyLocalizable(str);
                throw new RuggedException(message);
            }

            // Read the raster
            // ===============
            // To store the raster data
            double[] data = new double[this.geoidLatSize * this.geoidLonSize];
            // Get the raster
            Band band = geoidDataset.GetRasterBand(1);
            // We read all the raster once for all
            band.ReadRaster(0, 0, this.geoidLonSize, this.geoidLatSize, data);
            // Get the no data value from the raster
            Double[] noDataValue = new Double[1];
            band.GetNoDataValue(noDataValue);
            // test if the no data value exists
            Boolean noDataValueExist = false;
            if (noDataValue[0] != null) noDataValueExist = true;

            // Read the full raster all in once ...
            this.elevationsWholeEarth = new double[this.geoidLatSize][this.geoidLonSize];

            // from bottom left to upper right corner (GDAL convention)
            for (int iLat = this.geoidLatSize - 1; iLat >= 0; iLat--) {
                for (int jLon = 0; jLon < this.geoidLonSize; jLon++) {
                    // if jlon = 0 and ilat = rasterLatSize (721) : crash
                    // if iLat = 720 and jlon = rasterLonSize (1440) : crash

                    // Read the data with GDAL convention :
                    // iLat = 0 at North; iLat = rasterLatSize - 1 at South;
                    // jLon = 0 at West; jLon = rasterLonSize - 1 at East
                    double elevationOverEllipsoid = data[iLat * this.geoidLonSize + jLon];
                    if (noDataValueExist && (elevationOverEllipsoid == noDataValue[0])) {
                        elevationOverEllipsoid = 0.0;
                    }
                    // Set elevation for current latitude and longitude
                    // IMPORTANT : keep the GDAL convention [0][0] = North West
                    this.elevationsWholeEarth[iLat][jLon] = elevationOverEllipsoid;

                } // end loop jLon
            } // end loop iLat


            // Close Dataset and Band once the raster is read ...
            band.delete();
            band = null;
            geoidDataset.delete();
            geoidDataset = null;

            // Return the Geo Transform info
            return geoTransformInfo;
        }


        public double[][] getElevationsWholeEarth() {
            return elevationsWholeEarth;
        }
        
        public double[] getGeoidGeoTransformInfo() {
            return geoidGeoTransformInfo;
        }

        public int getGeoidLonSize() {
            return geoidLonSize;
        }

        public int getGeoidLatSize() {
            return geoidLatSize;
        }
        
        /** Open the geoid dataset
         * @param geoidFilePath the geoid file path
         * @return the geoid dataset
         */
        public org.gdal.gdal.Dataset openGeoid(String geoidFilePath) throws RuggedException {

            this.geoidDataset = (org.gdal.gdal.Dataset) org.gdal.gdal.gdal.Open(geoidFilePath, org.gdal.gdalconst.gdalconst.GA_ReadOnly);

            if (this.geoidDataset == null) {
                throw new RuggedException(RuggedMessages.UNKNOWN_TILE);

            }
            return this.geoidDataset;
        }
    }
    
    /**
     * Tool using GeoTransform info returned from GDAL.
     */
    static private class GdalTransformTools {

        /** GeoTransform info from Gdal. */
        private double[] gtInfo;

        /** x size (longitude in deg). */
        private double xSize;

        /** y size (latitude in deg). */
        private double ySize;

        /**
         * Constructor.
         * @param gtInfo geo transform info from gdal
         * @param xSize x size (longitude in deg)
         * @param ySize y size (latitude in deg)
         */
        public GdalTransformTools(final double[] gtInfo, final double xSize, final double ySize) {
            // We suppose gtInfo[5] and gtInfo[1] different from 0
            // no tests are performed about gtInfo values ... as org.gdal.gdal.Dataset.GetGeoTransform will give
            // a default transform value set to (0,1,0,0,0,1) if problems
            this.gtInfo = gtInfo;
            this.xSize = xSize;
            this.ySize = ySize;
        }

        /**
         * Get X (longitude) coord from pixel and line.
         * @param pixelNum pixel number
         * @param lineNum line number
         * @return X coord (longitude in degrees)  from pixel and line. For Gdal = Upper left corner of the cell.
         */
        public double getXFromPixelLine(final double pixelNum, final double lineNum) {
            return gtInfo[0] + gtInfo[1] * pixelNum + gtInfo[2] * lineNum;
        }

        /**
         * Get Y (latitude) coord from pixel and line.
         * @param pixelNum pixel number
         * @param lineNum line number
         * @return Y coord (latitude in degrees) from pixel and line. For Gdal = Upper left corner of the cell.
         */
        public double getYFromPixelLine(final double pixelNum, final double lineNum) {
            return gtInfo[3] + gtInfo[4] * pixelNum + gtInfo[5] * lineNum;
        }

        /**
         * Get pixel number from latitude and longitude.
         * @param lat latitude (deg)
         * @param lon longitude (deg)
         * @return pixel number (pixel number = index in longitude). For Gdal = Upper left corner of the cell.
         */
        public double getPixelNumFromLatLon(final double lat, final double lon) {
            // We suppose gtInfo[5] and gtInfo[1] different from 0
            double tmp = gtInfo[2] / gtInfo[5];
            double value = (lon - tmp * lat - gtInfo[0] + tmp * gtInfo[3]) / (gtInfo[1] - tmp * gtInfo[4]);

            // if value = xSize, don't convert value to get the inverse value from getLatMax and getLonMax
            if (value >= xSize) {
                value = value % xSize;
            }
            return value;
        }

        /**
         * Get line number from latitude and longitude.
         * @param lat latitude (deg)
         * @param lon longitude (deg)
         * @return line number (line number = index in latitude). For Gdal = Upper left corner of the cell.
         */
        public double getLineNumFromLatLon(final double lat, final double lon) {
            // We suppose gtInfo[1] and gtInfo[5] different from 0
            double tmp = gtInfo[4] / gtInfo[1];
            double value = (lat - tmp * lon - gtInfo[3] + tmp * gtInfo[0]) / (gtInfo[5] - tmp * gtInfo[2]);
            if (value >= ySize) { // if value = xSize, don't convert value to get the inverse value from getLatMax and getLonMax
                value = value % ySize;
            }
            return value;
        }
    }
    }
    
    
**To be noticed:**

if you are using a version of Rugged &le; 3.0, you need to replace the call:

     setDigitalElevationModel(srtmUpdater, SRTMelevationUpdater.NUMBER_TILES_CACHED, false)

by

     setDigitalElevationModel(srtmUpdater, SRTMelevationUpdater.NUMBER_TILES_CACHED)

and use a DEM with overlapping tiles as explained in [Direct location with a DEM](./direct-location-with-DEM.html).
  