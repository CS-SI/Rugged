

function DirectLocation()
clear all

%Put 'orekit-data.zip' in the current directory, if not I get an error in line 55  %org.orekit.errors.OrekitException: aucune donnée d'historique UTC-TAI n'a été chargée

% These seems to work if pasted to prompt.
% javaaddpath 'C:\ ... enter your path here ...\MATLAB'
% javaaddpath 'C:\.. enter your path here ...\MATLAB\orekit-7.1.jar'
% javaaddpath 'C:\.. enter your path here ...\\MATLAB\rugged-1.0.jar'
% javaaddpath 'C:\.. enter your path here... \\MATLAB\commons-math3-3.6.1.jar'
%%

import java.util.ArrayList;
import java.util.List;
import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.orekit.rugged.los.*;
import org.orekit.rugged.utils.*; 

%% Configure Orekit. The file orekit-data.zip must be in current dir
import org.orekit.data.ZipJarCrawler
import org.orekit.time.AbsoluteDate
import org.orekit.time.TimeScalesFactory
DM=org.orekit.data.DataProvidersManager.getInstance();
crawler=org.orekit.data.ZipJarCrawler('orekit-data.zip');
DM.clearProviders();
DM.addProvider(crawler);


%%

%The raw viewing direction of pixel i with respect to the instrument is defined by the vector:
rawDirs = ArrayList();
for i =1:2000
    %20° field of view, 2000 pixels
    rawDirs.add (Vector3D(0,i*deg2rad(20)/2000,1));
end

%The instrument is oriented 10° off nadir around the X-axis, we need to rotate the viewing
%direction to obtain the line of sight in the satellite frame
losBuilder = LOSBuilder(rawDirs);
losBuilder.addTransform(FixedRotation(ParameterType.FIXED, Vector3D.PLUS_I, deg2rad(10)));
lineOfSight = losBuilder.build();

 %%
 
 import org.orekit.time.AbsoluteDate;
 import org.orekit.time.TimeScalesFactory;
 import org.orekit.rugged.linesensor.LinearLineDatation;


%We use Orekit for handling time and dates, and Rugged for defining the datation model:
 utc = TimeScalesFactory.getUTC();
 absDate =  AbsoluteDate('2009-12-11T16:59:30.0', utc);
 lineDatation =  LinearLineDatation(absDate, 1, 20); 
 
 
 %%
 import org.orekit.rugged.linesensor.LineSensor;
%With the LOS and the datation now defined , we can initialize a line sensor object in Rugged:
lineSensor =  LineSensor('mySensor', lineDatation, Vector3D.ZERO, lineOfSight);
%%

import org.orekit.frames.*;
import org.orekit.utils.*;
%In our application, we simply need to know the name of the frames we are working with. Positions and
%velocities are given in the ITRF terrestrial frame, while the quaternions are given in EME2000
%inertial frame.
eme2000 = FramesFactory.getEME2000();
simpleEOP = true; % we don't want to compute tiny tidal effects at millimeter level
itrf = FramesFactory.getITRF(IERSConventions.IERS_2010, simpleEOP);
%%

satelliteQList =  ArrayList();
gps=TimeScalesFactory.getGPS();

addSatelliteQ(gps, satelliteQList, '2009-12-11T16:58:42.592937', -0.340236, 0.333952, -0.844012, -0.245684);
addSatelliteQ(gps, satelliteQList, '2009-12-11T16:59:06.592937', -0.354773, 0.329336, -0.837871, -0.252281);
addSatelliteQ(gps, satelliteQList, '2009-12-11T16:59:30.592937', -0.369237, 0.324612, -0.831445, -0.258824);
addSatelliteQ(gps, satelliteQList, '2009-12-11T16:59:54.592937', -0.3836, 0.319792, -0.824743, -0.265299);
addSatelliteQ(gps, satelliteQList, '2009-12-11T17:00:18.592937', -0.397834, 0.314883, -0.817777, -0.271695);
addSatelliteQ(gps, satelliteQList, '2009-12-11T17:00:42.592937', -0.411912, 0.309895, -0.810561, -0.278001);
addSatelliteQ(gps, satelliteQList, '2009-12-11T17:01:06.592937', -0.42581, 0.304838, -0.803111, -0.284206);
addSatelliteQ(gps, satelliteQList, '2009-12-11T17:01:30.592937', -0.439505, 0.299722, -0.795442, -0.290301);
addSatelliteQ(gps, satelliteQList, '2009-12-11T17:01:54.592937', -0.452976, 0.294556, -0.787571, -0.296279);
addSatelliteQ(gps, satelliteQList, '2009-12-11T17:02:18.592937', -0.466207, 0.28935, -0.779516, -0.302131);
           
%%
satellitePVList =  ArrayList();

addSatellitePV(gps, eme2000, itrf, satellitePVList, '2009-12-11T16:58:42.592937', -726361.466, -5411878.485, 4637549.599, -2463.635, -4447.634, -5576.736);
addSatellitePV(gps, eme2000, itrf, satellitePVList, '2009-12-11T16:59:04.192937', -779538.267, -5506500.533, 4515934.894, -2459.848, -4312.676, -5683.906);
addSatellitePV(gps, eme2000, itrf, satellitePVList, '2009-12-11T16:59:25.792937', -832615.368, -5598184.195, 4392036.13, -2454.395, -4175.564, -5788.201);
addSatellitePV(gps, eme2000, itrf, satellitePVList, '2009-12-11T16:59:47.392937', -885556.748, -5686883.696, 4265915.971, -2447.273, -4036.368, -5889.568);
addSatellitePV(gps, eme2000, itrf, satellitePVList, '2009-12-11T17:00:08.992937', -938326.32, -5772554.875, 4137638.207, -2438.478, -3895.166, -5987.957);
addSatellitePV(gps, eme2000, itrf, satellitePVList, '2009-12-11T17:00:30.592937', -990887.942, -5855155.21, 4007267.717, -2428.011, -3752.034, -6083.317);
addSatellitePV(gps, eme2000, itrf, satellitePVList, '2009-12-11T17:00:52.192937', -1043205.448, -5934643.836, 3874870.441, -2415.868, -3607.05, -6175.6);
addSatellitePV(gps, eme2000, itrf, satellitePVList, '2009-12-11T17:01:13.792937', -1095242.669, -6010981.571, 3740513.34, -2402.051, -3460.291, -6264.76);
addSatellitePV(gps, eme2000, itrf, satellitePVList, '2009-12-11T17:01:35.392937', -1146963.457, -6084130.93, 3604264.372, -2386.561, -3311.835, -6350.751);
addSatellitePV(gps, eme2000, itrf, satellitePVList, '2009-12-11T17:01:56.992937', -1198331.706, -6154056.146, 3466192.446, -2369.401, -3161.764, -6433.531);
addSatellitePV(gps, eme2000, itrf, satellitePVList, '2009-12-11T17:02:18.592937', -1249311.381, -6220723.191, 3326367.397, -2350.574, -3010.159, -6513.056);
%%

% import org.orekit.rugged.api.AlgorithmId;
% import org.orekit.rugged.api.BodyRotatingFrameId;
% import org.orekit.rugged.api.EllipsoidId;
% import org.orekit.rugged.api.InertialFrameId;
% import org.orekit.rugged.api.Rugged;
import org.orekit.rugged.api.*;
import org.orekit.rugged.errors.RuggedException;
import org.orekit.utils.*;
% import org.orekit.utils.CartesianDerivativesFilter;

rugged=RuggedBuilder();
rugged.setAlgorithm(AlgorithmId.IGNORE_DEM_USE_ELLIPSOID);
rugged.setEllipsoid(EllipsoidId.WGS84, BodyRotatingFrameId.ITRF);
rugged.setTimeSpan(absDate, absDate.shiftedBy(60.0), 0.01, 5 / lineSensor.getRate(0));
rugged.setTrajectory(InertialFrameId.EME2000,...
    satellitePVList, 4, CartesianDerivativesFilter.USE_P,...
    satelliteQList,  4,  AngularDerivativesFilter.USE_R);
rugged.addLineSensor(lineSensor);
Rugged=rugged.build();

%%

import org.orekit.bodies.GeodeticPoint;
 position = lineSensor.getPosition(); % This returns a zero vector since we set the relative position of the sensor w.r.T the satellite to 0.
 firstLineDate = lineSensor.getDate(0);
 los = lineSensor.getLos(firstLineDate, 0);
 upLeftPoint = Rugged.directLocation(firstLineDate, position, los);

fprintf('upper left point: Lat = %8.3f °, Lon = %8.3f °, h = %8.3f m\n',rad2deg(upLeftPoint.getLatitude()), rad2deg(upLeftPoint.getLongitude()),upLeftPoint.getAltitude() )

end

function satelliteQList=addSatelliteQ( gps,  satelliteQList ,absDate,q0,  q1,  q2,  q3) 
 import org.orekit.utils.TimeStampedAngularCoordinates;
import org.orekit.time.AbsoluteDate;
import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
 attitudeDate =  AbsoluteDate(absDate, gps);
 rotation =  Rotation(q0, q1, q2, q3, true);
 rotationRate = Vector3D.ZERO;
 rotationAcceleration = Vector3D.ZERO;
 pair =  TimeStampedAngularCoordinates(attitudeDate, rotation, rotationRate, rotationAcceleration); 
 satelliteQList.add(pair);
 
end

function  satellitePVList= addSatellitePV( gps,  eme2000,  itrf, satellitePVList, absDate, px,  py,  pz,  vx,  vy,  vz)
        import org.orekit.time.AbsoluteDate;
        import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
        import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
         import org.orekit.utils.TimeStampedPVCoordinates;
         import org.orekit.utils.PVCoordinates;
         import org.orekit.frames.Transform;
        ephemerisDate =  AbsoluteDate(absDate, gps);
        position =  Vector3D(px, py, pz);
        velocity =  Vector3D(vx, vy, vz);
        pvITRF =  PVCoordinates(position, velocity);
        transform = itrf.getTransformTo(eme2000, ephemerisDate);
        pvEME2000 = transform.transformPVCoordinates(pvITRF); 
        satellitePVList.add( TimeStampedPVCoordinates(ephemerisDate, pvEME2000.getPosition(), pvEME2000.getVelocity(), Vector3D.ZERO));
end

