%In this example we aim to create the rugged object based on the DIM file (in xml format).

%Dimap is a format file used to provide interior and exterior orientation
%elements, this format is used by several satellites such: SPOT5,
%Formosat, THEOS, ALSAT-2A, KOMPSAT-2 and other.
%Except SPOT5 and Formosat, the interior orientation parameters are
%given in the form of polynomial coefficients that allow the estimation of the
%look direction of a considered pixel in a frame called RLOS and also the
%Bias angles to transform the look directions from RLOS to RSAT.
%(see: DOI 10.1007/s12524-014-0380-x and doi:10.5194/isprsarchives-XL-1-W1-35-2013)
%as an example for THEOS Dimap file see:
%http://ortho-qgis.googlecode.com/svn/trunk/Chiangmai1Dimap.xml
%This example has been tested and works also using ALSAT-2A DIM file.

function dim2rugged()
clear all
%Put 'orekit-data.zip' in the current directory, if not I get an error in line 55  %org.orekit.errors.OrekitException: aucune donnée d'historique UTC-TAI n'a été chargée

% These seems to work if pasted to prompt.
% javaaddpath 'C:\ ... enter your path here ...\MATLAB'
% javaaddpath 'C:\.. enter your path here ...\MATLAB\orekit-7.1.jar'
% javaaddpath 'C:\.. enter your path here ...\\MATLAB\rugged-1.0.jar'
% javaaddpath 'C:\.. enter your path here... \\MATLAB\commons-math3-3.6.1.jar'

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

dimFile='Chiangmai1Dimap.xml';

%terrainaltitude is the altitude of the ground intersection point
terrainAltitude=800;

%this function returns a RuggedBuilder based on DIM file and a nominal terrainAltitude
ruggedBuilder= Dim2RuggedBuilderObj(dimFile,terrainAltitude);

%the terrainAltitude can be changed using ruggedBuilder.setConstantElevation(terrainAltitude);
%then dimRugged=ruggedBuilder.build(); must be called to create a new Rugged that
%allow the direct location at a different terrain altitude without parsing
%the DIM file again.
dimRugged=ruggedBuilder.build();

%This exemple consider the PAN image only where the sensor line is called
%PAN Sensor
lineSensor=dimRugged.getLineSensor('PAN Sensor');

%Get the LOS and the datation of the first pixel of the image
position = lineSensor.getPosition(); 
lineDate = lineSensor.getDate(0);
los = lineSensor.getLos(lineDate, 0);

%Get the corresponding ground coordinates of the considered pixel at terrainAltitude
upLeftPoint = dimRugged.directLocation(lineDate, position, los);

%Print the results
fprintf('upper left point: Lat = %8.6f °, Lon = %8.6f °, h = %8.2f m\n',rad2deg(upLeftPoint.getLatitude()), rad2deg(upLeftPoint.getLongitude()),upLeftPoint.getAltitude() )
 

end
function ruggedBuilder= Dim2RuggedBuilderObj(dimFile,terrainAltitude)
% This function creates a RuggedBuilder object based on DIM file
% The intersection are done at terrainAltitude, when this object is used for DirectLocation 
%% Configure Orekit. The file orekit-data.zip must be in current dir
import org.orekit.data.ZipJarCrawler
import org.orekit.time.AbsoluteDate
import org.orekit.time.TimeScalesFactory
DM=org.orekit.data.DataProvidersManager.getInstance();
crawler=org.orekit.data.ZipJarCrawler('orekit-data.zip');
DM.clearProviders();
DM.addProvider(crawler);
%%
%Read XML document and get Document Object Model node

xDoc = xmlread(dimFile);

%REFERENCE_TIME is the time of imaging of the REFERENCE_LINE 
Element='REFERENCE_TIME';
referenceTime=GetElementValue(Element,xDoc);

%FormatTime to get the read time in the standard format
referenceTime=FormatTime(referenceTime);

%LINE_PERIOD is the time required to record an image line
Element='LINE_PERIOD';

%LineRate is the inverse of the line period
lineRate=1/(str2double(GetElementValue(Element,xDoc)));

%REFERENCE_LINE the line used for datation reference
Element='REFERENCE_LINE';
referenceLine=str2double(GetElementValue(Element,xDoc));

%NROWS the number of Rows
Element='NROWS';
nRows=str2double(GetElementValue(Element,xDoc));

%NCOLS the number of columns
Element='NCOLS';
nCols=str2double(GetElementValue(Element,xDoc));

%YAW, PITCH, ROLL are the angles between RLOS and RSAT, where RLOS is the
%frame where the los are measured and RSAT is the frame tied to the
%spacecraft where the quatenions represent the rotaion between RSAT and
%EME2000

Element='YAW';
yaw=str2double(GetElementValue(Element,xDoc));

Element='PITCH';
pitch=str2double(GetElementValue(Element,xDoc));

Element='ROLL';
roll=str2double(GetElementValue(Element,xDoc));

%XLOS_i and YLOS_i are the polynomial coefficiens that allow the 
%calculation of LOS direction of each pixel in RLOS 
%(see: DOI 10.1007/s12524-014-0380-x)

Element='XLOS_3';
xLosPoly=str2double(GetElementValue(Element,xDoc));
Element='XLOS_2';
xLosPoly=[xLosPoly, str2double(GetElementValue(Element,xDoc))];
Element='XLOS_1';
xLosPoly=[xLosPoly, str2double(GetElementValue(Element,xDoc))];
Element='XLOS_0';
xLosPoly=[xLosPoly, str2double(GetElementValue(Element,xDoc))];

Element='YLOS_3';
yLosPoly=str2double(GetElementValue(Element,xDoc));
Element='YLOS_2';
yLosPoly=[yLosPoly, str2double(GetElementValue(Element,xDoc))];
Element='YLOS_1';
yLosPoly=[yLosPoly, str2double(GetElementValue(Element,xDoc))];
Element='YLOS_0';
yLosPoly=[yLosPoly, str2double(GetElementValue(Element,xDoc))];
%%
quaternions = xDoc.getElementsByTagName('Quaternion');
%read the Quaternions into the arraylist
satelliteQList=ParseQuaternions(quaternions);

%%
pv = xDoc.getElementsByTagName('Point');
%read the Positions/Velocities into the arraylist
satellitePVList=ParsePV(pv);

%%
%Build the LOS based on whole pushbroom size (ncols), the polynomial
%describing the LOS and bias angles (yaw,pitch,roll)
lineOfSight=LosBuilding(nCols,xLosPoly,yLosPoly,yaw,pitch,roll);

 %%
 import org.orekit.time.AbsoluteDate;
 import org.orekit.time.TimeScalesFactory;
 import org.orekit.rugged.linesensor.LinearLineDatation;

%setup of the datation

 gps = TimeScalesFactory.getGPS();
 absDate =  AbsoluteDate(referenceTime, gps);
 lineDatation =  LinearLineDatation(absDate, referenceLine, lineRate); 

 %%
 import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
 import org.orekit.rugged.linesensor.LineSensor;
 %With the LOS and the datation now defined , we can initialize a line sensor object in Rugged:
 lineSensor =  LineSensor('PAN Sensor', lineDatation, Vector3D.ZERO, lineOfSight);

%%
import org.orekit.rugged.api.*;
import org.orekit.rugged.errors.RuggedException;
import org.orekit.utils.*;

ruggedBuilder=RuggedBuilder();

%the intersection during the direct location will be at "terrainAltitude" 
ruggedBuilder.setConstantElevation(terrainAltitude);
ruggedBuilder.setAlgorithm(AlgorithmId.CONSTANT_ELEVATION_OVER_ELLIPSOID);
ruggedBuilder.setEllipsoid(EllipsoidId.WGS84, BodyRotatingFrameId.ITRF);

captureDuration=(nRows+100)/ lineSensor.getRate(0);

ruggedBuilder.setTimeSpan(absDate, absDate.shiftedBy(captureDuration), 0.001, 5 / lineSensor.getRate(0));
ruggedBuilder.setTrajectory(InertialFrameId.EME2000,...
              satellitePVList, 4, CartesianDerivativesFilter.USE_P,...
              satelliteQList,  4, AngularDerivativesFilter.USE_R);
ruggedBuilder.addLineSensor(lineSensor);


end
function lineOfSight=LosBuilding(nCols,xLosPoly,yLosPoly,yaw,pitch,roll)
import java.util.ArrayList;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.orekit.rugged.los.*;
import org.orekit.rugged.utils.*; 

rawDirs = ArrayList();

for i =1:nCols
    
    psiY = -polyval(xLosPoly,i)/1000;
    psiX = polyval(yLosPoly,i)/1000;                
    rawDirs.add (Vector3D(psiX,psiY,1));
    
end

losBuilder = LOSBuilder(rawDirs);
losBuilder.addTransform(FixedRotation(ParameterType.FIXED, Vector3D.PLUS_K,yaw));
losBuilder.addTransform(FixedRotation(ParameterType.FIXED, Vector3D.PLUS_J,pitch));
losBuilder.addTransform(FixedRotation(ParameterType.FIXED, Vector3D.PLUS_I,roll));

lineOfSight = losBuilder.build();

end

function valElement=GetElementValue(element,xDoc)
% valElement=GetElementValue(element,xDoc) returne a string contained in
% the corresponding "element" in the XML "xDoc" file
valElement=char(xDoc.getElementsByTagName(element).item(0).getTextContent);

end

function time=FormatTime(time)
%FormatTime replace the space contained the time string by "T"
position=strfind(time,' ');
time(position)='T';

end
function satellitePVList=ParsePV(pv)
%ParsePV parse the "Point" element in the dimap file to get an ArrayList
import java.util.ArrayList;
import java.util.List;
import org.orekit.time.TimeScalesFactory;
import org.orekit.frames.*;
import org.orekit.utils.*;

satellitePVList =  ArrayList();

gps=TimeScalesFactory.getGPS();
eme2000 = FramesFactory.getEME2000();
simpleEOP = true; % we don't want to compute tiny tidal effects at millimeter level
itrf = FramesFactory.getITRF(IERSConventions.IERS_2010, simpleEOP);


nPV = pv.getLength;
position=[];
velocity=[];

for i=0:nPV - 1
    entries = pv.item(i).getChildNodes;
    node = entries.getFirstChild;
while ~isempty(node)

    if strcmpi(node.getNodeName, 'TIME')
        T= FormatTime(char(node.getTextContent));
    end
    if strcmpi(node.getNodeName, 'Location')
        position= str2num(char(node.getTextContent));
    end
    if strcmpi(node.getNodeName, 'Velocity')
        velocity= str2num(char(node.getTextContent));
    end

    node = node.getNextSibling;
    
end
addSatellitePV(gps, eme2000, itrf, satellitePVList, T, position(1), position(2), position(3), velocity(1), velocity(2), velocity(3));

end


end


function satelliteQList=ParseQuaternions(quaternions)
%ParseQuaternions parse the "Quaternion" element in the dimap file to get an ArrayList
import java.util.ArrayList;
import java.util.List;
import org.orekit.time.TimeScalesFactory;

satelliteQList =  ArrayList();
gps=TimeScalesFactory.getGPS();

nQuaternions=quaternions.getLength;

for i=0:nQuaternions - 1
    entries = quaternions.item(i).getChildNodes;
    node = entries.getFirstChild;
while ~isempty(node)
    if strcmpi(node.getNodeName, 'TIME')
        T= FormatTime(char(node.getTextContent));
    end
    if strcmpi(node.getNodeName, 'Q0')
        Q0= str2double(char(node.getTextContent));
    end
    if strcmpi(node.getNodeName, 'Q1')
        Q1= str2double(char(node.getTextContent));
    end
    if strcmpi(node.getNodeName, 'Q2')
        Q2= str2double(char(node.getTextContent));
    end
    if strcmpi(node.getNodeName, 'Q3')
        Q3= str2double(char(node.getTextContent));
    end
    node = node.getNextSibling;
end
addSatelliteQ(gps, satelliteQList, T, Q0, Q1, Q2, Q3);
end

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
