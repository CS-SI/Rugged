/* Copyright 2013-2019 CS Systèmes d'Information
 * Licensed to CS Systèmes d'Information (CS) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.
 * CS licenses this file to You under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with
 * the License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package org.orekit.rugged.utils;

import java.text.NumberFormat;

import org.hipparchus.exception.MathRuntimeException;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.util.CompositeFormat;
import org.hipparchus.util.FastMath;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.errors.OrekitException;

public class GeodeticUtilities {

    /** Compute an (approximate) geodetic distance in meters between geodetic points (long1, lat1) and (long2, lat2).
     * TBN: Orekit does not have such a method
     * @param earthRadius Earth radius (m)
     * @param long1 longitude of first geodetic point (rad)
     * @param lat1 latitude of first geodetic point (rad)
     * @param long2 longitude of second geodetic point (rad)
     * @param lat2 latitude of second geodetic point (rad)
     * @return distance in meters
     */
    public static double computeDistanceInMeter(double earthRadius, final double long1, final double lat1,
                                                                    final double long2, final double lat2) {

        // get vectors on unit sphere from angular coordinates
        final Vector3D p1 = new Vector3D(long1, lat1);
        final Vector3D p2 = new Vector3D(long2, lat2);
        return earthRadius * Vector3D.angle(p1, p2);
    }
    
    /** Compute an (approximate) geodetic distance in meters between two geodetic points
     * TBN: Orekit does not have such a method
     * @param earthRadius Earth radius (m)
     * @param gp1 first geodetic point
     * @param gp2 second geodetic point
     * @return distance in meters
     */
    public static double computeDistanceInMeter(double earthRadius, final GeodeticPoint gp1, final GeodeticPoint gp2) {

        return computeDistanceInMeter(earthRadius, gp1.getLongitude(), gp1.getLatitude(), gp2.getLongitude(), gp2.getLatitude());
    }



    public static String toStringDMS(GeodeticPoint gp) {
        
        final NumberFormat format = CompositeFormat.getDefaultNumberFormat();
        format.setMaximumFractionDigits(1);
        DMSangle latDMS = convertLatitudeToDMS(FastMath.toDegrees(gp.getLatitude()));
        DMSangle lonDMS = convertLongitudeToDMS(FastMath.toDegrees(gp.getLongitude()));

        String latSign = "";
        if (latDMS.getCardinalPoint() == CardinalDirection.South) latSign = "-";
        String lonSign = "";
        if (lonDMS.getCardinalPoint() == CardinalDirection.West) lonSign = "-";

        return "{lat: " + latSign + 
                format.format(latDMS.getDegrees()) + "° " + 
                format.format(latDMS.getMinutes()) + "' " +
                format.format(latDMS.getSeconds()) + "'' " +
                " lon: " + lonSign + 
                format.format(lonDMS.getDegrees()) + "° " + 
                format.format(lonDMS.getMinutes()) + "' " +
                format.format(lonDMS.getSeconds()) + "'' " +
                "}";
    }

    /**
     * Convert longitude (in decimal degrees) in degrees/minutes/seconds
     * @param longitudeInDecimalDegrees
     * @return the DMS angle
     */
    static DMSangle convertLongitudeToDMS(double longitudeInDecimalDegrees) {

        String cardinalDirection;
        // Get the cardinal direction
        if (longitudeInDecimalDegrees >= 0.0){
            cardinalDirection= "E";
        } else {
            cardinalDirection= "W";
        }

        return convertToDMS(longitudeInDecimalDegrees, cardinalDirection);
    }

    /**
     * Convert latitude (in decimal degrees) in degrees/minutes/seconds
     * @param latitudeInDecimalDegrees
     * @return the DMSangle
     */
    public static DMSangle convertLatitudeToDMS(double latitudeInDecimalDegrees){

        String cardinalDirection;
        // Get the cardinal direction
        if (latitudeInDecimalDegrees >= 0.0){
            cardinalDirection= "N";
        } else {
            cardinalDirection= "S";
        }

        return convertToDMS(latitudeInDecimalDegrees, cardinalDirection);

    }

    /**
     * Convert angle (in decimal degrees) in degrees/minutes/seconds and add the associated cardinal direction
     * @param angleInDecimalDegrees angle in decimal degrees
     * @param cardinalDirection the associated cardinal direction
     * @return the DMSangle 
     */
    private static DMSangle convertToDMS(double angleInDecimalDegrees, String cardinalDirection) {

        // We know the cardinal direction so we work on |angleInDecimalDegrees| the positive value 
        double angleInDD = FastMath.abs(angleInDecimalDegrees);
        // Get the degrees part
        int degreesPart = (int) FastMath.floor(angleInDD);

        // Get the minutes part (always positive value)
        double minutesInDecimal = 60.*(angleInDD - degreesPart);
        int minutesPart = (int) FastMath.floor(minutesInDecimal);

        // Get the seconds (in decimal)
        double secondsInDecimal = 60.*(minutesInDecimal - minutesPart);

        // Due to rounding problem (at equator around 30 micrometre > diameter of human hair)
        if (secondsInDecimal > (60. - 1.e-8)) {
            secondsInDecimal = 0.0;
            minutesPart++;
            if (minutesPart == 60) {
                minutesPart = 0;
                degreesPart++;
            }
        }

        return new DMSangle(degreesPart, minutesPart, secondsInDecimal, cardinalDirection);
    }
}
class DMSangle {

    /**
     * Degrees part of the angle
     */
    private int degrees;

    /**
     * Minutes part of the angle
     */
    private int minutes;

    /**
     * Seconds part of the angle
     */
    private double seconds;

    /**
     * Cardinal direction
     */
    private CardinalDirection cardinalPoint;

    /**
     * Create a DMS angle for a GeodeticPoint (for instance)
     * @param degrees degrees part
     * @param minutes minutes part
     * @param seconds seconds part 
     * @param cardinalName cardinal direction
     */
    public DMSangle(int degrees, int minutes, double seconds, String cardinalName) {
        this.degrees = degrees;
        this.minutes = minutes;
        this.seconds = seconds;
        this.cardinalPoint = CardinalDirection.getCardinalDirectionFromName(cardinalName);
        if (this.cardinalPoint == null){
            throw new OrekitException(new MathRuntimeException(null, this.cardinalPoint));
        }
    }

    /**
     * @return the degrees part of the angle
     */
    public int getDegrees() {
        return degrees;
    }
    /**
     * @return the minutes part of the angle
     */
    public int getMinutes() {
        return minutes;
    }
    /**
     * @return the seconds part of the angle
     */
    public double getSeconds() {
        return seconds;
    }
    /**
     * @return the cardinal direction of the latitude or the longitude
     */
    public CardinalDirection getCardinalPoint() {
        return cardinalPoint;
    }
}

enum CardinalDirection {
    North("North","N"), 
    South("South","S"), 
    West("West","W"), 
    East("East","E");
    /**
     * Cardinal direction full name
     */
    private String cardinalFullName = null;

    /**
     * Cardinal direction short name
     */
    private String cardinalShortName = null;

    /**
     * Private constructor
     * @param fullName
     * @param shortName
     */
    private CardinalDirection(String fullName, String shortName){
        this.cardinalFullName = fullName;
        this.cardinalShortName = shortName;
    }

    /**
     * Get the cardinal direction from name (long or short)
     * @param cardinalName cardinal name (long or short)
     * @return the CardinalDirection
     */
    public static CardinalDirection getCardinalDirectionFromName(String cardinalName){
        // Get the cardinal direction from full name ...
        for (CardinalDirection currentName : CardinalDirection.values()) {
            if (currentName.cardinalFullName.equals(cardinalName)) {
                return currentName;
            }
        }
        // otherwise get for short name ...
        for (CardinalDirection currentName : CardinalDirection.values()) {
            if (currentName.cardinalShortName.equals(cardinalName)) {
                return currentName;
            }
        }
        return null;
    }
}