/* Copyright 2013-2014 CS Systèmes d'Information
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
package org.orekit.rugged.api;

/** Interface representing line datation model.
 * @author Luc Maisonobe
 */
public interface LineDatation {

    /** Get the date for a given line.
     * @param param lineNumber line number
     * @return date, as an offset in seconds from reference date
     */
    double getDate(double lineNumber);

    /** Get the line for a given date.
     * @param date, as an offset in seconds from reference date
     * @return line number
     */
    double getLine(double date);

}
