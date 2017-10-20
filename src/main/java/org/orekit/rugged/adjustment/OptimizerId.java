/* Copyright 2013-2017 CS Systèmes d'Information
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
package org.orekit.rugged.adjustment;


/** Enumerate for Optimizer used in Least square optimization.
 * @author Jonathan Guinet
 * @since 2.0
 */
public enum OptimizerId {

    /** Levenberg Marquadt. */
    LEVENBERG_MARQUADT,
    
    /** Gauss Newton with LU decomposition. */
    GAUSS_NEWTON_LU,
    
    /** Gauss Newton with QR decomposition. */
    GAUSS_NEWTON_QR
    
}
