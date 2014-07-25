/*
  planner.c - buffers movement commands and manages the acceleration profile plan
 Part of Grbl
 
 Copyright (c) 2009-2011 Simen Svale Skogsrud
 
 Grbl is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 Grbl is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
 */

/* The ring buffer implementation gleaned from the wiring_serial library by David A. Mellis. */

/*  
 Reasoning behind the mathematics in this module (in the key of 'Mathematica'):
 
 s == speed, a == acceleration, t == time, d == distance
 
 Basic definitions:
 
 Speed[s_, a_, t_] := s + (a*t) 
 Travel[s_, a_, t_] := Integrate[Speed[s, a, t], t]
 
 Distance to reach a specific speed with a constant acceleration:
 
 Solve[{Speed[s, a, t] == m, Travel[s, a, t] == d}, d, t]
 d -> (m^2 - s^2)/(2 a) --> estimate_acceleration_distance()
 
 Speed after a given distance of travel with constant acceleration:
 
 Solve[{Speed[s, a, t] == m, Travel[s, a, t] == d}, m, t]
 m -> Sqrt[2 a d + s^2]    
 
 DestinationSpeed[s_, a_, d_] := Sqrt[2 a d + s^2]
 
 When to start braking (di) to reach a specified destionation speed (s2) after accelerating
 from initial speed s1 without ever stopping at a plateau:
 
 Solve[{DestinationSpeed[s1, a, di] == DestinationSpeed[s2, a, d - di]}, di]
 di -> (2 a d - s1^2 + s2^2)/(4 a) --> intersection_distance()
 
 IntersectionDistance[s1_, s2_, a_, d_] := (2 a d - s1^2 + s2^2)/(4 a)
 */

#include "Marlin.h"
#include "planner.h"
#include "stepper.h"
#include "temperature.h"
#include "ultralcd.h"
#include "language.h"

//===========================================================================
//=============================public variables ============================
//===========================================================================
