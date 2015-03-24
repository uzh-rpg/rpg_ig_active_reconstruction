/* Copyright (c) 2015, Stefan Isler, islerstefan@bluewin.ch
*

math is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
math is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU Lesser General Public License for more details.
You should have received a copy of the GNU Lesser General Public License
along with math. If not, see <http://www.gnu.org/licenses/>.
*/ 

/// set of convenience functions and classes related to math

#pragma once
#include <utility>
#include <cmath>
#include <vector>

namespace st_is
{
  
extern double machine_precision;
/// checks whether _to_check is within machine_precision close to zero
bool approxIsZero( double _to_check );
/// checks whether _to_check is more than _to_check smaller than zero
bool approxLessZero( double _to_check );
/// checks whether _to_check is more than _to_check greater than zero
bool approxGreaterZero( double _to_check );
  
/** simple solver for the quadratic equation a*x² + bx + c = 0
*  Returns false if the roots are imaginary, otherwhise the two roots are stored in _roots - twice
*  the same value if only one root exists.
*/
bool roots( double _aCoeff, double _bCoeff, double _cCoeff, std::pair<double,double>& _roots );

/// class describing standard errors
struct StdError
{
  StdError();
  
  /// initializes the object directly from a double array
  StdError( std::vector<double>& _data );
  
  /// calculates the std error and returns a StdError object
  static StdError getStdError( std::vector<double>& _data );
  
  /// calculates and returns the mean of the data
  static double calcMean( std::vector<double>& _data );
  
  /// calculates and returns the variance of the data
  static double calcVariance( std::vector<double>& _data );
  
  double mean;
  double variance;
};

}