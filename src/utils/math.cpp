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

#include "utils/math.h"

#include <algorithm>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <boost/bind.hpp> 
#include <boost/ref.hpp>


namespace st_is
{
double machine_precision = 1.e-14;

bool approxIsZero( double _to_check )
{
  return _to_check>-machine_precision && _to_check<machine_precision;
}

bool approxLessZero( double _to_check )
{
  return _to_check<-machine_precision;
}

bool approxGreaterZero( double _to_check )
{
  return _to_check>machine_precision;
}
  

bool roots( double _aCoeff, double _bCoeff, double _cCoeff, std::pair<double,double>& _roots )
{
  double discriminant = sqrt( _bCoeff*_bCoeff - 4*_aCoeff*_cCoeff );
  
  if( discriminant<0 ) return false;
  
  _roots.first = (-_bCoeff + discriminant ) / (2*_aCoeff);
  _roots.second = (-_bCoeff - discriminant ) / (2*_aCoeff);
  return true;
}

StdError::StdError()
{
  
}

StdError::StdError( std::vector<double>& _data )
{
  using namespace boost::accumulators;
  
  accumulator_set<double, stats<tag::mean,tag::variance> > accumulator;
  std::for_each( _data.begin(), _data.end(), boost::bind<void>(boost::ref(accumulator), _1 ));
  
  mean = extract::mean(accumulator);
  variance = extract::variance(accumulator);
}

StdError StdError::getStdError( std::vector<double>& _data )
{
  return StdError(_data);
}

double StdError::calcMean( std::vector<double>& _data )
{
  return StdError(_data).mean;
}

double StdError::calcVariance( std::vector<double>& _data )
{
  return StdError(_data).variance;
}

}