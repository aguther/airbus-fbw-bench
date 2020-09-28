/**
 * Copyright 2019 Bradley J. Snyder <snyder.bradleyj@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef _PID_SOURCE_
#define _PID_SOURCE_

#include <iostream>
#include <cmath>
#include "pid.h"

using namespace std;

class PIDImpl {
 public:
  PIDImpl(
      double dt,
      double max,
      double min,
      double Kp,
      double Kd,
      double Ki,
      double errorWeightFactor
  );

  ~PIDImpl() = default;

  double calculate(
      double setpoint,
      double pv
  );

  double getDt() const;
  void setDt(double dt);

  double getMax() const;
  void setMax(double max);

  double getMin() const;
  void setMin(double min);

  double getKp() const;
  void setKp(double Kp);

  double getKd() const;
  void setKd(double Kd);

  double getKi() const;
  void setKi(double Ki);

  double getErrorWeightFactor() const;
  void setErrorWeightFactor(double errorWeightFactor);

 private:
  double mDt;
  double mMax;
  double mMin;
  double mKp;
  double mKd;
  double mKi;
  double mErrorWeightFactor;
  double mPreError;
  double mIntegral;
};

PID::PID(
    double dt,
    double max,
    double min,
    double Kp,
    double Kd,
    double Ki,
    double errorWeightFactor
) {
  pimpl = new PIDImpl(
      dt,
      max,
      min,
      Kp,
      Kd,
      Ki,
      errorWeightFactor
  );
}

double PID::calculate(
    double setpoint,
    double pv
) {
  return pimpl->calculate(setpoint, pv);
}

PID::~PID() {
  delete pimpl;
}

double PID::getDt() const {
  return pimpl->getDt();
}

void PID::setDt(
    double dt
) {
  pimpl->setDt(dt);
}

double PID::getMax() const {
  return pimpl->getMax();
}

void PID::setMax(
    double max
) {

}
double PID::getMin() const {
  return pimpl->getMin();
}

void PID::setMin(
    double min
) {
  pimpl->setMin(min);
}

double PID::getKp() const {
  return pimpl->getKp();
}

void PID::setKp(
    double Kp
) {
  pimpl->setKp(Kp);
}

double PID::getKd() const {
  return pimpl->getKd();
}

void PID::setKd(
    double Kd
) {
  pimpl->setKd(Kd);
}

double PID::getKi() const {
  return pimpl->getKi();
}

void PID::setKi(
    double Ki
) {
  pimpl->setKi(Ki);
}

double PID::getErrorWeightFactor() const {
  return pimpl->getErrorWeightFactor();
}

void PID::setErrorWeightFactor(
    double errorWeightFactor
) {
  pimpl->setErrorWeightFactor(errorWeightFactor);
}

/**
 * Implementation
 */
PIDImpl::PIDImpl(
    double dt,
    double max,
    double min,
    double Kp,
    double Kd,
    double Ki,
    double errorWeightFactor
) :
    mDt(dt),
    mMax(max),
    mMin(min),
    mKp(Kp),
    mKd(Kd),
    mKi(Ki),
    mErrorWeightFactor(errorWeightFactor),
    mPreError(0),
    mIntegral(0) {
}

double PIDImpl::calculate(
    double setpoint,
    double pv
) {
  // Calculate error
  double error = mErrorWeightFactor * (setpoint - pv);

  // Proportional term
  double Pout = mKp * error;

  // Integral term
  mIntegral += error * mDt;
  if (mIntegral > mMax) {
    mIntegral = mMax;
  } else if (mIntegral < mMin) {
    mIntegral = mMin;
  }
  double Iout = mKi * mIntegral * mErrorWeightFactor;

  // Derivative term
  double derivative = (error - mPreError) / mDt;
  double Dout = mKd * derivative;

  // Calculate total output
  double output = Pout + Iout + Dout;

  // Restrict to max/min
  if (output > mMax)
    output = mMax;
  else if (output < mMin)
    output = mMin;

  // Save error to previous error
  mPreError = error;

  return output;
}

double PIDImpl::getDt() const {
  return mDt;
}

void PIDImpl::setDt(
    double dt
) {
  mDt = dt;
}

double PIDImpl::getMax() const {
  return mMax;
}

void PIDImpl::setMax(
    double max
) {

}

double PIDImpl::getMin() const {
  return mMin;
}

void PIDImpl::setMin(
    double min
) {
  mMin = min;
}

double PIDImpl::getKp() const {
  return mKp;
}

void PIDImpl::setKp(
    double Kp
) {
  mKp = Kp;
}

double PIDImpl::getKd() const {
  return mKd;
}

void PIDImpl::setKd(
    double Kd
) {
  mKd = Kd;
}

double PIDImpl::getKi() const {
  return mKi;
}

void PIDImpl::setKi(
    double Ki
) {
  mKi = Ki;
}

double PIDImpl::getErrorWeightFactor() const {
  return mErrorWeightFactor;
}

void PIDImpl::setErrorWeightFactor(
    double errorWeightFactor
) {
  mErrorWeightFactor = errorWeightFactor;
  if (mErrorWeightFactor == 0) {
    mIntegral = 0;
  }
}

#endif
