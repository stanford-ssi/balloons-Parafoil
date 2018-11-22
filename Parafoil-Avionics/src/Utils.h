#ifndef UTILS_H
#define UTILS_H

#include <math.h>
//#include "spa.h"

#define pi 3.14159
#define SECONDS_PER_DAY (86400.)
#define DAYS_PER_SECOND (0.0000115741)
#define MAX_SUN_SPEED (360.0F/(60.0F*60.0F*24.0F))


/*
 * class: Biquad
 * -------------------
 * Implementation of a biquad IIR filter.
 * Pray for stability.
 */
class Biquad {
public:
  typedef struct {
    double a[3];
    double b[3];
  } Coeffs;
  Biquad(): x{0.0,0.0,0.0},y{0.0,0.0,0.0} ,coeffs{{0.0,0.0,0.0},{0.0,0.0,0.0}}{}
  Biquad(Coeffs coeffs): x{0.0,0.0,0.0},y{0.0,0.0,0.0},coeffs(coeffs){}
  float update(float input);
  void setSS(float val);
  void setCoeffs(Coeffs coeffs);
  double getSSGain();
  void shiftBias(float offset);
private:
  double x[3];
  double y[3];
  Coeffs coeffs;
};

class DBiquad {
public:
  typedef struct {
    double a[3];
    double b[4];
  } Coeffs;
  DBiquad(): x{0},y{0} ,coeffs{{0},{0}} {}
  DBiquad(Coeffs coeffs): x{0},y{0},coeffs(coeffs){}
  float update(float input);
  void setSS(float val);
  void setCoeffs(Coeffs coeffs);
private:
  double x[4];
  double y[3];
  Coeffs coeffs;
};


/*
 * class: AdjustableLowpass
 * -------------------
 * Adjustable 2nd order IIR lowpass filter. Wrapper for biquad
 *
 * F0: corner frequency of the filter
 * Q: Quality factor of the filter
 * Fs: Sample rate of the filter
 *
 * If you don't know how what you are doing, but just want a lowpass filter, then
 * do the following:
 * Call the constructor and give it the three settings. For general use, you don't need to
 * know what Q means, and just set it to 0.5. F0 is the corner frequency, which can be thought
 * of as the freuqency at which waves of higher frequency will be rejected by the filter, and
 * waves of lower freuqecny will be passed through by the filter. Sometimes it's easy to pick
 * this value exactly, but other times it's not and it's best to just run the filter on previous
 * data and tune the corner frequency until it "looks right". A lower corner frequency means more
 * smoothing, but also more delay between the outputs and the inputs. The corner frequency must be
 * less than half of the sample rate, and in most cases it will be much, much less.
 * The sample rate is the freuency at which you call the update function.
 * To get the filtered value from the filter, simply call update(input), where input is the raw
 * value that you are looking to filter. It will return the filtered value for that timestep.
 * You need to ensure that you call update() at a fixed frequency, best done with an interrupt timer.
 *
 * Lastly, these things are not foolproof so some spooky stuff can happen if you do things wrong.
 * Probably a good idea to check the final implementation and use with @johndean
 */
class AdjustableLowpass{
public:
  AdjustableLowpass(float F0, float Q, float Fs);
  AdjustableLowpass();
  void setSS(float v);
  void setQ(float Q);
  void setCorner(float F0);
  void setSampleRate(float Fs);
  float update(float input);
private:
  Biquad::Coeffs calcCoeffs();
  Biquad biquad;
  float Q;
  float F0;
  float Fs;
};



#endif
