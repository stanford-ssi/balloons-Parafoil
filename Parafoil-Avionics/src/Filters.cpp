#include "Filters.h"

 void Filters::filter(){

  float filteredvalue = this->lowpassfilter.update(0.5);
  float hello = filteredvalue+2;
  hello = hello + filteredvalue;
}

bool Filters::initialize(){


  float F0 = 20;
  float Q = 0.5;
  float Fs = 15;

  lowpassfilter.setQ(Q);
  lowpassfilter.setCorner(F0);
  lowpassfilter.setSampleRate(Fs);

  int FILTER_FREQ = 100;

//  timer.begin(filter, 1000000/FILTER_FREQ);
  return true;

}
