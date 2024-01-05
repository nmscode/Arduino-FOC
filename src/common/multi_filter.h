#ifndef MULTI_FILTER_H
#define MULTI_FILTER_H

#include "common/time_utils.h"
#include "common/foc_utils.h"

/**
 *  Multi filter class
 *  Implements a state variable filter to compute low-, high-, bandpass and notch filters 
 *  with variable Q (resonance) in one operation, as well as seamless switching between them
 */
class MultiFilter
{
public:
    /**
     * @param f - Filter frequency, i.e. the center of bandpass and notch filters, and the "-3dB" of high- and lowpass
     * @param q - Filter resonance
     */
    MultiFilter(float f=100.0f, float q=0.707f);

    enum returnType {
        MULTI_FILTER_LOWPASS,
        MULTI_FILTER_HIGHPASS,
        MULTI_FILTER_BANDPASS,
        MULTI_FILTER_NOTCH
    };

    float operator() (float x, float dt=NOT_SET);

    void setQ(float newQ);                      //!< Set filter resonance
    void setFrequency(float newFrequency);      //!< set filter frequency in Hz instead of time constant
    void setNotchDepth(float newNotchDepth);    //!< Set notch filter depth
    void setReturnType(returnType type);        //!< change between low-, high-, bandpass, or notch output as default return value

    float getFrequency() {return freq;}
    float getQ() {return q;}
    float getNotchdepth() {return notchDepth;}
    returnType getReturnType() {return defaultFilter;}

    //!< Get different filter outputs
    float getLp();
    float getHp();
    float getBp();
    float getNotch();

    //!< Get different filter outputs while updating the filter state
    float getLp(float x, float dt=NOT_SET);
    float getHp(float x, float dt=NOT_SET);
    float getBp(float x, float dt=NOT_SET);
    float getNotch(float x, float dt=NOT_SET);

protected:
    unsigned long timestamp_prev;  //!< Last execution timestamp

    float yl_prev; //!< filtered  lowpass value in previous execution step 
    float yh_prev; //!< filtered highpass value in previous execution step 
    float yb_prev; //!< filtered bandpass value in previous execution step 
    float yn_prev; //!< filtered notch value in previous execution step 

    float freq;                         //!< Multi filter frequency
    float timeConstFactor;              //!< precompute divided constant to save time in the loop
    float q = 0.707f;                   //!< filter resonance (quality factor)
    float notchDepth = 0.0f;            //!< notch filter cut depth. 0.0 -> cut everything at peak; 1.0 -> cut to about half amplitude; larger values tend towards no cutting/filtering
    float notchScalingfactor = 1.0f;    //!< precomputed on changing notchDepth to correct output scale

    float alpha1;
    float alpha2 = 1.0f/q;

    returnType defaultFilter = MULTI_FILTER_LOWPASS;
};

#endif // MULTI_FILTER_H