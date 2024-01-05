#include "MultiFilter.h"

MultiFilter::MultiFilter(float frequency, float q_factor)
    : yl_prev(0.0f)
    , yh_prev(0.0f)
    , yb_prev(0.0f)
    , yn_prev(0.0f)

{
    setQ(q_factor);
    setFrequency(frequency);
    timestamp_prev = _micros();
}


float MultiFilter::operator() (float x, float dt)
{
    // Implements a state variable filter
    unsigned long timestamp = _micros();
    if(dt == NOT_SET){
        dt = (timestamp - timestamp_prev)*1e-6f;
    }

    if (dt < 0.0f ) dt = 1e-3f;
    else if(dt > 0.3f) {
        yl_prev = x;
        yh_prev = x;
        yb_prev = x;
        yn_prev = x;
        timestamp_prev = timestamp;
        return x;
    }

    alpha1 = _constrain(2 * _sin(dt * timeConstFactor), 0.0f, 1.0f);

    float yh = x - yl_prev - alpha2 * yb_prev;
    float yb = alpha1 * yh + yb_prev;
    float yl = alpha1 * yb + yl_prev;
    yl_prev = yl;
    yh_prev = yh;
    yb_prev = yb;
    yn_prev = yl_prev + yh_prev + x * notchDepth;
    timestamp_prev = timestamp;
    
    switch (defaultFilter)
    {
    case MULTI_FILTER_LOWPASS:
        return yl_prev;
        break;
    case MULTI_FILTER_HIGHPASS:
        return yh_prev;
        break;
    case MULTI_FILTER_BANDPASS:
        return yb_prev * alpha2; // scale to [0:1] by multiplying with alpha2 (==1/q)
        break;
    case MULTI_FILTER_NOTCH:
        return yn_prev * notchScalingfactor;
        break;
    default:
        return yl_prev;
        break;
    }
}

float MultiFilter::getLp() {return yl_prev;}
float MultiFilter::getHp() {return yh_prev;}
float MultiFilter::getBp() {return yb_prev * alpha2;}
float MultiFilter::getNotch() {return yn_prev * notchScalingfactor;}

float MultiFilter::getLp(float x, float dt) 
{
    (*this)(x, dt);  // Call operator() on current instance of MultiFilter
    return yl_prev;
}
float MultiFilter::getHp(float x, float dt) 
{
    (*this)(x, dt);  // Call operator() on current instance of MultiFilter
    return yh_prev;
}
float MultiFilter::getBp(float x, float dt) 
{
    (*this)(x);  // Call operator() on current instance of MultiFilter
    return yb_prev * alpha2;
}
float MultiFilter::getNotch(float x, float dt) 
{
    (*this)(x, dt);  // Call operator() on current instance of MultiFilter
    return yn_prev * notchScalingfactor;
}

void MultiFilter::setQ(float newQ)
{
    if (newQ != 0.0f && newQ != -0.0f)
    {
        alpha2 = 1.0f / newQ;
        q = newQ;
    }else
    {
        alpha2 = 1.0f / 0.707f;
        q = 0.707f;
    }
}

void MultiFilter::setNotchDepth(float newNotchDepth)
{
    if (newNotchDepth >= 0.0f)
    {
        notchDepth = newNotchDepth;
    }else
    {
        notchDepth = 0.0f;
    }
    notchScalingfactor = 1.0f / (1.0f + notchDepth);
}

void MultiFilter::setFrequency(float newFrequency)
{
    // float newTf = 0.003f;
    if (newFrequency > 0.0f)
    {
        // newTf = 1.0f / (_2PI * newFrequency);
    }else
    {
        newFrequency = 100.0f;
        // newTf = 1e-3f;
    }
    (*this).timeConstFactor = (_PI * newFrequency);
    (*this).freq = newFrequency;
    // setTf(newTf);
}

void MultiFilter::setReturnType(returnType type)
{
    defaultFilter = type;
}