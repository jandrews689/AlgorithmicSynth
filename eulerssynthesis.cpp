#include "eulerssynthesis.h"


//Calculates the phase increment based from Sampling Frequency. 
template <int num_harmonics>
float clasEulersOscillator<num_harmonics>::fxCalculatePhaseInc(float frequency){
    return frequency * flSamplingRateRecip_;
};


// //Set Harmonic value
// template <int num_harmonics>
// float clasEulersOscillator<num_harmonics>::SetHarmonic(float value){
//     return value;
// };


template <int num_harmonics>
void clasEulersOscillator<num_harmonics>::Init(float SampleRate){
    flSamplingRate_ = SampleRate;
    flSamplingRateRecip_ = 1.0f / SampleRate;
    flFreq_ = 150.0f;
    flPhase_ = 0.0f;
    flPhaseInc_ = fxCalculatePhaseInc(flFreq_);
    flAmplitude_ = 0.0f;
    flReal_ = 0.0f;
    flImag_ = 0.0f;
    flRealAmp_ = 0.5f;
    flImagAmp_ = 0.5f;
    // flHarmonicAmp_ = 0.0f;
    uiHarmonicWave_ = LEAD_SQUARE;
    uiOctave_ = 0;
};


//Process the oscillator signal and send to output. 
template <int num_harmonics>
float clasEulersOscillator<num_harmonics>::Process(){
    int num = num_harmonics;

    flReal_ = 0.0f;
    flImag_ = 0.0f;

    //Computate the fundamental frequency of the real and imag parts using eulers. 
    float flExpReal = cosf(flPhase_);
    float flExpImag = sinf(flPhase_);

    //Store the exponent numbers into Pow for recursive multiplication
    float flPowReal = flExpReal;
    float flPowImag = flExpImag;

    for(float i = 2.0f; i<=num; i++){
        //Recursive computation of the harmonic components. 
        float tempReal = flPowReal;
        flPowReal = (flPowReal * flExpReal) - (flPowImag * flExpImag);
        flPowImag = (tempReal * flExpImag) + (flPowImag * flExpReal);

        //Harmonic Amplitude waveform 
        float flHarmonicAmp = GetHarmonicAmp(i);

        flImag_ += flHarmonicAmp * flPowImag * flImagAmp_;
        flReal_ += flHarmonicAmp * flPowReal * flRealAmp_;
    }


    //Add the fundamental frequency. and multiply by 50% to prevent going over 1.
    flReal_ += flExpReal * flRealAmp_;
    flImag_ += flExpImag * flImagAmp_;


    //Find the aboslute value of the signal. 
    float sum_val = fabsf(flReal_ + flImag_);


    //Normalise to between 0 - 1;
    if(sum_val > 1.0f)
    {
        float norm = 1.0f / sum_val;
        flReal_ *= norm;
        flImag_ *= norm;
    }


    //Combine the outputs.
    float flOutput = (flReal_ + flImag_);

    if (flOutput > 1.0f){
        flOutput = 1.0f;
    }


    //Increment phase
    flPhase_ += flPhaseInc_;
    if (flPhase_ > 1.0f){
        flPhase_ -= 1.0f;
    }

    return flOutput * flAmplitude_;
};


//Change freq
template <int num_harmonics>
void clasEulersOscillator<num_harmonics>::SetFreq(float frequency){
    flFreq_ = frequency;
    if (flFreq_ < 11) {flFreq_ = 11;}
    if (flFreq_ > 1000){ flFreq_ = 1000;}
    flPhaseInc_ = fxCalculatePhaseInc(flFreq_);
};


//Change amp
template <int num_harmonics>
void clasEulersOscillator<num_harmonics>::SetAmp(const float amplitude){
    flAmplitude_ = amplitude;
};


// //Change Waveform;
// template <int num_harmonics>
// void clasEulersOscillator<num_harmonics>::SetWaveform(const uint8_t waveform){
//     uiWaveform_ = waveform;
// };


// Adds a value between 0-1 to the current phase. Used for FM. 
template <int num_harmonics>
void clasEulersOscillator<num_harmonics>::PhaseAdd(float phase) { 
    flPhase_ += phase; 
};


//Get Current Frequency Setting
template <int num_harmonics>
float clasEulersOscillator<num_harmonics>::GetFreq(){
    return flFreq_;
};


//Set Harmonic Waveform
template <int num_harmonics>
void clasEulersOscillator<num_harmonics>::SetHarmonicWaveForm(uint8_t value){
    uiHarmonicWave_ = value;
    if (uiHarmonicWave_ <= 0) {uiHarmonicWave_ = 0;}
    if (uiHarmonicWave_ >= 3) {uiHarmonicWave_ = 3;}
};


//Set Harmonic Waveform
template <int num_harmonics>
uint8_t clasEulersOscillator<num_harmonics>::GetHarmonicWaveForm(){
    return uiHarmonicWave_;
};


//Get the harmonic waveform.  NOT TESTED. 
template <int num_harmonics>
float clasEulersOscillator<num_harmonics>::GetHarmonicAmp(float loop_inc){
    float flHarmonicAmp = 0.0f;
    int j = loop_inc;
    if (loop_inc == 0){loop_inc = 1;}
    switch(uiHarmonicWave_){
        case 0: //Saw tooth
            flHarmonicAmp = 1 / loop_inc;
            break;
        case 1: //Sqaurewave
            // flHarmonicAmp = (j % 2 == 0) ? 0.0f : 1.0f / loop_inc;
            flHarmonicAmp = sinf(loop_inc);
            break;
        case 2: //Impulse train
            flHarmonicAmp = 1.0f;
            break;
        case 3: //Bipolar impulse train
            flHarmonicAmp = (j % 2 == 0) ? -1.0f : 1.0f;
            break;
        default: flHarmonicAmp = 1 / loop_inc; 
            break;
    }
    return flHarmonicAmp;
};


//Set the octave of the oscillator
template <int num_harmonics>
void clasEulersOscillator<num_harmonics>::SetOctave(uint8_t uiOctave){
    if (uiOctave < 0){uiOctave = 0;}
    else if (uiOctave > 2){uiOctave = 2;}
    uiOctave_ = uiOctave;
};


//Get the octave of the oscillator
template <int num_harmonics>
uint8_t clasEulersOscillator<num_harmonics>::GetOctave(){
    return uiOctave_;
};


template <int num_harmonics>
float clasEulersOscillator<num_harmonics>::GetRealAmp(){
    return flRealAmp_;
}


template <int num_harmonics>
float clasEulersOscillator<num_harmonics>::GetImagAmp(){
    return flImagAmp_;

}


template <int num_harmonics>
void clasEulersOscillator<num_harmonics>::SetRealAmp(float adcInput){
    flRealAmp_ = (adcInput <= 0.f) ? 0.0f : (adcInput > 1.f) ? 1.0f : adcInput;

}


template <int num_harmonics>
void clasEulersOscillator<num_harmonics>::SetImagAmp(float adcInput){
    flImagAmp_ = (adcInput <= 0.f) ? 0.0f : (adcInput > 1.f) ? 1.0f : adcInput;

}


template class clasEulersOscillator<16>;