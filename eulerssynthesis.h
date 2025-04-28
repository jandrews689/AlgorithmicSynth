#pragma once
#ifndef EULERS_H
#define EULERS_H

#include <stdint.h>
#include "math.h"
#include "daisysp.h"
#include "daisy_seed.h"


template <int num_harmonics = 16>
class clasEulersOscillator
{
  private:

    enum LEAD_WAVE {
      LEAD_SAW,
      LEAD_SQUARE,
      LEAD_IMPUL,
      LEAD_BIPO
    } LEAD_WAVEFORM;

    #define PI_F 3.1415927410125732421875f
    float flSamplingRate_;
    float flSamplingRateRecip_;
    float flFreq_;
    float flPhase_;
    float flPhaseInc_;
    float flAmplitude_;
    float flReal_;
    float flImag_;
    float flRealAmp_;
    float flImagAmp_;
    // float flHarmonicAmp_;
    uint8_t uiHarmonicWave_;
    uint8_t uiOctave_; 


    //Calculates the phase increment based from Sampling Frequency. 
    float fxCalculatePhaseInc(float frequency);

    public:
        // clasEulersOscillator(DaisySeed& hardware) : hw(hardware) {}

        void Init(float SampleRate);

        //Process the oscillator signal and send to output. 
        float Process();


        // float SetRealImagAmp(float adcInput, float flRealPart, float flImagPart);


        // Adds a value between 0-1 to the current phase. Used for FM. 
        void PhaseAdd(float phase);


        //Change freq
        void SetFreq(float frequency);


        //Get Current Frequency Setting
        float GetFreq();


        //Change amp
        void SetAmp(const float amplitude);


        //Set imag amplutude
        // void SetImagAmp(float amplitude);
      

        //Set Harmonic Waveform
        void SetHarmonicWaveForm(uint8_t value);


        //Set Harmonic Waveform
        uint8_t GetHarmonicWaveForm();


        //Get the harmonic waveform.  NOT TESTED. 
        float GetHarmonicAmp(float loop_inc);


        //Set real amplitude
        // void SetRealAmp(float amplitude);


        //Set Octave of the oscillator
        void SetOctave(uint8_t uiOctave);


        //Get Octave of the oscillator
        uint8_t GetOctave();


        //Get real
        float GetRealAmp();


        //Get imag
        float GetImagAmp();


        //Set real
        void SetRealAmp(float adcInput);

        
        //Set imag
        void SetImagAmp(float adcInput);

};
#endif