#pragma once
#ifndef OSC_H
#define OSC_H

#include <stdint.h>
#include "math.h"

enum OSC_TYPE{
  OSC_NORMAL,
  OSC_LFO_SLOW,
  OSC_LFO_FAST
};

class clasOscillator
{
  private:
    #define PI_F 3.1415927410125732421875f
    float flSamplingRate_;
    float flSamplingRateRecip_;
    float flFreq_;
    float flPhase_;
    float flPhaseInc_;
    float flAmplitude_;
    float flPulseWidth_;
    uint8_t uiWaveform_;
    uint8_t uiRatio_;
    OSC_TYPE OSCILLATOR_TYPE_;
    float flDepth_;
    float flMaxDepth_;
    bool xTromoloMode_;
    bool xVibratoMode_;
    


    //Calculates the phase increment based from Sampling Frequency. 
    float fxCalculatePhaseInc(float frequency);

  public:



    enum DRONE_WAVE{
        WAVE_SIN,
        WAVE_TRI,
        WAVE_SAW,
        WAVE_SQUARE,
        // WAVE_EXP_D,
        // WAVE_GAUSSIAN,
        // WAVE_SUPERSAW,
        // WAVE_PARABOLA,
        // WAVE_TANH,
        // WAVE_PWM
    } DRONE_WAVEFORM;


    clasOscillator(){
    }
    ~clasOscillator(){}

    void Init(float SampleRate, OSC_TYPE type);

    //Process the oscillator signal and send to output. 
    float Process();

    //Change freq
    void SetFreq(float frequency);

    //Change amp
    void SetAmp(float amplitude);

    //Change DRONE_Waveform;
    void SetWaveform(uint8_t uiWaveform);

    // Adds a value between 0-1 to the current phase. Used for FM. 
    void PhaseAdd(float phase);

    //Get Current Frequency Setting
    float GetFreq();

    //Get current Amplitude setting
    float GetAmp();

    //Get current DRONE_waveform
    uint8_t GetWaveform();

    uint8_t GetRatio();

    void SetRatio(uint8_t uiRatio);

    float GetDepth();

    void SetDepth(float input);

    float Tromolo(float Signal);

    float Vibrato(float Signal);

    void SetTromolo();

    void SetVibrato();

    //Turns on tromolo mode
    bool GetTromoloMode();

    //Turns on vibrato mode
    bool GetVibratoMode();


    float LFOProcess(float flSignal);


};
#endif