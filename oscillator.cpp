#include "oscillator.h"

void clasOscillator::Init(float SampleRate, OSC_TYPE type){
    OSCILLATOR_TYPE_ = type;

    switch(OSCILLATOR_TYPE_){
        case OSC_NORMAL:
            flFreq_ = 100.0f;
            flAmplitude_ = 1.0f;
        break;
        case OSC_LFO_SLOW:
            flFreq_ = 2.5f;
            flAmplitude_ = 1.0f;
        break;
        case OSC_LFO_FAST:
            flFreq_ = 10.0f;
            flAmplitude_ = 1.0f;
        break;
    }

    flSamplingRate_ = SampleRate;
    flSamplingRateRecip_ = 1.0f / SampleRate;
    uiRatio_ = 1;
    flPhase_ = 0.0f;
    flPhaseInc_ = fxCalculatePhaseInc(flFreq_ * uiRatio_);
    flPulseWidth_ = 0.5f;
    uiWaveform_ = WAVE_SIN;
    
    flDepth_ = 1.0f;
    flMaxDepth_ = 0.5f;
}


float clasOscillator::Process(){
    float flOutput = 0.0f;
    switch(uiWaveform_){
        case WAVE_SIN:
            flOutput = sinf(flPhase_ * (2.0f * PI_F));
            break;
        case WAVE_TRI:
            flOutput = flPhase_ < flPulseWidth_ ? (4 * flPhase_ - 1) : (-4 * flPhase_ + 3);
            break;
        case WAVE_SAW:
            flOutput = (flPhase_ * 2) - 1;
            break;
        case WAVE_SQUARE:
            flOutput = flPhase_ < flPulseWidth_ ? 1.0f : -1.0f;
            break;
        default: flOutput = 0.0f; break;
    }

    //Increment phase
    flPhase_ += flPhaseInc_;
    if (flPhase_ > 1.0f){
        flPhase_ -= 1.0f;
    }

    return flOutput * flAmplitude_;
}


float clasOscillator::fxCalculatePhaseInc(float freq){
    return freq * flSamplingRateRecip_;
}

//Change freq
void clasOscillator::SetFreq(const float frequency){
    flFreq_ = frequency;
    switch(OSCILLATOR_TYPE_){
        case OSC_NORMAL:
            if (flFreq_ < 20) {flFreq_ = 20;}
            if (flFreq_ > 1000){ flFreq_ = 1000;}
        break;
        case OSC_LFO_SLOW:
            if (flFreq_ < 0.1f) {flFreq_ = 0.1f;}
            if (flFreq_ > 5.0f){ flFreq_ = 5.0f;}
        break;
        case OSC_LFO_FAST:
            if (flFreq_ < 5.0f) {flFreq_ = 5.0f;}
            if (flFreq_ > 20.0f){ flFreq_ = 20.0f;}
        break;
    }
    flPhaseInc_ = fxCalculatePhaseInc(flFreq_ * uiRatio_);
}


//Change amp
void clasOscillator::SetAmp(const float amplitude){
    flAmplitude_ = amplitude;
    if (flAmplitude_ <= 0.0f) {flAmplitude_ = 0.0f;}
    if (flAmplitude_ >= 1.0f) {flAmplitude_ = 1.0f;}
}

//Change Waveform;
void clasOscillator::SetWaveform(const uint8_t waveform){
    uiWaveform_ = waveform;
    if (uiWaveform_ <= 0) {uiWaveform_ = 0;}
    if (uiWaveform_ >= 3) {uiWaveform_ = 3;}
}

// Adds a value between 0-1 to the current phase. Used for FM. 
void clasOscillator::PhaseAdd(float phase) { 
    flPhase_ += phase; 
}

//Get Current Frequency 
float clasOscillator::GetFreq(){
    return flFreq_;
}


//Get Current Amplitude
float clasOscillator::GetAmp(){
    return flAmplitude_;
}


//Get Current Amplitude
float clasOscillator::GetDepth(){
    return flDepth_;
}

//Get Current Amplitude
void clasOscillator::SetDepth(float input){
    if (input <= 0.0f) {flDepth_ = 0.0f;}
    if (input >= flMaxDepth_) {flDepth_ = flMaxDepth_;}

    flAmplitude_ = flDepth_;
}


//Get current waveform
uint8_t clasOscillator::GetWaveform(){
    return uiWaveform_;
}


//Get current ratio
uint8_t clasOscillator::GetRatio(){
    return uiRatio_;
}

//Sets the oscillators ratio
void clasOscillator::SetRatio(uint8_t uiRatio){
    uiRatio_ = uiRatio;
    if (uiRatio_ <= 1) {uiRatio_ = 1;}
    if (uiRatio_ >= 9) {uiRatio_ = 9;}
    flPhaseInc_ = fxCalculatePhaseInc(flFreq_ * uiRatio_);
}


// float clasOscillator::LFOProcess(float flSignalInput){
//     return flSignalInput * Process();
// }


//Tromolo effect
float clasOscillator::Tromolo(float Signal){
    flMaxDepth_ = 0.5f;
    float flTromolo = Process() + 0.5;
    return Signal * flTromolo;
}

// //Vibrato effect 
// float clasOscillator::Vibrato(float Signal){
//     flMaxDepth_ = 12.0f;
//     float flVibrato = Process();
//     flVibrato /= 12;
//     float flOutput = pow(2, (flVibrato));
//     return (Signal * flOutput);
// }


//Vibrato effect 
float clasOscillator::Vibrato(float Signal){
    flMaxDepth_ = 12.0f;
    float flOutput = Process();
    // float flFrequency = Signal;
    flOutput = flOutput / 12.0;
    flOutput = pow(2, (flOutput));
    return (Signal * flOutput);
}


//Turns on tromolo mode
void clasOscillator::SetTromolo(){
    xVibratoMode_ = false;
    xTromoloMode_ = true;
}

//Turns on vibrato mode
void clasOscillator::SetVibrato(){
    xTromoloMode_ = false;
    xVibratoMode_ = true;
}

//Turns on tromolo mode
bool clasOscillator::GetTromoloMode(){
    return xTromoloMode_;
}

//Turns on vibrato mode
bool clasOscillator::GetVibratoMode(){
    return xVibratoMode_;
}


float clasOscillator::LFOProcess(float flSignal){
    float flOutput = 0.0f;
    if (xTromoloMode_){
        flOutput = Tromolo(flSignal);
    } else if (xVibratoMode_){
        flOutput = Vibrato(flSignal);
    }
    return flOutput;
}

