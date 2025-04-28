#pragma once 
#ifndef CONTROL_H
#define CONTROL_H

#include "encodercontrol.h"
#include "oscillator.h"
#include "leadvoice.h"
#include "daisysp.h"
#include "daisy_seed.h"
#include "math.h"
#include "adsr.h"
#include "eulerssynthesis.h"
#include "twoorderfilter.h"

using namespace daisysp;
using namespace daisy;
using namespace seed;

//INSTANTIATION/////////////////////////////////////////
extern DaisySeed hw;
extern clasEulersOscillator<16> objLeadVoice;
extern clasSecondOrderFilter objLPFLeadVoice;

clasOscillator aobjSingleOscillator[4];


// clasOscillator objRingMod;
// clasOscillator objCarrierA;
// clasOscillator objCarrierB;
// clasOscillator objCarrierC;


clasOscillator* objParent_;
clasOscillator* objChild_;

clasOscillator* ptrSingleOscillator_ = nullptr;

extern clasOscillator objLFOSlow;
extern clasOscillator objLFOFast;

clasEncoder objEncoderA;
clasEncoder objEncoderB;
clasEncoder objEncoderC;
clasEncoder objEncoderD;


Switch Btn1;
Switch Btn2;

AdcChannelConfig PT1_Effect3;
AdcChannelConfig PT2_Effect1;
AdcChannelConfig PT3_Filter1;
AdcChannelConfig PT4_Filter2;
AdcChannelConfig PT5_OSC_A_Amp;
AdcChannelConfig PT6_OSC_B_Amp;
AdcChannelConfig PT7_OSC_C_Amp;
AdcChannelConfig PT8_OSC_D_Amp;
AdcChannelConfig PT9_RealImag;
AdcChannelConfig PT10_Effect4;

clasADSR objADSR;

TimerHandle objEncTimerMultipler;
TimerHandle objResetTimer;
TimerHandle objResetMenuTimer;



//LFO MADNESS//////////////////////////////////////////////////////////
enum class VoiceSelection{
    None, 
    Drone,
    Single,
    Lead
};


// Holds the info for one LFO assignment:
struct LFOAssignment
{
    clasOscillator* ptrLFO;         //pointer to the LFO object (objLFOSlow or objLFOFast)
    clasOscillator* ptrSingleOscSelectLFO_; //Single oscillator pointer. 
    VoiceSelection  whichVoice;     //drone, single, or lead
    int iOscNumber;
    bool            xTremolo;       //which mode
    bool            xVibrato;
    
};

static LFOAssignment g_astruLfoSlots[2];
int g_iLFOElement = 0;  //Which LFO element 
//LFO MADNESS//////////////////////////////////////////////////////////

//Enumerator for the touch keyboard menu. 
enum TOUCH_MODE {
    T_NORMAL = 0, //This needs to be 0!
    T_FAST_LFO = 1,
    T_ADSR = 2,
    T_SLOW_LFO = 3,
    T_D_AMP = 4,
    T_RATIO = 5,
    T_MATCH = 7,
    T_D_SHAPE = 9,
    T_L_SHAPE = 11,
};

enum MENU_STATE {
    M_MAIN = 0,
    M_SUB = 1,
    M_MODE = 2
};

enum LFO_MODE {
    LFO_IDLE = 0,
    LFO_TROMOLO = 1,
    LFO_VIBRATO = 3,
    LFO_DRONE = 6,
    LFO_SINGLE = 8,
    LFO_LEAD = 10
};

enum WAVE_TYPE{
    WAVE_SIN,
    WAVE_TRI,
    WAVE_SAW,
    WAVE_SQUARE,
    WAVE_EXP_D,
    WAVE_GAUSSIAN,
    WAVE_SUPERSAW,
    WAVE_PARABOLA,
    WAVE_TANH,
    WAVE_PWM
};


//VARIABLES//////////////////////////////////////////

const int MAX_FREQ = 6000;
const int MIN_FREQ = 11;
const int PERIOD = 20000; 

volatile int iEncoderPos_ = 0;
volatile uint32_t uiLastTime_[4] = {0,0,0,0};
volatile float flSpeedFactor_[4] = {1.0f, 1.0f, 1.0f, 1.0f}; 
const float SPEED_SCALE_ = 2.0f;
const float MAX_INC_ = 20;

enum WAVE_TYPE WAVEFORM; // Correct definition
uint32_t uiLastUpdate;
uint8_t uiTouchKeyPressed;
bool xGate; //Signal to start the ADSR. 
float aflFreqTable[12]; //Array for the frequencies to stored in.
int iUserInput_ = T_NORMAL; 
bool xNormalMode = true;
float flLFOFastSignal;
float flLFOSlowSignal;
bool xPrintOnce = false;
int eMenuState = M_MAIN;
int eWhileState = M_MAIN;
int iPrevUserInput = -1;
int eMain = T_NORMAL;
int eSub = LFO_IDLE;
int eMode = LFO_IDLE;

int eResetMainMenuState = T_NORMAL;


//FUNCTIONS//////////////////////////////////////////////


//Analogue Pots Calibration
//Change the sensistivity of the signal when using potentiometers. 
//Linear -> a=0, b=1, c=0
//More sense low end -> a>0, b=1, c=0
//More sense high end -> a<0, b=1, c=0
float fxPolynomialMap(float inputsignal, float a, float b, float c){
    float flCorrected = a * pow(inputsignal, 2) + b * inputsignal + c;
    flCorrected = fmax(0.0, fmin(flCorrected, 1.0));
    return flCorrected;
}


void InitLFOAssignments()
{
    g_astruLfoSlots[0] = {nullptr, nullptr, VoiceSelection::None, -1, false, false};
    g_astruLfoSlots[1] = {nullptr, nullptr, VoiceSelection::None, -1, false, false};
}


void fxEncoderSetup(){
    objEncoderA.Init(D0, D1, D25, 1000); //RingMod
    objEncoderB.Init(D2, D3, D26, 1000); //CarrierA
    objEncoderC.Init(D4, D5, D27, 1000); //CarrierB
    objEncoderD.Init(D6, D7, D28, 1000); //CarrierC
}



void fxEncoderTimerInit(){
    TimerHandle::Config FreqMultiplerTimerConfig;
    FreqMultiplerTimerConfig.dir = TimerHandle::Config::CounterDir::UP;
    FreqMultiplerTimerConfig.enable_irq = true;
    FreqMultiplerTimerConfig.period = 9999;
    FreqMultiplerTimerConfig.periph = TimerHandle::Config::Peripheral::TIM_5;
    objEncTimerMultipler.Init(FreqMultiplerTimerConfig);
    objEncTimerMultipler.SetPrescaler(200000);//Sets rate in which the timer ticks, 1khz.
    // objEncTimerMultipler.SetCallback(fxTimerCB, nullptr);
    objEncTimerMultipler.Start();
}



void fxControlInit(){
    uiLastUpdate = System::GetNow();
    fxEncoderSetup();
}


//DroneVoiceOscillatorSetup
void fxDroneVoiceSetup(float flSample_rate){
    aobjSingleOscillator[0].Init(flSample_rate, OSC_NORMAL);
    aobjSingleOscillator[0].SetFreq(100);
    aobjSingleOscillator[0].SetWaveform(WAVE_SIN);

    aobjSingleOscillator[1].Init(flSample_rate, OSC_NORMAL);
    aobjSingleOscillator[1].SetFreq(100);
    aobjSingleOscillator[1].SetWaveform(WAVE_TRI);

    aobjSingleOscillator[2].Init(flSample_rate, OSC_NORMAL);
    aobjSingleOscillator[2].SetFreq(100);    
    aobjSingleOscillator[2].SetWaveform(WAVE_SAW);
    
    aobjSingleOscillator[3].Init(flSample_rate, OSC_NORMAL);
    aobjSingleOscillator[3].SetFreq(100);
    aobjSingleOscillator[3].SetWaveform(WAVE_SQUARE);

}



float fxEncoderSpeedMultiplier(int iEncoderID){
    uint32_t uiCurrentTime = objEncTimerMultipler.GetTick(); //Get the current tick of the system
    uint32_t uiDeltaTime_ = (uiCurrentTime - uiLastTime_[iEncoderID]) & 0xFFFFFFFF; //Make positive
    uiLastTime_[iEncoderID] = uiCurrentTime;  //Update lasttime 
    //Compute the speed factor based on increments within time. 
    if(uiDeltaTime_ > 0){
        float flSpeed = 10000.0f / uiDeltaTime_; 
        flSpeedFactor_[iEncoderID] = 1.0f + (flSpeed * SPEED_SCALE_);
        if(flSpeedFactor_[iEncoderID] > MAX_INC_ ) { flSpeedFactor_[iEncoderID] = MAX_INC_; }
    }
    return flSpeedFactor_[iEncoderID];
}


//Encoder Frequency Increment
void fxFrequencyInc(clasOscillator& osc, clasEncoder& enc, int iEncoderID){
    float flCurrentFreq = osc.GetFreq();
    int iDirection = enc.Increment();
    if (iDirection != 0) {
        float flSpeedScale = fxEncoderSpeedMultiplier(iEncoderID);
        flCurrentFreq += (flSpeedScale * iDirection);
        osc.SetFreq(flCurrentFreq);
    }
}


//Encoder Frequency Micro Increment 0.1 steps
void fxFrequencyMicroInc(clasOscillator& osc, clasEncoder& enc, int iEncoderID){
    float flCurrentFreq = osc.GetFreq();
    int iDirection = enc.Increment();
    if (iDirection != 0) {
        float flSpeedScale = fxEncoderSpeedMultiplier(iEncoderID);
        flCurrentFreq += ( (flSpeedScale * 0.1) * iDirection );
        osc.SetFreq(flCurrentFreq);
    }
}


//Reset Parent Oscillator
void resetParent(){
    objParent_ = nullptr;
}


//Reset Child Oscillator
void resetChild(){
    objChild_ = nullptr;
}


//Set Parent Oscillator
void setParent(clasOscillator* osc){
    objParent_ = osc;
    // hw.PrintLine("Parent Set");
    objResetTimer.Start();
}


//Set Child Oscillator
void setChild(clasOscillator* osc){
    objChild_ = osc;
    objChild_->SetFreq(objParent_->GetFreq());
    // hw.PrintLine("Child Set");
}


//Get Parent Oscillator
clasOscillator* getParent(){
    return objParent_;
}


//Get Child Oscillator
clasOscillator* getChild(){
    return objChild_;
}


//Callback function for the resetting of the parent child system. 
void fxResetTimerCallBack(void* data){
    hw.PrintLine("Reset");
    resetParent();
    resetChild();
    objResetTimer.Stop();
}


//Callback function for the resetting of the parent child system. 
void fxResetMenuTimerCallBack(void* data){
    hw.PrintLine("Menu Reset");
    eMenuState = M_MAIN;
    eMain = eResetMainMenuState;
    eSub = LFO_IDLE;
    eMode = LFO_IDLE;
    objResetTimer.Stop();
}


//Reset Timer Init
void fxResetTimerInit(){
    TimerHandle::Config ResetTimerConfig;
    ResetTimerConfig.dir = TimerHandle::Config::CounterDir::UP;
    ResetTimerConfig.enable_irq = true;
    ResetTimerConfig.period = PERIOD;
    ResetTimerConfig.periph = TimerHandle::Config::Peripheral::TIM_4;
    objResetTimer.Init(ResetTimerConfig);
    objResetTimer.SetPrescaler(39999); //Sets the rate in which the timer ticks, 5khz. 
    objResetTimer.SetCallback(fxResetTimerCallBack);
    objResetTimer.Start();
}

//Reset Timer Init
void fxResetMenuTimerInit(){
    TimerHandle::Config ResetTimerConfig;
    ResetTimerConfig.dir = TimerHandle::Config::CounterDir::UP;
    ResetTimerConfig.enable_irq = true;
    ResetTimerConfig.period = 60000; //5 second reset timer. 
    ResetTimerConfig.periph = TimerHandle::Config::Peripheral::TIM_3;
    objResetMenuTimer.Init(ResetTimerConfig);
    objResetMenuTimer.SetPrescaler(39999); //Sets the rate in which the timer ticks, 5khz. 
    objResetMenuTimer.SetCallback(fxResetMenuTimerCallBack);
    objResetMenuTimer.Start();
}


//Oscillator state machine 
void fxFrequencyMatch(clasOscillator& osc, clasEncoder& enc) {
    // enc.Debounce();
    if (enc.RisingEdge()){ //Sets signal high outside to ensure detects all rising edges. 
        // hw.PrintLine("Encoder Pressed");
        clasOscillator* objParent = getParent();

        if (objParent != nullptr) {
            setChild(&osc);
        } else {
            setParent(&osc);
        }
    }
}


void fxOscillatorSelect(clasOscillator* osc, clasEncoder& enc, int iEncoderNumber){

    if (enc.RisingEdge()){ //Sets signal high outside to ensure detects all rising edges. 
        ptrSingleOscillator_ = osc;
        hw.PrintLine("Osc %d Connected to LFO", iEncoderNumber);
    }

}


void fxEncoderTest(){
    objEncoderA.Debounce();
    if (objEncoderA.RisingEdge()){
        // hw.PrintLine("Encoder A Button Pressed");
    }

    objEncoderB.Debounce();
    if (objEncoderB.RisingEdge()){
        // hw.PrintLine("Encoder B Button Pressed");
    }  

    objEncoderC.Debounce();
    if (objEncoderC.RisingEdge()){
        // hw.PrintLine("Encoder C Button Pressed");
    }    

    objEncoderD.Debounce();
    if (objEncoderD.RisingEdge()){
        // hw.PrintLine("Encoder D Button Pressed");
    }

}


//Increments the oscillators amplitude
void IncOscillatorAmplitude(clasOscillator& osc, clasEncoder& enc){
    float flCurrentAmp = osc.GetAmp();
    int iDirection = enc.Increment();
    if (iDirection != 0) {
        flCurrentAmp += 0.1 * iDirection;
        osc.SetAmp(flCurrentAmp);
    }
}


//Increments the oscillators amplitude
void IncOscillatorDepth(clasOscillator& osc, clasEncoder& enc){
    float flCurrent = osc.GetDepth();
    int iDirection = enc.Increment();
    if (iDirection != 0) {
        flCurrent += 0.1 * iDirection;
        osc.SetDepth(flCurrent);
    }
}


//Changes the frequency shape of the oscillator.
void ChangeWaveform(clasOscillator& osc, clasEncoder& enc){
    uint8_t uiCurrentWaveform = osc.GetWaveform();
    int iDirection = enc.Increment();
    if (iDirection != 0) {
        uiCurrentWaveform += 1 * iDirection;
        osc.SetWaveform(uiCurrentWaveform);
    }
}


//Frequency modulation ratio
void FreqModulationRatio(clasOscillator& osc, clasEncoder& enc){
    float flModulatorRatio = osc.GetRatio();
    int iDirection = enc.Increment();
    if (iDirection != 0) {
        flModulatorRatio += 1 * iDirection;
        osc.SetRatio(flModulatorRatio);
    }
}



void FreqModulationCarrier(clasOscillator& oscA, clasOscillator& oscB, clasOscillator& oscC, clasEncoder& enc){
    fxFrequencyInc(aobjSingleOscillator[0], enc, 0);
    float flCarrierFrequency = aobjSingleOscillator[0].GetFreq();
    oscA.SetFreq(flCarrierFrequency);
    oscB.SetFreq(flCarrierFrequency);
    oscC.SetFreq(flCarrierFrequency);
    }


//Debounce 
bool fxMenuDebounce(){
    uint32_t uiNow = System::GetNow();
    bool xDebounce = false;

    if(uiNow - uiLastUpdate >= 100){
        uiLastUpdate = uiNow;
        xDebounce = true;
    }
    return xDebounce;
}


//Exponential semitone frequency updater.
void fxUpdateFrequencies(float adcInput){
    // Interpolate the low note and high note
    float flLowestFreq = 130.0f + adcInput * (520.0f - 130.0f);
    float flHighestFreq = 250.0f + adcInput * (990.0f - 250.0f);

    //Calculate the ratio of highest to lowest. 
    float flRatio = flHighestFreq / flLowestFreq;

    //Semitone distribution loop
    for (int i=0; i<12; i++){
        float flFraction = (float)i / 11.0f;
        aflFreqTable[i] = flLowestFreq * powf(flRatio, flFraction);
    }
}


//Changes the frequency shape of the oscillator.
void ChangeWaveform(clasEulersOscillator<16>& osc, clasEncoder& enc){
    uint8_t uiCurrentWaveform = osc.GetHarmonicWaveForm();
    int iDirection = enc.Increment();
    if (iDirection != 0) {
        uiCurrentWaveform += 1 * iDirection;
        osc.SetHarmonicWaveForm(uiCurrentWaveform);
    }
}


//Attack Decay Sustain Release Functionality 
void fxADSR(clasEncoder attack, clasEncoder decay, clasEncoder sustain, clasEncoder release, Switch adsrBtn){
    //Encoder debounce needs to happen outside of function to allow all other functions using encoders to work. 

    //Check if it is held
    if(adsrBtn.Pressed()){

        //Attack Update
        if ( attack.RisingEdge() ){ //Reset to default value. 
            objADSR.SetAttackRate(1.f);
        }
        int iDirA = attack.Increment();
        if (iDirA != 0){
            float flAttackTime = objADSR.GetAttackTime();
            flAttackTime += 0.25 * iDirA;
            objADSR.SetAttackRate(flAttackTime);
        }

        //Decay Update
        if ( decay.RisingEdge() ){ //Reset to default value. 
            objADSR.SetDecayRate(1.f);
        }
        int iDirB = decay.Increment();
        if (iDirB != 0){
            float flDecayTime = objADSR.GetDecayTime();
            flDecayTime += 0.25 * iDirB;
            objADSR.SetDecayRate(flDecayTime);
        }

        //Sustain Update
        if ( sustain.RisingEdge() ){ //Reset to default value. 
            objADSR.SetSustainLevel(0.7f);
        }
        int iDirC = sustain.Increment();
        if (iDirC != 0){
            float flSustainLevel = objADSR.GetSustainLevel();
            flSustainLevel += 0.1 * iDirC;
            objADSR.SetSustainLevel(flSustainLevel);
        }  

        //Release Update
        if ( release.RisingEdge() ){ //Reset to default value. 
            objADSR.SetReleaseRate(1.f);
        }
        int iDirD = release.Increment();
        if (iDirD != 0){
            float flReleaseTime = objADSR.GetReleaseTime();
            flReleaseTime += 0.25 * iDirD;
            objADSR.SetReleaseRate(flReleaseTime);
        }
    }
};


//User Interface Mode Menu
void fxModeMenu(int input){
    bool xPrintLine = false;

    //Menu debounce system
    if (fxMenuDebounce()){
        iUserInput_ = input;
        if (iPrevUserInput != iUserInput_){
            iPrevUserInput = iUserInput_;
            hw.PrintLine("Key press: %d", input);
            xPrintLine = true;
        }
    }

    if (eMenuState == M_MAIN){
        objResetMenuTimer.Start();

        //////////////////////////////////////////////////////////////////////////////////////
        //MAIN////////////////////////////////////////////////////////////////////////////////

        // if (xPrintLine) {hw.PrintLine("Entered the Main Menu");}
        eMain = iUserInput_;

        switch(eMain){
            case T_NORMAL:
            if (xPrintLine) {hw.PrintLine("Drone Normal Mode Selected");}
                // xNormalMode = true;
                eMain = T_NORMAL;
                break;

            case T_ADSR: //ADSR Control 
            if (xPrintLine) {hw.PrintLine("ADSR Mode Selected");}
                // fxADSR(objEncoderA, objEncoderB, objEncoderC, objEncoderD, Btn1);
                eMain = T_ADSR;
                break;
    
            case T_D_AMP:
            if (xPrintLine) {hw.PrintLine("Drone Amplitude Mode Selected");}

                eMain = T_D_AMP;
                break;
    
            case T_RATIO:
            if (xPrintLine) {hw.PrintLine("Drone Ratio Mode Selected");}
                // xNormalMode = false;
                eMain = T_RATIO;
                break;
    
            case T_MATCH:
            if (xPrintLine) {hw.PrintLine("Drone Match Mode Selected");}

                eMain = T_MATCH;
                break;
    
            case T_D_SHAPE:
            if (xPrintLine) {hw.PrintLine("Drone Shape Mode Selected");}

                eMain = T_D_SHAPE;
                break;
    
            case T_L_SHAPE:
            if (xPrintLine) {hw.PrintLine("Lead Shape Mode Selected");}
                // ChangeWaveform(objLeadVoice, objEncoderA);
                eMain = T_L_SHAPE;
                break;
    
            case T_FAST_LFO:
            if (xPrintLine) {hw.PrintLine("FAST_LFO Mode Selected");}
                eMenuState = M_SUB;
                //Store the address of the LFO

                g_iLFOElement = 0;

                g_astruLfoSlots[g_iLFOElement].ptrLFO = &objLFOFast;  // or &objLFOSlow
                g_astruLfoSlots[g_iLFOElement].whichVoice = VoiceSelection::None;
                g_astruLfoSlots[g_iLFOElement].xTremolo = false;
                g_astruLfoSlots[g_iLFOElement].xVibrato = false;


                eMain = T_FAST_LFO;

                break;
    
            case T_SLOW_LFO:
            if (xPrintLine) {hw.PrintLine("SLOW_LFO Mode Selected");}
                eMenuState = M_SUB;
                //Store the address of the LFO

                g_iLFOElement = 1;

                g_astruLfoSlots[g_iLFOElement].ptrLFO = &objLFOSlow;
                g_astruLfoSlots[g_iLFOElement].whichVoice = VoiceSelection::None;
                g_astruLfoSlots[g_iLFOElement].xTremolo = false;
                g_astruLfoSlots[g_iLFOElement].xVibrato = false;


                eMain = T_SLOW_LFO;
                break;
    
            default: 
                // if (xPrintLine) {hw.PrintLine("defaulted! Shit the Bed.........!!");} 
                xNormalMode = true;
                eMain = T_NORMAL;
                break;
            
            }

            //Updated the reset timers callback
            eResetMainMenuState = eMain;
            eWhileState = eMain;
            
        } else if (eMenuState == M_SUB){

            //////////////////////////////////////////////////////////////////////////////////////
            //SUB////////////////////////////////////////////////////////////////////////////////

            // if (xPrintLine) {hw.PrintLine("Entered the Sub Menu");}
            eSub = iUserInput_;

            switch(eSub){
                case LFO_IDLE: 
                    //Do nothing
                    eSub = LFO_IDLE;
                    break;

                case LFO_DRONE:
                if (xPrintLine) {hw.PrintLine("Drone Voice Connected to LFO");}
                    eMenuState = M_MODE;
                    
                    //Save which oscillator to manipulate with LFO. 
                    g_astruLfoSlots[g_iLFOElement].whichVoice = VoiceSelection::Drone;

                    eSub = LFO_IDLE;
                    break;

                case LFO_SINGLE:
                    eMenuState = M_MODE;

                    if(ptrSingleOscillator_ == nullptr){
                        fxOscillatorSelect(&aobjSingleOscillator[0], objEncoderA, 0);
                        fxOscillatorSelect(&aobjSingleOscillator[1], objEncoderB, 1);
                        fxOscillatorSelect(&aobjSingleOscillator[2], objEncoderC, 2);
                        fxOscillatorSelect(&aobjSingleOscillator[3], objEncoderD, 3);
                        eSub = LFO_SINGLE;
                    } else {
                        //Save which oscillator to manipulate with LFO. 
                        g_astruLfoSlots[g_iLFOElement].ptrSingleOscSelectLFO_ = ptrSingleOscillator_;
                        ptrSingleOscillator_ = nullptr; //Reset the nullpointer. 
                        g_astruLfoSlots[g_iLFOElement].whichVoice = VoiceSelection::Single;
                        eSub = LFO_IDLE;
                    }

                    break;

                case LFO_LEAD:
                if (xPrintLine) {hw.PrintLine("Lead Voice Connected to LFO");}
                    eMenuState = M_MODE;
                    
                    //Save which oscillator to manipulate with LFO. 
                    g_astruLfoSlots[g_iLFOElement].whichVoice = VoiceSelection::Lead;

                    eSub = LFO_IDLE;
                    break;

                default: 

                    break;
            }
        } else if (eMenuState == M_MODE) {

            //////////////////////////////////////////////////////////////////////////////////////
            //MODE////////////////////////////////////////////////////////////////////////////////

            // if (xPrintLine) {hw.PrintLine("Entered the Mode Menu");}
            int eMode = iUserInput_;

            switch(eMode){
                case LFO_IDLE:
                    //Do nothing

                    eMode = LFO_IDLE;
                    break;

                case LFO_TROMOLO: 
                if (xPrintLine) {hw.PrintLine("TROMOLO Mode Selected");}
                    
                    g_astruLfoSlots[g_iLFOElement].ptrLFO->SetTromolo();
                    g_astruLfoSlots[g_iLFOElement].xTremolo = true;
                    g_astruLfoSlots[g_iLFOElement].xVibrato = false;

                    eMenuState = M_MAIN; //Return back to Main Menu
                    eMode = LFO_IDLE;
                    break;

                case LFO_VIBRATO:
                if (xPrintLine) {hw.PrintLine("VIBRATO Mode Selected");}
                    
                    g_astruLfoSlots[g_iLFOElement].ptrLFO->SetVibrato();
                    g_astruLfoSlots[g_iLFOElement].xTremolo = false;
                    g_astruLfoSlots[g_iLFOElement].xVibrato = true;

                    eMenuState = M_MAIN; //Return back to Main Menu
                    eMode = LFO_IDLE;
                    break;

                default: 

                    // eMode = LFO_IDLE; 
                    // if (xPrintLine) {hw.PrintLine("MODE: Incorrect mode select: %d,  Please try again", eMode);}
                    break;
            }


        }


        iUserInput_ = 0; //Reset 
        uiTouchKeyPressed = -1; 


    
    //Add code here to reset everything after time delay. 
 }


//Cap Touch Press function 
void fxPressFunction(uint16_t uiPad, bool xMenuActive){
    // hw.PrintLine("Key %d touch", uiPad);
    uiTouchKeyPressed = uiPad;
    if (!xMenuActive) {
        objLeadVoice.SetFreq(aflFreqTable[uiPad]);
        //envelope start signal here. 
        xGate = true;
    }
}


//Cap Toucnh Release function 
void fxReleaseFunction(uint16_t uiPad){
    // hw.PrintLine("Key %d release", uiPad);
    uiTouchKeyPressed = -1;
    //Envelope stop signal
    
    xGate = false;
}



//CLASSES//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Capacitive Touch Class//////////////////////////////////////////////////////////////////////////////////////////////////////
class clasCapTouch{
    private:

        Mpr121I2C _objMpr121;
        DaisySeed _hw;
        uint16_t _uiState;
        void (*_ptrOnTouch)(uint16_t uiPad, bool xMenuActive); //function address placeholder
        void (*_ptrOnRelease)(uint16_t uiPad);

        void mConfig(){
            Mpr121I2C::Config config;
            if (_objMpr121.Init(config) == Mpr121I2C::OK){
                    _hw.PrintLine("MPR121 Initialized Successfully!");

            }   

            SetOnTouch(fxPressFunction);
            SetOnRelease(fxReleaseFunction);
        }

    public:

        void Init(DaisySeed hw){
            _hw = hw;
            mConfig();
        };


        // Connects the callback functions, inserts the function into the adress placeholder. 
        void SetOnTouch(void (*ptrOnTouch)(uint16_t uiPad,  bool xMenuActive)) { 
            _ptrOnTouch = ptrOnTouch;
        }

        void SetOnRelease(void (*ptrOnRelease)(uint16_t uiPad)) { 
            _ptrOnRelease = ptrOnRelease;
        }

        bool IsTouched(uint16_t uiPad) { 
            return _uiState & (1 << uiPad); 
        }

        bool HasTouch() { 
            return _uiState > 0; 
        }
    

        void Process() {
            uint16_t uiPad;
            bool xIsTouched;
            bool xWasTouched;
            auto state = _objMpr121.Touched();
            for (uint16_t i = 0; i < 12; i++) {
                uiPad = 1 << i;
                xIsTouched = state & uiPad;
                xWasTouched = _uiState & uiPad;
                if (_ptrOnTouch != nullptr && xIsTouched && !xWasTouched) {
                    _ptrOnTouch(i, Btn1.Pressed()); //Button added for mode. Uses call back function
                    // _hw.PrintLine("Key %d touch", i);
                } else if (_ptrOnRelease != nullptr && xWasTouched && !xIsTouched) {
                    _ptrOnRelease(i);
                    // _hw.PrintLine("Key %d release", i);

                }
            }
            _uiState = state;
        };

        
};


#endif