#include "daisysp.h"
#include "daisy_seed.h"
#include "math.h"
#include "adsr.h"
#include "dronevoice.h"
#include "eulerssynthesis.h"
#include "leadvoice.h"
#include "twoorderfilter.h"
#include "control.h"

using namespace daisysp;
using namespace daisy;
using namespace seed;

DaisySeed hw;
clasCapTouch objTouch;
clasEulersOscillator<16> objLeadVoice;
clasSecondOrderFilter objLPFLeadVoice;

clasOscillator objLFOSlow;
clasOscillator objLFOFast;

//TEST/////////////////////////////////////////////////
const float ADC_SCALAR = 1.25;

//VARIABLES////////////////////////////////////////////

float AmpLead;

float adcPT1;
float adcPT2;
float adcPT3;
float adcPT4;
float adcPT5;
float adcPT6;
float adcPT7;
float adcPT8;
float adcPT9;
float adcPT10;

//FILTERS//////////////////////////////////////////////
static OnePole objFilter1;
static OnePole objFilter2;

static clasSecondOrderFilter objLPFDrone;





//INTERRUPTS AND CALLBACKS//////////////////////////////////
//Main audio call back function 

//Cant use PrintLine in here because will be called before the port is opened. 
static void AudioCallback(AudioHandle::InterleavingInputBuffer  in,
                          AudioHandle::InterleavingOutputBuffer out,
                          size_t                                size)
{
    for(size_t i = 0; i < size; i += 2)
    {
        //VOICE OUTPUTS////////////////////////////////////////////////////////////////////

            //SINGLE OSCILLATORS////////////////////////////////////////////////////////////////////
            float aflSingleOutput[4]; //Individual outputs from the 4 oscillators. 

            aflSingleOutput[0] = aobjSingleOscillator[0].Process();
            aflSingleOutput[1]  = aobjSingleOscillator[1].Process();
            aflSingleOutput[2]  = aobjSingleOscillator[2].Process();
            aflSingleOutput[3]  = aobjSingleOscillator[3].Process();

            //LFO Processing Single Oscillator.
            for(int i = 0; i < 2; i++){
                //Check for nulls in order to proceed. 
                if( g_astruLfoSlots[i].ptrLFO != nullptr && 
                    g_astruLfoSlots[i].ptrSingleOscSelectLFO_ != nullptr && 
                    g_astruLfoSlots[i].whichVoice == VoiceSelection::Single){

                    //Processes the output of a single oscillator. 
                    aflSingleOutput[g_astruLfoSlots[i].iOscNumber] = g_astruLfoSlots[i].ptrLFO->LFOProcess(aflSingleOutput[g_astruLfoSlots[i].iOscNumber]);
                }
            }


            //DRONE VOICE//////////////////////////////////////////////////////////////////////////
            //Drone output from oscillator bank
            float DroneOutputSignal = ((
                aflSingleOutput[0] +
                aflSingleOutput[1] +
                aflSingleOutput[2] +
                aflSingleOutput[3] ) 
                * 0.25f);


            //LFO implementation for Drone voice
            for(int i = 0; i < 2; i++){
                //Check for nulls in order to proceed. 
                if(g_astruLfoSlots[i].ptrLFO != nullptr && g_astruLfoSlots[i].whichVoice == VoiceSelection::Drone){
                    //Processes the output of drone voice. 
                    DroneOutputSignal = g_astruLfoSlots[i].ptrLFO->LFOProcess(DroneOutputSignal);
                }
            }

            //Drone output into filter. 
            DroneOutputSignal = objLPFDrone.Process(DroneOutputSignal);
            

            //LEAD VOICE////////////////////////////////////////////////////////////////////////////////////
            //Lead output from oscillator
            float LeadOutputSignal = objLeadVoice.Process();

            //Lead ampilification from ADSR
            objLeadVoice.SetAmp(objADSR.Process(xGate));

            //LFO implementation for the Lead voice. 
            for(int i = 0; i < 2; i++){
                //Check for nulls in order to proceed. 
                if(g_astruLfoSlots[i].ptrLFO != nullptr && g_astruLfoSlots[i].whichVoice == VoiceSelection::Lead){
                    //Process the data. 
                    LeadOutputSignal = g_astruLfoSlots[i].ptrLFO->LFOProcess(LeadOutputSignal);
                }
            }

            //Lead output into filter. 
            LeadOutputSignal = objLPFLeadVoice.Process(LeadOutputSignal);
                

        //SOUND OUTPUT/////////////////////////////////////////////////////////////////////////

            //Combination output of lead and drone voices, normalised to 0-1 and controlled by master pot. 
            float flOutput = (LeadOutputSignal + DroneOutputSignal) * 0.5f * adcPT2; 


            out[i] = flOutput; // left out
            out[i + 1] = flOutput; // right out
    }
}

//MAIN/////////////////////////////////////////////////////////////
int main(void)
{
    // initialize seed hardware
    hw.Configure();
    hw.Init();
    hw.SetAudioBlockSize(4);
    float sample_rate = hw.AudioSampleRate();
    // hw.StartLog(true); //For the serial port. 
    fxEncoderTimerInit();
    fxResetTimerInit();
    fxResetMenuTimerInit();
    objADSR.Init(sample_rate);

    objTouch.Init(hw);
    InitLFOAssignments();

    //Controls Setup
    fxControlInit();

    //Buttons
    Btn1.Init(hw.GetPin(8), 1000, daisy::Switch::TYPE_MOMENTARY, daisy::Switch::POLARITY_INVERTED, daisy::Switch::Pull::PULL_UP);
    Btn2.Init(hw.GetPin(9), 1000, daisy::Switch::TYPE_MOMENTARY, daisy::Switch::POLARITY_INVERTED, daisy::Switch::Pull::PULL_UP);

    
    // Define ADC channels
    AdcChannelConfig adcConfig[10];
    adcConfig[0].InitSingle(hw.GetPin(15)); //PT1_Effect3 //Drone Out
    adcConfig[1].InitSingle(hw.GetPin(16)); //PT2_Effect1 //Lead Out
    adcConfig[2].InitSingle(hw.GetPin(17)); //PT3_Filter1 //Drone filter
    adcConfig[3].InitSingle(hw.GetPin(18)); //PT4_Filter2 //Lead filter
    adcConfig[4].InitSingle(hw.GetPin(19)); //PT5_OSC_A_Amp
    adcConfig[5].InitSingle(hw.GetPin(20)); //PT6_OSC_B_Amp
    adcConfig[6].InitSingle(hw.GetPin(21)); //PT7_OSC_C_Amp
    adcConfig[7].InitSingle(hw.GetPin(22)); //PT8_OSC_D_Amp
    adcConfig[8].InitSingle(hw.GetPin(23)); //PT9_RealImag //Real/Imag
    adcConfig[9].InitSingle(hw.GetPin(24)); //PT10_Effect4 //Lead Resonance


    // Initialize ADC with all channels
    hw.adc.Init(adcConfig, 10);
    hw.adc.Start();

    fxDroneVoiceSetup(sample_rate);
    objLeadVoice.Init(sample_rate);

    objLFOFast.Init(sample_rate, OSC_LFO_FAST);
    objLFOSlow.Init(sample_rate, OSC_LFO_SLOW);


    //Low Pass Filter for output. 
    objFilter1.Init();
    objFilter1.SetFilterMode(objFilter1.FILTER_MODE_LOW_PASS);
    objFilter1.SetFrequency(10);

    objLPFDrone.Init(sample_rate);
    objLPFLeadVoice.Init(sample_rate);
    objLPFLeadVoice.setFreq(20000);

    hw.StartAudio(AudioCallback);

    while(1) {

        objEncoderA.Debounce();
        objEncoderB.Debounce();
        objEncoderC.Debounce();
        objEncoderD.Debounce();
        Btn1.Debounce();
        Btn2.Debounce();


        if (Btn1.Pressed() && uiTouchKeyPressed != 255){ 
                fxModeMenu(uiTouchKeyPressed); 
        } else {

            switch(eWhileState){
                case T_NORMAL:

                    //Reset the Ratios to 1;
                    aobjSingleOscillator[1].SetRatio(1);
                    aobjSingleOscillator[2].SetRatio(1);
                    aobjSingleOscillator[3].SetRatio(1);
                    if (Btn2.Pressed()){ 
                        //Micro Frequency change of oscillators. 
                        fxFrequencyMicroInc(aobjSingleOscillator[0], objEncoderA, 0);
                        fxFrequencyMicroInc(aobjSingleOscillator[1], objEncoderB, 1);
                        fxFrequencyMicroInc(aobjSingleOscillator[2], objEncoderC, 2);
                        fxFrequencyMicroInc(aobjSingleOscillator[3], objEncoderD, 3);
                    } else {
                        //Normal frequency change of occillators. 
                        fxFrequencyInc(aobjSingleOscillator[0], objEncoderA, 0);
                        fxFrequencyInc(aobjSingleOscillator[1], objEncoderB, 1);
                        fxFrequencyInc(aobjSingleOscillator[2], objEncoderC, 2);
                        fxFrequencyInc(aobjSingleOscillator[3], objEncoderD, 3);
                    }

                    break;
                case T_ADSR:

                    fxADSR(objEncoderA, objEncoderB, objEncoderC, objEncoderD, Btn1);

                    break;

                case T_D_AMP:

                    IncOscillatorAmplitude(aobjSingleOscillator[0], objEncoderA);
                    IncOscillatorAmplitude(aobjSingleOscillator[1], objEncoderB);
                    IncOscillatorAmplitude(aobjSingleOscillator[2], objEncoderC);
                    IncOscillatorAmplitude(aobjSingleOscillator[3], objEncoderD);
                    break;

                case T_RATIO:

                    if (Btn2.Pressed()){ 
                        fxFrequencyMicroInc(aobjSingleOscillator[0], objEncoderA, 0);
                        float flCarrierFrequency = aobjSingleOscillator[0].GetFreq();
                        aobjSingleOscillator[1].SetFreq(flCarrierFrequency);
                        aobjSingleOscillator[2].SetFreq(flCarrierFrequency);
                        aobjSingleOscillator[3].SetFreq(flCarrierFrequency);
                    } else {
                        FreqModulationCarrier(aobjSingleOscillator[1], aobjSingleOscillator[2], aobjSingleOscillator[3], objEncoderA);
                        FreqModulationRatio(aobjSingleOscillator[1], objEncoderB);
                        FreqModulationRatio(aobjSingleOscillator[2], objEncoderC);
                        FreqModulationRatio(aobjSingleOscillator[3], objEncoderD);
                    }
                    break;

                case T_MATCH:
                    fxFrequencyMatch(aobjSingleOscillator[0], objEncoderA);
                    fxFrequencyMatch(aobjSingleOscillator[1], objEncoderB);
                    fxFrequencyMatch(aobjSingleOscillator[2], objEncoderC);
                    fxFrequencyMatch(aobjSingleOscillator[3], objEncoderD);
                    break;

                case T_D_SHAPE:                
                    ChangeWaveform(aobjSingleOscillator[0], objEncoderA);
                    ChangeWaveform(aobjSingleOscillator[1], objEncoderB);
                    ChangeWaveform(aobjSingleOscillator[2], objEncoderC);
                    ChangeWaveform(aobjSingleOscillator[3], objEncoderD);
                    break;

                case T_L_SHAPE:
                    ChangeWaveform(objLeadVoice, objEncoderA);
                    break;

                case T_FAST_LFO:
                    // fxFrequencyInc(objLFOSlow, objEncoderA, 0);
                    IncOscillatorDepth(objLFOSlow, objEncoderA); //Add this function 
                    ChangeWaveform(objLFOSlow, objEncoderB);
                    break;

                case T_SLOW_LFO:
                    // fxFrequencyInc(objLFOFast, objEncoderA, 0);
                    IncOscillatorDepth(objLFOFast, objEncoderA); //Add this function 
                    ChangeWaveform(objLFOFast, objEncoderB);
                    break;

            }
        }

        // //Potentiometers
        adcPT1 = ADC_SCALAR * (hw.adc.GetFloat(0));
        adcPT2 = ADC_SCALAR * (hw.adc.GetFloat(1));
        adcPT3 = ADC_SCALAR * (hw.adc.GetFloat(2));
        adcPT4 = ADC_SCALAR * (hw.adc.GetFloat(3));
        adcPT5 = ADC_SCALAR * (hw.adc.GetFloat(4));
        adcPT6 = ADC_SCALAR * (hw.adc.GetFloat(5));
        adcPT7 = ADC_SCALAR * (hw.adc.GetFloat(6));
        adcPT8 = ADC_SCALAR * (hw.adc.GetFloat(7));
        adcPT9 = ADC_SCALAR * (hw.adc.GetFloat(8));
        adcPT10 = ADC_SCALAR * (hw.adc.GetFloat(9));


        objLeadVoice.SetRealAmp(adcPT4);
        objLeadVoice.SetImagAmp(adcPT3);

        objLPFDrone.setFreqNormalised(adcPT5);
        objLPFDrone.setQ(adcPT6);
        objLPFLeadVoice.setFreqNormalised(adcPT7 * 10);
        objLPFLeadVoice.setQ(adcPT8);
        fxUpdateFrequencies(adcPT1);


        objLFOSlow.SetFreq(20 * adcPT10);
        objLFOFast.SetFreq(20 * adcPT9);

        objTouch.Process();

    }
}
