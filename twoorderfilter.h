#pragma once
#ifndef SECONDORD_H
#define SECONDORD_H

#include "daisysp.h"
#include "daisy_seed.h"
#include "math.h"


//cofficient structure
typedef struct {
    float den[3];
    float num[3];
} COEFFICIENTS;

//BELL CURVE EQ CLASS//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class clasSecondOrderFilter {
private:
    //Variables//////////////
    float Fs_;
    float Q_;
    float fcut_;
    float boost_;
    float T_;
    float pi_ = 3.14159265358979323846;
    float aflX_[3];
    float aflY_[3];
    const int MAX_FREQ = 1500;
    COEFFICIENTS stAanCoeff_;                    //Define ananlogue coefficients variable
    COEFFICIENTS stDigCoeff_;                    //Define digital coefficients variable


    //Methods////////////////

    //mBilinear coefficient generator. Takes the analogue coefficients and converts into digital by using the BZT.
    COEFFICIENTS mBilinear(COEFFICIENTS stAnaCoeff, float Fs_) {

        //Calculate k
        float k = Fs_*2;
        float k2 = k * k;

        //Apply the BZT//////////////////////////////////////////////////////////////////////////////
        float a0 = stAnaCoeff.den[0] * k2 + stAnaCoeff.den[1] * k + stAnaCoeff.den[2];
        float a1 = -2.0f * stAnaCoeff.den[0] * k2 + 2.0f * stAnaCoeff.den[2];
        float a2 = stAnaCoeff.den[0] * k2 - stAnaCoeff.den[1] * k + stAnaCoeff.den[2];
    
        float b0 = stAnaCoeff.num[0] * k2 + stAnaCoeff.num[1] * k + stAnaCoeff.num[2];
        float b1 = -2.0f * stAnaCoeff.num[0] * k2 + 2.0f * stAnaCoeff.num[2];
        float b2 = stAnaCoeff.num[0] * k2 - stAnaCoeff.num[1] * k + stAnaCoeff.num[2];
    
        // Normalize in-place (a0 becomes 1)
        float inv_a0 = 1.0f / a0;
    
        stDigCoeff_.num[0] = b0 * inv_a0;
        stDigCoeff_.num[1] = b1 * inv_a0;
        stDigCoeff_.num[2] = b2 * inv_a0;
    
        stDigCoeff_.den[0] = 1.0f;
        stDigCoeff_.den[1] = a1 * inv_a0;
        stDigCoeff_.den[2] = a2 * inv_a0;

        return stDigCoeff_;
    };

public:

    //Constructor
    // clBellCurveEQ_(){};

    //Initialise the class. 
    void Init(float sample_rate){
        Fs_ = sample_rate;
        Q_ = 0.707;
        fcut_ = 500;
        boost_ = 1;
        T_ = 1.0/Fs_;                           //Sampling period
    }


    void Update(){
        float Wo = 2*pi_*fcut_;                       //Calculate the angular cut-off freq_uency
        float Wop = 2.0 * Fs_ * tan(Wo*T_/2.0);       //Denormalised prewarp formula for continuous time 
        float flWopPowTwo = pow(Wop, 2);            //Optimisation. 
        //Analogue transfer function 
        stAanCoeff_.num[0] = 0;
        stAanCoeff_.num[1] = 0;
        stAanCoeff_.num[2] = flWopPowTwo;
        stAanCoeff_.den[0] = 1;
        stAanCoeff_.den[1] = (Wop/Q_);
        stAanCoeff_.den[2] = flWopPowTwo;

        stDigCoeff_ = mBilinear(stAanCoeff_, Fs_);      //Convert in mBilinear function. (BZT)
    };
        

    //Difference eq_uation function. Processes the sample using the filters coefficients. 
    float Process(float input){
        //Insert the input into the first element. 
        aflX_[0] = input;
        //Difference eq_uation updating array of float Y. 
        aflY_[0] = stDigCoeff_.num[0] * aflX_[0] + stDigCoeff_.num[1] * aflX_[1] + 
            stDigCoeff_.num[2] * aflX_[2] - stDigCoeff_.den[1] * aflY_[1] - stDigCoeff_.den[2] * aflY_[2];
        //Store the output for return.
        float temp = aflY_[0];
        //Move the recorded values across by 1 time delay
        for(int n = 2; n >=1; n--){ //Make sure to move y(n-1) into y(n-2) first
            aflX_[n] = aflX_[n-1];
            aflY_[n] = aflY_[n-1];
        }
        return temp;
    };
    

    //Get/Set methods for the class attributes. 
    float getFcut_(){
        return fcut_;
    };

    
    void setFreqNormalised(float normFreq) {

        // Clamp normalized frequency just below Nyquist
        // normFreq = normFreq < 0.497f ? normFreq : 0.497f; //Carrying out conditioning before entering the function therefore dont require. 

        //Un-normalise. 
        fcut_ = normFreq * MAX_FREQ;
    };

    void setFreq(float frequency){
        fcut_ = frequency;
        Update();
    }
    

    float getQ(){
        return Q_;
    };


    void setQ(float input){
        // Clamp input to [0.0, 1.0]
        if(input < 0.707f) input = 0.707f;
        if(input > 1.0f) input = 1.0f;
    
        // Map to useful Q range: 0.5 to 10 (or tweak as needed)
        Q_ = 0.707f + input * 9.5f;
        Update();
    };


    // float getBoost_(){
    //     return boost_;
    // };

    // void setBoost_(float input){
    //     boost_ = input;
    // };

    // //Returns the digital coefficients. 
    // COEFFICIENTS getDigitalCofficients(){
    //     return stDigCoeff_;
    // };

};


#endif