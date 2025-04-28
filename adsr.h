#pragma once
#ifndef ADSR_H
#define ADSR_H

#include <stdint.h>
#ifdef __cplusplus

enum eMode
{
    ADSR_IDLE    = 0,
    ADSR_ATTACK  = 1,
    ADSR_DECAY   = 2,
    ADSR_SUSTAIN = 3,
    ADSR_RELEASE = 4
};


class clasADSR
{
    private:
    //Attributes/////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////

    float flSampleRate_;
    float flDt_;

    float flAttackTime_;
    float flDecayTime_;
    float flReleaseTime_;
    float flSustainLevel_;
    
    float flX0Attack_;
    float flX0Decay_;
    float flX0Release_;

    float flBAttack_;
    float flBDecay_;
    float flBRelease_;

    eMode uiMode_;
    bool xNoteOn_;
    float flY_;
    float flYPrev_;

    //Methods/////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////

    //Recursive Output Equation
    float mRecursiveEquation(float X0, float b, float YPrev){
        float Y = X0 + b * YPrev;
        flYPrev_ = Y;
        return Y;
    }
    public:

    //Methods/////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////

    //Initalise the object and setup the rates. 
    void Init(float flSampleRate){
        flSampleRate_ = flSampleRate;
        flDt_ = 1.0f / flSampleRate;
        flAttackTime_ = 1.0f;
        flDecayTime_ = 1.0f;
        flReleaseTime_ = 1.0f;
        flSustainLevel_ = 0.05f;
        
        SetAttackRate(flAttackTime_);
        SetDecayRate(flDecayTime_);
        SetReleaseRate(flReleaseTime_);
    };   


    float Process(bool xSignalOn){

        //Signal ON / OFF
        if (xSignalOn && !xNoteOn_){
            uiMode_ = ADSR_ATTACK;
        } else if (!xSignalOn && xNoteOn_) {
            uiMode_ = ADSR_RELEASE;
        }
        xNoteOn_ = xSignalOn;

        //Switch case for ADSR sections. 
        switch(uiMode_)
        {
            case ADSR_IDLE:
                flY_ = 0.0f;
                break;

            case ADSR_ATTACK:
                flY_ = mRecursiveEquation(flX0Attack_, flBAttack_, flYPrev_);
                
                if (flY_ >= 1.0f) { //Change condition
                    flY_ = 1.0f;
                    uiMode_ = ADSR_DECAY; 
                }
                break;

            case ADSR_DECAY:
                flY_ = mRecursiveEquation(flX0Decay_, flBDecay_, flYPrev_);

                if (flY_ <= flSustainLevel_) { //Change condition
                    flY_ = flSustainLevel_;
                    uiMode_ = ADSR_SUSTAIN;
                }
                break;

            case ADSR_SUSTAIN:
                flY_ = flSustainLevel_;
                break;

            case ADSR_RELEASE:
                flY_ = mRecursiveEquation(flX0Release_, flBRelease_, flYPrev_);

                if (flY_ <= 0.0f){ //Change condition
                    flY_ = 0.0f;
                    uiMode_ = ADSR_IDLE;
                }
                break;

            default:
                uiMode_ = ADSR_IDLE;
                break;
        }
        return flY_;
    };


    //Getters/////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////

    float GetAttackTime(){
        return flAttackTime_;
    }


    float GetDecayTime(){
        return flDecayTime_;
    }


    float GetReleaseTime(){
        return flReleaseTime_;
    }


    float GetSustainLevel(){
        return flSustainLevel_;
    }


    //Setters///////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////

    //Sets the sustain level
    inline void SetSustainLevel(float level){ 
        level = (level <= 0.f) ? -0.01f // forces envelope into idle 
                                        : (level > 1.f) ? 1.f : level; //clamps to <= 1.0f;
        flSustainLevel_ = level;
    }


    //Sets the Attack rate
    void SetAttackRate(float seconds){
        flAttackTime_ = seconds;
        flAttackTime_ = (flAttackTime_ <= 0.f) ? 0.001 : (flAttackTime_ > 10.0f) ? 10.0f : flAttackTime_; //Clamps value
        float TCO = 1 - 0.99;
        float alpha = (-log( (1 + TCO ) / ( TCO ) ) / flAttackTime_ );
        flBAttack_ = exp(alpha * flDt_); 
        flX0Attack_ = ( ( 1 + TCO ) * ( 1 - flBAttack_ ) );
    }


    //Sets the Decay rate
    void SetDecayRate(float seconds){
        flDecayTime_ = seconds;
        flDecayTime_ = (flDecayTime_ <= 0.f) ? 0.001 : (flDecayTime_ > 10.0f) ? 10.0f : flDecayTime_; //Clamps value
        float TCO = exp(-3);
        float alpha = (-log( (1 + TCO ) / ( TCO ) ) / flDecayTime_ );
        flBDecay_ = exp(alpha * flDt_); 
        flX0Decay_ = ( ( flSustainLevel_ - TCO ) * ( 1 - flBDecay_ ) );
    }


    //Sets the Release rate
    void SetReleaseRate(float seconds){
        flReleaseTime_ = seconds;
        flReleaseTime_ = (flReleaseTime_ <= 0.f) ? 0.001 : (flReleaseTime_ > 15.0f) ? 15.0f : flReleaseTime_; //Clamps value
        float TCO = exp(-3);
        float alpha = (-log( (1 + TCO ) / ( TCO ) ) / flReleaseTime_ );
        flBRelease_ = exp(alpha * flDt_); 
        flX0Release_ = ( ( - TCO ) * ( 1 - flBRelease_ ) );
    }

};

#endif
#endif
