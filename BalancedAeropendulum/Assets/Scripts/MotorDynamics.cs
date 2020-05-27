using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/*
 * Simple dynamic model of a motor
 * 
 * Uses gain cascaded with a first-order lag.
 * 
 */

public class MotorDynamics {
    
    public float gainPWMToForce;
    public float timeConstant;

    public float T;

    private float prevInput;

    public float output;

    public MotorDynamics(float gainPWMToForce, float timeConstant, float T)
    {

        this.gainPWMToForce = gainPWMToForce;
        this.timeConstant   = timeConstant;

        this.T = T;

        Reset();

    }

    public void Reset()
    {

        prevInput  = 0.0f;
        output     = 0.0f;

    }

    /* Update motor dynamics. Output = Force (N) */
    public float Update(float throttleSetting)
    {

        /* Compute output */
        output = (gainPWMToForce * T * (throttleSetting + prevInput) + (2.0f * timeConstant - T) * output)
               / (2.0f * timeConstant + T);

        /* Store current input */
        prevInput  = throttleSetting;

        /* Return output */
        return output;

    }

}
