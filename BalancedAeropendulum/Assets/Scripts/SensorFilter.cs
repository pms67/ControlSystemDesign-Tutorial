using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/*
 * 
 * First-Order Low-Pass Filter 
 *
 */

public class SensorFilter {

    public float tau; /* Filter time constant */
    public float T;   /* Sample time */

    public float output;

    private float prevInput;

    public SensorFilter(float tau, float T)
    {

        this.tau = tau;
        this.T   = T;

        output = 0.0f;
        prevInput = 0.0f;

    }

    public float Update(float input) {

        output = (T * (input + prevInput) + (2.0f * tau - T) * output) / (2.0f * tau + T);

        prevInput = input;

        return output;

    }

}
