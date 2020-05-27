using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PID {

    /* Controller gains */
    public float Kp;
    public float Ki;
    public float Kd;

    /* Derivative low-pass filter time constant */
    public float tau;

    /* Output limits */
    public float limMin;
    public float limMax;

    /* Sample time (in seconds) */
    public float T;

    /* Controller "memory" */
    private float integrator;
    private float prevError;        /* Required for integrator */
    private float differentiator;
    private float prevMeasurement;  /* Required for differentiator */

    /* Controller output */
    public float output;

    /* Constructor */
	public PID(float Kp, float Ki, float Kd, float tau, float limMin, float limMax, float T)
    {

        /* Store controller parameters */
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;

        this.tau = tau;

        this.limMin = limMin;
        this.limMax = limMax;

        this.T = T;

        /* Clear controller variables */
        Reset();

    }

    /* Clear PID state */
    public void Reset()
    {
        integrator      = 0.0f;
        prevError       = 0.0f;
        differentiator  = 0.0f;
        prevMeasurement = 0.0f;

        output = 0.0f;
    }

    public float Update(float setpoint, float measurement)
    {

        /*
        * Error signal
        */
        float error = setpoint - measurement;


        /*
        * Proportional
        */
        float proportional = Kp * error;


        /*
        * Integral
        */
        integrator = integrator + 0.5f * Ki * T * (error + prevError);


        /* Anti-wind-up via dynamic integrator clamping */
        float limMinInt, limMaxInt;

        /* Compute integrator limits */
        if (limMax > proportional)
        {

            limMaxInt = limMax - proportional;

        }
        else
        {

            limMaxInt = 0.0f;

        }

        if (limMin < proportional)
        {

            limMinInt = limMin - proportional;

        }
        else
        {

            limMinInt = 0.0f;

        }

        /* Clamp integrator */
        if (integrator > limMaxInt)
        {

            integrator = limMaxInt;

        }
        else if (integrator < limMinInt)
        {

            integrator = limMinInt;

        }


        /*
        * Derivative (band-limited differentiator)
        */
        differentiator = (2.0f * Kd * ((0.0f - measurement) - (0.0f - prevMeasurement))    /* Note: derivative on measurement! */
                       + (2.0f * tau - T) * differentiator)
                       / (2.0f * tau + T);
        
        /*
        * Compute output and apply limits
        */
        output = proportional + integrator + differentiator;

        if (output > limMax) {

            output = limMax;

        } else if (output < limMin) {

            output = limMin;

        }

        /* Store error and measurement for later use */
        prevError = error;
        prevMeasurement = measurement;

        /* Return controller output */
        return output;

    }

}
