using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SemiQuadDynamics {

    private float massMotor;           /* Mass of a single motor */
    private float massRod;             /* Mass of rod */
    private float lengthRod;           /* Length of rod */
    private float frictionCoefficient; /* Friction coefficient at pivot */

    private float momentOfInertia;     /* Moment of inertia about pivot */

    private float T;                   /* Sample time */

    /* Angle and derivatives (all in rad (per s, per s^2)) */
    private float theta;
    private float thetaDot;
    private float thetaDblDot;

    /* Disturbance */
    private float thetaDblDotDisturbance;

    /* System output */
    public float output;

    /* Constructor */
    public SemiQuadDynamics(float massMotor, float massRod, float lengthRod, float frictionCoefficient, float T)
    {

        /* Store system parameters */
        this.massMotor           = massMotor;
        this.massRod             = massRod;
        this.lengthRod           = lengthRod;
        this.frictionCoefficient = frictionCoefficient;

        this.T = T;

        /* Compute moment of inertia about pivot */
        momentOfInertia = 0.5f * massMotor * lengthRod * lengthRod + (massRod * lengthRod * lengthRod) / 12.0f;

        /* Clear state */
        Initialise(0.0f, 0.0f);

        /* Clear disturbance */
        thetaDblDotDisturbance = 0.0f;

    }

    /* Set system's initial state */
    public void Initialise(float thetaRad, float thetaDotRad)
    {
        theta    = thetaRad;
        thetaDot = thetaDotRad;
    }

    /* Update system's state. Input force F, sample time T */
    public float Update(float forceLeft, float forceRight)
    {

        /* Calculate angular acceleration */
        thetaDblDot = (0.5f * lengthRod * forceRight - 0.5f * lengthRod * forceLeft - frictionCoefficient * thetaDot) / momentOfInertia + thetaDblDotDisturbance;

        /* Remove impulse disturbance */
        thetaDblDotDisturbance = 0.0f;
 
        
        /* Simple Euler integration */
        thetaDot = thetaDot + thetaDblDot * T;
        theta    = theta    + thetaDot    * T;

        /* Limit maximum swing to +- 30 deg */
        if (theta > 0.61086523819f)
        {

            theta = 0.61086523819f;

        } else if (theta < -0.61086523819f)
        {

            theta = -0.61086523819f;

        }

        /* Set output */
        output = theta;

        /* Return angular displacement */
        return output;

    }

    public void SetDisturbance(float magnitude)
    {

        thetaDblDotDisturbance = magnitude;

    }


}
