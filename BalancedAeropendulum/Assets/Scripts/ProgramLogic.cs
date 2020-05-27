using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class ProgramLogic : MonoBehaviour {

    /* System parameters */
    private const float massMotor           = 0.0025f;
    private const float massRod             = 0.0180f;
    private const float lengthRod           = 0.3000f;
    private const float frictionCoefficient = 0.1200f;

    /* Motor coefficient (i.e. conversion from motor command to actual motor thrust in Newtons) */
    private const float motorConversionCoefficient = 0.02f;

    /* Needed to access transforms (visualise rotations) */
    public GameObject rotatingAssembly;
    public GameObject propLeft;
    public GameObject propRight;

    /* UI */
    public Slider sliderKp;
    public Slider sliderKi;
    public Slider sliderKd;
    public Slider sliderTau;
    public Slider sliderSensorTau;
    public Slider sliderT;
    public Slider sliderSetpoint;
    public Slider sliderDisturbance;
    public Slider sliderSensorNoise;
    public Slider sliderMotorLag;

    public Text textAngle;
    public Text textControlSignal;

    public Text textKp;
    public Text textKi;
    public Text textKd;
    public Text textTau;
    public Text textSensorTau;
    public Text textT;
    public Text textSetpoint;
    public Text textDisturbance;
    public Text textSensorNoise;
    public Text textMotorLag;

    public Button buttonSetpoint;
    public Button buttonDisturbance;
    public Button buttonResetSystem;

    /* Sample time */
    private const float sampleTimeSystem = 0.001f;

    /* System dynamics */
    private SemiQuadDynamics system;

    /* Motor dynamics */
    private MotorDynamics motorLeft;
    private MotorDynamics motorRight;
    public const float baseMotorSetting = 50.0f;

    /* Controller */
    private PID controller;
    private float timerController;
    private float setpoint;

    /* Sensor filter */
    SensorFilter sensorFilter;

	void Start () {

        /* Set sample time (T) */
        Time.fixedDeltaTime = sampleTimeSystem;

        /* Initialise system */
        system = new SemiQuadDynamics(massMotor, massRod, lengthRod, frictionCoefficient, sampleTimeSystem);
        system.Initialise(0.0f, 0.0f); /* Set initial state */

        /* Initialise motor(s) */
        motorLeft  = new MotorDynamics(motorConversionCoefficient, 0.1f, sampleTimeSystem);
        motorRight = new MotorDynamics(motorConversionCoefficient, 0.1f, sampleTimeSystem);

        /* Initialise controller */
        controller = new PID(1.0f, 0.0f, 0.0f, 0.01f, -50.0f, 50.0f, 0.01f);

        /* Initialise sensor filter */
        sensorFilter = new SensorFilter(0.01f, 0.01f);

        /* Clear timer */
        timerController = 0.0f;

        /* Initial setpoint */
        setpoint = 0.0f;

        /* Initialise GUI */
        InitGUI();
        
	}
	
	void FixedUpdate () {

        /* Get elapsed time since last frame */
        float T = Time.fixedDeltaTime;

        /* Check if it's time to update PID controller */
        if (timerController >= controller.T)
        {

            /* Simulate measurement (i.e. true output + random Gaussian noise) */
            float sensorOutput = system.output + GenerateNormalRandom(0.0f, sliderSensorNoise.value);

            /* Filter measurement */
            sensorFilter.Update(sensorOutput);

            /* Calculate controller output */
            controller.Update(setpoint, sensorFilter.output);

            /* Update UI */
            textControlSignal.text = controller.output.ToString("F2");

            /* Reset timer */
            timerController = 0.0f;

        } else
        {
            /* Increment counter */
            timerController += sampleTimeSystem;

        }

        /* Get motor force */
        motorLeft.Update(baseMotorSetting - controller.output);
        motorRight.Update(baseMotorSetting + controller.output);

        /* Update system dynamics */
        system.Update(motorLeft.output, motorRight.output);

        /* Update 3D view */
        rotatingAssembly.transform.eulerAngles = new Vector3(0.0f, 0.0f, RadiansToDegrees(system.output));

        float propSpeedLeft = Mathf.Pow( (baseMotorSetting - controller.output * 10), 2.0f) * 0.25f;
        float propSpeedRight = Mathf.Pow((baseMotorSetting + controller.output * 10), 2.0f) * 0.25f;
        propLeft.transform.Rotate(Vector3.back * propSpeedLeft * T);
        propRight.transform.Rotate(Vector3.back * propSpeedRight * T);
        
        /* Update UI text */
        textAngle.text = RadiansToDegrees(system.output).ToString("F2");
        
    }

    /*
     * 
     * GUI FUNCTIONS
     * 
     */

    void InitGUI()
    {
        /* Add listeners to UI elements */

        /* Sliders */
        sliderKp.onValueChanged.AddListener(delegate { SliderChange(1); });
        sliderKi.onValueChanged.AddListener(delegate { SliderChange(2); });
        sliderKd.onValueChanged.AddListener(delegate { SliderChange(3); });
        sliderTau.onValueChanged.AddListener(delegate { SliderChange(4); });
        sliderSensorTau.onValueChanged.AddListener(delegate { SliderChange(5); });
        sliderT.onValueChanged.AddListener(delegate { SliderChange(6); });
        sliderSetpoint.onValueChanged.AddListener(delegate { SliderChange(7); });
        sliderDisturbance.onValueChanged.AddListener(delegate { SliderChange(8); });
        sliderSensorNoise.onValueChanged.AddListener(delegate { SliderChange(9); });
        sliderMotorLag.onValueChanged.AddListener(delegate { SliderChange(10); });

        /* Buttons */
        buttonSetpoint.onClick.AddListener(ChangeSetpoint);
        buttonDisturbance.onClick.AddListener(ImpulseDisturbance);
        buttonResetSystem.onClick.AddListener(ResetSystem);
    }
    
    void SliderChange(int sliderIndex)
    {
        switch (sliderIndex)
        {
            case 1:
                textKp.text   = sliderKp.value.ToString("F2");
                controller.Kp = sliderKp.value;
                break;
            case 2:
                textKi.text   = sliderKi.value.ToString("F2");
                controller.Ki = sliderKi.value;
                break;
            case 3:
                textKd.text   = sliderKd.value.ToString("F2");
                controller.Kd = sliderKd.value;
                break;
            case 4:
                textTau.text   = sliderTau.value.ToString("F2");
                controller.tau = sliderTau.value;
                break;
            case 5:
                textSensorTau.text = sliderSensorTau.value.ToString("F2");
                sensorFilter.tau   = sliderSensorTau.value;
                break;
            case 6:
                textT.text     = sliderT.value.ToString("F2") + "s";
                controller.T   = sliderT.value;
                sensorFilter.T = sliderT.value;
                break;
            case 7:
                textSetpoint.text = sliderSetpoint.value.ToString("F0");
                break;
            case 8:
                textDisturbance.text = sliderDisturbance.value.ToString("F0");
                break;
            case 9:
                textSensorNoise.text = sliderSensorNoise.value.ToString("F2");
                break;
            case 10:
                textMotorLag.text = sliderMotorLag.value.ToString("F2");
                motorLeft.timeConstant  = sliderMotorLag.value;
                motorRight.timeConstant = sliderMotorLag.value;
                break;

        }
    }

    void ChangeSetpoint()
    {
        setpoint = DegreesToRadians(sliderSetpoint.value);
    }

    void ImpulseDisturbance()
    {
        system.SetDisturbance(sliderDisturbance.value);
    }

    void ResetSystem()
    {
        system.Initialise(0.0f, 0.0f);
    }

    /*
     * 
     * MISC. FUNCTIONS
     * 
     */

    float DegreesToRadians(float deg)
    {
        return (deg * Mathf.PI / 180.0f);
    }

    float RadiansToDegrees(float rad)
    {
        return (rad * 180.0f / Mathf.PI);
    }

    float GenerateNormalRandom(float mu, float sigma) /* Credit: https://gist.github.com/ciscoslot/d2d57b351e4852d5e03a30663fdcd87b */
    {
        float rand1 = Random.Range(0.0f, 1.0f);
        float rand2 = Random.Range(0.0f, 1.0f);

        float n = Mathf.Sqrt(-2.0f * Mathf.Log(rand1)) * Mathf.Cos((2.0f * Mathf.PI) * rand2);

        return (mu + sigma * n);
    }

}
