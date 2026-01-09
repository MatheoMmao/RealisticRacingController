using System.Diagnostics.CodeAnalysis;
using Unity.Properties;
using Unity.VisualScripting;
using UnityEngine;

public class CarPhysics : MonoBehaviour
{
    [Header("Outside Input")]
    [SerializeField]
    [Tooltip("Air density in km/m^3")]
    float airDensity = 1.29f;

    [SerializeField][Range(0f, 1f)] float throttleValue;

    [Header("Engine Input")]
    [SerializeField]
    float minEngineTorque = 400;

    [SerializeField] float maxEngineTorque = 500;
    [SerializeField] float minEngineRPM = 1000;
    [SerializeField] float maxEngineRPM = 6000;

    [SerializeField]
    AnimationCurve RPMToTorqueCurve = new AnimationCurve(new Keyframe(0, 0, 0, 0.5f),
        new Keyframe(0.8f, 1, 0, 0), new Keyframe(1, 0, -4, 0));

    [SerializeField] float RPM;

    [Header("Car Input")][SerializeField] float engineForce;

    [SerializeField]
    [Tooltip("Coefficient of friction depend on the car shape")]
    float frictionCoef = 0.3f;

    [SerializeField]
    [Tooltip("Frontal area of the car in m^2")]
    float frontalArea = 2.2f;

    [SerializeField]
    [Tooltip("Must be approx. 30 times the drag Resistance (0.5 * frictionCoef * frontalArea * airDensity) * 30")]
    float rollResistance = 12.8f;

    [SerializeField]
    [Tooltip("The mass of the car in kg")]
    float mass = 100;

    [SerializeField] float gearRatio = 2.66f;
    [SerializeField] float differentialRatio = 3.42f;
    [SerializeField][Range(0f, 1f)] float transmissionEfficiency = 0.7f;
    [SerializeField] Vector3 centerOfGravity;

    [Header("Wheels")]
    [SerializeField] MyWheelCollider[] wheelColliders;

    [SerializeField] WheelControl[] wheelControls;
    [SerializeField] float wheelFrictionCoef = 1.5f;

    [Header("Forces")][SerializeField] Vector2 tractionForce;
    [SerializeField] Vector2 dragForce;
    [SerializeField] Vector2 rollResistanceForce;
    [SerializeField] Vector2 longitudinalForce;

    [Header("Output")][SerializeField] Vector2 velocity;
    [SerializeField] Vector2 acceleration;
    [SerializeField] float engineTorque;
    [SerializeField] float wheelTorque;
    [SerializeField] float[] wheelMaxFriction;

    Rigidbody rb;
    Vector2 wheelBase;
    [SerializeField] float[] wheelWeightSupported;
    [SerializeField] float wheelsRPM;
    [SerializeField] float wheelsSpeed;

    private void Start()
    {
        rb = GetComponent<Rigidbody>();
        //wheelWeightSupported = new float[wheelControls.Length];
        //wheelMaxFriction = new float[wheelControls.Length];

        //if (wheelControls.Length > 0)
        //{
        //    Vector3 frontMaxWheelPos = wheelControls[0].transform.position;
        //    Vector3 rearMaxWheelPos = wheelControls[0].transform.position;
        //    Vector3 leftMaxWheelPos = wheelControls[0].transform.position;
        //    Vector3 rightMaxWheelPos = wheelControls[0].transform.position;
        //    foreach (var control in wheelControls)
        //    {
        //        if (control.transform.position.z > frontMaxWheelPos.z)
        //        {
        //            frontMaxWheelPos = control.transform.position;
        //        }

        //        if (control.transform.position.z < rearMaxWheelPos.z)
        //        {
        //            rearMaxWheelPos = control.transform.position;
        //        }

        //        if (control.transform.position.x > rightMaxWheelPos.x)
        //        {
        //            rightMaxWheelPos = control.transform.position;
        //        }

        //        if (control.transform.position.x < leftMaxWheelPos.x)
        //        {
        //            leftMaxWheelPos = control.transform.position;
        //        }
        //    }

        //    wheelBase = new Vector2(frontMaxWheelPos.z - rearMaxWheelPos.z, rightMaxWheelPos.x - leftMaxWheelPos.x);
        //}
    }

    private void FixedUpdate()
    {
        wheelsRPM = CalculateWheelRPM();
        RPM = CalculateEngineRPM(wheelsRPM, gearRatio, differentialRatio);
        RPM = Mathf.Clamp(RPM, minEngineRPM, maxEngineRPM);
        engineTorque = GetEngineTorque(RPM, throttleValue);


        //for (int i = 0; i < wheelControls.Length; i++)
        //{
        //    Vector2 wheelPos = new Vector2(wheelControls[i].transform.localPosition.z - centerOfGravity.z,
        //        wheelControls[i].transform.localPosition.x - centerOfGravity.x);
        //    wheelWeightSupported[i] =
        //        CalculateWheelWeightSupported(wheelPos, wheelBase, mass, centerOfGravity.y, acceleration);
        //    wheelMaxFriction[i] = CalculateWheelMaxFriction(wheelFrictionCoef, wheelWeightSupported[i]);

        //    if (wheelControls[i].motorized)
        //    {
        //        wheelControls[i].WheelCollider.motorTorque = wheelTorque;
        //    }
        //}

        //wheelsSpeed = CalculateWheelSpeed(wheelsRPM);

        ////tractionForce = CalculateTraction(velocity.normalized, engineTorque);
        dragForce = CalculateDragForce(frictionCoef, frontalArea, airDensity, velocity);

        rb.AddForce(dragForce);
        //rollResistanceForce = CalculateRollResistanceForce(rollResistance, velocity);
        //longitudinalForce = CalculateLongitudinalForce(tractionForce, dragForce, rollResistanceForce);

        //acceleration = CalculateAcceleration(longitudinalForce, mass);

        //velocity += Time.fixedDeltaTime * acceleration;

        foreach (var collider in wheelColliders)
        {
            wheelTorque = CalculateWheelTorque(engineTorque, gearRatio, differentialRatio, transmissionEfficiency,
                collider.GetWheelRadius());
            collider.SetWheelTorque(wheelTorque);
        }

        Debug.Log(rb.linearVelocity.magnitude);
    }

    float CalculateEngineRPM(float wheelRPM, float gearRatio, float differentialRatio)
    {
        return wheelRPM * gearRatio * differentialRatio;
    }

    float GetEngineTorque(float RPM, float throttleValue)
    {
        return throttleValue *
               (RPMToTorqueCurve.Evaluate((RPM - minEngineRPM) / (maxEngineRPM - minEngineRPM)) *
                   (maxEngineTorque - minEngineTorque) + minEngineTorque);
    }

    float CalculateWheelTorque(float torqueEngine, float gearRatio, float differentialRatio,
        float transmissionEfficiency, float wheelRadius)
    {
        return torqueEngine * gearRatio * differentialRatio * transmissionEfficiency ;
    }

    float CalculateWheelRPM()
    {
        float sum = 0;
        int nbDrivenWheel = 0;
        foreach (MyWheelCollider wheel in wheelColliders)
        {
            if (wheel.IsMotorized())
            {
                sum += wheel.GetRPM();
                nbDrivenWheel++;
            }
        }

        return sum / nbDrivenWheel;
    }

    Vector2 CalculateTraction(Vector2 direction, float engineForce)
    {
        // Temp to test
        if (direction.magnitude == 0)
        {
            direction.x += 0.00001f;
        }

        return direction * engineForce;
    }

    Vector2 CalculateDragForce(float frictionCoef, float frontalArea, float airDensity, Vector2 velocity)
    {
        return -0.5f * frictionCoef * frontalArea * airDensity * velocity.magnitude * velocity;
    }

    Vector2 CalculateRollResistanceForce(float rollResistance, Vector2 velocity)
    {
        return -0.5f * 0.3f * 2.2f * 1.29f * 30 * velocity;
    }

    Vector2 CalculateLongitudinalForce(Vector2 tractionForce, Vector2 dragForce, Vector2 rollResistanceForce)
    {
        return tractionForce + dragForce + rollResistanceForce;
    }

    Vector2 CalculateAcceleration(Vector2 netForce, float mass)
    {
        return netForce / mass;
    }

    float CalculateWheelWeightSupported(Vector2 wheelPos, Vector2 wheelBase, float carMass, float heightCG,
        Vector2 acceleration)
    {
        float coefWheelPos = -(wheelPos.x != 0 ? wheelPos.x / Mathf.Abs(wheelPos.x) : 1);

        float frontBackweight = (Mathf.Abs(wheelPos.x) / wheelBase.x) * carMass +
                                (heightCG / wheelBase.x) * carMass * acceleration.x * coefWheelPos;

        coefWheelPos = -(wheelPos.y != 0 ? wheelPos.y / Mathf.Abs(wheelPos.y) : 1);

        return (Mathf.Abs(wheelPos.y) / wheelBase.y) * frontBackweight * 9.81f +
               (heightCG / wheelBase.y) * carMass * acceleration.y * coefWheelPos;
    }

    float CalculateWheelMaxFriction(float frictionCoef, float weightSupported)
    {
        return frictionCoef * weightSupported;
    }

    float CalculateWheelSpeed(float wheelRPM)
    {
        return wheelRPM / 60 * 2.14f * 3.6f;
    }
}