using UnityEditor;
using UnityEngine;
using UnityEngine.InputSystem.HID;

public class MyWheelCollider : MonoBehaviour
{
    [Header("New")]
    [SerializeField] bool steerable = false;
    [SerializeField] bool motorized = false;

    [Header("Old")]
    [Header("Suspension")]
    [SerializeField] private float minSpringLength = 0.05f; // Recomended to not put 0 as the min distance
    [SerializeField] private float springLength = 1f;
    [SerializeField, Range(0f, 1f)] private float restRatio = 0.5f;
    [SerializeField] private float springStrength = 30000f;
    [SerializeField] private float damperStrength = 4500f;
    [SerializeField] private float wheelMass = 30f;

    [Header("Wheel")]
    [SerializeField] private float wheelRadius = 0.25f;
    [SerializeField] private Transform wheelTransform; // Visual wheel

    [Header("Vehicle")]
    [SerializeField] private Rigidbody carRigidbody;

    private float restLength;
    private float currentLength;
    private float springVelocity;

    // Store visual wheel’s rest local position
    private Vector3 wheelVisualStartLocalPos;

    public bool debugWheel = false;
    bool grounded = false;

    private void Start()
    {
        restLength = springLength * restRatio;
        currentLength = restLength;
        springVelocity = 0f;

        if (wheelTransform != null)
            wheelVisualStartLocalPos = wheelTransform.localPosition; // save starting position
        else
            Debug.LogError("No wheelTransform (visual wheel) has been assigned!");
    }

    private void FixedUpdate()
    {
        UpdateWheelUpForce();

        if (grounded)
        {
            // Temporary steer and acceleration to test suspension
            if (steerable)
            {
                float dir = Input.GetAxis("Horizontal");

                carRigidbody.AddForceAtPosition(dir * transform.right * 1000, transform.position);
            }

            if (motorized)
            {
                float move = Input.GetAxis("Vertical");

                carRigidbody.AddForceAtPosition(move * transform.forward * 3000, transform.position);
            }

            Vector3 velocity = carRigidbody.GetPointVelocity(transform.position);

            float steeringVelocity = Vector3.Dot(transform.right, velocity);

            float acceleration = -(steeringVelocity * 0.5f) / Time.fixedDeltaTime;

            carRigidbody.AddForceAtPosition(transform.right * acceleration * wheelMass, transform.position);

            float carSpeed = Vector3.Dot(transform.forward, velocity);


            carRigidbody.AddForceAtPosition((-carSpeed * 0.5f) / Time.fixedDeltaTime * transform.forward, transform.position);
        }
    }


    void UpdateWheelUpForce()
    {
        Ray ray = new Ray(transform.position, -transform.up);

        if (Physics.Raycast(ray, out RaycastHit hit, springLength + wheelRadius))
        {
            grounded = true;

            float targetLength = hit.distance - wheelRadius;
            targetLength = Mathf.Clamp(targetLength, minSpringLength, springLength);

            float springCompression = restLength - targetLength;

            float springForce = springStrength * springCompression;
            float damperForce = damperStrength * (currentLength - targetLength) / Time.fixedDeltaTime;

            float suspensionForce = springForce + damperForce;

            if (suspensionForce > 0)
            {
                carRigidbody.AddForceAtPosition(suspensionForce * transform.up, hit.point);
            }

            // VISUAL WHEEL: follow ground

            if (hit.distance<wheelTransform.localPosition.y-wheelVisualStartLocalPos.y + (springLength / 2 + wheelRadius))
            wheelTransform.localPosition = wheelVisualStartLocalPos + (springLength/2 + wheelRadius - hit.distance) * Vector3.up;

            currentLength = targetLength;
        }
        else
        {
            grounded = false;

            // no suspension force in air

            // VISUAL WHEEL: drop to rest position
            Vector3 restPos = transform.position - transform.up * restLength;
            wheelTransform.position = restPos;
        }
    }

    private void OnDrawGizmos()
    {
        Handles.DrawWireDisc(transform.position - springLength * restRatio * transform.up,
                             transform.right, wheelRadius);

        Handles.DrawLine(transform.position, transform.position - springLength * transform.up);
        if (!debugWheel)
            return;

        Color color = Handles.color;
        Handles.color = Color.green;
        Handles.ArrowHandleCap(0, wheelTransform.position, Quaternion.Euler(-90, 0, 0), springVelocity + 1f, EventType.Repaint);

        Handles.color = Color.red;
        Handles.ArrowHandleCap(0, wheelTransform.position, Quaternion.Euler(0, 90, 0), 1, EventType.Repaint);

        Handles.color = Color.blue;
        Handles.ArrowHandleCap(0, wheelTransform.position, Quaternion.Euler(0, 0, 90), 1, EventType.Repaint);
        Handles.color = color;
    }
}
