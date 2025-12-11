using Unity.VisualScripting;
using UnityEngine;

public class Test : MonoBehaviour
{
    [SerializeField] Transform wheelTransform;

    [SerializeField] float velocity = 0;
    [SerializeField] float springLength = 1;
    [SerializeField] float restPos = 0.5f;


    [SerializeField] float springStrength = 10;
    [SerializeField] float dampingStrength = 4;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        float offset = restPos-wheelTransform.localPosition.y;

        float force = (springStrength * offset) - (velocity * dampingStrength);

        velocity += force * Time.fixedDeltaTime;

        wheelTransform.localPosition += Vector3.up * velocity * Time.fixedDeltaTime;
        if (wheelTransform.localPosition.y < 0)
        {
            wheelTransform.localPosition = new Vector3(wheelTransform.localPosition.x, 0, wheelTransform.localPosition.z);
        }
        if (wheelTransform.localPosition.y> springLength)
        {
            wheelTransform.localPosition = new Vector3(wheelTransform.localPosition.x, springLength, wheelTransform.localPosition.z);
        }
        Debug.Log($"{offset} {force}");
    }
}
