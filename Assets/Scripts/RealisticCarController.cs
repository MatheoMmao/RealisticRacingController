using Unity.VisualScripting;
using UnityEngine;

public class RealisticCarController : MonoBehaviour
{


    CarPhysics carPhysics;

    private void Start()
    {
        carPhysics = GetComponent<CarPhysics>();
    }

    private void FixedUpdate()
    {

    }
}
