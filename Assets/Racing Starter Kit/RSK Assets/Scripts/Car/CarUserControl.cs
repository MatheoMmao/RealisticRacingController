using UnityEngine;
/// <summary>
/// send player input info to car controller
/// </summary>
namespace SpinMotion
{
    [RequireComponent(typeof (CarController))]
    public class CarUserControl : MonoBehaviour
    {
        private CarController m_Car; // the car controller we want to use

        private void Awake()
        {
            // get the car controller
            m_Car = GetComponent<CarController>();
        }

        private void FixedUpdate()
        {
            // pass the input to the car!
            float h = MobileInputManager.GetAxis("Horizontal");
            float v = MobileInputManager.GetAxis("Vertical");
#if !MOBILE_INPUT
            float handbrake = MobileInputManager.GetAxis("Jump");
            m_Car.Move(h, v, v, handbrake);
#else
            m_Car.Move(h, v, v, 0f);
#endif
        }
    }
}
