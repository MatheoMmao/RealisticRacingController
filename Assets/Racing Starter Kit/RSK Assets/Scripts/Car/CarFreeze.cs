using UnityEngine;
/// <summary>
/// used during the pre-race 3,2,1,go countdown to avoid players moving
/// you can also disable CarUserControl but it won't let players burnout their wheels (boring)
/// </summary>
namespace SpinMotion
{
    public class CarFreeze : MonoBehaviour
    {
        public GameEvents gameEvents;
        public bool startFreezed = true;
        private Rigidbody carRigidbody;

        private void Awake()
        {
            gameEvents.ToggleCarFreezeEvent.AddListener(OnToggleCarFreeze);
            carRigidbody = GetComponent<Rigidbody>();
            if (startFreezed)
                OnToggleCarFreeze(true);
        }

        private void OnToggleCarFreeze(bool isFreezed)
        {
            if (isFreezed)
            {
                carRigidbody.constraints = RigidbodyConstraints.FreezePositionX | RigidbodyConstraints.FreezePositionZ | RigidbodyConstraints.FreezeRotation;
            }
            else
            {
                carRigidbody.constraints = RigidbodyConstraints.None;
            }
        }
    }
}