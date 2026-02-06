using UnityEngine;
using System.Collections;
/// <summary>
/// use two raycasts, one at each side of the car to detect obstacles and steer the other way
/// </summary>
namespace SpinMotion
{
    public class AICarAvoidanceSmartSteering : MonoBehaviour
    {
        public CarController aiCarController;
        public float raycastDistance = 2f;
        public float smartSteeringRaycastCheckDelay = 0.5f;
        public Vector3 leftRaycastOffset;
        public Vector3 rightRaycastOffset;

        private bool canCheckRaycast = true;
        private Vector3 leftRayOrigin;
        private Vector3 rightRayOrigin;

        private void Update()
        {
            if (canCheckRaycast)
            {
                StartCoroutine(CheckSmartSteering());
            }
        }

        private IEnumerator CheckSmartSteering()
        {
            canCheckRaycast = false;
            bool leftHit = Physics.Raycast(CalculateLeftRayOrigin(), transform.forward, raycastDistance, LayerMask.GetMask("Player"));
            bool rightHit = Physics.Raycast(CalculateRightRayOrigin(), transform.forward, raycastDistance, LayerMask.GetMask("Player"));
            
            if (leftHit && !rightHit)
            {
                aiCarController.forceSteering = -1;
            }
            else if (rightHit && !leftHit)
            {
                aiCarController.forceSteering = 1;
            }
            else
            {
                aiCarController.forceSteering = 0;
            }

            yield return new WaitForSeconds(smartSteeringRaycastCheckDelay);
            canCheckRaycast = true;
        }

        private void OnDrawGizmos()
        {
            Gizmos.color = Color.red;
            Gizmos.DrawLine(CalculateLeftRayOrigin(), leftRayOrigin + transform.forward * raycastDistance);
            Gizmos.DrawLine(CalculateRightRayOrigin(), rightRayOrigin + transform.forward * raycastDistance);
        }

        private Vector3 CalculateLeftRayOrigin()
        {
            return leftRayOrigin = transform.position + leftRaycastOffset - transform.right * 0.5f;
        }

        private Vector3 CalculateRightRayOrigin()
        {
            return rightRayOrigin = transform.position + rightRaycastOffset + transform.right * 0.5f;
        }
    }
}