using UnityEngine;
/// <summary>
/// GetColliderTrigger() is used by AIWaypointTracker to compare InstanceID and see which AI car triggered that tracker
/// </summary>
namespace SpinMotion
{
    [RequireComponent(typeof(Collider))]
    public class AICarWaypointTrackerColliderTrigger : MonoBehaviour
    {
        private Collider colliderTrigger;
        public Collider GetColliderTrigger() { return colliderTrigger; }

        private void Awake()
        {
            colliderTrigger = GetComponent<Collider>();
        }
    }
}