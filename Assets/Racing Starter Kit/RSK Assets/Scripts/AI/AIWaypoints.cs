using System.Linq;
using UnityEngine;
/// <summary>
/// grab all Ai Waypoint boxes as children of this transform to add it to the AI Waypoint Runtime Set
/// </summary>
namespace SpinMotion
{
    public class AIWaypoints : MonoBehaviour
    {
        public GameEvents gameEvents;
        public AIWaypointSet aiWaypointSet;

        private void Start()
        {
            var waypoints = GetComponentsInChildren<Transform>().ToList();
            waypoints.RemoveAt(0); // unity adds parent transform so remove it, leave only child
            aiWaypointSet.Items.Clear();
            foreach (var waypoint in waypoints)
                if (waypoint.TryGetComponent<MeshRenderer>(out var meshRenderer))
                {
                    aiWaypointSet.Add(new AIWaypoint { aiWaypointTransform = waypoint, aiWaypointMeshRenderer = meshRenderer });
                    meshRenderer.enabled = false;
                }
        }
    }

    [System.Serializable]
    public class AIWaypoint
    {
        public Transform aiWaypointTransform;
        public MeshRenderer aiWaypointMeshRenderer;
    }
}