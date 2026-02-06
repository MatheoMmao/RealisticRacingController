using System.Collections;
using System.Collections.Generic;
using UnityEngine;
/// <summary>
/// This script will move the AI waypoint tracker when the AI car triggers a point. The AI car will always follow the tracker
/// and the tracker will take the position of the next point that is placed on the racetrack
/// in this way the AI car will roam around the track all the time
/// (enable AI tracker & AI waypoints mesh renderer to understand better)
/// </summary>
namespace SpinMotion
{
    public class AIWaypointTracker : MonoBehaviour
    {
        public GameEvents gameEvents;
        public AIWaypointSet aiWaypointSet;
        public Vector3 trackerScale;

        private List<AIWaypoint> aiWaypoints = new();
        private Transform currentWaypoint;
        private int currentIndex;
        private Collider aiCarCollider;
        private Collider waypointTrackerCollider;
        
        private void Awake()
        {
            gameEvents.RestartRaceEvent.AddListener(OnRestartRace);
            waypointTrackerCollider = GetComponent<BoxCollider>();
            transform.localScale = trackerScale;
            ResetTracker();
        }

        private void OnRestartRace()
        {
            ResetTracker();
        }

        private void ResetTracker()
        {
            this.aiWaypoints = aiWaypointSet.Items;
            currentIndex = 0;
            UpdateTrackerPosition();
        }

        public void SetupAICarCollider(Collider aiCarCollider)
        {
            this.aiCarCollider = aiCarCollider;
        }

        IEnumerator OnTriggerEnter(Collider collider)
        {
            if (collider.GetInstanceID() == aiCarCollider.GetInstanceID())
            {
                waypointTrackerCollider.enabled = false;
                currentIndex++;
                if (currentIndex >= aiWaypoints.Count)
                    currentIndex = 0;

                UpdateTrackerPosition();
                yield return new WaitForSeconds(0.1f);
                waypointTrackerCollider.enabled = true;
            }
        }

        private void UpdateTrackerPosition()
        {
            currentWaypoint = aiWaypoints[currentIndex].aiWaypointTransform;
            this.transform.position = currentWaypoint.position;
        }
    }
}