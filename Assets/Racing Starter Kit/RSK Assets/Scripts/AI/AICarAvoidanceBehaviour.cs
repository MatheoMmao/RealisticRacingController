using UnityEngine;
using System.Collections;
using System.Linq;
/// <summary>
/// This script detects the AI car speed to see if the car it’s stuck so it will start going reverse for 1 second to get back on track
/// Also, it uses a Box collider with IsTrigger option checked to see if an AI Car or the player car is in front of this car to brake
/// </summary>
namespace SpinMotion
{
    [RequireComponent(typeof(Collider))]
    public class AICarAvoidanceBehaviour : MonoBehaviour
    {
        public GameEvents gameEvents;
        public CarController aiCarController;
        public Rigidbody aiCarRigidbody;
        
        public float slowSpeedThreshold = 0.25f;
        public float reducedSpeed = 15f;

        private WheelCollider[] allWheelColliders;
        private Coroutine checkReverseCoroutine;
        private float aiCarSpeed;
        private bool checkReverse, startReverse;
        private float normalSteering, normalTopSpeed;
        
        private void Awake()
        {
            gameEvents.RaceStartedEvent.AddListener(OnRaceStarted);
            gameEvents.RestartRaceEvent.AddListener(OnRestartRace);
            gameEvents.RaceFinishedEvent.AddListener(OnRaceFinished);

            allWheelColliders = aiCarController.transform.GetComponentsInChildren<WheelCollider>();
            if (allWheelColliders.Count() == 0)
                Debug.LogWarning("No wheel colliders were found for AI car avoidance detection");
            
            normalSteering = aiCarController.m_MaximumSteerAngle;
            normalTopSpeed = aiCarController.m_Topspeed;
        }

        private void OnRaceStarted()
        {
            StopCheckReverse();
            checkReverseCoroutine = StartCoroutine(CheckReverseCoroutine());
        }

        private void OnRestartRace()
        {
            StopCheckReverse();
        }

        private void OnRaceFinished(RaceFinishType raceFinishType)
        {
            StopCheckReverse();
        }

        private void StopCheckReverse()
        {
            if (checkReverseCoroutine != null)
                StopCoroutine(checkReverseCoroutine);

            checkReverse = false;
        }

        private IEnumerator CheckReverseCoroutine()
        {
            // the script will wait 3 seconds so the AI car can get enough speed after starting the race
            // otherwise if you don't wait, the car always start at 0 speed so it will trigger the reverse coroutine
            yield return new WaitForSeconds(3); //now that 3 seconds passed we can check if the AI car needs to use the 1 second reverse
            checkReverse = true;
        }

        private void Update()
        {
            aiCarSpeed = aiCarRigidbody.linearVelocity.magnitude;

            if (aiCarSpeed < slowSpeedThreshold && checkReverse)
            {
                checkReverse = false;
                startReverse = true;
            }

            if (startReverse)
            {
                startReverse = false;
                aiCarController.m_FullTorqueOverAllWheels *= -1; // reverse enabled
                aiCarController.m_MaximumSteerAngle = 0; // block turns to back up in reverse straight
                StartCoroutine(ReverseCoroutine());
            }
        }

        private IEnumerator ReverseCoroutine()
        {
            aiCarController.forceBraking = -1;// failsafe for glitchy wheel colliders: (prevents getting stalled)
            yield return new WaitForSeconds(1);
            aiCarController.forceBraking = 0;// unapply failsafe after 1s, resume normal reverse coroutine
            aiCarController.m_FullTorqueOverAllWheels *= -1;
            // after one second, the car will be able to turn again (we don't want to turn while reversing, go straight backing up)
            yield return new WaitForSeconds(1);
            aiCarController.m_MaximumSteerAngle = normalSteering;
            StopCheckReverse();
            checkReverseCoroutine = StartCoroutine(CheckReverseCoroutine());
        }
        
        // set "Player" layer on player car colliders prefabs and "AIPlayer" for AI car colliders
        void OnTriggerEnter(Collider collider)
        {
            if (collider.gameObject.layer == LayerMask.NameToLayer("Player")
                || collider.gameObject.layer == LayerMask.NameToLayer("AIPlayer"))
            {
                aiCarController.m_Topspeed = reducedSpeed;
            }
        }
        
        void OnTriggerExit(Collider collider)
        {
            if (collider.gameObject.layer == LayerMask.NameToLayer("Player")
                || collider.gameObject.layer == LayerMask.NameToLayer("AIPlayer"))
            {
                aiCarController.m_Topspeed = normalTopSpeed;
            }
        }
    }
}