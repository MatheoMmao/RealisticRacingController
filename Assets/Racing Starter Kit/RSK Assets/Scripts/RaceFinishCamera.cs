using UnityEngine;
/// <summary>
/// enable this camera when finishing the race
/// </summary>
namespace SpinMotion
{
    [RequireComponent(typeof(Camera))]
    public class RaceFinishCamera : MonoBehaviour
    {
        public GameEvents gameEvents;
        private Camera finishCamera;
        private AudioListener audioListener;

        private void Awake()
        {
            finishCamera = GetComponent<Camera>();
            audioListener = GetComponent<AudioListener>();

            gameEvents.RestartRaceEvent.AddListener(OnRestartRace);
            gameEvents.RaceFinishedEvent.AddListener(OnRaceFinished);
        }

        private void OnRestartRace()
        {
            finishCamera.enabled = false;
            audioListener.enabled = false;
        }

        private void OnRaceFinished(RaceFinishType raceFinishType)
        {
            finishCamera.enabled = true;
            audioListener.enabled = true;
        }
    }
}