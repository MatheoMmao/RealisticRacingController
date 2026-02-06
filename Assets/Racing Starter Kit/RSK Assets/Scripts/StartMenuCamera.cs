using UnityEngine;
/// <summary>
/// disable main menu camera when playing a race
/// </summary>
namespace SpinMotion
{
    [RequireComponent(typeof(Camera))]
    public class StartMenuCamera : MonoBehaviour
    {
        public GameEvents gameEvents;
        private Camera startMenuCamera;
        private AudioListener audioListener;

        private void Awake()
        {
            gameEvents.OnClickPlayRaceEvent.AddListener(OnClickPlayRace);
            startMenuCamera = GetComponent<Camera>();
            audioListener = GetComponent<AudioListener>();
        }

        private void OnClickPlayRace()
        {
            startMenuCamera.enabled = false;
            audioListener.enabled = false;
        }
    }
}