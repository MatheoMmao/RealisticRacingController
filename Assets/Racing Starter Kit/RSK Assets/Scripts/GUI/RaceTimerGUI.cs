using TMPro;
using UnityEngine;
/// <summary>
/// manage race timer (play, pause and restart) as requested by events
/// </summary>
namespace SpinMotion
{
    public class RaceTimerGUI : MonoBehaviour
    {
        public GameEvents gameEvents;
        public RaceManagerItem raceManager;
        public TMP_Text timerTMP;

        private float remainingTime;
        private bool isRunning = false;

        private void Start()
        {
            if (!raceManager.Item.isTimedRace)
            {
                timerTMP.gameObject.SetActive(false);
                gameObject.SetActive(false);
                return;
            }
            remainingTime = raceManager.Item.raceTimerSeconds;
            
            gameEvents.RaceStartedEvent.AddListener(OnRaceStarted);
            gameEvents.ToggleGamePauseEvent.AddListener(OnToggleGamePause);
            gameEvents.RestartRaceEvent.AddListener(OnRestartRace);
            gameEvents.RaceFinishedEvent.AddListener(OnRaceFinished);
        }

        private void OnRaceStarted()
        {
            if (remainingTime > 0)
                isRunning = true;
        }

        private void OnToggleGamePause(bool isPaused)
        {
            if (!isPaused)
            {
                if (remainingTime > 0)
                    isRunning = true;
            }
            else
            {
                isRunning = false;
            }
        }

        private void Update()
        {
            if (isRunning && remainingTime > 0)
            {
                remainingTime -= Time.deltaTime;
                timerTMP.text = ((int)remainingTime).ToString();
                if (remainingTime <= 0)
                {
                    remainingTime = 0;
                    isRunning = false;
                    gameEvents.RaceTimeoutEvent.Invoke();
                }
            }
        }

        private void OnRestartRace()
        {
            remainingTime = raceManager.Item.raceTimerSeconds;
            isRunning = false;
            timerTMP.text = string.Empty;
        }

        private void OnRaceFinished(RaceFinishType raceFinishType)
        {
            isRunning = false;
        }
    }
}
