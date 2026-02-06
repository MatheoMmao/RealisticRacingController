using UnityEngine;
/// <summary>
/// setup the game state to begin, restart and finish a race
/// and manage current race data and flags
/// </summary>
namespace SpinMotion
{
    public class RaceManager : MonoBehaviour
    {
        public RaceManagerItem raceManagerRuntimeItem;
        public GameEvents gameEvents;
        [Header("Race Timer Settings:")]
        public bool isTimedRace;
        public int raceTimerSeconds;

        public bool IsRaceInProgress() { return isRaceInProgress; }
        private bool isRaceInProgress;

        private void Awake()
        {
            raceManagerRuntimeItem.Set(this);

            gameEvents.RaceStartedEvent.AddListener(OnRaceStarted);
            gameEvents.RaceFinishedEvent.AddListener(OnRaceFinished);
            // gui buttons:
            gameEvents.OnClickPlayRaceEvent.AddListener(OnPlayRace);
            gameEvents.OnClickRestartRaceEvent.AddListener(OnRestartRace);
        }

        private void OnPlayRace()
        {
            gameEvents.SpawnPlayersEvent.Invoke();
            gameEvents.PreRaceUpdateGuiEvent.Invoke();
            gameEvents.PlayPreRaceCountdownEvent.Invoke();
            gameEvents.ChangeToRaceCamerasEvent.Invoke();
        }

        // 3,2,1 countdown ended:
        private void OnRaceStarted()
        {
            isRaceInProgress = true;
        }

        private void OnRaceFinished(RaceFinishType raceFinishType)
        {
            isRaceInProgress = false;
        }

        private void OnRestartRace()
        {
            isRaceInProgress = false;
            
            gameEvents.RestartRaceEvent.Invoke();
            gameEvents.PreRaceUpdateGuiEvent.Invoke();
            gameEvents.PlayPreRaceCountdownEvent.Invoke();
        }
    }

    public class RaceData
    {
        public static int CheckpointsCount;
        public static int AiBotsSelected;
        public static int LapsSelected;
    }
}