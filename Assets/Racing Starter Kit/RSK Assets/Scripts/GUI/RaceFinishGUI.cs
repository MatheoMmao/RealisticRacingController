using TMPro;
using UnityEngine;
using UnityEngine.UI;
/// <summary>
/// display on screen the final player status when completing the race (timeout or finish pos)
/// </summary>
namespace SpinMotion
{
    public class RaceFinishGUI : MonoBehaviour
    {
        public RealTimeRacePositionsItem realTimeRacePositions;

        public GameEvents gameEvents;
        public GameObject raceUI;
        public GameObject raceFinishPanel;
        public TMP_Text raceFinishTMP;
        public Button raceFinishRestartButton;
        public Button raceFinishContinueButton;

        private void Awake()
        {
            gameEvents.RaceFinishedEvent.AddListener(OnRaceFinished);
            raceFinishRestartButton.onClick.AddListener(OnClickRestart);
            raceFinishContinueButton.onClick.AddListener(OnClickContinue);
        }

        private void OnRaceFinished(RaceFinishType raceFinishType)
        {
            raceUI.SetActive(false);
            raceFinishPanel.SetActive(true);

            var playerRacePos = realTimeRacePositions.Item.GetPlayerRacePosition(0);
            switch(raceFinishType)
            {
                case RaceFinishType.Lose:
                    raceFinishTMP.text = playerRacePos + RacePositionGUI.CardinalPos(playerRacePos) + " Place" + "\n" + "Race Lost";
                    break;
                case RaceFinishType.Win:
                    raceFinishTMP.text = playerRacePos + RacePositionGUI.CardinalPos(playerRacePos) + " Place" + "\n" + "Race Won";
                    break;
                case RaceFinishType.Timeout:
                    raceFinishTMP.text = playerRacePos + RacePositionGUI.CardinalPos(playerRacePos) + " Place" + "\n" + "Race Finished by timeout";
                    break;
            }
        }

        private void OnClickRestart()
        {
            raceUI.SetActive(true);
            raceFinishPanel.SetActive(false);

            gameEvents.OnClickRestartRaceEvent.Invoke();
        }

        private void OnClickContinue()
        {
            gameEvents.OnClickRestartGameEvent.Invoke();
        }
    }
}