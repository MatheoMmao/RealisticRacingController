using TMPro;
using UnityEngine;
/// <summary>
/// display player current laps done (counted by RealTimeRacePositions script)
/// </summary>
namespace SpinMotion
{
    public class LapsGUI : MonoBehaviour
    {
        public GameEvents gameEvents;
        public RealTimeRacePositionsItem realTimeRacePositions;
        
        public TMP_Text currentLapTMP;
        public TMP_Text raceLapsTMP;
        
        private void Awake()
        {
            gameEvents.PreRaceUpdateGuiEvent.AddListener(OnPreRaceUpdateGUI);
            gameEvents.LapCompletedEvent.AddListener(OnLapCompleted);
        }

        private void OnPreRaceUpdateGUI()
        {
            raceLapsTMP.text = RaceData.LapsSelected.ToString();
            currentLapTMP.text = "0";
        }

        private void OnLapCompleted()
        {
            var lapsDone = realTimeRacePositions.Item.LapScores[0];
            if (lapsDone > RaceData.LapsSelected) return;

            currentLapTMP.text = lapsDone.ToString();
        }
    }
}