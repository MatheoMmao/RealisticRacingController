using TMPro;
using UnityEngine;
/// <summary>
/// display player current position (calculated by RealTimeRacePositions script)
/// </summary>
namespace SpinMotion
{
    public class RacePositionGUI : MonoBehaviour
    {
        public RaceManagerItem raceManager;
        public RealTimeRacePositionsItem realTimeRacePositions;
        public TMP_Text racePositionTMP;
        public string racePositionNotAvailableText = "--";

        private void Update()
        {
            if (raceManager.Item.IsRaceInProgress())
            {
                var playerRacePos = realTimeRacePositions.Item.GetPlayerRacePosition(0);
                racePositionTMP.text = playerRacePos + CardinalPos(playerRacePos) + "\n" + " Place";
            }
            else
            {
                racePositionTMP.text = racePositionNotAvailableText;
            }
        }

        public static string CardinalPos(int i)
        {
            if (i % 100 >= 11 && i % 100 <= 13)
            {
                return "th";
            }

            switch (i % 10)
            {
                case 1: return "st"; // 1st, 21st, 31st...
                case 2: return "nd"; // 2nd, 22nd, 32nd...
                case 3: return "rd"; // 3rd, 23rd, 33rd...
                default: return "th"; // 4th, 5th, ..., 11th, 12th...
            }
        }
    }
}