using TMPro;
using UnityEngine;
/// <summary>
/// display race timer and manage lap completion / race begin states
/// </summary>
namespace SpinMotion
{
    public class LapTimeGUI : MonoBehaviour
    {
        public GameEvents gameEvents;
        public RaceManagerItem raceManager;

        public TMP_Text minutesTMP;
        public TMP_Text secondsTMP;
        public TMP_Text millisecondsTMP;

        public int minuteCount, secondCount; // minutes and seconds are ints
        public float milliCount; // milliseconds are floats
        public string milliDisplay; // milliseconds are also stated as strings to show less numbers after the comma (,)

        private string initMinutesText;
        private string initSecondsText;
        private string initMillisecondsText;

        private void Awake()
        {
            gameEvents.PreRaceUpdateGuiEvent.AddListener(OnPreRaceUpdateGUI);
            gameEvents.LapCompletedEvent.AddListener(OnLapCompleted);

            initMinutesText = minutesTMP.text;
            initSecondsText = secondsTMP.text;
            initMillisecondsText = millisecondsTMP.text;
        }

        private void OnPreRaceUpdateGUI()
        {
            minuteCount = 0;
            secondCount = 0;
            milliCount = 0;
            milliDisplay = string.Empty;

            minutesTMP.text = initMinutesText;
            secondsTMP.text = initSecondsText;
            millisecondsTMP.text = initMillisecondsText;
        }

        private void OnLapCompleted()
        {
            minuteCount = 0;
            secondCount = 0;
            milliCount = 0;
        }

        private void Update()
        {
            if (!raceManager.Item.IsRaceInProgress()) return;

            milliCount += Time.deltaTime * 10;
            milliDisplay = milliCount.ToString("F0"); // here we set the milliseconds display numbers after the comma
            millisecondsTMP.text = "" + milliDisplay;

            if (milliCount >= 10)
            {
                milliCount = 0;
                secondCount += 1;
            }

            if (secondCount <= 9)
            {
                secondsTMP.text = "0" + secondCount + ".";
            }
            else
            {
                secondsTMP.text = "" + secondCount + ".";
            }

            if (secondCount >= 60)
            {
                secondCount = 0;
                minuteCount += 1;
            }

            if (minuteCount <= 9)
            {
                minutesTMP.text = "0" + minuteCount + ":";
            }
            else
            {
                minutesTMP.text = "" + minuteCount + ":";
            }
        }
    }
}