using UnityEngine;
using UnityEngine.UI;
/// <summary>
/// start race and switch main menu UI with race UI
/// </summary>
namespace SpinMotion
{
    public class MenuGUI : MonoBehaviour
    {
        public GameEvents gameEvents;
        public Button playRaceButton;
        public GameObject menuUI;
        public GameObject raceUI;

        private void Awake()
        {
            playRaceButton.onClick.AddListener(OnClickPlayRace);
        }

        private void OnClickPlayRace()
        {
            menuUI.SetActive(false);
            raceUI.SetActive(true);
            gameEvents.OnClickPlayRaceEvent.Invoke();
        }
    }
}