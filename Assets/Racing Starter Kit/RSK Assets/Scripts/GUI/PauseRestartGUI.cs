using UnityEngine;
using UnityEngine.UI;
/// <summary>
/// display pause menu options and manage them (restart game, restart race, resume race, pause/unpause)
/// </summary>
namespace SpinMotion
{
    public class PauseRestartGUI : MonoBehaviour
    {
        public GameEvents gameEvents;
        public GameObject pauseMenuContainer;
        public Button pauseButton;
        public Button resumeButton;
        public Button restartButton;
        public Button quitButton;

        private void Awake()
        {
            gameEvents.ToggleGamePauseEvent.AddListener(OnToggleGamePause);

            pauseButton.onClick.AddListener(OnClickPause);
            resumeButton.onClick.AddListener(OnClickResume);
            restartButton.onClick.AddListener(OnClickRestartRace);
            quitButton.onClick.AddListener(OnClickQuitRace);
        }

        private void OnToggleGamePause(bool isPaused)
        {
            pauseMenuContainer.SetActive(isPaused);
        }

        private void OnClickPause()
        {
            
            gameEvents.OnClickTogglePauseEvent.Invoke();
        }

        private void OnClickResume()
        {
            gameEvents.OnClickTogglePauseEvent.Invoke();
        }

        private void OnClickRestartRace()
        {
            gameEvents.OnClickTogglePauseEvent.Invoke();
            gameEvents.OnClickRestartRaceEvent.Invoke();
        }

        private void OnClickQuitRace()
        {
            gameEvents.OnClickTogglePauseEvent.Invoke();
            gameEvents.OnClickRestartGameEvent.Invoke();
        }
    }
}