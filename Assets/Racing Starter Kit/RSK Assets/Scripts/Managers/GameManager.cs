using UnityEngine;
using UnityEngine.SceneManagement;
/// <summary>
/// manage pause and fast-forward debug states, also reloads the scene if requested by events
/// </summary>
namespace SpinMotion
{
    public class GameManager : MonoBehaviour
    {
        public GameEvents gameEvents;
        public RaceManagerItem raceManager;
        private bool paused;

        private void Awake()
        {
            // game render settings:
            Application.targetFrameRate = 60;
            QualitySettings.vSyncCount = 0;
            // gui buttons:
            gameEvents.OnClickRestartGameEvent.AddListener(OnRestartGame);
            gameEvents.OnClickTogglePauseEvent.AddListener(OnTogglePause);
        }

        private void OnRestartGame()
        {
            SceneManager.LoadScene(SceneManager.GetActiveScene().buildIndex);
        }

        private void Update()
        {
            if (Input.GetKeyDown(KeyCode.Escape))
            {
                TogglePause();
            }
            
            if (Input.GetKeyDown(KeyCode.Tab))
            {
                if (Time.timeScale == 1)
                    Time.timeScale = 3;
                else if (Time.timeScale == 3)
                    Time.timeScale = 1;
            }
        }

        private void TogglePause()
        {
            OnTogglePause();
        }

        private void OnTogglePause()
        {
            if (!raceManager.Item.IsRaceInProgress()) return;
            
            paused = !paused;
            gameEvents.ToggleGamePauseEvent.Invoke(paused);
            AudioListener.volume = paused ? 0f: 1f;
            Time.timeScale = paused ? 0f: 1f;
        }
    }
}