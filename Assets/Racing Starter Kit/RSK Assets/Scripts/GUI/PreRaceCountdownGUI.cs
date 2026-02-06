using System.Collections;
using TMPro;
using UnityEngine;
/// <summary>
/// display a 3,2,1,go countdown and change the race state to begin (race timer, unfreeze cars...)
/// </summary>
namespace SpinMotion
{
    public class PreRaceCountdownGUI : MonoBehaviour
    {
        public GameEvents gameEvents;
        public TMP_Text countdownTMP;
        private Coroutine raceStartCountdown;

        private void Awake()
        {
            gameEvents.PlayPreRaceCountdownEvent.AddListener(OnPlayRace);
        }

        private void OnPlayRace()
        {
            if (raceStartCountdown != null) { StopCoroutine(raceStartCountdown); }
            raceStartCountdown = StartCoroutine(RaceStartCountdown());
        }

        // it can also be done with an Animator and changing the text on the animation clip
        private IEnumerator RaceStartCountdown()
        {
            gameEvents.ToggleCarFreezeEvent.Invoke(true);
            yield return new WaitForSeconds(0.5f);
            countdownTMP.gameObject.SetActive(true);
            countdownTMP.text = "3";
            gameEvents.PlayAudioSfxEvent.Invoke(SFXType.GetReadySFX);

            yield return new WaitForSeconds(1);
            countdownTMP.text = "2";
            gameEvents.PlayAudioSfxEvent.Invoke(SFXType.GetReadySFX);

            yield return new WaitForSeconds(1);
            countdownTMP.text = "1";
            gameEvents.PlayAudioSfxEvent.Invoke(SFXType.GetReadySFX);
            
            yield return new WaitForSeconds(1);
            countdownTMP.text = "GO!";
            gameEvents.PlayAudioSfxEvent.Invoke(SFXType.GoSFX);
            gameEvents.ToggleCarFreezeEvent.Invoke(false);
            gameEvents.RaceStartedEvent.Invoke();

            yield return new WaitForSeconds(1);
            countdownTMP.gameObject.SetActive(false);
        }
    }
}