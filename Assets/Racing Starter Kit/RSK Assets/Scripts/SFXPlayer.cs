using System.Collections.Generic;
using UnityEngine;
/// <summary>
/// only used on pre-race 3,2,1,go countdown but you can add more sfx easily with this system
/// </summary>
namespace SpinMotion
{
    public enum SFXType
    {
        GetReadySFX,
        GoSFX
    }

    public class SFXPlayer : MonoBehaviour
    {
        public GameEvents gameEvents;
        public List<AudioSource> sfxs = new();

        private void Awake()
        {
            gameEvents.PlayAudioSfxEvent.AddListener(OnPlayAudioSfx);
        }

        private void OnPlayAudioSfx(SFXType sfxType)
        {
            switch(sfxType)
            {
                case SFXType.GetReadySFX:
                    sfxs[0].Play();
                    break;
                case SFXType.GoSFX:
                    sfxs[1].Play();
                    break;
            }
        }
    }
}