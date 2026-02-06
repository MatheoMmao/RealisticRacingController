using System.Collections.Generic;
using UnityEngine;
/// <summary>
/// use the provided key on input settings to toggle between 3 cameras, add or replace with your own
/// </summary>
namespace SpinMotion
{
    public class PlayerChangeCameras : MonoBehaviour
    {
        public GameEvents gameEvents;
        public List<PlayerCarCameraController> playerCarCameras = new();
        [Header("Edit > Project Settings > Input - Set your axes name here:")]
        public string changeCameraInput;
        
        private int currentCameraIndex;

        private void Awake()
        {
            foreach (var camera in playerCarCameras)
                camera.gameObject.SetActive(false);

            gameEvents.ChangeToRaceCamerasEvent.AddListener(OnChangeToRaceCameras);
        }

        private void OnChangeToRaceCameras()
        {
            playerCarCameras[currentCameraIndex].gameObject.SetActive(true);
        }

        private void Update()
        {
            // you can also use KeyCode but GetButtonDown also works for UI buttons if mobile is intended
            if (Input.GetButtonDown(changeCameraInput))
            {
                currentCameraIndex++;
                if (currentCameraIndex == playerCarCameras.Count)
                    currentCameraIndex = 0;
                
                // disable GameObject instead of Camera to avoid having multiple AudioListeners
                for (int i = 0; i < playerCarCameras.Count; i++)
                {
                    if (i == currentCameraIndex)
                    {
                        playerCarCameras[i].gameObject.SetActive(true);
                    }
                    else
                    {
                        playerCarCameras[i].gameObject.SetActive(false);
                    }
                }
            }
        }
    }
}