using UnityEngine;
/// <summary>
/// cross platform input Unity standard assets code
/// </summary>
namespace SpinMotion
{
    public class MobileButtonHandler : MonoBehaviour
    {
        public string Name;

        void OnEnable()
        {

        }

        public void SetDownState()
        {
            MobileInputManager.SetButtonDown(Name);
        }


        public void SetUpState()
        {
            MobileInputManager.SetButtonUp(Name);
        }


        public void SetAxisPositiveState()
        {
            MobileInputManager.SetAxisPositive(Name);
        }


        public void SetAxisNeutralState()
        {
            MobileInputManager.SetAxisZero(Name);
        }


        public void SetAxisNegativeState()
        {
            MobileInputManager.SetAxisNegative(Name);
        }

        public void Update()
        {

        }
    }
}
