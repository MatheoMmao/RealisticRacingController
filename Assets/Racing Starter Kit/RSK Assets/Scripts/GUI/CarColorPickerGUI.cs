using UnityEngine;
using UnityEngine.UI;
/// <summary>
/// player can select from 3 sliders the color of the car, CarColor script will apply it
/// </summary>
namespace SpinMotion
{
    public class CarColorPickerGUI : MonoBehaviour
    {
        public Image colorPreviewImage;
        [Header("UI Sliders")]
        public Slider hueSlider;
        public Slider saturationSlider;
        public Slider brightnessSlider;
        
        public static Color SelectedCarColor;

        private void Awake()
        {
            hueSlider.onValueChanged.AddListener(delegate { UpdateColor(); });
            saturationSlider.onValueChanged.AddListener(delegate { UpdateColor(); });
            brightnessSlider.onValueChanged.AddListener(delegate { UpdateColor(); });

            UpdateColor();
        }

        private void UpdateColor()
        {
            // Create an RGB color from the HSV values from the Sliders
            var rgbColor = Color.HSVToRGB(hueSlider.value, saturationSlider.value, brightnessSlider.value);
            colorPreviewImage.color = rgbColor;
            SelectedCarColor = rgbColor;
        }
    }
}
