using UnityEngine;
/// <summary>
/// uses the base SelectorGUI and overrides QuantityUpdated to refresh RaceData on how many AI bots will the race have
/// </summary>
namespace SpinMotion
{
    public class BotSelectorGUI : SelectorGUI
    {
        [Header("Ensure having enough AI car spawn points")]
        [Range(0, 13)]public int maxBots;

        private void Awake()
        {
            min = 0;
            max = maxBots;
        }

        protected override void QuantityUpdated()
        {
            RaceData.AiBotsSelected = quantity;
        }
    }
}