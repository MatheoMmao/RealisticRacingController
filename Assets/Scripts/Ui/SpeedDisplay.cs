using TMPro;
using UnityEngine;

public class SpeedDisplay : MonoBehaviour
{
    TMP_Text text;

    RealisticCarController car;

    private void Start()
    {
        text = GetComponent<TMP_Text>();
        car = FindAnyObjectByType<RealisticCarController>();
    }

    private void Update()
    {
    }
}
