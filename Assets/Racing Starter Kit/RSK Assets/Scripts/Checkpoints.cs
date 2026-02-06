using System.Collections.Generic;
using System.Linq;
using UnityEngine;
/// <summary>
/// gather all provided checkpoints on the public list to track them in the race for all cars
/// </summary>
namespace SpinMotion
{
    public class Checkpoints : MonoBehaviour
    {
        public List<Checkpoint> checkpoints = new();

        private void Start()
        {
            checkpoints.Clear();
            checkpoints.AddRange(GetComponentsInChildren<Checkpoint>().ToList());
            for (int i = 0; i < checkpoints.Count; i++)
            {
                checkpoints[i].SetCheckpointNumber(i + 1);
                checkpoints[i].gameObject.name = checkpoints[i].gameObject.name + (i + 1).ToString();
                //unparent checkpoints so we can get precise transforms values for RealTimeRacePositions class
                checkpoints[i].transform.SetParent(null);
            }
            RaceData.CheckpointsCount = checkpoints.Count;
        }
    }
}