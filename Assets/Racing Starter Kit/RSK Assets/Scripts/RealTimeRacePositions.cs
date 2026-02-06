using System.Collections.Generic;
using UnityEngine;
/// <summary>
/// measures a total score for each player
/// the score is laps done (1000 pts.) + checkpoints passed (100 pts.) + distance to the last checkpoint
/// Checkpoint script calculates distance from car to its transform and adds that value to the final score
/// the final score comparison results on the race positions (1st, 2nd, 3rd)
/// </summary>
namespace SpinMotion
{
    public class RealTimeRacePositions : MonoBehaviour
    {
        public RealTimeRacePositionsItem realTimeRacePositionsRuntimeItem;
        public GameEvents gameEvents;
        public RaceManagerItem raceManager;
        
        public List<CheckpointTracker> CarCheckpointTrackers = new();
        public List<double> DistanceFromCheckpointToCarTrackers = new();
        public List<int> CheckpointScores = new();
        public List<int> LapScores = new();
        public List<double> RacePositionTotalScores = new();

        private void Awake()
        {
            realTimeRacePositionsRuntimeItem.Set(this);
            gameEvents.PlayersCheckpointTrackersAssignedEvent.AddListener(OnPlayersCheckpointTrackersAssigned);
            gameEvents.RaceStartedEvent.AddListener(OnRaceStarted);
        }

        private void OnPlayersCheckpointTrackersAssigned(List<CheckpointTracker> playersCheckpointTrackers)
        {
            CarCheckpointTrackers.Clear();
            CarCheckpointTrackers.AddRange(playersCheckpointTrackers);
        }

        private void OnRaceStarted()
        {
            //clear previous races values when starting a new race
            DistanceFromCheckpointToCarTrackers.Clear();
            CheckpointScores.Clear();
            LapScores.Clear();
            RacePositionTotalScores.Clear();
            
            CarCheckpointTrackers.Sort((a, b) => a.GetCarRacePositionIndex().CompareTo(b.GetCarRacePositionIndex()));

            //here we add a checkpoint, distance, laps, score and player number position for all cars in the scene
            for (int i = 0; i < CarCheckpointTrackers.Count; i++)
            {
                DistanceFromCheckpointToCarTrackers.Add(0);
                CheckpointScores.Add(RaceData.CheckpointsCount);
                LapScores.Add(0);
                RacePositionTotalScores.Add(0);
            }
        }

        public void Update()
        {
            if (!raceManager.Item.IsRaceInProgress()) return;
            
            //final score comparison between all players:
            for (int i = 0; i < CarCheckpointTrackers.Count; i++)
            {
                RacePositionTotalScores[i] = DistanceFromCheckpointToCarTrackers[i] + (CheckpointScores[i] * 100) + (LapScores[i] * 10000);
            }
        }

        //get the maximum position comparing cars scores
        public int GetPlayerRacePosition(int nPlayer)
        {
            var maxPos = 1;

            for (int i = 0; i < RacePositionTotalScores.Count; i++)
            {
                if (RacePositionTotalScores[nPlayer] < RacePositionTotalScores[i])
                    maxPos = maxPos + 1;
            }

            return maxPos;
        }
    }
}