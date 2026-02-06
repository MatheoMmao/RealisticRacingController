using System.Collections.Generic;
using UnityEngine;
/// <summary>
/// spawn player and AI bots, manages race start position, and restart race pos/rotations
/// </summary>
namespace SpinMotion
{
    public enum PlayerType
    {
        Player,
        AI
    }
    
    public enum PlayerSpawnIndex
    {
        First,
        Last,
        Custom
    }

    public class PlayersSpawner : MonoBehaviour
    {
        public GameEvents gameEvents;
        public GameObject playerPrefab;
        public GameObject aiCarPrefab;
        public GameObject aiWaypointTrackerPrefab;
        public List<Transform> spawnPoints = new();

        [Header("Player Spawn Settings")]
        public PlayerSpawnIndex playerSpawnIndex = PlayerSpawnIndex.First;
        public int customPlayerSpawnIndex = 0; // only used if playerSpawnIndex is Custom

        private List<(GameObject go, Vector3 spawnPos, Quaternion spawnRot)> spawnedPlayers = new();
        private List<CheckpointTracker> playersCheckpointTrackers = new();

        private void Awake()
        {
            if (spawnPoints.Count == 0)
            {
                Debug.LogError("No spawn points assigned");
            }

            gameEvents.SpawnPlayersEvent.AddListener(OnSpawnPlayers);
            gameEvents.RestartRaceEvent.AddListener(OnRestartRace);

            foreach (var spawnPoint in spawnPoints)
                if (spawnPoint.TryGetComponent<Renderer>(out var renderer)) { renderer.enabled = false; }
        }

        private void OnSpawnPlayers()
        {
            int totalSpawns = spawnPoints.Count;
            int aiCount = RaceData.AiBotsSelected;

            // determine player spawn index
            int playerSpawnIndex = 0;
            switch (this.playerSpawnIndex)
            {
                case PlayerSpawnIndex.Last:
                    playerSpawnIndex = Mathf.Clamp(aiCount, 0, totalSpawns - 1); // player spawns after all AI
                    break;
                case PlayerSpawnIndex.Custom:
                    playerSpawnIndex = Mathf.Clamp(customPlayerSpawnIndex, 0, totalSpawns - 1);
                    break;
                case PlayerSpawnIndex.First:
                default:
                    playerSpawnIndex = 0;
                    break;
            }

            for (int i = 0; i <= aiCount; i++)
            {
                if (i == 0)
                {
                    // spawn player at the determined index
                    var player = Instantiate(playerPrefab, spawnPoints[playerSpawnIndex].position, spawnPoints[playerSpawnIndex].rotation);
                    spawnedPlayers.Add((player, player.transform.position, player.transform.rotation));

                    var checkpointTracker = player.GetComponentInChildren<CheckpointTracker>();
                    checkpointTracker.SetCarRacePositionIndex(0);
                    playersCheckpointTrackers.Add(checkpointTracker);
                }
                else
                {
                    // spawn AI at their respective indexes
                    int aiSpawnIdx = (i <= playerSpawnIndex) ? i - 1 : i; // adjust AI index if it overlaps with player
                    aiSpawnIdx = Mathf.Clamp(aiSpawnIdx, 0, totalSpawns - 1);

                    var aiCar = Instantiate(aiCarPrefab, spawnPoints[aiSpawnIdx].position, spawnPoints[aiSpawnIdx].rotation);
                    spawnedPlayers.Add((aiCar, aiCar.transform.position, aiCar.transform.rotation));

                    var aiTracker = Instantiate(aiWaypointTrackerPrefab).GetComponent<AIWaypointTracker>();
                    aiTracker.SetupAICarCollider(aiCar.GetComponentInChildren<AICarWaypointTrackerColliderTrigger>().GetColliderTrigger());

                    aiCar.GetComponent<CarAIControl>().SetTarget(aiTracker.transform); // replace with your car controller ai target to aim/follow

                    var checkpointTracker = aiCar.GetComponentInChildren<CheckpointTracker>();
                    checkpointTracker.SetCarRacePositionIndex(i);
                    playersCheckpointTrackers.Add(checkpointTracker);
                }
            }
            gameEvents.PlayersCheckpointTrackersAssignedEvent.Invoke(playersCheckpointTrackers);
        }

        private void OnRestartRace()
        {
            // reallocate players position and rotation to initial spawn points
            foreach (var player in spawnedPlayers)
            {
                player.go.transform.position = player.spawnPos;

                // check for main and child Rigidbodies
                Rigidbody[] rigidbodies = player.go.GetComponentsInChildren<Rigidbody>();
                foreach (var rb in rigidbodies)
                {
                    rb.linearVelocity = Vector3.zero;
                    rb.angularVelocity = Vector3.zero;
                    rb.rotation = player.spawnRot;
                }

                if (rigidbodies.Length == 0)
                {
                    player.go.transform.rotation = player.spawnRot;
                }
            }
        }
    }
}