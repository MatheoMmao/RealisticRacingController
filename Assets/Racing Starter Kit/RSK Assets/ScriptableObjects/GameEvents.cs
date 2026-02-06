using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;

namespace SpinMotion
{
    [CreateAssetMenu(fileName = "GameEvents", menuName = "Scriptable Objects/GameEvents")]
    public class GameEvents : ScriptableObject
    {
        public ToggleCarFreezeEvent ToggleCarFreezeEvent = new();
        public PlayAudioSfxEvent PlayAudioSfxEvent = new();
        public ToggleGamePauseEvent ToggleGamePauseEvent = new();

        public SpawnPlayersEvent SpawnPlayersEvent = new();
        public PlayPreRaceCountdownEvent PlayPreRaceCountdownEvent = new();
        public PreRaceUpdateGuiEvent PreRaceUpdateGuiEvent = new();
        public ChangeToRaceCamerasEvent ChangeToRaceCamerasEvent = new();

        public PlayersCheckpointTrackersAssignedEvent PlayersCheckpointTrackersAssignedEvent = new();
        public RaceStartedEvent RaceStartedEvent = new();
        public CheckpointPassedEvent CheckpointPassedEvent = new();
        public LapCompletedEvent LapCompletedEvent = new();
        public RaceFinishedEvent RaceFinishedEvent = new();
        public RestartRaceEvent RestartRaceEvent = new();
        public RaceTimeoutEvent RaceTimeoutEvent = new();

        public OnClickRestartRaceEvent OnClickRestartRaceEvent = new();
        public OnClickRestartGameEvent OnClickRestartGameEvent = new();
        public OnClickPlayRaceEvent OnClickPlayRaceEvent = new();
        public OnClickTogglePauseEvent OnClickTogglePauseEvent = new();
    }

    public class ToggleCarFreezeEvent : UnityEvent<bool> {}
    public class PlayAudioSfxEvent : UnityEvent<SFXType> {}
    public class ToggleGamePauseEvent : UnityEvent<bool> {}

    public class SpawnPlayersEvent : UnityEvent {}
    public class PlayPreRaceCountdownEvent : UnityEvent {}
    public class PreRaceUpdateGuiEvent : UnityEvent {}
    public class ChangeToRaceCamerasEvent : UnityEvent {}

    public class PlayersCheckpointTrackersAssignedEvent : UnityEvent<List<CheckpointTracker>> {}
    public class RaceStartedEvent : UnityEvent {}
    public class CheckpointPassedEvent : UnityEvent<int, int> {}
    public class LapCompletedEvent : UnityEvent {}
    public class RaceFinishedEvent : UnityEvent<RaceFinishType> {}
    public class RestartRaceEvent : UnityEvent {}
    public class RaceTimeoutEvent : UnityEvent {}

    public class OnClickRestartRaceEvent : UnityEvent {}
    public class OnClickRestartGameEvent : UnityEvent {}
    public class OnClickPlayRaceEvent : UnityEvent {}
    public class OnClickTogglePauseEvent : UnityEvent {}
}