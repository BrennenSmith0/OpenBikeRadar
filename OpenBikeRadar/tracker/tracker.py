"""
tracker.py

Maintains stable tracked targets across successive radar frames.
"""

from __future__ import annotations

from ld2451.frame_parser import RadarTarget
from ld2451.enums import Direction
from tracker.target import TrackedTarget


from config import (
    TRACK_MATCH_ANGLE,
    TRACK_MIN_DISTANCE_BUFFER,
    TRACK_BUFFER_PERCENT,
    TRACK_MAX_MISSED,
)


class Tracker:
    """
    Tracks radar targets over time.

    The tracker predicts where each tracked target should be based on
    its previous speed and direction, then matches new radar detections
    against those predictions.
    """

    def __init__(self):
        self.targets: list[TrackedTarget] = []
        self.next_id = 1

    def update(
        self,
        radar_targets: list[RadarTarget],
        dt: float,
    ) -> list[TrackedTarget]:
        """
        Update the tracker with a new radar frame.

        Parameters
        ----------
        radar_targets
            Targets parsed from the latest radar frame.

        dt
            Seconds since the previous radar frame.
        """

        matched_tracks = set()

        #
        # Match each radar target
        #
        for radar in radar_targets:

            best_track = None
            best_score = float("inf")

            for track in self.targets:

                if track.id in matched_tracks:
                    continue

                #
                # Predict where this target should be now.
                #
                travel = track.speed * dt

                if track.direction == Direction.APPROACHING:
                    predicted_distance = track.distance - travel
                else:
                    predicted_distance = track.distance + travel

                distance_error = abs(
                    radar.distance - predicted_distance
                )

                angle_error = abs(
                    radar.angle - track.angle
                )

                tolerance = max(
                    TRACK_MIN_DISTANCE_BUFFER,
                    travel * TRACK_BUFFER_PERCENT,
                )

                if distance_error > tolerance:
                    continue

                if angle_error > TRACK_MATCH_ANGLE:
                    continue

                #
                # Lower score = better match.
                #
                score = (
                    distance_error
                    + angle_error * 0.2
                )

                if score < best_score:
                    best_score = score
                    best_track = track

            #
            # Existing target
            #
            if best_track is not None:

                best_track.distance = radar.distance
                best_track.angle = radar.angle
                best_track.speed = radar.speed
                best_track.direction = radar.direction
                best_track.snr = radar.snr

                best_track.age += 1
                best_track.missed_frames = 0

                matched_tracks.add(best_track.id)

            #
            # New target
            #
            else:

                self.targets.append(
                    TrackedTarget(
                        id=self.next_id,
                        distance=radar.distance,
                        angle=radar.angle,
                        speed=radar.speed,
                        direction=radar.direction,
                        snr=radar.snr,
                    )
                )

                matched_tracks.add(self.next_id)
                self.next_id += 1

        #
        # Age unmatched tracks
        #
        survivors = []

        for track in self.targets:

            if track.id not in matched_tracks:
                track.missed_frames += 1

            if track.missed_frames <= TRACK_MAX_MISSED:
                survivors.append(track)

        self.targets = survivors

        return self.targets