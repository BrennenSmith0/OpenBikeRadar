"""
tracker.py

Maintains stable tracked targets across successive radar frames.
"""

from __future__ import annotations

from ld2451.frame_parser import RadarTarget
from ld2451.enums import Direction
from tracker.target import TrackedTarget


import config 



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
        #
        # Try to match each radar target
        #
        for radar in radar_targets:

            best_track = None
            best_score = float("inf")

            for track in self.targets:

                if track.id in matched_tracks:
                    continue

                #
                # Predict where the target should be.
                #
                travel = (track.speed /3.6)* dt

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

                #
                # Slow targets jitter much more than they move.
                #
                if track.speed < config.TRACK_SLOW_SPEED_THRESHOLD:
                    distance_tolerance = config.TRACK_STATIONARY_BUFFER

                elif track.speed < config.TRACK_FAST_SPEED_THRESHOLD:
                    distance_tolerance = config.TRACK_SLOW_BUFFER

                else:
                    distance_tolerance = max(
                    config.TRACK_MIN_DISTANCE_BUFFER,
                    travel * config.TRACK_BUFFER_PERCENT,
                )

                #
                # Normalized score.
                #
                score = (
                    distance_error / distance_tolerance
                    + angle_error / config.TRACK_MATCH_ANGLE
                )

                #
                # Reject obviously bad matches.
                #
                if score >= config.TRACK_MATCH_SCORE:
                    continue

                if score < best_score:
                    best_score = score
                    best_track = track

            #
            # Update existing track
            #
            if best_track is not None:

                best_track.distance = radar.distance
                best_track.angle = radar.angle
                best_track.speed = radar.speed
                best_track.direction = radar.direction
                best_track.snr = radar.snr

                best_track.age += 1
                best_track.missed_frames = 0

                if radar.direction == Direction.APPROACHING:
                    best_track.has_approached = True

                matched_tracks.add(best_track.id)

            #
            # Create new track
            #
            else:
                has_approached = radar.direction == Direction.APPROACHING

                self.targets.append(
                    TrackedTarget(
                        id=self.next_id,
                        distance=radar.distance,
                        angle=radar.angle,
                        speed=radar.speed,
                        direction=radar.direction,
                        snr=radar.snr,
                        has_approached=has_approached,   # ← new
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

            if track.missed_frames <= config.TRACK_MAX_MISSED:
                survivors.append(track)

        self.targets = survivors

        return self.targets