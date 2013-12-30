#!/usr/bin/python
"""
Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.

This script is used to read a feature tracks file and draw the tracks as
randomly colored poly-lines.  This provides a visualization for a quick
visual assessment of the quality of the tracks (e.g. number of tracks and
their distribution, number of bad tracks, etc.)
"""

from optparse import OptionParser

import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
import random

class TrackState(object):
    """Represents the state of a feature track at a particular frame of video.
    """
    def __init__(self, track_id, frame_num,
                 location=(0.0, 0.0),
                 magnitude=1.0,
                 scale=1.0,
                 angle=0.0):
        """Constructor"""
        self.track_id = track_id
        self.frame_num = frame_num
        self.location = location
        self.magnitude = magnitude
        self.scale = scale
        self.angle = angle


def parse_track_state_line(line):
    """Parse a line of a feature track file and produce a track state.
    """
    if not line or line[0] == '#':
        return None
    tokens = line.split(' ')
    if len(tokens) != 7:
        return None
    track_id = int(tokens[0])
    frame_num = int(tokens[1])
    location = map(float, tokens[2:4])
    magnitude = float(tokens[4])
    scale = float(tokens[5])
    angle = float(tokens[6])

    return TrackState(track_id, frame_num, location, magnitude, scale, angle)


def load_tracks(filename):
    """Load a feature track file into a map of tracks ids to track states.
    """
    track_map = {}
    with open(filename, 'r') as f:
        for line in f:
            ts = parse_track_state_line(line)
            if not ts:
                continue
            if ts.track_id not in track_map:
                track_map[ts.track_id] = [ts]
            else:
                track_map[ts.track_id].append(ts)

    return track_map


def main():
    usage = "usage: %prog [options] track_file"
    description = "Read a feature track file and plot the tracks"
    parser = OptionParser(usage=usage, description=description)

    (options, args) = parser.parse_args()

    track_filename = args[0]

    tracks = load_tracks(track_filename)

    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.set_xlim(2084)
    ax.set_ylim(2084)
    for track_id, track in tracks.iteritems():
        if len(track) < 10:
            continue
        coords = [ts.location for ts in track]
        codes = [Path.MOVETO] + [Path.LINETO] * (len(track)-1)
        path = Path(coords, codes, closed=False)
        rand_color = (random.random(), random.random(), random.random())
        patch = patches.PathPatch(path, facecolor='none', edgecolor=rand_color)
        ax.add_patch(patch)

    plt.show()


if __name__ == "__main__":
    main()
