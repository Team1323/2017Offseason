package com.team1323.frc2017.paths;

import java.util.ArrayList;

import com.team1323.frc2017.paths.PathBuilder.Waypoint;
import com.team254.lib.util.control.Path;
import com.team254.lib.util.math.RigidTransform2d;
import com.team254.lib.util.math.Rotation2d;
import com.team254.lib.util.math.Translation2d;

public class StartToHopperRed implements PathContainer{
	@Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(16.5, 19.75, 0, 0));
        sWaypoints.add(new Waypoint(85, 19.75, 35, 60));
        sWaypoints.add(new Waypoint(85, -32, 0, 60));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0.0));
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}
