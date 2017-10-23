package com.team1323.frc2017.paths;

import java.util.ArrayList;

import com.team1323.frc2017.paths.PathBuilder.Waypoint;
import com.team254.lib.util.control.Path;
import com.team254.lib.util.math.RigidTransform2d;
import com.team254.lib.util.math.Rotation2d;
import com.team254.lib.util.math.Translation2d;

public class StartToBoilerGearBlue implements PathContainer{
	
	@Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        RigidTransform2d startPose = new RigidTransform2d(new Translation2d(), new Rotation2d());
        RigidTransform2d endPose = new RigidTransform2d(new Translation2d(96.0, -36.0), Rotation2d.fromDegrees(-60.0));
        Translation2d middlePoint = startPose.intersection(endPose);
        sWaypoints.add(new Waypoint(startPose.getTranslation(), 0, 0));
        sWaypoints.add(new Waypoint(middlePoint, 25, 80));
        sWaypoints.add(new Waypoint(endPose.getTranslation(), 0, 80));

        System.out.println(middlePoint.x() + ", " + middlePoint.y());
        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(0, 0), Rotation2d.fromDegrees(180.0));
    }

    @Override
    public boolean isReversed() {
        return true;
    }
}
