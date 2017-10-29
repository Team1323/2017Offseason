package com.team1323.frc2017.auto.actions;

import com.team1323.frc2017.subsystems.Drive;
import com.team254.lib.util.math.Rotation2d;

public class TurnToHeadingAction implements Action {

    private Rotation2d mTargetHeading;
    private Drive mDrive;

    public TurnToHeadingAction(Rotation2d heading) {
        mTargetHeading = heading;
        mDrive = Drive.getInstance();
    }

    @Override
    public boolean isFinished() {
        return mDrive.distanceIsOnTarget();
    }

    @Override
    public void update() {
        // Nothing done here, controller updates in mEnabedLooper in robot
    }

    @Override
    public void done() {
    }

    @Override
    public void start() {
        //mDrive.setWantTurnToHeading(mTargetHeading);
    	mDrive.setPositionSetpoint(mTargetHeading);
    }
}
