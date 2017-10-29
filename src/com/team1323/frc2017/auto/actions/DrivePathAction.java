package com.team1323.frc2017.auto.actions;

import com.team1323.frc2017.paths.PathContainer;
import com.team1323.frc2017.subsystems.Drive;
import com.team254.lib.util.control.Path;

/**
 * Drives the robot along the Path defined in the PathContainer object. The action finishes once the robot reaches the
 * end of the path.
 * 
 * @see PathContainer
 * @see Path
 * @see Action
 */
public class DrivePathAction implements Action {

    private PathContainer mPathContainer;
    private Path mPath;
    private Drive mDrive = Drive.getInstance();
    private boolean waitForCompletion = false;

    public DrivePathAction(PathContainer p, boolean waitForCompletion) {
        mPathContainer = p;
        mPath = mPathContainer.buildPath();
        this.waitForCompletion = waitForCompletion;
    }

    @Override
    public boolean isFinished() {
        return mDrive.isDoneWithPath() || !waitForCompletion;
    }

    @Override
    public void update() {
        // Nothing done here, controller updates in mEnabedLooper in robot
    }

    @Override
    public void done() {
        // TODO: Perhaps set wheel velocity to 0?
    }

    @Override
    public void start() {
        mDrive.setWantDrivePath(mPath, mPathContainer.isReversed());
    }
}
