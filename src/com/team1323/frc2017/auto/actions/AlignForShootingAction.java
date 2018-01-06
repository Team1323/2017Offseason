package com.team1323.frc2017.auto.actions;

import com.team1323.frc2017.subsystems.Drive;
import com.team1323.frc2017.subsystems.Drive.DriveControlState;
import com.team1323.lib.util.DriveSignal;

public class AlignForShootingAction implements Action{
	private Drive drive;
	
	public AlignForShootingAction(){
		drive = Drive.getInstance();
	}
	
	@Override
	public boolean isFinished(){
		return drive.isDoneWithTurn();
	}
	
	@Override
	public void start(){
		drive.configureTalonsForPositionControl();
		//drive.setState(DriveControlState.DRIVE_TOWARDS_GOAL_APPROACH);
		drive.setWantDriveTowardsGoal();
	}
	
	@Override
	public void update(){
		
	}
	
	@Override
	public void done(){
		drive.setOpenLoop(DriveSignal.NEUTRAL);
	}
}
