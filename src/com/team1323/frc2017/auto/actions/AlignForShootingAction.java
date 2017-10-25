package com.team1323.frc2017.auto.actions;

import com.team1323.frc2017.subsystems.Drive;

public class AlignForShootingAction implements Action{
	private Drive drive;
	
	public AlignForShootingAction(){
		drive = Drive.getInstance();
	}
	
	@Override
	public boolean isFinished(){
		return drive.currentControlState() == Drive.DriveControlState.AIM_TO_GOAL &&
				drive.isOnTarget();
	}
	
	@Override
	public void start(){
		drive.setWantDriveTowardsGoal();
	}
	
	@Override
	public void update(){
		
	}
	
	@Override
	public void done(){
		
	}
}
