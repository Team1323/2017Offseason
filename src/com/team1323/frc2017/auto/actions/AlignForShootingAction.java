package com.team1323.frc2017.auto.actions;

import com.team1323.frc2017.subsystems.Drive;
import com.team1323.frc2017.subsystems.Drive.DriveControlState;

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
		drive.setState(DriveControlState.DRIVE_TOWARDS_GOAL_APPROACH);
	}
	
	@Override
	public void update(){
		
	}
	
	@Override
	public void done(){
		
	}
}
