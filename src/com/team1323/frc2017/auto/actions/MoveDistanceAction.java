package com.team1323.frc2017.auto.actions;

import com.team1323.frc2017.subsystems.Drive;

public class MoveDistanceAction implements Action{
	private Drive drive;
	private double distance;
	
	public MoveDistanceAction(double distance){
		this.distance = distance;
		drive = Drive.getInstance();
	}
	
	@Override
	public boolean isFinished(){
		return drive.distanceIsOnTarget();
	}
	
	@Override
	public void start(){
		drive.setPositionSetpoint(distance, distance);
	}
	
	@Override
	public void update(){
		
	}
	
	@Override
	public void done(){
		
	}
}
