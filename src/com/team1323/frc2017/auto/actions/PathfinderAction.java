package com.team1323.frc2017.auto.actions;

import com.team1323.frc2017.subsystems.Drive;

import jaci.pathfinder.Trajectory;

public class PathfinderAction implements Action{
	private Drive drive;
	private Trajectory trajectory;
	private boolean isReversed;
	private boolean useHeadingCorrection;
	private boolean waitForCompletion;
	
	public PathfinderAction(Trajectory trajectory, boolean isReversed, boolean useHeadingCorrection, boolean waitForCompletion){
		drive = Drive.getInstance();
		this.trajectory = trajectory;
		this.isReversed = isReversed;
		this.useHeadingCorrection = useHeadingCorrection;
		this.waitForCompletion = waitForCompletion;
	}
	
	@Override
	public boolean isFinished(){
		return !waitForCompletion || drive.isDoneWithPathfinder();
	}
	
	@Override
	public void start(){
		drive.setWantFollowPathfinder(trajectory, isReversed, useHeadingCorrection);
	}
	
	@Override
	public void update(){
		
	}
	
	@Override
	public void done(){
		
	}
}
