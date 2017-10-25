package com.team1323.frc2017.auto.actions;

import com.team1323.frc2017.subsystems.GearIntake;

public class ScoreGearAction implements Action{
	@Override
	public boolean isFinished(){
		return GearIntake.getInstance().getState() == GearIntake.State.IDLE;
	}
	
	@Override
	public void start(){
		GearIntake.getInstance().score();
	}
	
	@Override
	public void update(){
		
	}
	
	@Override
	public void done(){
		
	}
}
