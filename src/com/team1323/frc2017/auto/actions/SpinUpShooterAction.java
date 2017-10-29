package com.team1323.frc2017.auto.actions;

import com.team1323.frc2017.subsystems.Shooter;

public class SpinUpShooterAction implements Action{
	public SpinUpShooterAction(){
		
	}
	
	@Override
	public boolean isFinished(){
		return Shooter.getInstance().isOnTarget();
	}
	
	@Override
	public void start(){
		Shooter.getInstance().setState(Shooter.State.OverCompensate);
	}
	
	@Override
	public void update(){
		
	}
	
	@Override
	public void done(){
		
	}
}
