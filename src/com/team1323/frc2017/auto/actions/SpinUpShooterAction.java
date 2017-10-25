package com.team1323.frc2017.auto.actions;

import com.team1323.frc2017.subsystems.Shooter;

public class SpinUpShooterAction extends RunOnceAction implements Action{
	@Override
	public void runOnce(){
		Shooter.getInstance().setState(Shooter.State.OverCompensate);
	}
}
