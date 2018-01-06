package com.team1323.frc2017.auto.actions;

import com.team1323.frc2017.subsystems.BallIntake;
import com.team1323.frc2017.subsystems.DoubleDyeRotor;
import com.team1323.frc2017.subsystems.Shooter;

public class BeginShootingAction extends RunOnceAction implements Action{
	@Override
	public void runOnce(){
		DoubleDyeRotor.getInstance().startFeeding();
		//BallIntake.getInstance().forward();
		Shooter.getInstance().setState(Shooter.State.SpinUp);
	}
}
