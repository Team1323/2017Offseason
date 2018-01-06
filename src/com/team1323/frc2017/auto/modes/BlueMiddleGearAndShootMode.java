package com.team1323.frc2017.auto.modes;

import com.team1323.frc2017.auto.AutoModeBase;
import com.team1323.frc2017.auto.AutoModeEndedException;
import com.team1323.frc2017.auto.actions.AlignForShootingAction;
import com.team1323.frc2017.auto.actions.BeginShootingAction;
import com.team1323.frc2017.auto.actions.PathfinderAction;
import com.team1323.frc2017.auto.actions.ScoreGearAction;
import com.team1323.frc2017.auto.actions.SpinUpShooterAction;
import com.team1323.frc2017.subsystems.Drive;
import com.team1323.frc2017.subsystems.Pigeon;

public class BlueMiddleGearAndShootMode extends AutoModeBase{
	@Override
	public void routine() throws AutoModeEndedException{
		Pigeon.getInstance().setAngle(0.0);
		runAction(new PathfinderAction(Drive.getInstance().middlePegTrajectory, true, true, true));
		runAction(new ScoreGearAction());
		runAction(new PathfinderAction(Drive.getInstance().blueMiddlePegToShootTrajectory, false, false, true));
		runAction(new AlignForShootingAction());
		runAction(new SpinUpShooterAction());
		runAction(new BeginShootingAction());
	}
}
