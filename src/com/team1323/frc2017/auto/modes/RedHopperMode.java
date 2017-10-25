package com.team1323.frc2017.auto.modes;

import java.util.Arrays;

import com.team1323.frc2017.auto.AutoModeBase;
import com.team1323.frc2017.auto.AutoModeEndedException;
import com.team1323.frc2017.auto.actions.Action;
import com.team1323.frc2017.auto.actions.AlignForShootingAction;
import com.team1323.frc2017.auto.actions.BeginShootingAction;
import com.team1323.frc2017.auto.actions.DrivePathAction;
import com.team1323.frc2017.auto.actions.ParallelAction;
import com.team1323.frc2017.auto.actions.ResetPoseFromPathAction;
import com.team1323.frc2017.auto.actions.SpinUpShooterAction;
import com.team1323.frc2017.paths.PathContainer;
import com.team1323.frc2017.paths.StartToHopperRed;

public class RedHopperMode extends AutoModeBase{
	@Override
	public void routine() throws AutoModeEndedException{
		PathContainer hopperPath = new StartToHopperRed();
		runAction(new ResetPoseFromPathAction(hopperPath));
		runAction(new DrivePathAction(hopperPath));
		
		runAction(new ParallelAction(Arrays.asList(new Action[] {
				new AlignForShootingAction(),
				new SpinUpShooterAction()
		})));
		runAction(new BeginShootingAction());
	}
}
