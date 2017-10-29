package com.team1323.frc2017.auto.modes;

import java.util.Arrays;

import com.team1323.frc2017.auto.AutoModeBase;
import com.team1323.frc2017.auto.AutoModeEndedException;
import com.team1323.frc2017.auto.actions.Action;
import com.team1323.frc2017.auto.actions.AlignForShootingAction;
import com.team1323.frc2017.auto.actions.BeginShootingAction;
import com.team1323.frc2017.auto.actions.DrivePathAction;
import com.team1323.frc2017.auto.actions.MoveDistanceAction;
import com.team1323.frc2017.auto.actions.ParallelAction;
import com.team1323.frc2017.auto.actions.ResetPoseFromPathAction;
import com.team1323.frc2017.auto.actions.SeriesAction;
import com.team1323.frc2017.auto.actions.SpinUpShooterAction;
import com.team1323.frc2017.auto.actions.TurnToHeadingAction;
import com.team1323.frc2017.auto.actions.WaitAction;
import com.team1323.frc2017.paths.PathContainer;
import com.team1323.frc2017.paths.StartToHopperBlue;
import com.team254.lib.util.math.Rotation2d;

public class BlueHopperMode extends AutoModeBase{
	@Override
	public void routine() throws AutoModeEndedException{
		PathContainer hopperPath = new StartToHopperBlue();
		runAction(new ResetPoseFromPathAction(hopperPath));
		runAction(new ParallelAction(Arrays.asList(new Action[]{
				new DrivePathAction(hopperPath, false),
				new WaitAction(5.0)
		})));
		runAction(new SeriesAction(Arrays.asList(new Action[]{
				new MoveDistanceAction(-24.0),
				new TurnToHeadingAction(Rotation2d.fromDegrees(90))
		})));
		runAction(new ParallelAction(Arrays.asList(new Action[] {
				new AlignForShootingAction(),
				//new SpinUpShooterAction()
		})));
		runAction(new SpinUpShooterAction());
		runAction(new BeginShootingAction());
	}
}
