package com.team1323.frc2017.auto.modes;

import java.util.Arrays;

import com.team1323.frc2017.auto.AutoModeBase;
import com.team1323.frc2017.auto.AutoModeEndedException;
import com.team1323.frc2017.auto.actions.Action;
import com.team1323.frc2017.auto.actions.AlignForShootingAction;
import com.team1323.frc2017.auto.actions.BeginShootingAction;
import com.team1323.frc2017.auto.actions.MoveDistanceAction;
import com.team1323.frc2017.auto.actions.ParallelAction;
import com.team1323.frc2017.auto.actions.PathfinderAction;
import com.team1323.frc2017.auto.actions.SeriesAction;
import com.team1323.frc2017.auto.actions.SpinUpShooterAction;
import com.team1323.frc2017.auto.actions.TurnToHeadingAction;
import com.team1323.frc2017.auto.actions.WaitAction;
import com.team1323.frc2017.subsystems.Drive;
import com.team1323.frc2017.subsystems.Pigeon;
import com.team254.lib.util.math.Rotation2d;

public class BlueHopperMode extends AutoModeBase{
	@Override
	public void routine() throws AutoModeEndedException{
		/*PathContainer hopperPath = new StartToHopperBlue();
		runAction(new ResetPoseFromPathAction(hopperPath));*/
		Pigeon.getInstance().setAngle(0.0);
		runAction(new ParallelAction(Arrays.asList(new Action[]{
				new PathfinderAction(Drive.getInstance().blueHopperTrajectory, false, false, false),
				new WaitAction(5.0)
		})));
		runAction(new SeriesAction(Arrays.asList(new Action[]{
				new MoveDistanceAction(-24.0),
				new TurnToHeadingAction(Rotation2d.fromDegrees(70))
		})));
		runAction(new ParallelAction(Arrays.asList(new Action[] {
				new AlignForShootingAction(),
				//new SpinUpShooterAction()
		})));
		runAction(new SpinUpShooterAction());
		runAction(new BeginShootingAction());
	}
}
