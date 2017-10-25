package com.team1323.frc2017.auto.modes;

import com.team1323.frc2017.auto.AutoModeBase;
import com.team1323.frc2017.auto.AutoModeEndedException;
import com.team1323.frc2017.auto.actions.DrivePathAction;
import com.team1323.frc2017.auto.actions.ResetPoseFromPathAction;
import com.team1323.frc2017.auto.actions.ScoreGearAction;
import com.team1323.frc2017.paths.PathContainer;
import com.team1323.frc2017.paths.StartToCenterGearBlue;

public class BlueMiddleGearAndShootMode extends AutoModeBase{
	@Override
	public void routine() throws AutoModeEndedException{
		PathContainer gearPath = new StartToCenterGearBlue();
		runAction(new ResetPoseFromPathAction(gearPath));
		runAction(new DrivePathAction(gearPath));
		runAction(new ScoreGearAction());
		
	}
}