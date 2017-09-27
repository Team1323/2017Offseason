package com.team1323.frc2017;

import com.team1323.frc2017.subsystems.BallIntake;
import com.team1323.frc2017.subsystems.Drive;
import com.team1323.frc2017.subsystems.GearIntake;

public class RobotSystem {
	public Drive drive;
	public BallIntake ballIntake;
	public GearIntake gearIntake;
	
	private static RobotSystem instance;
	public static RobotSystem getInstance(){
		if(instance == null)instance = new RobotSystem();
		return instance;
	}
	public RobotSystem(){
		drive = Drive.getInstance();
		ballIntake = BallIntake.getInstance();
		gearIntake = GearIntake.getInstance();
	}
}
