package com.team1323.frc2017;

import com.team1323.frc2017.subsystems.BallIntake;
import com.team1323.frc2017.subsystems.DoubleDyeRotor;
import com.team1323.frc2017.subsystems.Drive;
import com.team1323.frc2017.subsystems.GearIntake;
import com.team1323.frc2017.subsystems.Hanger;
import com.team1323.frc2017.subsystems.Pigeon;
import com.team1323.frc2017.subsystems.Shooter;

import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.wpilibj.CameraServer;

public class RobotSystem {
	public Pigeon pidgey;
	public Drive drive;
	public BallIntake ballIntake;
	public GearIntake gearIntake;
	public DoubleDyeRotor dyeRotors;
	public Shooter shooter;
	public Hanger hanger;
		
	private static RobotSystem instance;
	public static RobotSystem getInstance(){
		if(instance == null)instance = new RobotSystem();
		return instance;
	}
	public RobotSystem(){
		pidgey = Pigeon.getInstance();
		drive = Drive.getInstance();
		ballIntake = BallIntake.getInstance();
		gearIntake = GearIntake.getInstance();
		dyeRotors = DoubleDyeRotor.getInstance();
		shooter = Shooter.getInstance();
		hanger = Hanger.getInstance();
	}
	
	public void initCamera(){
    	UsbCamera usbCamera = new UsbCamera("USB Camera 0", 0);
    	usbCamera.setVideoMode(PixelFormat.kMJPEG, 640, 480, 30);
    	MjpegServer mjpegServer2 = new MjpegServer("serve_Blur", 1182);
    	mjpegServer2.setSource(usbCamera);
	}
}
