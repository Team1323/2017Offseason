package com.team1323.frc2017;

import com.team1323.frc2017.loops.Looper;
import com.team1323.io.LogitechJoystick;
import com.team1323.io.SteeringWheel;
import com.team1323.io.Xbox;
import com.team1323.lib.util.CrashTracker;

import edu.wpi.first.wpilibj.IterativeRobot;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	private RobotSystem robot;
	
	private LogitechJoystick driverJoystick;
	private SteeringWheel wheel;
	private Xbox coDriver;
	private CheesyDriveHelper cheesyDriveHelper = new CheesyDriveHelper();
	
	Looper enabledLooper = new Looper();
	Looper disabledLooper = new Looper();
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		try{
			CrashTracker.logRobotInit();
			robot = RobotSystem.getInstance();
			wheel = new SteeringWheel(0);
			driverJoystick = new LogitechJoystick(1);
			coDriver = new Xbox(2);
			zeroAllSensors();
			
			enabledLooper.register(robot.drive.getLoop());
		}catch(Throwable t){
			CrashTracker.logThrowableCrash(t);
			throw(t);
		}
	}
	
	public void zeroAllSensors(){
		robot.drive.zeroSensors();
	}
	
	public void outputAllToSmartDashboard(){
		robot.drive.outputToSmartDashboard();
		robot.ballIntake.outputToSmartDashboard();
	}
	
	public void stopAll(){
		robot.drive.stop();
		robot.ballIntake.stop();
	}
	
	public void coDriverStop(){
		robot.ballIntake.stop();
	}

	@Override
	public void disabledInit(){
		try{
			CrashTracker.logDisabledInit();
			
			enabledLooper.stop();
			disabledLooper.start();
		}catch(Throwable t){
			CrashTracker.logThrowableCrash(t);
			throw(t);
		}
	}
	
	@Override
	public void autonomousInit() {
		try{
			CrashTracker.logAutoInit();
			
			zeroAllSensors();
			
			disabledLooper.stop();
			enabledLooper.start();
		}catch(Throwable t){
			CrashTracker.logThrowableCrash(t);
			throw(t);
		}
	}

	@Override
	public void teleopInit(){
		try{
			CrashTracker.logTeleopInit();
			
			disabledLooper.stop();
			enabledLooper.start();
		}catch(Throwable t){
			CrashTracker.logThrowableCrash(t);
			throw(t);
		}
	}

	@Override
	public void disabledPeriodic(){
		try{
			stopAll();
			outputAllToSmartDashboard();
		}catch(Throwable t){
			CrashTracker.logThrowableCrash(t);
			throw(t);
		}
	}
	
	@Override
	public void autonomousPeriodic(){
		try{
			outputAllToSmartDashboard();
		}catch(Throwable t){
			CrashTracker.logThrowableCrash(t);
			throw(t);
		}
	}
	
	@Override
	public void teleopPeriodic() {
		try{
			wheel.update();
			driverJoystick.update();
			coDriver.update();
			
			robot.drive.setOpenLoop(cheesyDriveHelper.cheesyDrive(-driverJoystick.getYAxis(), wheel.getWheelTurn(), wheel.leftBumper.isBeingPressed()));
			
			
			if(coDriver.rightBumper.wasPressed()){
				robot.ballIntake.forward();
			}else if(coDriver.leftBumper.wasPressed()){
				robot.ballIntake.reverse();
			}
			
			if(coDriver.backButton.wasPressed()){
				coDriverStop();
			}
		}catch(Throwable t){
			CrashTracker.logThrowableCrash(t);
			throw(t);
		}
	}
}

