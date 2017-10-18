package com.team1323.frc2017;

import com.team1323.frc2017.loops.Looper;
import com.team1323.frc2017.loops.RobotStateEstimator;
import com.team1323.frc2017.loops.VisionProcessor;
import com.team1323.frc2017.subsystems.Drive;
import com.team1323.frc2017.subsystems.GearIntake;
import com.team1323.io.LogitechJoystick;
import com.team1323.io.SteeringWheel;
import com.team1323.io.Xbox;
import com.team1323.lib.util.CrashTracker;
import com.team1323.lib.util.DriveSignal;
import com.team254.lib.util.math.Rotation2d;

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
			enabledLooper.register(robot.gearIntake.getLoop());
			enabledLooper.register(RobotStateEstimator.getInstance());
			enabledLooper.register(VisionProcessor.getInstance());
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
		robot.gearIntake.outputToSmartDashboard();
		robot.dyeRotors.outputToSmartDashboard();
		robot.hanger.outputToSmartDashboard();
		robot.shooter.outputToSmartDashboard();
	}
	
	public void stopAll(){
		robot.drive.stop();
		robot.ballIntake.stop();
		robot.gearIntake.stop();
		robot.dyeRotors.stop();
		robot.hanger.stop();
		robot.shooter.stop();
	}
	
	public void coDriverStop(){
		robot.ballIntake.stop();
		robot.gearIntake.stop();
		robot.dyeRotors.stop();
		robot.hanger.stop();
		robot.shooter.stop();
	}

	@Override
	public void disabledInit(){
		try{
			CrashTracker.logDisabledInit();
			
			enabledLooper.stop();
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
			
			if(Math.abs(driverJoystick.getYAxis()) > 0 || wheel.leftBumper.isBeingPressed()){
				robot.drive.setOpenLoop(cheesyDriveHelper.cheesyDrive(-driverJoystick.getYAxis(), 
						wheel.getWheelTurn(), wheel.leftBumper.isBeingPressed(), true));
			}else if(robot.drive.currentControlState() == Drive.DriveControlState.OPEN_LOOP){
				robot.drive.setOpenLoop(DriveSignal.NEUTRAL);
			}
			
			if(coDriver.rightBumper.wasPressed()){
				robot.ballIntake.forward();
			}else if(coDriver.leftBumper.wasPressed()){
				robot.ballIntake.reverse();
			}
			
			if(coDriver.aButton.isBeingPressed() || driverJoystick.thumbButton.isBeingPressed()){
				robot.gearIntake.setState(GearIntake.State.INTAKING);
			}else if(robot.gearIntake.getState() == GearIntake.State.INTAKING){
				robot.gearIntake.setState(GearIntake.State.HOLDING);
			}
			
			if(driverJoystick.triggerButton.isBeingPressed()){
				robot.gearIntake.score();
			}
			
			if(coDriver.rightTrigger.isBeingPressed()){
				robot.dyeRotors.startFeeding();
			}
			
			if(coDriver.startButton.isBeingPressed()){
				robot.hanger.hang();
			}
			
			if(coDriver.leftTrigger.wasPressed()){
				robot.shooter.setSpeed(Constants.kShootingSpeed);
			}
			
			if(coDriver.yButton.wasPressed()){
				robot.drive.setPositionSetpoint(Rotation2d.fromDegrees(90));
			}
			
			if(coDriver.backButton.wasPressed() || wheel.yButton.wasPressed()){
				coDriverStop();
			}
			
			outputAllToSmartDashboard();
		}catch(Throwable t){
			CrashTracker.logThrowableCrash(t);
			throw(t);
		}
	}
}

