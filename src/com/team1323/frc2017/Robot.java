package com.team1323.frc2017;

import com.team1323.frc2017.loops.Looper;
import com.team1323.frc2017.loops.RobotStateEstimator;
import com.team1323.frc2017.loops.VisionProcessor;
import com.team1323.frc2017.paths.PathContainer;
import com.team1323.frc2017.paths.StartToBoilerBlue;
import com.team1323.frc2017.subsystems.Drive;
import com.team1323.frc2017.subsystems.GearIntake;
import com.team1323.frc2017.subsystems.Shooter;
import com.team1323.io.LogitechJoystick;
import com.team1323.io.SteeringWheel;
import com.team1323.io.Xbox;
import com.team1323.lib.util.CrashTracker;
import com.team1323.lib.util.DriveSignal;
import com.team254.lib.util.math.RigidTransform2d;
import com.team254.lib.util.math.Rotation2d;
import com.team254.lib.util.math.Translation2d;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	private RobotSystem robot;
	private RobotState robotState = RobotState.getInstance();
	
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
			enabledLooper.register(robot.gearIntake.getLoop());
			enabledLooper.register(robot.shooter.getLoop());
			enabledLooper.register(robot.pidgey.getLoop());
			enabledLooper.register(RobotStateEstimator.getInstance());
			enabledLooper.register(VisionProcessor.getInstance());
			
			disabledLooper.register(robot.pidgey.getLoop());
			disabledLooper.register(VisionProcessor.getInstance());
		}catch(Throwable t){
			CrashTracker.logThrowableCrash(t);
			throw(t);
		}
	}
	
	public void zeroAllSensors(){
		robot.drive.zeroSensors();
		robot.pidgey.setAngle(0);
		robotState.reset(Timer.getFPGATimestamp(), new RigidTransform2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0)));
	}
	
	public void outputAllToSmartDashboard(){
		robot.drive.outputToSmartDashboard();
		robot.ballIntake.outputToSmartDashboard();
		robot.gearIntake.outputToSmartDashboard();
		robot.dyeRotors.outputToSmartDashboard();
		robot.hanger.outputToSmartDashboard();
		robot.shooter.outputToSmartDashboard();
		robotState.outputToSmartDashboard();
		robot.pidgey.outputToSmartDashboard();
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
			
			if(robot.gearIntake.isScoring()){
				robot.drive.setOpenLoop(DriveSignal.NEUTRAL);
			}else if(robot.drive.currentControlState() == Drive.DriveControlState.OPEN_LOOP){
				robot.drive.setOpenLoop(cheesyDriveHelper.cheesyDrive(-driverJoystick.getYAxis(), 
						wheel.getWheelTurn(), wheel.leftBumper.isBeingPressed(), true));
			}else if(Math.abs(driverJoystick.getYAxis()) > 0.5 || wheel.leftBumper.isBeingPressed()){
				robot.drive.setOpenLoop(cheesyDriveHelper.cheesyDrive(-driverJoystick.getYAxis(), 
						wheel.getWheelTurn(), wheel.leftBumper.isBeingPressed(), true));
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
			
			if(robot.gearIntake.needsToNotify()){
				coDriver.rumble(2, 1);
			}
			
			if(coDriver.rightTrigger.isBeingPressed()){
				robot.dyeRotors.startFeeding();
			}
			
			if(coDriver.startButton.isBeingPressed()){
				robot.hanger.hang();
			}
			
			if(coDriver.leftTrigger.wasPressed()){
				robot.shooter.setState(Shooter.State.SpinUp);
			}
			
			if(coDriver.yButton.wasPressed()){
				robot.dyeRotors.stopArms();
				robot.dyeRotors.reverseRollers();
			}
			
			if(coDriver.xButton.wasPressed()){
				PathContainer pc = new StartToBoilerBlue();
				RobotState.getInstance().reset(Timer.getFPGATimestamp(), pc.getStartPose());
				robot.pidgey.setAngle((int)pc.getStartPose().getRotation().getDegrees());
				robot.drive.setWantDrivePath(pc.buildPath(), pc.isReversed());
			}
			
			if(coDriver.backButton.wasPressed()){
				coDriverStop();
			}
			
			outputAllToSmartDashboard();
		}catch(Throwable t){
			CrashTracker.logThrowableCrash(t);
			throw(t);
		}
	}
}

