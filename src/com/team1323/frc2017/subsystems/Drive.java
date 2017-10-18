package com.team1323.frc2017.subsystems;

import java.util.Optional;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;
import com.ctre.CANTalon.VelocityMeasurementPeriod;
import com.team1323.frc2017.Constants;
import com.team1323.frc2017.Ports;
import com.team1323.frc2017.RobotState;
import com.team1323.frc2017.loops.Loop;
import com.team1323.frc2017.vision.ShooterAimingParameters;
import com.team1323.lib.util.DriveSignal;
import com.team1323.lib.util.Kinematics;
import com.team1323.lib.util.Util;
import com.team254.lib.util.control.Path;
import com.team254.lib.util.control.PathFollower;
import com.team254.lib.util.math.RigidTransform2d;
import com.team254.lib.util.math.Rotation2d;
import com.team254.lib.util.math.Twist2d;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drive extends Subsystem{
	private final CANTalon leftMaster, leftSlave, rightMaster, rightSlave;
	private final Solenoid shifter;
	private Pigeon pidgey;
	private boolean isHighGear = false;
	private boolean isBrakeMode = false;
	public boolean isHighGear(){
		return isHighGear;
	}
	public boolean isBrakeMode() {
        return isBrakeMode;
    }
	
	private static final int kLowGearPositionControlSlot = 0;
    private static final int kHighGearVelocityControlSlot = 1;
	
	// The robot drivetrain's various states.
    public enum DriveControlState {
        OPEN_LOOP, // open loop voltage control
        VELOCITY_SETPOINT, // velocity PID control
        PATH_FOLLOWING, // used for autonomous driving
        AIM_TO_GOAL, // turn to face the boiler
        TURN_TO_HEADING, // turn in place
        DRIVE_TOWARDS_GOAL_COARSE_ALIGN, // turn to face the boiler, then DRIVE_TOWARDS_GOAL_COARSE_ALIGN
        DRIVE_TOWARDS_GOAL_APPROACH // drive forwards until we are at optimal shooting distance
    }

    /**
     * Check if the drive talons are configured for velocity control
     */
    protected static boolean usesTalonVelocityControl(DriveControlState state) {
        if (state == DriveControlState.VELOCITY_SETPOINT || state == DriveControlState.PATH_FOLLOWING) {
            return true;
        }
        return false;
    }

    /**
     * Check if the drive talons are configured for position control
     */
    protected static boolean usesTalonPositionControl(DriveControlState state) {
        if (state == DriveControlState.AIM_TO_GOAL ||
                state == DriveControlState.TURN_TO_HEADING ||
                state == DriveControlState.DRIVE_TOWARDS_GOAL_COARSE_ALIGN ||
                state == DriveControlState.DRIVE_TOWARDS_GOAL_APPROACH) {
            return true;
        }
        return false;
    }
    
    // Control states
    private DriveControlState mDriveControlState;
    public DriveControlState currentControlState(){
    	return mDriveControlState;
    }
    
    // Controllers
    private RobotState mRobotState = RobotState.getInstance();
    private PathFollower mPathFollower;

    // These gains get reset below!!
    private Rotation2d mTargetHeading = new Rotation2d();
    private Path mCurrentPath = null;
    
    private boolean mIsOnTarget = false;
    private boolean mIsApproaching = false;
	
	private static Drive instance = null;
	
	public static Drive getInstance(){
		if(instance == null){
			instance = new Drive();
		}
		return instance;
	}
	
	private Drive(){
		leftMaster = new CANTalon(Ports.DRIVE_LEFT_MASTER);
		leftSlave = new CANTalon(Ports.DRIVE_LEFT_SLAVE);
		rightMaster = new CANTalon(Ports.DRIVE_RIGHT_MASTER);
		rightSlave = new CANTalon(Ports.DRIVE_RIGHT_SLAVE);
		
		shifter = new Solenoid(20, Ports.DRIVE_SHIFTER);
		setHighGear(true);
		
		leftMaster.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		rightMaster.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		
		leftMaster.reverseOutput(true);
		leftMaster.reverseSensor(true);
		rightMaster.reverseSensor(false);
		rightMaster.reverseOutput(false);
		
		leftMaster.setStatusFrameRateMs(CANTalon.StatusFrameRate.Feedback, 5);
		rightMaster.setStatusFrameRateMs(CANTalon.StatusFrameRate.Feedback, 5);
		
		leftMaster.SetVelocityMeasurementPeriod(VelocityMeasurementPeriod.Period_10Ms);
		leftMaster.SetVelocityMeasurementWindow(32);
		rightMaster.SetVelocityMeasurementPeriod(VelocityMeasurementPeriod.Period_10Ms);
		rightMaster.SetVelocityMeasurementWindow(32);
		
		CANTalon.FeedbackDeviceStatus leftSensorPresent = leftMaster
                .isSensorPresent(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
        if (leftSensorPresent != CANTalon.FeedbackDeviceStatus.FeedbackStatusPresent) {
            DriverStation.reportError("Could not detect left encoder: " + leftSensorPresent, false);
        }
        CANTalon.FeedbackDeviceStatus rightSensorPresent = rightMaster
                .isSensorPresent(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
        if (rightSensorPresent != CANTalon.FeedbackDeviceStatus.FeedbackStatusPresent) {
            DriverStation.reportError("Could not detect right encoder: " + rightSensorPresent, false);
        }
		
		leftMaster.changeControlMode(TalonControlMode.PercentVbus);
		leftMaster.set(0);
		leftSlave.changeControlMode(TalonControlMode.Follower);
		leftSlave.set(Ports.DRIVE_LEFT_MASTER);
		rightMaster.changeControlMode(TalonControlMode.PercentVbus);
		rightMaster.set(0);
		rightSlave.changeControlMode(TalonControlMode.Follower);
		rightSlave.set(Ports.DRIVE_RIGHT_MASTER);
		
		pidgey = Pigeon.getInstance();
		
		reloadGains();
		
		setOpenLoop(DriveSignal.NEUTRAL);
	}
	
	private final Loop loop = new Loop(){
		@Override
		public void onStart( double timestamp){
			setOpenLoop(DriveSignal.NEUTRAL);
		}
		
		@Override
		public void onLoop(double timestamp){
			synchronized (Drive.this){
				switch(mDriveControlState){
				case OPEN_LOOP:
					return;
				default:
					break;
				}
			}
		}
		
		@Override
		public void onStop(double timestamp){
			setOpenLoop(DriveSignal.NEUTRAL);
		}
	};
	
	public Loop getLoop(){
		return loop;
	}
	
	protected synchronized void setLeftRightPower(double left, double right){
		leftMaster.set(-left);
		rightMaster.set(right);
	}
	
	public synchronized void setOpenLoop(DriveSignal signal){
		if(mDriveControlState != DriveControlState.OPEN_LOOP){
			leftMaster.changeControlMode(TalonControlMode.PercentVbus);
			rightMaster.changeControlMode(TalonControlMode.PercentVbus);
			leftMaster.configNominalOutputVoltage(0.0, 0.0);
            rightMaster.configNominalOutputVoltage(0.0, 0.0);
			mDriveControlState = DriveControlState.OPEN_LOOP;
			setBrakeMode(false);
		}
		setLeftRightPower(signal.leftMotor, signal.rightMotor);
	}
	
	public void setHighGear(boolean highGear){
		isHighGear = highGear;
		shifter.set(highGear);
	}

    public synchronized void setBrakeMode(boolean on) {
        if (isBrakeMode != on) {
        	isBrakeMode = on;
            rightMaster.enableBrakeMode(on);
            rightSlave.enableBrakeMode(on);
            leftMaster.enableBrakeMode(on);
            leftSlave.enableBrakeMode(on);
        }
    }
	
    public synchronized void setVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
        configureTalonsForSpeedControl();
        mDriveControlState = DriveControlState.VELOCITY_SETPOINT;
        updateVelocitySetpoint(left_inches_per_sec, right_inches_per_sec);
    }
    
    public synchronized void setPositionSetpoint(double left_delta_inches, double right_delta_inches){
    	configureTalonsForPositionControl();
    	mDriveControlState = DriveControlState.DRIVE_TOWARDS_GOAL_APPROACH;
    	updatePositionSetpoint(getLeftDistanceInches() + left_delta_inches, getRightDistanceInches() + right_delta_inches);
    }
    public synchronized void setPositionSetpoint(Rotation2d delta_heading){
    	configureTalonsForPositionControl();
    	mDriveControlState = DriveControlState.AIM_TO_GOAL;
    	Kinematics.DriveVelocity wheel_delta = Kinematics.inverseKinematics(new Twist2d(0, 0, delta_heading.getRadians()));
    	updatePositionSetpoint(getLeftDistanceInches() + wheel_delta.left, getRightDistanceInches() + wheel_delta.right);
    }
    
	 /**
     * Configures talons for velocity control
     */
    private void configureTalonsForSpeedControl() {
        if (!usesTalonVelocityControl(mDriveControlState)) {
            // We entered a velocity control state.
            leftMaster.changeControlMode(CANTalon.TalonControlMode.Speed);
            leftMaster.setNominalClosedLoopVoltage(12.0);
            leftMaster.setProfile(kHighGearVelocityControlSlot);
            leftMaster.configNominalOutputVoltage(Constants.kDriveHighGearNominalOutput,
                    -Constants.kDriveHighGearNominalOutput);
            rightMaster.changeControlMode(CANTalon.TalonControlMode.Speed);
            rightMaster.setNominalClosedLoopVoltage(12.0);
            rightMaster.setProfile(kHighGearVelocityControlSlot);
            rightMaster.configNominalOutputVoltage(Constants.kDriveHighGearNominalOutput,
                    -Constants.kDriveHighGearNominalOutput);
            setBrakeMode(true);
        }
    }
    
    /**
     * Configures talons for position control
     */
    private void configureTalonsForPositionControl() {
        if (!usesTalonPositionControl(mDriveControlState)) {
            // We entered a position control state.
            leftMaster.changeControlMode(CANTalon.TalonControlMode.MotionMagic);
            leftMaster.setNominalClosedLoopVoltage(12.0);
            leftMaster.setProfile(kLowGearPositionControlSlot);
            leftMaster.configNominalOutputVoltage(Constants.kDriveLowGearNominalOutput,
                    -Constants.kDriveLowGearNominalOutput);
            leftMaster.setAllowableClosedLoopErr(0);
            rightMaster.changeControlMode(CANTalon.TalonControlMode.MotionMagic);
            rightMaster.setNominalClosedLoopVoltage(12.0);
            rightMaster.setProfile(kLowGearPositionControlSlot);
            rightMaster.configNominalOutputVoltage(Constants.kDriveLowGearNominalOutput,
                    -Constants.kDriveLowGearNominalOutput);
            rightMaster.setAllowableClosedLoopErr(0);
            setBrakeMode(true);
        }
    }
    
    private synchronized void updateVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
        if (usesTalonVelocityControl(mDriveControlState)) {
            final double max_desired = Math.max(Math.abs(left_inches_per_sec), Math.abs(right_inches_per_sec));
            final double scale = max_desired > Constants.kDriveHighGearMaxSetpoint
                    ? Constants.kDriveHighGearMaxSetpoint / max_desired : 1.0;
            leftMaster.set(inchesPerSecondToRpm(left_inches_per_sec * scale));
            rightMaster.set(inchesPerSecondToRpm(right_inches_per_sec * scale));
        } else {
            System.out.println("Hit a bad velocity control state");
            leftMaster.set(0);
            rightMaster.set(0);
        }
    }
    
    private synchronized void updatePositionSetpoint(double left_position_inches, double right_position_inches) {
        if (usesTalonPositionControl(mDriveControlState)) {
            leftMaster.set(inchesToRotations(left_position_inches));
            rightMaster.set(inchesToRotations(right_position_inches));
        } else {
            System.out.println("Hit a bad position control state");
            leftMaster.set(0);
            rightMaster.set(0);
        }
    }
	
	public double getLeftDistanceInches(){
		return rotationsToInches(leftMaster.getPosition());
	}
	
	public double getRightDistanceInches(){
		return rotationsToInches(rightMaster.getPosition());
	}
	
	public double getLeftVelocityInchesPerSec(){
		return rpmToInchesPerSecond(leftMaster.getSpeed());
	}
	
	public double getRightVelocityInchesPerSec(){
		return rpmToInchesPerSecond(rightMaster.getSpeed());
	}
	
	public double rotationsToInches(double rotations){
		return rotations * (Constants.kDriveWheelDiameterInches * Math.PI);
	}
	
	public double inchesToRotations(double inches){
		return inches / (Constants.kDriveWheelDiameterInches * Math.PI);
	}
	
	public double rpmToInchesPerSecond(double rpm){
		return rotationsToInches(rpm) / 60;
	}
	
	public double inchesPerSecondToRpm(double inchesPerSecond){
		return inchesToRotations(inchesPerSecond) * 60;
	}
	
	/**
     * Update the heading at which the robot thinks the boiler is.
     * 
     * Is called periodically when the robot is auto-aiming towards the boiler.
     */
    private void updateGoalHeading(double timestamp) {
        Optional<ShooterAimingParameters> aim = mRobotState.getAimingParameters();
        if (aim.isPresent()) {
            mTargetHeading = aim.get().getRobotToGoal();
        }
    }
    
    /**
     * Turn the robot to a target heading.
     * 
     * Is called periodically when the robot is auto-aiming towards the boiler.
     */
    private void updateTurnToHeading(double timestamp) {
        if (Shooter.getInstance().isShooting()) {
            // Do not update heading while shooting - just base lock. By not updating the setpoint, we will fight to
            // keep position.
            return;
        }
        final Rotation2d field_to_robot = mRobotState.getLatestFieldToVehicle().getValue().getRotation();

        // Figure out the rotation necessary to turn to face the goal.
        final Rotation2d robot_to_target = field_to_robot.inverse().rotateBy(mTargetHeading);

        // Check if we are on target
        final double kGoalPosTolerance = 0.75; // degrees
        final double kGoalVelTolerance = 5.0; // inches per second
        if (Math.abs(robot_to_target.getDegrees()) < kGoalPosTolerance
                && Math.abs(getLeftVelocityInchesPerSec()) < kGoalVelTolerance
                && Math.abs(getRightVelocityInchesPerSec()) < kGoalVelTolerance) {
            // Use the current setpoint and base lock.
            mIsOnTarget = true;
            updatePositionSetpoint(getLeftDistanceInches(), getRightDistanceInches());
            return;
        }

        Kinematics.DriveVelocity wheel_delta = Kinematics
                .inverseKinematics(new Twist2d(0, 0, robot_to_target.getRadians()));
        updatePositionSetpoint(wheel_delta.left + getLeftDistanceInches(),
                wheel_delta.right + getRightDistanceInches());
    }
    
    /**
     * Essentially does the same thing as updateTurnToHeading but sends the robot into the DRIVE_TOWARDS_GOAL_APPROACH
     * state if it detects we are not at an optimal shooting range
     */
    private void updateDriveTowardsGoalCoarseAlign(double timestamp) {
        updateGoalHeading(timestamp);
        updateTurnToHeading(timestamp);
        mIsApproaching = true;
        if (mIsOnTarget) {
            // Done coarse alignment.

            Optional<ShooterAimingParameters> aim = mRobotState.getAimingParameters();
            if (aim.isPresent()) {
                final double distance = aim.get().getRange();

                if (Math.abs(distance - Constants.kOptimalShootingRange) < 1.0) {
                    // Don't drive, just shoot.
                    mDriveControlState = DriveControlState.AIM_TO_GOAL;
                    mIsApproaching = false;
                    mIsOnTarget = false;
                    updatePositionSetpoint(getLeftDistanceInches(), getRightDistanceInches());
                    return;
                }
            }

            mDriveControlState = DriveControlState.DRIVE_TOWARDS_GOAL_APPROACH;
            mIsOnTarget = false;
        }
    }
    
    /**
     * Drives the robot straight forwards until it is at an optimal shooting distance. Then sends the robot into the
     * AIM_TO_GOAL state for one final alignment
     */
    private void updateDriveTowardsGoalApproach(double timestamp) {
        Optional<ShooterAimingParameters> aim = mRobotState.getAimingParameters();
        mIsApproaching = true;
        if (aim.isPresent()) {
            final double distance = aim.get().getRange();
            double error = distance - Constants.kOptimalShootingRange;
            final double kGoalPosTolerance = 1.0; // inches
            if (Util.epsilonEquals(error, 0.0, kGoalPosTolerance)) {
                // We are on target. Switch back to auto-aim.
                mDriveControlState = DriveControlState.AIM_TO_GOAL;
                RobotState.getInstance().resetVision();
                mIsApproaching = false;
                updatePositionSetpoint(getLeftDistanceInches(), getRightDistanceInches());
                return;
            }
            updatePositionSetpoint(getLeftDistanceInches() + error, getRightDistanceInches() + error);
        } else {
            updatePositionSetpoint(getLeftDistanceInches(), getRightDistanceInches());
        }
    }
    
    /**
     * Called periodically when the robot is in path following mode. Updates the path follower with the robots latest
     * pose, distance driven, and velocity, the updates the wheel velocity setpoints.
     */
    private void updatePathFollower(double timestamp) {
        RigidTransform2d robot_pose = mRobotState.getLatestFieldToVehicle().getValue();
        Twist2d command = mPathFollower.update(timestamp, robot_pose,
                RobotState.getInstance().getDistanceDriven(), RobotState.getInstance().getPredictedVelocity().dx);
        if (!mPathFollower.isFinished()) {
            Kinematics.DriveVelocity setpoint = Kinematics.inverseKinematics(command);
            updateVelocitySetpoint(setpoint.left, setpoint.right);
        } else {
            updateVelocitySetpoint(0, 0);
        }
    }

    public synchronized boolean isOnTarget() {
        return mIsOnTarget;
    }

    public synchronized boolean isAutoAiming() {
        return mDriveControlState == DriveControlState.AIM_TO_GOAL;
    }

    /**
     * Configures the drivebase for auto aiming
     */
    public synchronized void setWantAimToGoal() {
        if (mDriveControlState != DriveControlState.AIM_TO_GOAL) {
            mIsOnTarget = false;
            configureTalonsForPositionControl();
            mDriveControlState = DriveControlState.AIM_TO_GOAL;
            updatePositionSetpoint(getLeftDistanceInches(), getRightDistanceInches());
            mTargetHeading = Rotation2d.fromDegrees(pidgey.getAngle());
        }
    }
    
    public synchronized void reloadGains() {
        leftMaster.setPID(Constants.kDriveLowGearPositionKp, Constants.kDriveLowGearPositionKi,
                Constants.kDriveLowGearPositionKd, Constants.kDriveLowGearPositionKf,
                Constants.kDriveLowGearPositionIZone, Constants.kDriveLowGearPositionRampRate,
                kLowGearPositionControlSlot);
        leftMaster.setMotionMagicCruiseVelocity(Constants.kDriveLowGearMaxVelocity);
        leftMaster.setMotionMagicAcceleration(Constants.kDriveLowGearMaxAccel);
        rightMaster.setPID(Constants.kDriveLowGearPositionKp, Constants.kDriveLowGearPositionKi,
                Constants.kDriveLowGearPositionKd, Constants.kDriveLowGearPositionKf,
                Constants.kDriveLowGearPositionIZone, Constants.kDriveLowGearPositionRampRate,
                kLowGearPositionControlSlot);
        rightMaster.setMotionMagicCruiseVelocity(Constants.kDriveLowGearMaxVelocity);
        rightMaster.setMotionMagicAcceleration(Constants.kDriveLowGearMaxAccel);

        leftMaster.setPID(Constants.kDriveHighGearVelocityKp, Constants.kDriveHighGearVelocityKi,
                Constants.kDriveHighGearVelocityKd, Constants.kDriveHighGearVelocityKf,
                Constants.kDriveHighGearVelocityIZone, Constants.kDriveHighGearVelocityRampRate,
                kHighGearVelocityControlSlot);
        rightMaster.setPID(Constants.kDriveHighGearVelocityKp, Constants.kDriveHighGearVelocityKi,
                Constants.kDriveHighGearVelocityKd, Constants.kDriveHighGearVelocityKf,
                Constants.kDriveHighGearVelocityIZone, Constants.kDriveHighGearVelocityRampRate,
                kHighGearVelocityControlSlot);
    }
	
	@Override
	public synchronized void stop(){
		setOpenLoop(DriveSignal.NEUTRAL);
	}
	
	@Override
	public void outputToSmartDashboard(){
		SmartDashboard.putNumber("Left Drive Encoder", leftMaster.getPosition());
		SmartDashboard.putNumber("Right Drive Encoder", rightMaster.getPosition());
		SmartDashboard.putNumber("Right Master Voltage", rightMaster.getOutputVoltage());
		SmartDashboard.putNumber("Right Slave Voltage", rightSlave.getOutputVoltage());
		SmartDashboard.putNumber("Left Master Voltage", leftMaster.getOutputVoltage());
		SmartDashboard.putNumber("Left Slave Voltage", leftSlave.getOutputVoltage());
		SmartDashboard.putNumber("Right Master Current", rightMaster.getOutputCurrent());
		SmartDashboard.putNumber("Left Master Current", leftMaster.getOutputCurrent());
		SmartDashboard.putNumber("Left Slave Current", leftSlave.getOutputCurrent());
		SmartDashboard.putNumber("Right Slave Current", rightSlave.getOutputCurrent());
		
		SmartDashboard.putNumber("Left Drive Speed", getLeftVelocityInchesPerSec());
		SmartDashboard.putNumber("Right Drive Speed", getRightVelocityInchesPerSec());
		SmartDashboard.putNumber("Left Enc Velocity", leftMaster.getEncVelocity());
		SmartDashboard.putNumber("Right Enc Velocity", rightMaster.getEncVelocity());
		
		SmartDashboard.putNumber("Left Drive Setpoint", leftMaster.getSetpoint());
		SmartDashboard.putNumber("Right Drive Setpoint", rightMaster.getSetpoint());
	}
	
	@Override
	public synchronized void zeroSensors(){
		rightMaster.setPosition(0);
		leftMaster.setPosition(0);
	}
	
	
}
