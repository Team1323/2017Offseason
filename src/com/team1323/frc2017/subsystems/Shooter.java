package com.team1323.frc2017.subsystems;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;
import com.team1323.frc2017.Constants;
import com.team1323.frc2017.Ports;
import com.team1323.frc2017.loops.Loop;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends Subsystem{
	private CANTalon rightSlave1, rightSlave2, leftMaster, leftSlave;
	private static Shooter instance = new Shooter();
	public static Shooter getInstance(){
		return instance;
	}
	public Shooter(){
		leftMaster = new CANTalon(Ports.SHOOTER_MASTER);
		leftSlave = new CANTalon(Ports.SHOOTER_SLAVE_LEFT);
		rightSlave1 = new CANTalon(Ports.SHOOTER_SLAVE_RIGHT_1);
		rightSlave2 = new CANTalon(Ports.SHOOTER_SLAVE_RIGHT_2);
		
		leftMaster.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		leftMaster.setStatusFrameRateMs(CANTalon.StatusFrameRate.General, 2);
		leftMaster.setStatusFrameRateMs(CANTalon.StatusFrameRate.AnalogTempVbat, 2);
		leftMaster.setStatusFrameRateMs(CANTalon.StatusFrameRate.Feedback, 5);
		leftMaster.SetVelocityMeasurementPeriod(CANTalon.VelocityMeasurementPeriod.Period_10Ms);
		leftMaster.SetVelocityMeasurementWindow(32);
		leftMaster.reverseOutput(false);
		leftMaster.reverseSensor(false);
		leftMaster.setPID(4.0, 0.00, 40.0, 0.0285, 0, 0.0, 0);
		leftMaster.setPID(2.0, 0.0, 0.0, 0.028, 0, 0.0, 1);
		leftMaster.setProfile(1);
		leftMaster.configNominalOutputVoltage(+0f, -0f);
		leftMaster.configPeakOutputVoltage(+12f, -0f);
		leftMaster.setNominalClosedLoopVoltage(12.0f);
		leftMaster.setVoltageRampRate(0.0);
		leftMaster.setAllowableClosedLoopErr(0);
		leftMaster.enableBrakeMode(false);
		leftMaster.changeControlMode(TalonControlMode.PercentVbus);
		leftMaster.set(0);
		
		leftSlave.reverseOutput(false);
		leftSlave.enableBrakeMode(false);
		leftSlave.changeControlMode(TalonControlMode.Follower);
		leftSlave.set(Ports.SHOOTER_MASTER);
		rightSlave1.reverseOutput(false);
		rightSlave1.enableBrakeMode(false);
		rightSlave1.changeControlMode(TalonControlMode.Follower);
		rightSlave1.set(Ports.SHOOTER_MASTER);
		rightSlave2.reverseOutput(false);
		rightSlave2.enableBrakeMode(false);
		rightSlave2.changeControlMode(TalonControlMode.Follower);
		rightSlave2.set(Ports.SHOOTER_MASTER);
		
		if(leftMaster.isSensorPresent(FeedbackDevice.CtreMagEncoder_Relative) != 
				CANTalon.FeedbackDeviceStatus.FeedbackStatusPresent){
			DriverStation.reportWarning("Could not detect shooter encoder!", false);
		}
	}
	
	public enum State{
		Off, SpinUp, Hold
	}
	private State currentState = State.Off;
	public State getState(){
		return currentState;
	}
	public void setState(State newState){
		currentState = newState;
	}
	
	private final Loop shooterLoop = new Loop(){
		@Override
		public void onStart(double timestamp){
			setState(State.Off);
			setOpenLoop(0.0);
		}
		
		@Override
		public void onLoop(double timestamp){
			synchronized(Shooter.this){
				switch(currentState){
				case Off:
					setOpenLoop(0.0);
					break;
				case SpinUp:
					leftMaster.setVoltageRampRate(240.0);
					setSpeed(Constants.kShootingSpeed);
					if(isOnTarget())setState(State.Hold);
					break;
				case Hold:
					leftMaster.setVoltageRampRate(0.0);
					break;
				}
			}
		}
		
		@Override
		public void onStop(double timestamp){
			setState(State.Off);
			setOpenLoop(0.0);
		}
	};
	
	public Loop getLoop(){
		return shooterLoop;
	}
	
	public void setSpeed(double rpm){
		leftMaster.changeControlMode(TalonControlMode.Speed);
		leftMaster.set(rpm);
	}
	
	public void setOpenLoop(double percent){
		leftMaster.changeControlMode(TalonControlMode.PercentVbus);
		leftMaster.set(percent);
	}
	
	public boolean isOnTarget(){
		return leftMaster.getControlMode() == CANTalon.TalonControlMode.Speed &&
				Math.abs(leftMaster.getSetpoint() - leftMaster.getSpeed()) < Constants.kShooterAllowableError;
	}
	
	public boolean isShooting(){
		return leftMaster.getControlMode() == CANTalon.TalonControlMode.Speed &&
				leftMaster.getSetpoint() != 0.0;
	}
	
	@Override
	public synchronized void stop(){
		setOpenLoop(0);
	}
	
	@Override
	public synchronized void zeroSensors(){
		//no-op
	}
	
	@Override
	public void outputToSmartDashboard(){
		SmartDashboard.putNumber("Shooter Speed", leftMaster.getSpeed());
		if(leftMaster.getSpeed() > 2500){
			System.out.println(leftMaster.getSpeed());
		}
		SmartDashboard.putNumber("Shooter Master Current", leftMaster.getOutputCurrent());
		SmartDashboard.putNumber("Shooter Slave 1 Current", leftSlave.getOutputCurrent());
		SmartDashboard.putNumber("Shooter Slave 2 Current", rightSlave1.getOutputCurrent());
		SmartDashboard.putNumber("Shooter Slave 3 Current", rightSlave2.getOutputCurrent());
	}
}
