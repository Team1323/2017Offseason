package com.team1323.frc2017.subsystems;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;
import com.team1323.frc2017.Constants;
import com.team1323.frc2017.Ports;

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
		leftMaster.SetVelocityMeasurementPeriod(CANTalon.VelocityMeasurementPeriod.Period_10Ms);
		leftMaster.SetVelocityMeasurementWindow(32);
		leftMaster.reverseOutput(false);
		leftMaster.reverseSensor(false);
		leftMaster.setPID(4.0, 0.00, 40, 0.027, 0, 0.0, 0);
		leftMaster.setProfile(0);
		leftMaster.configNominalOutputVoltage(+0f, -0f);
		leftMaster.configPeakOutputVoltage(+12f, -0f);
		leftMaster.setNominalClosedLoopVoltage(12.0f);
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
		
	}
}
