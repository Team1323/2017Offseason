package com.team1323.frc2017.subsystems;

import com.ctre.CANTalon;
import com.ctre.CANTalon.StatusFrameRate;
import com.ctre.CANTalon.TalonControlMode;
import com.team1323.frc2017.Ports;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DoubleDyeRotor extends Subsystem{
	private CANTalon leftInner, rightInner, arm, slideFeeder;
	
	private boolean isInThread = false;
	private boolean isFeeding = false;
	public boolean isFeeding(){
		return isFeeding;
	}
	
	private static DoubleDyeRotor instance = null;
	public static DoubleDyeRotor getInstance(){
		if(instance == null){
			instance = new DoubleDyeRotor();
		}
		return instance;
	}
	public DoubleDyeRotor(){
		leftInner = new CANTalon(Ports.DYE_ROLLER_1);
		rightInner = new CANTalon(Ports.DYE_ROLLER_2);
		arm = new CANTalon(Ports.DYE_ARMS);
		slideFeeder = new CANTalon(Ports.SLIDE_FEEDER);
		
		leftInner.setStatusFrameRateMs(StatusFrameRate.Feedback, 20);
		rightInner.setStatusFrameRateMs(StatusFrameRate.Feedback, 20);
		arm.setStatusFrameRateMs(StatusFrameRate.Feedback, 20);
		slideFeeder.setStatusFrameRateMs(StatusFrameRate.Feedback, 20);
		
		leftInner.enableBrakeMode(false);
		rightInner.enableBrakeMode(false);
		arm.enableBrakeMode(false);
		slideFeeder.enableBrakeMode(false);
		
		leftInner.changeControlMode(TalonControlMode.Voltage);
		rightInner.changeControlMode(TalonControlMode.Voltage);
		arm.changeControlMode(TalonControlMode.Voltage);
		slideFeeder.changeControlMode(TalonControlMode.Voltage);
		
		leftInner.setVoltageRampRate(96.0);
		rightInner.setVoltageRampRate(96.0);
		arm.setVoltageRampRate(12.0);
		slideFeeder.setVoltageRampRate(96.0);
		
		slideFeeder.setCurrentLimit(30);
		slideFeeder.EnableCurrentLimit(true);
		arm.setCurrentLimit(60);
		arm.EnableCurrentLimit(true);
	}
	
	public void rightRollerForward(){
		rightInner.set(12.0);
	}
	
	public void leftRollerForward(){
		leftInner.set(-12.0);
	}
	
	public void rollersForward(){
		rightRollerForward();
		leftRollerForward();
	}
	
	public void reverseRollers(){
		isFeeding = false;
		rightInner.set(-12.0);
		leftInner.set(12.0);
	}
	
	public void armsForward(){
		arm.set(-0.8*12.0);
	}
	
	public void armsReversed(){
		arm.set(12.0);
	}
	
	public void slideFeederForward(){
		slideFeeder.set(12.0);
	}
	
	public void stopRollers(){
		rightInner.set(0);
		leftInner.set(0);
	}
	
	public void stopArms(){
		arm.set(0);
	}
	
	public void stopSlideFeeder(){
		slideFeeder.set(0);
	}
	
	public void startFeeding(){
		if(!isInThread){
			StartFeeding t = new StartFeeding();
			t.start();
		}
	}
	
	public class StartFeeding extends Thread{
		public void run(){
			isInThread = true;
			isFeeding = true;
			slideFeederForward();
			try {
				Thread.sleep(250);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			rightRollerForward();
			leftRollerForward();
			try {
				Thread.sleep(250);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			armsForward();
			isInThread = false;
		}
	}
	
	@Override
	public synchronized void stop(){
		isFeeding = false;
		rightInner.set(0);
		leftInner.set(0);
		arm.set(0);
		slideFeeder.set(0);
	}
	
	@Override
	public synchronized void zeroSensors(){
		//no-op
	}
	
	@Override
	public void outputToSmartDashboard(){
		SmartDashboard.putNumber("Left Dye Roller Voltage", leftInner.getOutputVoltage());
		SmartDashboard.putNumber("Right Dye Roller Voltage", rightInner.getOutputVoltage());
		SmartDashboard.putNumber("Dye Arm Voltage", arm.getOutputVoltage());
		SmartDashboard.putNumber("Left Dye Roller Current", leftInner.getOutputCurrent());
		SmartDashboard.putNumber("Right Dye Roller Current", rightInner.getOutputCurrent());
		SmartDashboard.putNumber("Dye Arm Current", arm.getOutputCurrent());
		SmartDashboard.putNumber("Slide Feeder Current", slideFeeder.getOutputCurrent());
	}
}
