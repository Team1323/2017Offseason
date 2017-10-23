package com.team1323.frc2017.subsystems;

import com.ctre.CANTalon;
import com.ctre.CANTalon.StatusFrameRate;
import com.ctre.CANTalon.TalonControlMode;
import com.team1323.frc2017.Ports;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DoubleDyeRotor extends Subsystem{
	private CANTalon leftInner, rightInner, arm, slideFeeder;
	
	private boolean isInThread = false;
	
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
		
		leftInner.setStatusFrameRateMs(StatusFrameRate.Feedback, 1000);
		rightInner.setStatusFrameRateMs(StatusFrameRate.Feedback, 1000);
		arm.setStatusFrameRateMs(StatusFrameRate.Feedback, 1000);
		slideFeeder.setStatusFrameRateMs(StatusFrameRate.Feedback, 1000);
		
		leftInner.enableBrakeMode(false);
		rightInner.enableBrakeMode(false);
		arm.enableBrakeMode(false);
		slideFeeder.enableBrakeMode(false);
		
		leftInner.changeControlMode(TalonControlMode.PercentVbus);
		rightInner.changeControlMode(TalonControlMode.PercentVbus);
		arm.changeControlMode(TalonControlMode.PercentVbus);
		slideFeeder.changeControlMode(TalonControlMode.PercentVbus);
		
		leftInner.setVoltageRampRate(24.0);
		rightInner.setVoltageRampRate(24.0);
		arm.setVoltageRampRate(24.0);
		slideFeeder.setVoltageRampRate(24.0);
		
		slideFeeder.setCurrentLimit(30);
		slideFeeder.EnableCurrentLimit(true);
	}
	
	public void rightRollerForward(){
		rightInner.set(1.0);
	}
	
	public void leftRollerForward(){
		leftInner.set(-1.0);
	}
	
	public void rollersForward(){
		rightRollerForward();
		leftRollerForward();
	}
	
	public void reverseRollers(){
		rightInner.set(-1.0);
		leftInner.set(1.0);
	}
	
	public void armsForward(){
		arm.set(-1.0);
	}
	
	public void slideFeederForward(){
		slideFeeder.set(1.0);
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
			rightRollerForward();
			leftRollerForward();
			slideFeederForward();
			try {
				Thread.sleep(500);
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
		/*SmartDashboard.putNumber("Left Dye Roller Voltage", leftInner.getOutputVoltage());
		SmartDashboard.putNumber("Right Dye Roller Voltage", rightInner.getOutputVoltage());
		SmartDashboard.putNumber("Dye Arm Voltage", arm.getOutputVoltage());
		SmartDashboard.putNumber("Left Dye Roller Current", leftInner.getOutputCurrent());
		SmartDashboard.putNumber("Right Dye Roller Current", rightInner.getOutputCurrent());
		SmartDashboard.putNumber("Dye Arm Current", arm.getOutputCurrent());
		SmartDashboard.putNumber("Slide Feeder Current", slideFeeder.getOutputCurrent());
		if(slideFeeder.getOutputCurrent() > 0.5){
		System.out.println(slideFeeder.getOutputCurrent());
		}*/
	}
}
