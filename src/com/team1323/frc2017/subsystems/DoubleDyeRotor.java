package com.team1323.frc2017.subsystems;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;
import com.team1323.frc2017.Ports;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DoubleDyeRotor extends Subsystem{
	private CANTalon leftInner, rightInner, arm;
	
	private boolean isInThread = false;
	
	private static DoubleDyeRotor instance = new DoubleDyeRotor();
	public static DoubleDyeRotor getInstance(){
		return instance;
	}
	public DoubleDyeRotor(){
		leftInner = new CANTalon(Ports.SWEEPER_ROLLER_1);
		rightInner = new CANTalon(Ports.SWEEPER_ROLLER_2);
		arm = new CANTalon(Ports.SWEEPER_ARMS);
		
		leftInner.enableBrakeMode(false);
		rightInner.enableBrakeMode(false);
		arm.enableBrakeMode(false);
		
		leftInner.changeControlMode(TalonControlMode.PercentVbus);
		rightInner.changeControlMode(TalonControlMode.PercentVbus);
		arm.changeControlMode(TalonControlMode.PercentVbus);
	}
	
	public void rightRollerForward(){
		rightInner.set(-1.0);
	}
	
	public void leftRollerForward(){
		leftInner.set(1.0);
	}
	
	public void reverseRollers(){
		rightInner.set(1.0);
		leftInner.set(-1.0);
	}
	
	public void armsForward(){
		arm.set(0.8);
	}
	
	public void stopRollers(){
		rightInner.set(0);
		leftInner.set(0);
	}
	
	public void stopArms(){
		arm.set(0);
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
		rightInner.set(0);
		leftInner.set(0);
		arm.set(0);
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
	}
}
