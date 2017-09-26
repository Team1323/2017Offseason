package com.team1323.frc2017.subsystems;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;
import com.team1323.frc2017.Ports;
import com.team1323.frc2017.loops.Loop;
import com.team1323.lib.util.DriveSignal;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drive extends Subsystem{
	private final CANTalon leftMaster, leftSlave, rightMaster, rightSlave;
	private final Solenoid shifter;
	private boolean isHighGear = false;
	public boolean isHighGear(){
		return isHighGear;
	}
	
	public enum ControlState{
		OPEN_LOOP
	}
	private ControlState controlState = ControlState.OPEN_LOOP;
	public ControlState getControlState(){
		return controlState;
	}
	
	private static Drive instance = new Drive();
	
	public static Drive getInstance(){
		return instance;
	}
	
	private Drive(){
		leftMaster = new CANTalon(Ports.DRIVE_LEFT_MASTER);
		leftSlave = new CANTalon(Ports.DRIVE_LEFT_SLAVE);
		rightMaster = new CANTalon(Ports.DRIVE_RIGHT_MASTER);
		rightSlave = new CANTalon(Ports.DRIVE_RIGHT_SLAVE);
		
		shifter = new Solenoid(20, Ports.DRIVE_SHIFTER);
		setHighGear(true);
		
		leftMaster.setStatusFrameRateMs(CANTalon.StatusFrameRate.Feedback, 10);
		rightMaster.setStatusFrameRateMs(CANTalon.StatusFrameRate.Feedback, 10);
		
		leftMaster.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		rightMaster.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		
		leftMaster.changeControlMode(TalonControlMode.PercentVbus);
		leftMaster.set(0);
		leftSlave.changeControlMode(TalonControlMode.Follower);
		leftSlave.set(Ports.DRIVE_LEFT_MASTER);
		rightMaster.changeControlMode(TalonControlMode.PercentVbus);
		rightMaster.set(0);
		rightSlave.changeControlMode(TalonControlMode.Follower);
		rightSlave.set(Ports.DRIVE_RIGHT_MASTER);
		
		setOpenLoop(DriveSignal.NEUTRAL);
	}
	
	private final Loop loop = new Loop(){
		@Override
		public void onStart(){
			setOpenLoop(DriveSignal.NEUTRAL);
		}
		
		@Override
		public void onLoop(){
			synchronized (Drive.this){
				switch(controlState){
				case OPEN_LOOP:
					return;
				default:
					break;
				}
			}
		}
		
		@Override
		public void onStop(){
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
		if(controlState != ControlState.OPEN_LOOP){
			leftMaster.changeControlMode(TalonControlMode.PercentVbus);
			rightMaster.changeControlMode(TalonControlMode.PercentVbus);
		}
		setLeftRightPower(signal.leftMotor, signal.rightMotor);
	}
	
	public void setHighGear(boolean highGear){
		isHighGear = highGear;
		shifter.set(highGear);
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
	}
	
	@Override
	public synchronized void zeroSensors(){
		rightMaster.setPosition(0);
		leftMaster.setPosition(0);
	}
	
	
}
