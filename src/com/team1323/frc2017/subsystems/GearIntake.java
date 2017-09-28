package com.team1323.frc2017.subsystems;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;
import com.team1323.frc2017.Ports;
import com.team1323.frc2017.loops.Loop;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GearIntake extends Subsystem{
	private CANTalon intakeMotor;
	private Solenoid gearFlaps;
	
	private boolean isScoring = false;
	
	private static GearIntake instance = new GearIntake();
	public static GearIntake getInstance(){
		return instance;
	}
	
	public GearIntake(){
		intakeMotor = new CANTalon(Ports.GEAR_INTAKE);
		intakeMotor.setCurrentLimit(50);
		intakeMotor.EnableCurrentLimit(true);
		intakeMotor.changeControlMode(TalonControlMode.PercentVbus);
		gearFlaps = new Solenoid(20, Ports.LEFT_GEAR_FLAP);
	}
	
	public enum State{
		INTAKING, HOLDING, SCORING, IDLE
	}
	private State currentState = State.HOLDING;
	public State getState(){
		return currentState;
	}
	public void setState(State state){
		currentState = state;
	}
	
	private final Loop loop = new Loop(){
		@Override
		public void onStart(){
			setState(State.HOLDING);
		}
		
		@Override
		public void onLoop(){
			synchronized (GearIntake.this){
				switch(currentState){
				case INTAKING:
					retractGearFlaps();
					intakeForward();
					break;
				case HOLDING:
					retractGearFlaps();
					stopIntake();
					break;
				case SCORING:
					break;
				case IDLE:
					stopIntake();
					extendGearFlaps();
					break;
				default:
					break;
				}
			}
		}
		
		@Override
		public void onStop(){
			setState(State.HOLDING);
		}
	};
	public Loop getLoop(){
		return loop;
	}
	
	public void intakeForward(){
		intakeMotor.set(-1.0);
	}
	
	public void intakeReverse(){
		intakeMotor.set(1.0);
	}
	
	public void stopIntake(){
		intakeMotor.set(0);
	}
	
	public void extendGearFlaps(){
		gearFlaps.set(true);
	}
	
	public void retractGearFlaps(){
		gearFlaps.set(false);
	}
	
	@Override
	public synchronized void stop(){
		stopIntake();
		retractGearFlaps();
		setState(State.HOLDING);
	}
	
	public void score(){
		setState(State.SCORING);
		if(!isScoring){
			ScoringThread t = new ScoringThread();
			t.start();
		}
	}
	
	public class ScoringThread extends Thread{
		public void run(){
			isScoring = true;
			intakeForward();
			Timer.delay(0.2);
			extendGearFlaps();
			Timer.delay(0.5);
			stopIntake();
			setState(GearIntake.State.IDLE);
			isScoring = false;
		}
	}
	
	@Override
	public synchronized void zeroSensors(){
		//no-op
	}
	
	@Override
	public void outputToSmartDashboard(){
		SmartDashboard.putNumber("Gear Intake Voltage", intakeMotor.getOutputVoltage());
		SmartDashboard.putNumber("Gear Intake Current", intakeMotor.getOutputCurrent());
	}
}
