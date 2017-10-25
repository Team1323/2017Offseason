package com.team1323.frc2017;

public class Constants {
	public static final double kLooperDt = 0.005;
	
	public static final double kDriveWheelDiameterInches = 4.0;
	public static final double kTrackWidthInches = 31.5;
    public static final double kTrackScrubFactor = 1.0;
	
	public static final double kShootingSpeed = 3100.0;
	public static final double kShooterAllowableError = 100.0;//rpm
	public static final double kOptimalShootingRange = 85.6;
    
 // Goal tracker constants
    public static final double kMaxGoalTrackAge = 1.0;
    public static final double kMaxTrackerDistance = 18.0;
    public static final double kCameraFrameRate = 30.0;
    public static final double kTrackReportComparatorStablityWeight = 1.0;
    public static final double kTrackReportComparatorAgeWeight = 1.0;
    
 // Pose of the camera frame w.r.t. the robot frame
    public static final double kCameraXOffset = 0.0;
    public static final double kCameraYOffset = 0.0;
    public static final double kCameraZOffset = 22.75;
    public static final double kCameraPitchAngleDegrees = 90-58.5;
    public static final double kCameraYawAngleDegrees = 0.0;
    public static final double kCameraDeadband = 0.0;
    
    public static final double kBoilerTargetTopHeight = 88.0;
    public static final double kBoilerRadius = 7.5;
    
 // PID gains for drive velocity loop (HIGH GEAR)
    // Units: setpoint, error, and output are in inches per second.
    public static double kDriveHighGearVelocityKp = 0.6;//1.2;
    public static double kDriveHighGearVelocityKi = 0.0;
    public static double kDriveHighGearVelocityKd = 144.0;
    public static double kDriveHighGearVelocityKf = 1023.0/5190.0;
    public static int kDriveHighGearVelocityIZone = 0;
    public static double kDriveHighGearVelocityRampRate = 240.0;
    public static double kDriveHighGearNominalOutput = 0.5;
    public static double kDriveHighGearMaxSetpoint = 13.5 * 12.0; // 13.5 fps
    
    public static final double kDriveLowGearPositionKp = 0.8;
    public static final double kDriveLowGearPositionKi = 0.002;
    public static final double kDriveLowGearPositionKd = 254.0;
    public static final double kDriveLowGearPositionKf = 1023.0/5190.0;
    public static final int kDriveLowGearPositionIZone = 300;
    public static final double kDriveLowGearPositionRampRate = 240.0; // V/s
    public static final double kDriveLowGearNominalOutput = 0.5; // V
    public static final double kDriveLowGearMaxVelocity = 10.0 * 12.0 * 60.0 / (Math.PI * kDriveWheelDiameterInches); // 10 fps
                                                                                                               // in RPM
    public static final double kDriveLowGearMaxAccel = 18.0 * 12.0 * 60.0 / (Math.PI * kDriveWheelDiameterInches); // 18 fps/s
                                                                                                             // in RPM/s
    
 // Path following constants
    public static double kMinLookAhead = 12.0; // inches
    public static double kMinLookAheadSpeed = 9.0; // inches per second
    public static double kMaxLookAhead = 24.0; // inches
    public static double kMaxLookAheadSpeed = 120.0; // inches per second
    public static double kDeltaLookAhead = kMaxLookAhead - kMinLookAhead;
    public static double kDeltaLookAheadSpeed = kMaxLookAheadSpeed - kMinLookAheadSpeed;

    public static double kInertiaSteeringGain = 0.001;//0.0125; // angular velocity command is multiplied by this gain *
                                                     // our speed
                                                     // in inches per sec
    public static double kSegmentCompletionTolerance = 0.1; // inches
    public static double kPathFollowingMaxAccel = 120.0; // inches per second^2
    public static double kPathFollowingMaxVel = 120.0; // inches per second
    public static double kPathFollowingProfileKp = 1.0;
    public static double kPathFollowingProfileKi = 0.03;
    public static double kPathFollowingProfileKv = 0.02;
    public static double kPathFollowingProfileKffv = 1.0;
    public static double kPathFollowingProfileKffa = 0.0;
    public static double kPathFollowingGoalPosTolerance = 0.75;
    public static double kPathFollowingGoalVelTolerance = 12.0;
    public static double kPathStopSteeringDistance = 9.0;
    
 // Phone
    public static final int kAndroidAppTcpPort = 8254;
}
