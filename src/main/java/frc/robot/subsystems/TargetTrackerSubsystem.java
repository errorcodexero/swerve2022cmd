package frc.robot.subsystems;

import org.xero1425.XeroRobot;
import org.xero1425.subsystems.XeroSubsystem;

import frc.robot.subsystems.LimeLightSubsystem.LedMode;

public class TargetTrackerSubsystem extends XeroSubsystem {
    private final static String SubsystemName = "tracker" ;

    private LimeLightSubsystem limelight_ ;
    private TurretSubsystem turret_ ;
    private boolean tracking_ ;
    private boolean hasTarget_ ;
    private int lastCount_ ;
    private double distance_ ;
    private double desiredAngle_ ;

    private double target_tracker_offset_angle_ ;
    private double threshold_ ;
    private int lost_count_ ;

    public TargetTrackerSubsystem(XeroRobot robot, LimeLightSubsystem limelight, TurretSubsystem turret) throws Exception {
        super(robot, SubsystemName) ;
        
        limelight_ = limelight ;
        turret_ = turret ;
        tracking_ = false ;
        lastCount_ = 0 ;
        hasTarget_ = false ;

        target_tracker_offset_angle_ = getSetting("camera-offset-angle").getDouble() ;
        threshold_ = getSetting("threshold").getDouble() ;
        lost_count_ = getSetting("lost-count").getInteger() ;
    }
    
    @Override
    public void periodic() {
        if (tracking_) {
            if (limelight_.isTargetDetected()) {
                hasTarget_ = true ;
                lastCount_ = 0 ;
                distance_ = limelight_.getDistance() ;
                desiredAngle_ = -(limelight_.getYaw() - target_tracker_offset_angle_) + turret_.getPosition() ;
            }
            else {                
                lastCount_++ ;
                if (lastCount_ > lost_count_) {
                    hasTarget_ = false ;
                }
            }
        }
        else {
            hasTarget_ = false ;
            distance_ = 0.0 ;
            desiredAngle_ = 0.0 ;
        }

        turret_.setTarget(desiredAngle_, threshold_);
    }
  
    @Override
    public void simulationPeriodic() {
    }

    public double getDistance() {
        return distance_ ;
    }

    public boolean hasTarget() {
        return hasTarget_ ;
    }

    public double desiredAngle() {
        return desiredAngle_ ;
    }

    public void startTracking() {
        limelight_.setLedMode(LedMode.ForceOn);
        tracking_ = true ;
    }

    public void stopTracking() {
        limelight_.setLedMode(LedMode.ForceOff);
        tracking_ = false ;
    }
}
