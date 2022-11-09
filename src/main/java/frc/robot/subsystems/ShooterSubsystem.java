package frc.robot.subsystems;

import org.xero1425.XeroRobot;
import org.xero1425.subsystems.XeroSubsystem;

public class ShooterSubsystem extends XeroSubsystem {
    private final static String SubsystemName = "shooter" ;

    public ShooterSubsystem(XeroRobot robot) throws Exception {
        super(robot, SubsystemName) ;
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }
  
    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }

    public void stop() {
    }

    public void setShootingParams(double hood, double velocity) {
    }

    public double getHood() {
        return 0.0 ;
    }

    public double getVelocity() {
        return 0.0 ;
    }
}
