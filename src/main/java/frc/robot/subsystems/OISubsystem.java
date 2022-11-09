package frc.robot.subsystems;

import org.xero1425.XeroRobot;
import org.xero1425.subsystems.XeroOISubsystem;

public class OISubsystem extends XeroOISubsystem {
    private final static String SubsystemName = "oi" ;

    public OISubsystem(XeroRobot robot, XeroOISubsystem.DriverType type) throws Exception {
        super(robot, SubsystemName, type) ;
    }

    @Override
    public void periodic() {
    }
  
    @Override
    public void simulationPeriodic() {
    }
}
