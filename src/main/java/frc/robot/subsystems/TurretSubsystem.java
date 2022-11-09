package frc.robot.subsystems;

import org.xero1425.XeroRobot;
import org.xero1425.subsystems.motorsubsystem.XeroMotorEncoderSubsystem;

public class TurretSubsystem extends XeroMotorEncoderSubsystem {
    private final static String SubsystemName = "turret" ;

    public TurretSubsystem(XeroRobot robot) throws Exception {
        super(robot, SubsystemName, true) ;        
    }

    @Override
    public void periodic() {
        super.periodic();
    }
  
    @Override
    public void simulationPeriodic() {
    }
}
