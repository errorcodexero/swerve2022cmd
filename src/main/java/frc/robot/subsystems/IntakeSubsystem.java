package frc.robot.subsystems;

import org.xero1425.XeroRobot;
import org.xero1425.subsystems.Intake2MotorSubsystem;

public class IntakeSubsystem extends Intake2MotorSubsystem {
    private final static String SubsystemName = "intake" ;


    public IntakeSubsystem(XeroRobot robot) throws Exception {
        super(robot, SubsystemName) ;
    }

    @Override
    public void periodic() {
        super.periodic();
    }
        
    @Override
    public void simulationPeriodic() {
        super.simulationPeriodic();
    }
}
