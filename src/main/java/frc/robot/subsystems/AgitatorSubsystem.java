package frc.robot.subsystems;

import org.xero1425.XeroRobot;
import org.xero1425.motors.BadMotorRequestException;
import org.xero1425.motors.MotorController;
import org.xero1425.motors.MotorRequestFailedException;
import org.xero1425.subsystems.motorsubsystem.XeroMotorSubsystem;

public class AgitatorSubsystem extends XeroMotorSubsystem {
    private final static String SubsystemName = "agitator" ;

    private MotorController motor_ ;
    private double power_on_ ;
    private double power_eject_ ;

    public AgitatorSubsystem(XeroRobot robot) throws Exception {
        super(robot, SubsystemName) ;

        power_on_ = getSetting("power:on").getDouble() ;
        power_eject_ = getSetting("power:off").getDouble() ;
    }

    @Override
    public void periodic() {
        super.periodic();
    }
  
    @Override
    public void simulationPeriodic() {
        super.simulationPeriodic();
    }

    public void on() throws BadMotorRequestException, MotorRequestFailedException {
        motor_.set(power_on_) ;
    }

    public void eject() throws BadMotorRequestException, MotorRequestFailedException {
        motor_.set(power_eject_) ;
    }

    public void off() throws BadMotorRequestException, MotorRequestFailedException {
        motor_.set(0.0) ;
    }
}
