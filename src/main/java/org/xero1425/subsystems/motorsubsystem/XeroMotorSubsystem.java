package org.xero1425.subsystems.motorsubsystem;

import org.xero1425.XeroRobot;
import org.xero1425.motors.MotorController;
import org.xero1425.subsystems.XeroSubsystem;

public class XeroMotorSubsystem extends XeroSubsystem {
    private MotorController motor_ ;
    private double power_ ;

    public XeroMotorSubsystem(XeroRobot robot, String name) throws Exception {
        super(robot, name) ;

        motor_ = robot.getMotorFactory().createMotor(getName(), "subsystems:" + getName() + ":hw:motors") ;
    }

    @Override
    public void periodic() {
        super.periodic();
    }
  
    @Override
    public void simulationPeriodic() {
        super.simulationPeriodic();
    }

    public MotorController getMotorController() {
        return motor_ ;
    }

    public void setPower(double p) {
        power_ = p ;
    }

    public double getPower() {
        return power_ ;
    }
}
