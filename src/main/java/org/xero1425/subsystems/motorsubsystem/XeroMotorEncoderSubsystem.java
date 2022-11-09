package org.xero1425.subsystems.motorsubsystem;

import org.xero1425.XeroRobot;

public class XeroMotorEncoderSubsystem extends XeroMotorSubsystem {

    private XeroEncoder encoder_ ;
    private double target_ ;
    private double threshold_ ;
    private boolean tracking_ ;

    public XeroMotorEncoderSubsystem(XeroRobot robot, String name, boolean angle) throws Exception {
        super(robot, name) ;

        String encname = "subsystems:" + name + ":hw:encoder" ;
        encoder_ = new XeroEncoder(robot, encname, angle, getMotorController()) ;        

        tracking_ = false ;
    }    

    @Override
    public void periodic() {
        super.periodic();
    }
  
    @Override
    public void simulationPeriodic() {
        super.simulationPeriodic();
    }    

    public double getPosition() {
        return encoder_.getPosition() ;
    }

    public void setTarget(double target, double threshold) {
        target_ = target ;
        threshold_ = threshold ;
        tracking_ = true ;
    }

    public boolean isAtTarget() {
        return tracking_ && Math.abs(target_ - getPosition()) < threshold_ ;
    }
}