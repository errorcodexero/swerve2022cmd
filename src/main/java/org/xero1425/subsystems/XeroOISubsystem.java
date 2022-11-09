package org.xero1425.subsystems;

import org.xero1425.XeroRobot;

public class XeroOISubsystem extends XeroSubsystem {
    public enum DriverType
    {
        Swerve,
        TankDrive
    } ;

    private DriverType type_ ;

    public XeroOISubsystem(XeroRobot robot, String name, DriverType type) throws Exception
    {
        super(robot, name) ;

        type_ = type ;
    }    
}
