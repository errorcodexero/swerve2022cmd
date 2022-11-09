package org.xero1425.subsystems;

import org.xero1425.XeroRobot;
import org.xero1425.message.MessageLogger;
import org.xero1425.settings.MissingParameterException;
import org.xero1425.settings.SettingsValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class XeroSubsystem extends SubsystemBase {
    private XeroRobot robot_ ;
    
    public XeroSubsystem(XeroRobot robot, String name) throws Exception {
        robot_ = robot ;

        setName(name) ;
        robot.addSubsystem(this) ;
    }    

    public MessageLogger getMessageLogger() {
        return robot_.getMessageLogger() ;
    }

    public SettingsValue getProperty(String name) {
        return null ;
    }

    public SettingsValue getSetting(String name) throws MissingParameterException {
        return robot_.getSettingsSupplier().get("subsystems:" + getName() + ":" + name) ;
    }
}
