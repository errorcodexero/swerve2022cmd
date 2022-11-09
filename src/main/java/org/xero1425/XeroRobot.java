package org.xero1425;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.xero1425.message.MessageDestination;
import org.xero1425.message.MessageDestinationFile;
import org.xero1425.message.MessageDestinationThumbFile;
import org.xero1425.message.MessageLogger;
import org.xero1425.message.MessageType;
import org.xero1425.motors.MotorFactory;
import org.xero1425.paths.XeroPathManager;
import org.xero1425.settings.ISettingsSupplier;
import org.xero1425.settings.JsonSettingsParser;
import org.xero1425.settings.SettingsValue;
import org.xero1425.simulator.engine.SimulationEngine;
import org.xero1425.subsystems.XeroSubsystem;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;

public class XeroRobot extends TimedRobot {
    private RobotPaths robot_paths_ ;
    private MessageLogger logger_;
    private ISettingsSupplier settings_ ;
    private PlotManager plot_mgr_ ;
    private int logger_id_ ;
    private MotorFactory motors_ ;
    private XeroPathManager paths_ ;
    
    private Map<String, XeroSubsystem> subsystems_ ;

    /// \brief The "subsystem" name for the message logger for this class
    public static final String LoggerName = "xerorobot" ;

    public XeroRobot() {
        super() ;

        // Generate the paths to the various important places (logfile directory, settings file, path follow paths directoryh, etc.)
        robot_paths_ = new RobotPaths(XeroRobot.isSimulation(), getName());

        subsystems_ = new HashMap<String, XeroSubsystem>() ;

        enableMessageLogger();
        readParamsFile();
        enableMessagesFromSettingsFile() ;

        if (XeroRobot.isSimulation()) {
            String str = SimArgs.InputFileName;
            if (str == null)
                str = getSimulationFileName() ;

            if (str == null) {
                System.out.println("The code is setup to simulate, but the derived robot class did not provide a stimulus file") ;
                System.out.println("Not initializing the Xero1425 Simulation engine - assuming Romi robot") ;
            }
            else {
                SimulationEngine.initializeSimulator(this, logger_);
                addRobotSimulationModels() ;
                SimulationEngine.getInstance().initAll(str) ;
            }
        }        

        // Create the motor factor
        motors_ = new MotorFactory(logger_, settings_);

        // Create the plot manager
        plot_mgr_ = new PlotManager("/XeroPlot");

        try {
            loadPathsFile();
        } catch (Exception ex) {
            logger_.startMessage(MessageType.Error) ;
            logger_.add("caught exception reading path files -").add(ex.getMessage()).endMessage();
        }
    }

    /// \brief Returns the motor factory
    /// \returns the motor factory
    public MotorFactory getMotorFactory() {
        return motors_;
    }

    public XeroPathManager getPathManager() {
        return paths_ ;
    }

    public void addSubsystem(XeroSubsystem sub) throws Exception {
        if (subsystems_.containsKey(sub.getName())) {
            throw new Exception("A subsystem with the name '" + sub.getName() + "' already exists") ;
        }

        subsystems_.put(sub.getName(), sub) ;
    }

    public XeroSubsystem getSubsystemByName(String name) {
        return subsystems_.get(name) ;
    }

    public MessageLogger getMessageLogger() {
        return logger_ ;
    }

    public ISettingsSupplier getSettingsSupplier() {
        return settings_ ;
    }

    /// \brief add specific models to the simulation, expected to be overridden by the derived class
    protected void addRobotSimulationModels() {
    }    
    
    /// \brief Returns the simulation stimulus file (JSON) name.  This method should be overridden by the
    /// derived robot specific class.
    /// \returns the simulation stimulus file (JSON) name
    protected String getSimulationFileName() {
        return null ;
    }

    /// \brief Returns the current robot time in seconds
    /// \returns the current robot time in seconds
    public double getTime() {
        return Timer.getFPGATimestamp();
    }

    // Override this method in the top level robot class
    public String getName() {
        return "BLAH" ;
    }

    @Override
    public void robotInit() {
        super.robotInit() ;
    }

    /// \brief load the paths file from the paths file directory
    protected void loadPathsFile() throws Exception {
        XeroPathManager mgr = getPathManager() ;

        try (Stream<Path> walk = Files.walk(Paths.get(mgr.getBaseDir()))) {
            List<String> result = walk.map(x -> x.toString()).filter(f -> f.endsWith("_main.csv")).collect(Collectors.toList());
            for(String name : result) {
                int index = name.lastIndexOf(File.separator) ;
                if (index != -1) {
                    name = name.substring(index + 1) ;
                    name = name.substring(0, name.length() - 9) ;
                    mgr.loadPath(name) ;
                }
            }
        }
        catch(IOException ex) {
        }
    }    

    private void enableMessageLogger() {
        String logfile = SimArgs.LogFileName ;
        MessageDestination dest ;

        logger_ = new MessageLogger();
        logger_.setTimeSource(new RobotTimeSource());

        if (logfile != null) {
            dest = new MessageDestinationFile(logfile) ;
        }
        else {
            dest = new MessageDestinationThumbFile(robot_paths_.logFileDirectory(), 250);
        }
        logger_.addDestination(dest);
    }

    protected boolean isPracticeBot() {
        return true ;
    }

    private void readParamsFile() {
        JsonSettingsParser file = new JsonSettingsParser(logger_);

        String bot ;
        if (isPracticeBot())
            bot = "PRACTICE" ;
        else
            bot = "COMPETITION" ;

        file.addDefine(bot) ;
        logger_.startMessage(MessageType.Info).add("reading params for bot ").addQuoted(bot).endMessage() ;

        String filename = robot_paths_.deployDirectory() + getName() + ".jsonc" ;

        File f = new File(filename) ;
        if (!f.exists()) {
            filename = robot_paths_.deployDirectory() + getName() + ".json" ;
            f = new File(filename) ;
            if (!f.exists()) {
                //
                // There is no params file
                //
                logger_.startMessage(MessageType.Error).add("no settings file exists for this robot").endMessage();
                return ;
            }
        }

        if (!file.readFile(filename)) {
            logger_.startMessage(MessageType.Error).add("error reading parameters file").endMessage();
        }

        settings_ = file ;
    }

    private void enableMessagesFromSettingsFile() {
        String path = "system:messages" ;
        ISettingsSupplier p = getSettingsSupplier() ;
        MessageLogger m = getMessageLogger() ;

        var keys = p.getAllKeys(path) ;
        if (keys != null) {
            for(String key : keys)
            {
                try {
                    String longkey = path + ":" + key ;
                    SettingsValue v = p.get(longkey) ;
                    if (v.isBoolean() && v.getBoolean())
                    {
                        m.enableSubsystem(key) ;
                    }
                }
                catch(Exception ex)
                {
                }
            }
        }
    }    
}
