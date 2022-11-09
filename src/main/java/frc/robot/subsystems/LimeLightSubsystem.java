package frc.robot.subsystems ;

import org.xero1425.XeroRobot;
import org.xero1425.subsystems.XeroSubsystem;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

/// \file

/// \brief The limelight subsystem supports the LimeLight camera.  It is expected that a class that is game
/// specific will be derived from this class.
public class LimeLightSubsystem extends XeroSubsystem {
    private final static String SubsystemName = "limelight" ;

    private double distance_ ;
    private double yaw_ ;

    private double camera_angle_ ;
    private double camera_height_ ;
    private double target_height_ ;

    // The camera mode (vision or driver)
    private CamMode cam_mode_ ;

    // The LED mode
    private LedMode led_mode_ ;

    // The pipeline number we are running
    private int pipeline_ ;

    // If true, the target values are valid
    private boolean tv_ ;

    // If true, the limelight is connected (via the Network Tables)
    private boolean connected_ ;

    // The TX value from the limelight, see the limelight documentation for more details
    private double tx_ ;

    // The TY value from the limelight, see the limelight documentation for more details    
    private double ty_ ;

    // The TA value from the limelight, see the limelight documentation for more details    
    private double ta_ ;

    // The total latency from the current values (TV, TX, TY, TA) to when the image was seen by the camera
    private double total_latency_ ;

    // The camera latency from the current values (TV, TX, TY, TA) to when the image was seen by the camera
    private double camera_latency_ ;

    // The network latency from the current values (TV, TX, TY, TA) to when the image was seen by the camera
    private double network_latency_ ;

    // The network tables entry for the limelight
    private NetworkTable nt_ ;

    // The amount of time to wait for the limelight to connect
    private double limelight_timeout_ ;

    // Last time valid connection to limelight detected 
    // (also reset at of execution, and start of switch to vision processing mode)
    private double last_connection_time_ ;

    // If true, we have printed a message about not finding the limelight
    private boolean limelight_not_found_ ;

    /// \brief  The name of the table to read for limelight information
    public final static String LimeLightTableName = "limelight";

    /// \brief  The key to write for camera mode
    public final static String CamModeKeyName = "camMode" ;

    /// \brief  The key to write for LED mode
    public final static String LedModeKeyName = "ledMode" ;

    /// \brief The key to write for the pipeline number
    public final static String PipelineKeyName = "pipeline" ;

    /// \brief Create a new limelight subsystem
    /// \param parent the subsystem that manages this subsystem
    /// \param name the name of hte subsystem
    public LimeLightSubsystem(XeroRobot robot) throws Exception {
        super(robot, SubsystemName) ;
        
        last_connection_time_ = Timer.getFPGATimestamp() ;

        led_mode_ = LedMode.Invalid ;
        cam_mode_ = CamMode.Invalid ;
        pipeline_ = -1 ;

        limelight_not_found_ = false ;

        camera_latency_ = getSetting("camera-latency").getDouble() ;
        network_latency_ = getSetting("network-latency").getDouble() ;
        limelight_timeout_ = getSetting("timeout").getDouble() ;

        nt_ = NetworkTableInstance.getDefault().getTable(LimeLightTableName) ;

        setLedMode(LedMode.ForceOff);
        setCamMode(CamMode.VisionProcessing) ;
        setPipeline(0);

        camera_angle_ = getSetting("camera-angle").getDouble() ;
        camera_height_ = getSetting("camera-height").getDouble() ;
        target_height_ = getSetting("target-height").getDouble() ;
        distance_ = 0 ;
        yaw_ = 0 ;
    }

    /// \brief the mode for the camera
    public enum CamMode
    {
        VisionProcessing,           ///< Processing vision targets on the field
        DriverCamera,               ///< Relaying the image to the driver on the driver station
        Invalid                     ///< Invalid camera mode
    } ;

    /// \brief the mode for the LED
    public enum LedMode
    {
        UseLED,                     ///< Use the LED per the pipeline
        ForceOff,                   ///< Force the LED to the off state
        ForceBlink,                 ///< Force the LED to a blinking state
        ForceOn,                    ///< Force the LED to the on state
        Invalid                     ///< Invalid state
    } ;

    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }

    /// \brief This method computes the state of the camera
    @Override
    public void periodic() {
        if (cam_mode_ == CamMode.VisionProcessing)
        {
            if (nt_.containsKey("tv"))
            {
                connected_ = true ;
                limelight_not_found_ = false;
                last_connection_time_ = Timer.getFPGATimestamp() ;
                double value = nt_.getEntry("tv").getNumber(0.0).doubleValue() ;
                if (value < 0.01)
                {
                    tv_ = false ;
                }
                else
                {
                    tv_ = true ;
                    tx_ = nt_.getEntry("tx").getNumber(0.0).doubleValue() ;
                    ty_ = nt_.getEntry("ty").getNumber(0.0).doubleValue() ;
                    ta_ = nt_.getEntry("ta").getNumber(0.0).doubleValue() ;
                    total_latency_ = nt_.getEntry("tl").getNumber(0.010).doubleValue() + camera_latency_ + network_latency_ ;
                }
            }
            else
            {
                connected_ = false ;
                if ((Timer.getFPGATimestamp() - last_connection_time_) > limelight_timeout_)
                {
                    if (!limelight_not_found_)
                    {
                        DriverStation.reportError("limelight not detected", false);
                        limelight_not_found_ = true ;
                    }
                }
            }
        }
        else
        {
            tv_ = false ;
        }

        if (isLimeLightConnected() && isTargetDetected())
        {
            distance_ = (target_height_ - camera_height_) / Math.tan(Math.toRadians(camera_angle_ + getTY())) ;
            yaw_ = getTX() ;
        }
    }

    /// \brief Returns true if the limelight is detected. Currently only set in vision processing mode.
    /// \returns true if the limelight is detected
    public boolean isLimeLightConnected() {
        return connected_ ;
    }

    /// \brief Returns true if the limelight detects a target
    /// \returns true if the limelight detects a target
    public boolean isTargetDetected() {
        return tv_ ;
    }

    /// \brief Returns the X angle from the camera to the target
    /// \returns the X angle from the camera to the target
    public double getTX() {
        return tx_ ;
    }

    /// \brief Returns the Y angle from the camera to the target
    /// \returns the Y angle from the camera to the target
    public double getTY() {
        return ty_ ;
    }

    /// \brief Returns the area of the target detected
    /// \returns the area of target detected
    public double getTA() {
        return ta_ ;
    }

    /// \brief Set the camera mode
    /// \param mode the desired camera mode
    public void setCamMode(CamMode mode) {
        if (cam_mode_ != mode)
        {
            switch(mode) {
                case VisionProcessing:
                    nt_.getEntry(CamModeKeyName).setNumber(0) ;
                    last_connection_time_ = Timer.getFPGATimestamp() ;
                    break ;
                case DriverCamera:
                    nt_.getEntry(CamModeKeyName).setNumber(1) ;
                    break ;
                case Invalid:
                    break ;
            }

            cam_mode_ = mode ;
        }
    }

    /// \brief Returns the current camera mode
    /// \returns the current camera mode
    public CamMode getCamMode() {
        return cam_mode_ ;
    }

    /// \brief Set the LED mode for the camera
    /// \param mode the desired LED mode
    public void setLedMode(LedMode mode) {
        if (led_mode_ != mode)
        {
            switch(mode)
            {
                case UseLED:
                    nt_.getEntry(LedModeKeyName).setNumber(0) ;
                    break ;
                case ForceOff:
                    nt_.getEntry(LedModeKeyName).setNumber(1) ;
                    break ;
                case ForceBlink:
                    nt_.getEntry(LedModeKeyName).setNumber(2) ;
                    break ;
                case ForceOn:
                    nt_.getEntry(LedModeKeyName).setNumber(3) ;
                    break ;
                case Invalid:
                    break ;                                                                                
            }

            led_mode_ = mode ;
        }
    }

    /// \brief Returns the LED mode for the camera
    /// \returns the LED mode for the camera
    public LedMode getLedMode() {
        return led_mode_ ;
    }

    /// \brief Set the desired pipeline to run on the camera.  See the Limelight documentation for more information
    /// on pipelines and how they are used.
    /// \param which the pipeline to use
    public void setPipeline(int which) {
        if (which != pipeline_)
        {
            nt_.getEntry(PipelineKeyName).setNumber(which) ;
            pipeline_ = which ;
        }
    }

    /// \brief Returns the current pipeline in use
    /// \returns the current pipeline in use
    public int getPipeline() {
        return pipeline_ ;
    }

    /// \brief Returns the total latency from the camera image to values in the network tables
    /// \returns the total latency from the camera image to values in the network tables
    public double getTotalLatency() {
        return total_latency_ ;
    }

    /// \brief Returns a human readable string describing the subsystem
    /// \returns a human readable string describing the subsystem
    @Override
    public String toString() {
        return "limelight" ;

    }

    public double getDistance() {
        return distance_ ;
    }

    public double getYaw() {
        return yaw_ ;
    }
} ;