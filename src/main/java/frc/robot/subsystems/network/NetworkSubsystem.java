package frc.robot.subsystems.network;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * Manages NetworkTables dashboard values and auto mode selection.
 * Centralizes all tunable parameters for easy access and modification
 * through the driver station dashboard without redeploying code.
 */
public class NetworkSubsystem {
    /**
     * Available autonomous/teleop operating modes.
     */
    public static enum TeleopMode {
        NONE,
        RELATIVE,
        KROUZKY_ALONE,
        LASERY_POLE,
        LASERY_VEZ,
        KROUZKY,
        BLUDISTE,
        PREDMETY,
        SLALOM_MANUAL,
        SLALOM_AUTONOM,
        CENTER_WHEELS,
        TEST,
    }

    public static enum AutoMode {
        NONE,
        LASERY_VEZ,
        BLUDISTE,
        PREDMETY,
        TEST
    }

    /** SendableChooser for auto mode selection on dashboard */
    public static final SendableChooser<TeleopMode> teleopModeChooser = new SendableChooser<>();
    public static final SendableChooser<AutoMode> autoModeChooser = new SendableChooser<>();

    // QR Parameter
    public static final DashboardValue<Boolean> ENABLE_QR_SCANNING = new DashboardValue<>(
        "Camera Tower/QR Detector/enable", false);

    // Controller parameters
    public static final DashboardValue<Double> TURN_SENSITIVITY = new DashboardValue<>(
        "Controller/Turn sensitivity", 0.7);
    public static final DashboardValue<Double> JOYSTICK_DEADZONE = new DashboardValue<>(
        "Controller/Joystick deadzone", 0.07);
    public static final DashboardValue<Double> TRIGGER_AXIS_DEADZONE = new DashboardValue<>(
        "Controller/Trigger axis deadzone", 0.05);

        
    // Chassis parameters
    public static final DashboardValue<Boolean> ZERO_ANGLE = new DashboardValue<>(
        "Chassis/Zero angle", false);
    public static final DashboardValue<Double> MAX_SPEED = new DashboardValue<>(
        "Chassis/Max speed (ms^-1)", 2.5);
    public static final DashboardValue<Double> MAX_ANGULAR_SPEED = new DashboardValue<>(
        "Chassis/Max angular speed (rads^-1)", 1.8);
    public static final DashboardValue<Double> SPEED_CURVE = new DashboardValue<Double>(
        "Chassis/Speed curve", 2.3);
    public static final DashboardValue<Boolean> DRIVE_RELATIVE = new DashboardValue<Boolean>(
        "Chassis/Drive relative", true);
    public static final DashboardValue<Boolean> OVERRIDE_LOW_VOLTAGE_LIMIERS = new DashboardValue<>(
        "Chassis/Advanced/Override low voltage limiters", true);
    public static final DashboardValue<Boolean> DISPLACEMENT_CORRECTION = new DashboardValue<>(
        "Chassis/Advanced/Displacement correction", false);

    // Chassis/advanced parameters
    public static final DashboardValue<Double> MAX_ACCELERATION = new DashboardValue<>(
        "Chassis/Advanced/Max acceleration (ms^-2)", 3.0);
    public static final DashboardValue<Double> MAX_ANGULAR_ACCELERATION = new DashboardValue<>(
        "Chassis/Advanced/Max angular acceleration (rads^-2)", 2*Math.PI);

    // Laser turret parameters
    public static final DashboardValue<Double> LASER_MOTOR_MAX_SPEED = new DashboardValue<>(
        "Laser Turret/Motor speed", 1.0);

    // Camera tower parameters
    public static final DashboardValue<Double> CAMERA_MOTOR_MAX_SPEED = new DashboardValue<>(
        "Camera Tower/Motor speed", 0.5);

    // Arm parameters
    public static final DashboardValue<Double> ARM_BRUSH_SPEED = new DashboardValue<>(
        "Arm/Brush speed", 0.2);
    public static final DashboardValue<Double> ARM_SWING_SPEED = new DashboardValue<>(
        "Arm/Swing Speed", 0.4);
    public static final DashboardValue<Boolean> ARM_BUTTON_OVERRIDE = new DashboardValue<>(
        "Arm/Button override", false);

    // Debug parameters
    public static final DashboardValue<Double> DEBUG_ARM_ENCODER_VALUE = new DashboardValue<> (
        "Debug/Arm encoder value", 0.0);
    public static final DashboardValue<Boolean> DEBUG_LIMIT_SWICH_READING = new DashboardValue<> (
        "Debug/Limit switch reading", false);
    public static final DashboardValue<Boolean> DEBUG_DISABLE_LASER = new DashboardValue<>(
        "Debug/Debug disable laser", false);
    public static final DashboardValue<Boolean> CHILD_MODE_ENABLED = new DashboardValue<>(
        "Debug/Child mode enabled", false);


    private static double backupMaxSpeed = 1.0;
    private static double backupRotationSpeed = 1.0;

    /**
     * Initializes the NetworkSubsystem by setting up the auto mode chooser
     * and pushing all default values to NetworkTables.
     * Must be called once during robot initialization.
     */
    public static void Init() {
        teleopModeChooser.setDefaultOption("None", TeleopMode.NONE);
        teleopModeChooser.addOption("Krouzky alone", TeleopMode.KROUZKY_ALONE);
        teleopModeChooser.addOption("Lasery Pole", TeleopMode.LASERY_POLE);
        teleopModeChooser.addOption("Lasery Vez", TeleopMode.LASERY_VEZ);
        teleopModeChooser.addOption("Krouzky", TeleopMode.KROUZKY);
        teleopModeChooser.addOption("Bludiste", TeleopMode.BLUDISTE);
        teleopModeChooser.addOption("Predmety", TeleopMode.PREDMETY);
        teleopModeChooser.addOption("Slalom manual", TeleopMode.SLALOM_MANUAL);
        teleopModeChooser.addOption("Slalom autonom", TeleopMode.SLALOM_AUTONOM);
        teleopModeChooser.addOption("Center Wheels", TeleopMode.CENTER_WHEELS);
        teleopModeChooser.addOption("Test", TeleopMode.TEST);
        teleopModeChooser.addOption("Relative", TeleopMode.RELATIVE);
        SmartDashboard.putData("Teleop Mode", NetworkSubsystem.teleopModeChooser);

        autoModeChooser.setDefaultOption("None", AutoMode.NONE);
        autoModeChooser.addOption("Lasery Vez", AutoMode.LASERY_VEZ);
        autoModeChooser.addOption("Bludiste", AutoMode.BLUDISTE);
        autoModeChooser.addOption("Predmety", AutoMode.PREDMETY);
        autoModeChooser.addOption("Test", AutoMode.TEST);

        SmartDashboard.putData("Auto Mode", NetworkSubsystem.autoModeChooser);

        SmartDashboard.putData("Child mode", new InstantCommand(() -> {
            CHILD_MODE_ENABLED.set(!CHILD_MODE_ENABLED.get());
            
            double swap = MAX_SPEED.get();
            MAX_SPEED.set(backupMaxSpeed);
            backupMaxSpeed = swap;

            swap = MAX_ANGULAR_SPEED.get();
            MAX_ANGULAR_SPEED.set(backupRotationSpeed);
            backupRotationSpeed = swap;
        }));


        for (DashboardValue<?> value : DashboardValue.values) {
            value.setDefault();
        }
    }
}
