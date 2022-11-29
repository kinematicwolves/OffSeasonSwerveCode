package frc.robot;

public class Constants {
    // d -> drive motors
    // s -> steer motors
    // enc -> encoders
    // joys -> joystick (if otther devices are used use the ID of that device instead)
    public final static int dFL_ID = 1; // fl -> foward left
    public final static int sFL_ID = 2; 
    public final static int encFL_ID = 3;
    public final static int dFR_ID = 4; // fr -> foward right
    public final static int sFR_ID = 5;
    public final static int encFR_ID = 6;
    public final static int dBL_ID = 7; // bl -> back left
    public final static int sBL_ID = 8;
    public final static int encBL_ID = 9;
    public final static int dBR_ID = 10; // br -> back right
    public final static int sBR_ID = 11;
    public final static int encBR_ID = 12;

    public final static int[] driveMotors = {dFL_ID, dFR_ID, dBL_ID, dBR_ID};
    public final static int[] steerMotors = {sFL_ID, sFR_ID, sBL_ID, sBR_ID};
    public final static int[] encoders = {encFL_ID, encFR_ID, encBL_ID, encBR_ID};

    public final static int joys_ID = 0;

    // To prevent the robot from reacting to joystick noise, blind areas are added around the origin
    // ranging from 0 to 1
    public final static double xBlind = 0.25;
    public final static double yBlind = 0.25;
    public final static double zBlind = 0.25;

    // Distances between the modules in meters
    // length refers to front/back distance, width refers to left/right distance
    public final static double Length = 0.65;
    public final static double Width = 0.65;
    
    // PID parameters: fine tune these for optimal performance
    public final static double kp = 0;
    public final static double ki = 0;
    public final static double kd = 0;
    public final static double tolerance = 0;

    // This is used to control the speed of the drive motors. Make it larger if the robot is too slow.
    public final static double driveMod = 1;

    // Since the encoders of the Mk4 modules are not aligned with the chassis, offsets are needed.
    // in degrees, and don't forget to tune the encoders in Phoenix Tuner to "boot to absolute position!"
    public final static double offsetFL = 10;
    public final static double offsetFR = 10;
    public final static double offsetBL = 10;
    public final static double offsetBR = 10;

        /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = Length; // FIXME Measure and set trackwidth
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = Width; // FIXME Measure and set wheelbase

    public static final int DRIVETRAIN_PIGEON_ID = 0; // FIXME Set Pigeon ID

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = dFL_ID; // FIXME Set front left module drive motor ID
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = sFL_ID; // FIXME Set front left module steer motor ID
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = encFL_ID; // FIXME Set front left steer encoder ID
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(33.48); // FIXME Measure and set front left steer offset

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = dFR_ID; // FIXME Set front right drive motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = sFR_ID; // FIXME Set front right steer motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = encFR_ID; // FIXME Set front right steer encoder ID
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(70.6); // FIXME Measure and set front right steer offset

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = dBL_ID; // FIXME Set back left drive motor ID
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = sBL_ID; // FIXME Set back left steer motor ID
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = encBL_ID; // FIXME Set back left steer encoder ID
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(141.8); // FIXME Measure and set back left steer offset

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = dBR_ID; // FIXME Set back right drive motor ID
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = sBR_ID; // FIXME Set back right steer motor ID
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = encBR_ID; // FIXME Set back right steer encoder ID
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(353.7); // FIXME Measure and set back right steer offset
}