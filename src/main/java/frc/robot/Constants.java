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
    public final static double xBlind = 0.1;
    public final static double yBlind = 0.1;
    public final static double zBlind = 0.1;

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
}