package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
	private Joystick joys = new Joystick(Constants.joys_ID); 

	private Translation2d FL2d = new Translation2d(Constants.Length / 2, Constants.Width / 2);
	private Translation2d FR2d = new Translation2d(Constants.Length / 2, -Constants.Width / 2);
	private Translation2d BL2d = new Translation2d(-Constants.Length / 2, Constants.Width / 2);
	private Translation2d BR2d = new Translation2d(-Constants.Length / 2, -Constants.Width / 2);
	private SwerveDriveKinematics SDK = new SwerveDriveKinematics(FL2d, FR2d, BL2d, BR2d);

	private SwerveModuleState[] states;
	private ChassisSpeeds cSpeeds;

	private CANCoder encoderFL = new CANCoder(Constants.encoders[0]);
	private CANCoder encoderFR = new CANCoder(Constants.encoders[1]);
	private CANCoder encoderBL = new CANCoder(Constants.encoders[2]);
	private CANCoder encoderBR = new CANCoder(Constants.encoders[3]);

	private TalonFX steerFL = new TalonFX(Constants.steerMotors[0]);
	private TalonFX steerFR = new TalonFX(Constants.steerMotors[1]);
	private TalonFX steerBL = new TalonFX(Constants.steerMotors[2]);
	private TalonFX steerBR = new TalonFX(Constants.steerMotors[3]);

	private TalonFX driveFL = new TalonFX(Constants.driveMotors[0]);
	private TalonFX driveFR = new TalonFX(Constants.driveMotors[1]);
	private TalonFX driveBL = new TalonFX(Constants.driveMotors[2]);
	private TalonFX driveBR = new TalonFX(Constants.driveMotors[3]);

	private PIDController pidFL = new PIDController(Constants.kp, Constants.ki, Constants.kd);
	private PIDController pidFR = new PIDController(Constants.kp, Constants.ki, Constants.kd);
	private PIDController pidBL = new PIDController(Constants.kp, Constants.ki, Constants.kd);
	private PIDController pidBR = new PIDController(Constants.kp, Constants.ki, Constants.kd);

	public void motorPID(double angle, CANCoder encoder, TalonFX motor, PIDController pid, double offset) {
		double currentPos = encoder.getPosition() + offset;
			SmartDashboard.putNumber("Current Encoder Position", currentPos);

		pid.setSetpoint(angle);
		double out = pid.calculate(currentPos);
			SmartDashboard.putNumber("Out", out); // I would give this a better label but I don't know what "out" is
													  // It looks like it's the setpoint but I want to double check with Ash or Armando

		motor.set(ControlMode.PercentOutput, out);
	}

	@Override
	public void robotInit() {
		pidFL.setTolerance(Constants.tolerance);
		pidFR.setTolerance(Constants.tolerance);
		pidBL.setTolerance(Constants.tolerance);
		pidBR.setTolerance(Constants.tolerance);
	}

	@Override
	public void robotPeriodic() {
	}

	@Override
	public void autonomousInit() {
	}

	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void teleopInit() {
	}

	@Override
	public void teleopPeriodic() {
		double joysY = joys.getY();
		double joysX = joys.getX();
		double joysZ = joys.getZ();

		double vX = 0; // Velocity X
		double vY = 0; // Velocity Y
		double omega = 0; // Idk why this is named omega when it's really just velocity Z

	// No idea what the math is doing here
		if (Math.abs(joysY) > Constants.xBlind) {
			vX = joysY*-.1;
		}
		if (Math.abs(joysX) > Constants.yBlind) {
			vY = joysX * -1*.1;
		}
		if (Math.abs(joysZ) > Constants.zBlind) {
			omega = joysZ*.2;
		}  

		cSpeeds = new ChassisSpeeds(vX, vY, omega);
		states = SDK.toSwerveModuleStates(cSpeeds);
	
	// I think this will print chassis speed but I don't know what the unit is
		SmartDashboard.putNumber("x Velocity: ", vX);
		SmartDashboard.putNumber("y Velocity: ", vY); 
		SmartDashboard.putNumber("Omega: ", omega); 

		driveFL.set(ControlMode.PercentOutput, states[0].speedMetersPerSecond * Constants.driveMod);
		driveFR.set(ControlMode.PercentOutput, states[1].speedMetersPerSecond * Constants.driveMod);
		driveBL.set(ControlMode.PercentOutput, states[2].speedMetersPerSecond * Constants.driveMod);
		driveBR.set(ControlMode.PercentOutput, states[3].speedMetersPerSecond * Constants.driveMod);

	// This is the way to actually print out the data from before but I'm not sure if your trying to get the speed of the drive motors?
	// If so, I don't think this will work
	// I actually don't know what it will print but I kinda want to find out anyways so I'll just leave it here
		SmartDashboard.putNumber("Foward Left Speed (Meters per second): ", states[0].speedMetersPerSecond * Constants.driveMod);
		SmartDashboard.putNumber("Foward Right Speed (Meters per second): ", states[1].speedMetersPerSecond * Constants.driveMod);
		SmartDashboard.putNumber("Back Left Speed (Meters per second): ", states[2].speedMetersPerSecond * Constants.driveMod);
		SmartDashboard.putNumber("Back Right Speed (Meters per second): ", states[3].speedMetersPerSecond * Constants.driveMod);

	/*
	 * Equation to get wheel speed in meters per second:
	 * RPM = senser units / encoder revolutions * 1000 * 60 * gear ratio
	 * RPM * 2pi * wheel radius / 60 = wheel speed meters per second
	 * 
	 * I would write it in code but I don't have the gear ration or the wheel radius
	 */

	// I don't know if this should be commented out
		/*
		steerFL.set(ControlMode.PercentOutput, states[0].speedMetersPerSecond * Constants.driveMod);
		steerFR.set(ControlMode.PercentOutput, states[1].speedMetersPerSecond * Constants.driveMod);
		steerBL.set(ControlMode.PercentOutput, states[2].speedMetersPerSecond * Constants.driveMod);
		steerBR.set(ControlMode.PercentOutput, states[3].speedMetersPerSecond * Constants.driveMod);
		*/
		
		double angleFL = states[0].angle.getDegrees();
		double angleFR = states[1].angle.getDegrees();
		double angleBL = states[2].angle.getDegrees();
		double angleBR = states[3].angle.getDegrees();
	 
		SmartDashboard.putNumber("Fornt Left Angle (degrees): ", angleFL);
		SmartDashboard.putNumber("Front Right Angle (degrees): ", angleFR);
		SmartDashboard.putNumber("Back Left Angle (degrees): ", angleBL);
		SmartDashboard.putNumber("Back Right Angle (degrees): ", angleBR);

		motorPID(angleFL, encoderFL, steerFL, pidFL, Constants.offsetFL);
		motorPID(angleFR, encoderFR, steerFR, pidFR, Constants.offsetFR);
		motorPID(angleBL, encoderBL, steerBL, pidBL, Constants.offsetBL);
		motorPID(angleBR, encoderBR, steerBR, pidBR, Constants.offsetBR);
		
	// I have no idea what the hell is going on here but these are all just printing constants
	// I don't think shuffle board will even print these so specify what exactly your looking for
		/*
		System.out.print("E FL: "); //com.ctre.phoenix.sensors.CANCoder
		System.out.print(encoderFL);
		System.out.print(", E FR: ");
		System.out.print(encoderFR);
		System.out.print(", E BL: ");
		System.out.print( encoderBL);
		System.out.print(", E BR: ");
		System.out.println(encoderBR);*/

		/*
		System.out.print("E FL: ");
		System.out.print(steerFL); //com.ctre.phoenix.motorcontrol.can.TalonFX
		System.out.print(", E FR: ");
		System.out.print(steerFR);
		System.out.print(", E BL: ");
		System.out.print( steerBL);
		System.out.print(", E BR: ");
		System.out.println(steerBR);*/

		/*
		System.out.print("E FL: "); //edu.wpi.first.math.controller.PIDController
		System.out.print(pidFL);
		System.out.print(", E FR: ");
		System.out.print(pidFR);
		System.out.print(", E BL: ");
		System.out.print( pidBL);
		System.out.print(", E BR: ");
		System.out.println(pidBR);*/

	}

	@Override
	public void disabledInit() {
	}

	@Override
	public void disabledPeriodic() {
	}

	@Override
	public void testInit() {
	}

	@Override
	public void testPeriodic() {
	}
}
