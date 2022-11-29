package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.constraint.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

public class Robot extends TimedRobot {

	// Change the initialization heree if the control scheme is not a joystick
	private Joystick joys = new Joystick(Constants.joys_ID);

	private Translation2d FL2d = new Translation2d(Constants.Length / 2, Constants.Width / 2);
	private Translation2d FR2d = new Translation2d(Constants.Length / 2, -Constants.Width / 2);
	private Translation2d BL2d = new Translation2d(-Constants.Length / 2, Constants.Width / 2);
	private Translation2d BR2d = new Translation2d(-Constants.Length / 2, -Constants.Width / 2);
	private SwerveDriveKinematics SDK = new SwerveDriveKinematics(FL2d, FR2d, BL2d, BR2d);

	private SwerveModuleState[] states;
	private ChassisSpeeds cSpeeds;

  private DrivetrainSubsystem m_drivetrainSubsytem = new DrivetrainSubsystem();
	// Change the initialization here if motors are not TalonFXs or if encoders are
	// not CANcoders
	// private CANCoder encoderFL = new CANCoder(Constants.encoders[0]);
	// private CANCoder encoderFR = new CANCoder(Constants.encoders[1]);
	// private CANCoder encoderBL = new CANCoder(Constants.encoders[2]);
	// private CANCoder encoderBR = new CANCoder(Constants.encoders[3]);

	// private TalonFX steerFL = new TalonFX(Constants.steerMotors[0]);
	// private TalonFX steerFR = new TalonFX(Constants.steerMotors[1]);
	// private TalonFX steerBL = new TalonFX(Constants.steerMotors[2]);
	// private TalonFX steerBR = new TalonFX(Constants.steerMotors[3]);

	// private TalonFX driveFL = new TalonFX(Constants.driveMotors[0]);
	// private TalonFX driveFR = new TalonFX(Constants.driveMotors[1]);
	// private TalonFX driveBL = new TalonFX(Constants.driveMotors[2]);
	// private TalonFX driveBR = new TalonFX(Constants.driveMotors[3]);

	// private PIDController pidFL = new PIDController(Constants.kp, Constants.ki, Constants.kd);
	// private PIDController pidFR = new PIDController(Constants.kp, Constants.ki, Constants.kd);
	// private PIDController pidBL = new PIDController(Constants.kp, Constants.ki, Constants.kd);
	// private PIDController pidBR = new PIDController(Constants.kp, Constants.ki, Constants.kd);

	public void motorPID(double angle, CANCoder encoder, TalonFX motor, PIDController pid, double offset) {
		double currentPos = encoder.getPosition() + offset;

		//System.out.println(currentPos);
		//System.out.println("");

		pid.setSetpoint(angle);
		double out = pid.calculate(currentPos);

		//System.out.println(out);
		//System.out.println("");

		motor.set(ControlMode.PercentOutput, out);
	}

	@Override
	public void robotInit() {
		// pidFL.setTolerance(Constants.tolerance);
		// pidFR.setTolerance(Constants.tolerance);
		// pidBL.setTolerance(Constants.tolerance);
		// pidBR.setTolerance(Constants.tolerance);

   // steerFL.config_K
	}

	@Override
	public void robotPeriodic() {
    m_drivetrainSubsytem.periodic();
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
		double theta = 1;
		double omega = 0;

		if (Math.abs(joysY) > Constants.xBlind) {
			vX = -joysY;
		}
		if (Math.abs(joysX) > Constants.yBlind) {
			vY = -1 * joysX;
		}
		if (Math.abs(joysZ) > Constants.zBlind) {
			omega = -joysZ*0.6;
		}
		
		/*
		System.out.print("vX: ");
		System.out.print(vX);
		System.out.print(", vY: ");
		System.out.print(vY);
		System.out.print(", Omega: ");
		System.out.println(omega);
		*/

		// cSpeeds = new ChassisSpeeds(vX, vY, omega);

    m_drivetrainSubsytem.drive(new ChassisSpeeds(vX * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, vY * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, omega * 2 * Math.PI));

		// states = SDK.toSwerveModuleStates(cSpeeds);

		//System.out.println(states[0].speedMetersPerSecond * Constants.driveMod);
		/*
		System.out.print("1: ");
		System.out.print(states[0].speedMetersPerSecond * Constants.driveMod);
		System.out.print(", 2: ");
		System.out.print( states[1].speedMetersPerSecond * Constants.driveMod);
		System.out.print(", 3: ");
		System.out.print( states[2].speedMetersPerSecond * Constants.driveMod);
		System.out.print(", 4: ");
		System.out.println(states[3].speedMetersPerSecond * Constants.driveMod);
		*/
		// driveFL.set(ControlMode.PercentOutput, states[0].speedMetersPerSecond * Constants.driveMod);
		// driveFR.set(ControlMode.PercentOutput, states[1].speedMetersPerSecond * Constants.driveMod);
		// driveBL.set(ControlMode.PercentOutput, states[2].speedMetersPerSecond * Constants.driveMod);
		// driveBR.set(ControlMode.PercentOutput, states[3].speedMetersPerSecond * Constants.driveMod);
		

		/*
		steerFL.set(ControlMode.PercentOutput, states[0].speedMetersPerSecond * Constants.driveMod);
		steerFR.set(ControlMode.PercentOutput, states[1].speedMetersPerSecond * Constants.driveMod);
		steerBL.set(ControlMode.PercentOutput, states[2].speedMetersPerSecond * Constants.driveMod);
		steerBR.set(ControlMode.PercentOutput, states[3].speedMetersPerSecond * Constants.driveMod);
		*/
		// double angleFL = states[0].angle.getDegrees();
		// double angleFR = states[1].angle.getDegrees();
		// double angleBL = states[2].angle.getDegrees();
		// double angleBR = states[3].angle.getDegrees();
		/*
		System.out.print("Angle FL: ");
		System.out.print(angleFL);
		System.out.print(", Angle FR: ");
		System.out.print(angleFR);
		System.out.print(", Angle BL: ");
		System.out.print( angleBL);
		System.out.print(", Angle BR: ");
		System.out.println(angleBR);*/

		// Optimization due to encoder offset problems.
		// Delete the if statement if it is causing problems.
		
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

		// motorPID(angleFL, encoderFL, steerFL, pidFL, Constants.offsetFL);
		// motorPID(angleFR, encoderFR, steerFR, pidFR, Constants.offsetFR);
		// motorPID(angleBL, encoderBL, steerBL, pidBL, Constants.offsetBL);
		// motorPID(angleBR, encoderBR, steerBR, pidBR, Constants.offsetBR);
	}

	@Override
	public void disabledInit() {
	}

	@Override
	public void disabledPeriodic() {
    m_drivetrainSubsytem.drive(new ChassisSpeeds());
	}

	@Override
	public void testInit() {
	}

	@Override
	public void testPeriodic() {
	}
}
