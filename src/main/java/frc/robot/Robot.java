package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
	private Joystick joys = new Joystick(Constants.joys_ID); 

  	private DrivetrainSubsystem m_drivetrainSubsytem = new DrivetrainSubsystem();

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
	public void robotInit() {}

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
    	m_drivetrainSubsytem.drive(new ChassisSpeeds(vX * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, vY * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, omega * 2 * Math.PI));
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
