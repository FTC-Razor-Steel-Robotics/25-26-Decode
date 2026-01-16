package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public abstract class Robot {
	//Create new classes for our drive motors and intake/shooting motors
	//This will split them into groups in the dashboard, making it a bit less cluttered
	//For reference, this is also done in MecanumDrive.java
	protected static abstract class DriveConfig {
		public static String frontLeftDriveString;
		public static String backLeftDriveDriveString;
		public static String frontRightDriveString;
		public static String backRightDriveString;
	}

	protected static abstract class ShooterConfig {
		public static String shooterString;
		public static String intakeString;

		public static double[] shooterSpeeds;

		public static int shooterSpeedIndex = 0;
	}

	//Create instances of the above classes so that way we can actually use them
//	public static DriveConfig DRIVE_CONFIG;
//	public static ShooterConfig SHOOTER_CONFIG;

	protected HardwareMap hardwareMap;
	protected Telemetry telemetry;

	protected DcMotor frontLeftDrive;
	protected DcMotor backLeftDrive;
	protected DcMotor frontRightDrive;
	protected DcMotor backRightDrive;

	protected DcMotor intake;
	protected DcMotor shooter;

	public Robot(HardwareMap hwMap, Telemetry telem) {
		//Create copies of the hardware map and telemetry so we can use them throughout the class
		hardwareMap = hwMap;
		telemetry = telem;
	}

	public void driveMecanum(double axial, double lateral, double yaw) {
		// Combine the joystick requests for each axis-motion to determine each wheel's power.
		// Set up a variable for each drive wheel to save the power level for telemetry.
		double frontLeftPower  = axial + lateral + yaw;
		double frontRightPower = axial - lateral - yaw;
		double backLeftPower   = axial - lateral + yaw;
		double backRightPower  = axial + lateral - yaw;

		// Normalize the values so no wheel power exceeds 100%
		// This ensures that the robot maintains the desired motion.
		double max;
		max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
		max = Math.max(max, Math.abs(backLeftPower));
		max = Math.max(max, Math.abs(backRightPower));

		if (max > 1.0) {
			frontLeftPower  /= max;
			frontRightPower /= max;
			backLeftPower   /= max;
			backRightPower  /= max;
		}

		frontLeftDrive.setPower(frontLeftPower);
		frontRightDrive.setPower(frontRightPower);
		backLeftDrive.setPower(backLeftPower);
		backRightDrive.setPower(backRightPower);

		telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
		telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);
	}

	public void fireShooter(boolean spin, boolean toggleSpeed) {
		//Cycle through the array and go back to 0 if we reach the end
		if (toggleSpeed) {
			if (++ShooterConfig.shooterSpeedIndex >= ShooterConfig.shooterSpeeds.length)
				ShooterConfig.shooterSpeedIndex = 0;
		}

		double shooterSpeed = ShooterConfig.shooterSpeeds[ShooterConfig.shooterSpeedIndex];
		telemetry.addData("Shooter Speed", shooterSpeed);

		shooter.setPower(spin ? shooterSpeed : 0);
	}

	public abstract void setIntakeSpeed(double speed);
}
