package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class Robot {
	//Create new classes for our drive motors and intake/shooting motors
	//This will split them into groups in the dashboard, making it a bit less cluttered
	//For reference, this is also done in MecanumDrive.java
	protected static abstract class DriveConfig {
		//You can thank this Stack Overflow answer for this method of doing things
		//https://stackoverflow.com/a/2371302
		public abstract String[] getDriveStrings();
		public abstract boolean[] getDriveReversals();
	}

	protected static abstract class ShooterConfig {
		public abstract String getShooterString();
		public abstract double[] getShooterSpeeds();
		public abstract double[] getShooterVoltages();
		public abstract double[] getShooterSpeedsCompensated();
	}

	//TODO: Make sure that the parallel odometry wheels are on ports 0 and 3 for best performance
	protected static abstract class RRConfig {
		public abstract String[] getOdomStrings();
		public abstract RevHubOrientationOnRobot.LogoFacingDirection getLogoDirection();
		public abstract RevHubOrientationOnRobot.UsbFacingDirection getUSBDirection();
	}

	//Create instances of the above classes so that way we can actually use them
	public static DriveConfig driveConfig;
	public static ShooterConfig shooterConfig;
	public static RRConfig rrConfig;

	protected int shooterSpeedIndex = 0;

	protected HardwareMap hardwareMap;
	protected Telemetry telemetry;

	protected DcMotor frontLeftDrive;
	protected DcMotor backLeftDrive;
	protected DcMotor frontRightDrive;
	protected DcMotor backRightDrive;

	protected VoltageSensor voltageSensor;

	protected DcMotorEx shooter;

	protected Robot(HardwareMap hwMap, Telemetry telem) {
		//Create copies of the hardware map and telemetry so we can use them throughout the class
		hardwareMap = hwMap;
		telemetry = telem;

		//Initialize our voltage sensor
		voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
	}

	protected void initDrive() {
		//Initialize drive motors
		String[] driveStrings = driveConfig.getDriveStrings();
		frontLeftDrive = hardwareMap.get(DcMotor.class, driveStrings[0]);
		backLeftDrive = hardwareMap.get(DcMotor.class, driveStrings[1]);
		frontRightDrive = hardwareMap.get(DcMotor.class, driveStrings[2]);
		backRightDrive = hardwareMap.get(DcMotor.class, driveStrings[3]);

		boolean[] driveReversals = driveConfig.getDriveReversals();
		frontLeftDrive.setDirection(driveReversals[0] ?
				DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
		backLeftDrive.setDirection(driveReversals[1] ?
				DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
		frontRightDrive.setDirection(driveReversals[2] ?
				DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
		backRightDrive.setDirection(driveReversals[3] ?
				DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

		frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
	}

	protected void initShooter() {
		shooter = hardwareMap.get(DcMotorEx.class, shooterConfig.getShooterString());
		shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		shooter.setDirection(DcMotorSimple.Direction.REVERSE);
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
			if (++shooterSpeedIndex >= shooterConfig.getShooterSpeeds().length)
				shooterSpeedIndex = 0;
		}

		double shooterSpeedCompensated = shooterConfig.getShooterSpeedsCompensated()[shooterSpeedIndex];
		telemetry.addData("Battery Voltage", voltageSensor.getVoltage());
		telemetry.addData("Shooter Speed Compensated", shooterSpeedCompensated);
		telemetry.addData("Shooter Speed", shooterSpeedCompensated / voltageSensor.getVoltage());

		shooter.setPower(spin ? shooterSpeedCompensated / voltageSensor.getVoltage() : 0);
	}

	public abstract void spinIntake(double speed);

	//Easier sleep function that doesn't need to be encapsulated in a try/catch
	//Hopefully ignoring the exception won't come to bite back
	protected void robotSleep(long ms) {
		try {
			Thread.sleep(ms);
		} catch (InterruptedException e) {}
	}
}
