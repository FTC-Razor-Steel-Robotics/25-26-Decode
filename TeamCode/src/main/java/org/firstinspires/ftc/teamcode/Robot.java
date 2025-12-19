package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Robot {
	//Create new classes for our drive motors and intake/shooting motors
	//This will split them into groups in the dashboard, making it a bit less cluttered
	//For reference, this is also done in MecanumDrive.java
	public static class DriveConfig {
		public String frontLeftDriveString = "FL/LO";
		public String backLeftDriveDriveString = "RL";
		public String frontRightDriveString = "FR/RO";
		public String backRightDriveString = "RR/BO";
	}

	public static class ShooterConfig {
		public String shooterString = "Shooter";
		public String intakeString = "Intake";
		public String triggerString = "Trigger";

		public double[] shooterSpeeds = {0.8, 0.75, 0.7};

		public double triggerUpPos = 1;
		public double triggerDownPos = 0;
	}

	//This could go into ShooterConfig, but it's not necessary
	int shooterSpeedIndex = 0;

	//Create instances of the above classes so that way we can actually use them
	public static DriveConfig DRIVE_CONFIG = new DriveConfig();
	public static ShooterConfig SHOOTER_CONFIG = new ShooterConfig();

	private HardwareMap hardwareMap;
	private Telemetry telemetry;

	private DcMotor frontLeftDrive;
	private DcMotor backLeftDrive;
	private DcMotor frontRightDrive;
	private DcMotor backRightDrive;

	private DcMotor intake;
	private DcMotor shooter;
	private Servo trigger;

	public Robot(HardwareMap hwMap, Telemetry telem) {
		//Create copies of the hardware map and telemetry so we can use them throughout the class
		hardwareMap = hwMap;
		telemetry = telem;

		//Initialize drive motors
		frontLeftDrive = hardwareMap.get(DcMotor.class, DRIVE_CONFIG.frontLeftDriveString);
		backLeftDrive = hardwareMap.get(DcMotor.class, DRIVE_CONFIG.backLeftDriveDriveString);
		frontRightDrive = hardwareMap.get(DcMotor.class, DRIVE_CONFIG.frontRightDriveString);
		backRightDrive = hardwareMap.get(DcMotor.class, DRIVE_CONFIG.backRightDriveString);

		frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
		backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
		frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
		backRightDrive.setDirection(DcMotor.Direction.REVERSE);

		frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		//Initialize the other motors
		shooter = hardwareMap.get(DcMotor.class, SHOOTER_CONFIG.shooterString);
		intake = hardwareMap.get(DcMotor.class, SHOOTER_CONFIG.intakeString);
		trigger = hardwareMap.get(Servo.class, SHOOTER_CONFIG.triggerString);

		shooter.setDirection(DcMotorSimple.Direction.REVERSE);

		//Set servo default positions
		moveTrigger(false);
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

	public void spinShooter(boolean spin, boolean toggleSpeed) {
		//Cycle through the array and go back to 0 if we reach the end
		if (toggleSpeed) {
			if (++shooterSpeedIndex >= SHOOTER_CONFIG.shooterSpeeds.length)
				shooterSpeedIndex = 0;
		}

		double shooterSpeed = SHOOTER_CONFIG.shooterSpeeds[shooterSpeedIndex];
		telemetry.addData("Shooter Speed", shooterSpeed);

		shooter.setPower(spin ? shooterSpeed : 0);
	}

	public void moveTrigger(boolean up) {
		double triggerPos = up ? SHOOTER_CONFIG.triggerUpPos
								: SHOOTER_CONFIG.triggerDownPos;
		telemetry.addData("Trigger Pos", triggerPos);

		trigger.setPosition(triggerPos);
	}

	public void setIntakeSpeed(double speed) {
		telemetry.addData("Intake Speed", speed);

		intake.setPower(speed);
	}
}
