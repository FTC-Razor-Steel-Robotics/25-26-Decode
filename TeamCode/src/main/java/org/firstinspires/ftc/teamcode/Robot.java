package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Robot {
	public static String frontLeftDriveString = "FL/LO";
	public static String backLeftDriveDriveString = "RL";
	public static String frontRightDriveString = "FR/RO";
	public static String backRightDriveString = "RR/BO";
	public static String shooterString = "Shooter";
	public static String intakeString = "Intake";
	public static String triggerString = "Trigger";

	public static double[] shooterSpeeds = {0.8, 0.75, 0.7};
	int shooterSpeedIndex = 0;

	private HardwareMap hardwareMap;
	private DcMotor frontLeftDrive;
	private DcMotor backLeftDrive;
	private DcMotor frontRightDrive;
	private DcMotor backRightDrive;
	private DcMotor intake;
	private DcMotor shooter;
	private Servo trigger;

	public void init(HardwareMap hardwareMap) {
		this.hardwareMap = hardwareMap;

		//Initialize drive motors
		frontLeftDrive = hardwareMap.get(DcMotor.class, frontLeftDriveString);
		backLeftDrive = hardwareMap.get(DcMotor.class, backLeftDriveDriveString);
		frontRightDrive = hardwareMap.get(DcMotor.class, frontRightDriveString);
		backRightDrive = hardwareMap.get(DcMotor.class, backRightDriveString);

		frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
		backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
		frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
		backRightDrive.setDirection(DcMotor.Direction.REVERSE);

		frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		//Initialize the other motors
		shooter = hardwareMap.get(DcMotor.class, shooterString);
		intake = hardwareMap.get(DcMotor.class, intakeString);
		trigger = hardwareMap.get(Servo.class,triggerString);

		shooter.setDirection(DcMotorSimple.Direction.REVERSE);

		//Set servo default positions
		trigger.setPosition(0);
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
	}

	public void spinShooter(boolean spin, boolean toggleSpeed) {
		if (toggleSpeed) {
			if (++shooterSpeedIndex >= shooterSpeeds.length)
				shooterSpeedIndex = 0;
		}

		double shooterSpeed = shooterSpeeds[shooterSpeedIndex];

		shooter.setPower(spin ? shooterSpeed : 0);
	}
}
