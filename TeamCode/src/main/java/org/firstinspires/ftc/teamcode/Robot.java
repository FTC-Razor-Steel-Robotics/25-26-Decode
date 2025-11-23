package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

//@Config
public class Robot {
	private String frontLeftDriveString = "FL/LO";
	private String backLeftDriveDriveString = "RL";
	private String frontRightDriveString = "FR/RO";
	private String backRightDriveString = "RR/BO";

	private HardwareMap hardwareMap;
	private DcMotor frontLeftDrive;
	private DcMotor backLeftDrive;
	private DcMotor frontRightDrive;
	private DcMotor backRightDrive;
	private Gamepad gamepad1;
	private Gamepad gamepad2;

	public Robot(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
		this.hardwareMap = hardwareMap;
		this.gamepad1 = gamepad1;
		this.gamepad2 = gamepad2;
	}

	public void init() {
		frontLeftDrive = hardwareMap.get(DcMotor.class, frontLeftDriveString);
		backLeftDrive = hardwareMap.get(DcMotor.class, backLeftDriveDriveString);
		frontRightDrive = hardwareMap.get(DcMotor.class, frontRightDriveString);
		backRightDrive = hardwareMap.get(DcMotor.class, backRightDriveString);

		frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
		backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
		frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
		backRightDrive.setDirection(DcMotor.Direction.FORWARD);

		frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
	}

	public void driveMotors() {
		// POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
		double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
		double lateral =  gamepad1.left_stick_x;
		double yaw     =  gamepad1.right_stick_x;

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
}
