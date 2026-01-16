package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class CompBot extends Robot {
	public static class CompDriveConfig extends Robot.DriveConfig {
		public static String frontLeftDriveString = "FL/LO";
		public static String backLeftDriveDriveString = "RL";
		public static String frontRightDriveString = "FR/RO";
		public static String backRightDriveString = "RR/BO";
	}

	public static class CompShooterConfig extends ShooterConfig {
		public static String shooterString = "Shooter";
		public static String intakeString = "Intake";
		public static String triggerString = "Trigger";

		public static double[] shooterSpeeds = {0.8, 0.75, 0.7};

		public static double triggerUpPos = 1;
		public static double triggerDownPos = 0;
	}

	private Servo trigger;

	public CompBot(HardwareMap hwMap, Telemetry telem) {
		super(hwMap, telem);

		//Initialize drive motors
		frontLeftDrive = hardwareMap.get(DcMotor.class, MentorBot.MentorDriveConfig.frontLeftDriveString);
		backLeftDrive = hardwareMap.get(DcMotor.class, MentorBot.MentorDriveConfig.backLeftDriveDriveString);
		frontRightDrive = hardwareMap.get(DcMotor.class, MentorBot.MentorDriveConfig.frontRightDriveString);
		backRightDrive = hardwareMap.get(DcMotor.class, MentorBot.MentorDriveConfig.backRightDriveString);

		frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
		backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
		frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
		backRightDrive.setDirection(DcMotor.Direction.REVERSE);

		frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		//Initialize the other motors
		shooter = hardwareMap.get(DcMotor.class, CompShooterConfig.shooterString);
		intake = hardwareMap.get(DcMotor.class, CompShooterConfig.intakeString);

		shooter.setDirection(DcMotorSimple.Direction.REVERSE);

		trigger = hardwareMap.get(Servo.class, CompShooterConfig.triggerString);

		//Set servo default positions
		moveTrigger(false);
	}

	public void moveTrigger(boolean up) {
		double triggerPos = up ? CompShooterConfig.triggerUpPos
				: CompShooterConfig.triggerDownPos;
		telemetry.addData("Trigger Pos", triggerPos);

		trigger.setPosition(triggerPos);
	}

	public void setIntakeSpeed(double speed) {
		telemetry.addData("Intake Speed", speed);

		intake.setPower(speed);
	}
}
