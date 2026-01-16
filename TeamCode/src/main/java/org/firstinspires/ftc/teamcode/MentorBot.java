package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class MentorBot extends Robot {
	public static class MentorDriveConfig extends DriveConfig {
		public static String frontLeftDriveString = "FL/LO";
		public static String backLeftDriveDriveString = "RL";
		public static String frontRightDriveString = "FR/FO";
		public static String backRightDriveString = "RR";
	}

	public static class MentorShooterConfig extends ShooterConfig {
		public static String shooterString = "Shooter";
		public static String intakeString = "Intake";
		public static String triggerString = "Trigger";

		public double[] shooterSpeeds = {0.8, 0.75, 0.7};

		public double triggerUpPos = 1;
		public double triggerDownPos = 0;
	}

	public MentorBot(HardwareMap hwMap, Telemetry telem) {
		super(hwMap, telem);

		//Initialize drive motors
		frontLeftDrive = hardwareMap.get(DcMotor.class, MentorDriveConfig.frontLeftDriveString);
		backLeftDrive = hardwareMap.get(DcMotor.class, MentorDriveConfig.backLeftDriveDriveString);
		frontRightDrive = hardwareMap.get(DcMotor.class, MentorDriveConfig.frontRightDriveString);
		backRightDrive = hardwareMap.get(DcMotor.class, MentorDriveConfig.backRightDriveString);

		frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
		backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
		frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
		backRightDrive.setDirection(DcMotor.Direction.FORWARD);

		frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
	}

	public void setIntakeSpeed(double speed) {

	}
}
