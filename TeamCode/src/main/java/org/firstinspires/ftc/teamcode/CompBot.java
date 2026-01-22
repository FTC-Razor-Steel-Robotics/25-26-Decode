package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.configs.CompDriveConfig;
import org.firstinspires.ftc.teamcode.configs.CompRRConfig;
import org.firstinspires.ftc.teamcode.configs.CompShooterConfig;
import org.firstinspires.ftc.teamcode.configs.DriveConfig;
import org.firstinspires.ftc.teamcode.configs.RRConfig;
import org.firstinspires.ftc.teamcode.configs.ShooterConfig;

public class CompBot extends Robot {
	private Servo trigger;
	private DcMotor intake;

	public static DriveConfig driveConfig = new CompDriveConfig();
	public static ShooterConfig shooterConfig = new CompShooterConfig();
	public static RRConfig rrConfig = new CompRRConfig();

	public CompBot(HardwareMap hwMap, Telemetry telem) {
		super(hwMap, telem, RobotType.COMP_BOT);

		//Initialize drive motors
		frontLeftDrive = hardwareMap.get(DcMotor.class, CompDriveConfig.frontLeftDriveString);
		backLeftDrive = hardwareMap.get(DcMotor.class, CompDriveConfig.backLeftDriveDriveString);
		frontRightDrive = hardwareMap.get(DcMotor.class, CompDriveConfig.frontRightDriveString);
		backRightDrive = hardwareMap.get(DcMotor.class, CompDriveConfig.backRightDriveString);

		frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
		backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
		frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
		backRightDrive.setDirection(DcMotor.Direction.REVERSE);

		frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		//Initialize the other motors
		shooter = hardwareMap.get(DcMotorEx.class, CompShooterConfig.shooterString);
		intake = hardwareMap.get(DcMotor.class, CompShooterConfig.intakeString);

		shooter.setDirection(DcMotorSimple.Direction.REVERSE);

		trigger = hardwareMap.get(Servo.class, CompShooterConfig.triggerString);

		//Set servo default positions
		moveTrigger(false);
	}

	public void spinIntake(double speed) {

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
