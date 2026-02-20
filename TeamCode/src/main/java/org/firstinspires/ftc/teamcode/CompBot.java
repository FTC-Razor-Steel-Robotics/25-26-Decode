package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
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
    private Servo Guard;
	private DcMotor intake;

	public static DriveConfig driveConfig = new CompDriveConfig();
	public static ShooterConfig shooterConfig = new CompShooterConfig();
	public static RRConfig rrConfig = new CompRRConfig();

	public CompBot(HardwareMap hwMap, Telemetry telem) {
		super(hwMap, telem);

		super.driveConfig = driveConfig;
		super.shooterConfig = shooterConfig;
		super.rrConfig = rrConfig;

		initDrive();
		initShooter();

		//Set servo default positions
		moveTrigger(false);
        moveGuard(false);
	}

	public void initShooter() {
		//Initialize shooter motor
		super.initShooter();

		//Initialize other hardware
		intake = hardwareMap.get(DcMotor.class, CompShooterConfig.intakeString);
		trigger = hardwareMap.get(Servo.class, CompShooterConfig.triggerString);
        Guard = hardwareMap.get(Servo.class,CompShooterConfig.guardString);
	}


	public void spinIntake(double speed) {
		intake.setPower(speed);
	}

	public void moveTrigger(boolean up) {
        double triggerPos = up ? CompShooterConfig.triggerDownPos
                : CompShooterConfig.triggerUpPos;
//		telemetry.addData("Trigger Pos", triggerPos);
        trigger.setPosition(triggerPos);
    }
    public void moveGuard(boolean up){
        double Clear = up ? CompShooterConfig.GuardDown
                : CompShooterConfig.GuardUp;
        Guard.setPosition(Clear);
	}

	public void setIntakeSpeed(double speed) {
		intake.setPower(speed);
	}
}
