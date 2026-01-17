package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@Config
@TeleOp(name = "Motor Test", group = "Mentor")
public class MotorTest extends LinearOpMode {
	public static String frontLeftDriveString = "FL/LO";
	public static String backLeftDriveDriveString = "RL";
	public static String frontRightDriveString = "FR/FO";
	public static String backRightDriveString = "RR";
	String shooterString = "shooter";
	String shooterDeliverString = "shooterDeliver";
	String carouselDeliverString = "carouselDeliver";
	String carouselString = "carousel";

	protected DcMotor frontLeftDrive;
	protected DcMotor backLeftDrive;
	protected DcMotor frontRightDrive;
	protected DcMotor backRightDrive;
	DcMotorEx shooter;
	Servo shooterDeliver;
	Servo carouselDeliver;
	Servo carousel;
	VoltageSensor voltageSensor;

	public static double shooterBaseSpeed = 0.6;
	public static double batMin = 11;
	public static double batMax = 14;
	public static double voltageCompScaling = 1;
	public static double shooterDeliverLimit = 0.5;
	public static double carouselDeliverMin = 0;
	public static double carouselDeliverMid = 0.5;
	public static double carouselDeliverMax = 1;

	@Override
	public void runOpMode() {
		telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

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

		shooter = hardwareMap.get(DcMotorEx.class, shooterString);
		shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		shooter.setDirection(DcMotorSimple.Direction.REVERSE);

		shooterDeliver = hardwareMap.get(Servo.class, shooterDeliverString);
		shooterDeliver.setDirection(Servo.Direction.REVERSE);
		carouselDeliver = hardwareMap.get(Servo.class, carouselDeliverString);
		carouselDeliver.setDirection(Servo.Direction.REVERSE);
		carousel = hardwareMap.get(Servo.class, carouselString);

		voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

		waitForStart();

		while (opModeIsActive()) {
//			shooter.setPower(gamepad1.y ? shooterMax : 0);
			shooter.setPower(gamepad1.y ? shooterBaseSpeed * (batMin / (voltageSensor.getVoltage() * voltageCompScaling)) : 0);
			telemetry.addData("Motor vel", shooter.getVelocity());
			telemetry.addData("Motor Power", shooter.getPower());
			telemetry.addData("Battery Voltage", voltageSensor.getVoltage());

			shooterDeliver.setPosition(gamepad1.right_trigger * shooterDeliverLimit);

//			carouselDeliver.setPosition(gamepad1.left_trigger * (carouselDeliverMax - carouselDeliverMin) + carouselDeliverMin);
			if (gamepad1.x) {
				carouselDeliver.setPosition(carouselDeliverMin);
			} else if (gamepad1.a) {
				carouselDeliver.setPosition(carouselDeliverMid);
			} else if (gamepad1.b) {
				carouselDeliver.setPosition(carouselDeliverMax);
			}
			telemetry.addData("Carousel Deliver Pos", carouselDeliver.getPosition());

			telemetry.update();
		}
	}
}
