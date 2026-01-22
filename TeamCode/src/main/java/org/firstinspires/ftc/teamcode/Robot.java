package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.configs.DriveConfig;
import org.firstinspires.ftc.teamcode.configs.RRConfig;
import org.firstinspires.ftc.teamcode.configs.ShooterConfig;

@Config
public abstract class Robot {
	public enum RobotType {
		COMP_BOT,
		MENTOR_BOT
	}

	public static RobotType DEFAULT_ROBOT_TYPE = RobotType.MENTOR_BOT;
	public final RobotType robotType;

	//Create instances of our configs
	public DriveConfig driveConfig;
	public ShooterConfig shooterConfig;
	public RRConfig rrConfig;

	protected int shooterSpeedIndex = 0;

	protected HardwareMap hardwareMap;
	protected Telemetry telemetry;

	protected DcMotor frontLeftDrive;
	protected DcMotor backLeftDrive;
	protected DcMotor frontRightDrive;
	protected DcMotor backRightDrive;

	protected VoltageSensor voltageSensor;

	protected DcMotorEx shooter;

	protected Robot(HardwareMap hwMap, Telemetry telem, RobotType type) {
		//Create copies of the hardware map and telemetry so we can use them throughout the class
		hardwareMap = hwMap;
		telemetry = telem;

		robotType = type;

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
		telemetry.addData("Shooter Speed", shooterSpeedCompensated / voltageSensor.getVoltage());
		telemetry.addData("Shooter Speed Compensated", shooterSpeedCompensated);

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
