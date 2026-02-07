package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.configs.CompDriveConfig;

@TeleOp(name = "Competition TeleOp", group = "Competition")
public class CompTeleOp extends LinearOpMode {
	private ElapsedTime runtime = new ElapsedTime();

	CompBot robot;

	@Override
	public void runOpMode() {
		telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

		robot = new CompBot(hardwareMap, telemetry);

		telemetry.addData("Status", "Initialized");
		telemetry.update();

		waitForStart();
		runtime.reset();

		double driveSpeed = 1;
		int shooterSpeedIndex = 0;
		boolean spinShooter = false;

		while (opModeIsActive()) {
			//Drive
			if (gamepad1.left_bumper)
				driveSpeed = CompDriveConfig.slowSpeed;
			else
				driveSpeed = 1;

			robot.driveMecanum(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, driveSpeed);

			//Shooter
			if (gamepad2.x) {
				shooterSpeedIndex = 0;
				telemetry.addData("Shooter setting", "FAR");
			} else if (gamepad2.a) {
				shooterSpeedIndex = 1;
				telemetry.addData("Shooter setting", "MED");
			} else if (gamepad2.b) {
				shooterSpeedIndex = 2;
				telemetry.addData("Shooter setting", "CLOSE");
			}

			if (gamepad2.dpad_up)
				spinShooter = true;
			else if (gamepad2.dpad_down)
				spinShooter = false;

			robot.fireShooter(spinShooter, shooterSpeedIndex);

			robot.moveTrigger(gamepad2.right_trigger > 0.2);

			robot.spinIntake(-gamepad2.left_stick_y);

			telemetry.addData("Status", "Run Time: " + runtime.toString());
			telemetry.update();
		}
	}
}
