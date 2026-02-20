package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.configs.CompDriveConfig;
import org.firstinspires.ftc.teamcode.configs.CompShooterConfig;

@TeleOp(name="Competition TeleOp", group="Competition")
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
        double triggerStart = 0;

        while (opModeIsActive()) {
            //Drive
            if (gamepad1.left_bumper)
                driveSpeed = CompDriveConfig.slowSpeed;
            else if (gamepad1.right_bumper)
                driveSpeed = CompDriveConfig.superSlowSpeed;
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

            if (gamepad2.left_bumper)
                robot.moveGuard(true);
            else if (gamepad2.right_bumper)
                robot.moveGuard(false);

            robot.fireShooter(spinShooter, shooterSpeedIndex);

            if (gamepad1.right_trigger > 0.2 && gamepad2.right_trigger > 0.2) {
                if (triggerStart == 0)
                    triggerStart = runtime.milliseconds();
                robot.moveGuard(true);
                robot.spinIntake(0);

                if (runtime.milliseconds() - triggerStart > CompShooterConfig.guardDelay)
                    robot.moveTrigger(true);
            } else if (gamepad2.left_bumper){
                robot.moveGuard(true);
                robot.spinIntake(0);
                if (triggerStart == 0)
                    triggerStart = runtime.milliseconds();

                if (runtime.milliseconds() - triggerStart > CompShooterConfig.guardDelay)
                    robot.spinIntake(-gamepad2.left_stick_y);
            }else if (gamepad2.right_bumper){
            robot.moveGuard(false);
            }else {
                robot.moveTrigger(false);
                robot.moveGuard(false);
                robot.spinIntake(-gamepad2.left_stick_y);

                triggerStart = 0;
            }



			telemetry.addData("Status", "Run Time: " + runtime.toString());
			telemetry.update();
		}
	}
}
