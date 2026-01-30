package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.configs.MentorLiftConfig;

@TeleOp(name = "Mentor TeleOp", group = "Mentor")
public class MentorTeleOp extends LinearOpMode {
	private ElapsedTime runtime = new ElapsedTime();

	MentorBot robot;

	@Override
	public void runOpMode() {
		telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

		robot = new MentorBot(hardwareMap, telemetry);

		// Wait for the game to start (driver presses START)
		telemetry.addData("Status", "Initialized");
		telemetry.update();

		waitForStart();
		runtime.reset();

		boolean rightBumperPrev = false;
		boolean dpadUpPrev = false;
		boolean dpadDownPrev = false;
		boolean dpadLeftPrev = false;
		boolean dpadRightPrev = false;

		while (opModeIsActive()) {
			robot.driveMecanum(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

			//Shooter
			telemetry.addData("Bumper cur", gamepad1.right_bumper);
			telemetry.addData("Bumper prev", rightBumperPrev);
			robot.fireShooter(gamepad1.right_trigger > 0.5, gamepad1.right_bumper && !rightBumperPrev);

			robot.moveShooterDeliver(gamepad1.a);

			//Intake
			if (gamepad1.dpad_up && !dpadUpPrev)
				robot.autoIntake(gamepad1);
			else if (gamepad1.dpad_down && !dpadDownPrev)
				robot.autoDispense(gamepad1);

			if (gamepad1.b)
				robot.moveCarouselDeliver(MentorBot.CarouselDeliverPos.SHOOTER);
			else
				robot.moveCarouselDeliver(MentorBot.CarouselDeliverPos.CAROUSEL);

			robot.cycleCarousel(gamepad1.dpad_left && !dpadLeftPrev,
								gamepad1.dpad_right && !dpadRightPrev);

			//Lift
			if (robot.liftConfig.liftEnabled)
				robot.moveLift(gamepad1.y, gamepad1.a);

			rightBumperPrev = gamepad1.right_bumper;
			dpadUpPrev = gamepad1.dpad_up;
			dpadDownPrev = gamepad1.dpad_down;
			dpadLeftPrev = gamepad1.dpad_left;
			dpadRightPrev = gamepad1.dpad_right;

			// Show the elapsed game time and wheel power.
			telemetry.addData("Status", "Run Time: " + runtime.toString());
			telemetry.update();
		}
	}
}
