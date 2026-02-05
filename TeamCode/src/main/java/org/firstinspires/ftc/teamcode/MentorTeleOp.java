package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.configs.MentorLiftConfig;

import java.util.List;

@TeleOp(name = "Mentor TeleOp", group = "Mentor")
public class MentorTeleOp extends LinearOpMode {
	private ElapsedTime runtime = new ElapsedTime();

	MentorBot robot;

	Limelight3A limelight;

	IMU imu;

	@Override
	public void runOpMode() {
		telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

		robot = new MentorBot(hardwareMap, telemetry);

		Robot.getRobotType(hardwareMap);
		telemetry.addData("Active configuration", Robot.configFile);

		limelight = hardwareMap.get(Limelight3A.class, "limelight");
		limelight.setPollRateHz(100);
		limelight.pipelineSwitch(0);
		limelight.start();

		imu = hardwareMap.get(IMU.class, "imu");

		telemetry.addData("Status", "Initialized");
		telemetry.update();

		waitForStart();
		runtime.reset();

		boolean rightBumperPrev = false;
		boolean dpadUpPrev = false;
		boolean dpadDownPrev = false;
		boolean dpadLeftPrev = false;
		boolean dpadRightPrev = false;

		int shooterSpeedIndex = 0;

		while (opModeIsActive()) {
			//Sensor fusion with our IMU to get better position estimates
//			limelight.updateRobotOrientation(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

			//TODO: Reject positioning if we see obelisk tags
			LLResult result = limelight.getLatestResult();
			if (result != null && result.isValid()) {
				Pose3D pose = result.getBotpose();
				//For use with sensor fusion
//				Pose3D pose = result.getBotpose_MT2();

				List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
				for (LLResultTypes.FiducialResult fiducial : fiducials) {
					int id = fiducial.getFiducialId();
					double distance = fiducial.getRobotPoseTargetSpace().getPosition().z;
					telemetry.addData("Fiducial " + id, -distance * 39.37007874015748031496  + " inches away");
				}
			}

			robot.driveMecanum(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

			//Shooter
			if (gamepad1.right_bumper && !rightBumperPrev) {
				//Cycle through the array and go back to 0 if we reach the end
				if (++shooterSpeedIndex >= robot.shooterConfig.getShooterSpeeds().length)
					shooterSpeedIndex = 0;
			}

			robot.fireShooter(gamepad1.right_trigger > 0.5, shooterSpeedIndex);

			robot.moveShooterDeliver(gamepad1.a);

			//Intake
			if (gamepad1.dpad_up && !dpadUpPrev)
				robot.autoIntake(gamepad1);
			else if (gamepad1.dpad_down && !dpadDownPrev)
				robot.autoDispense(gamepad1);

			//Only allow carousel deliver to move if we are not running our auto intake or dispense
			if (!robot.intakeBusy) {
				if (gamepad1.b)
					robot.moveCarouselDeliver(MentorBot.CarouselDeliverPos.SHOOTER);
				else
					robot.moveCarouselDeliver(MentorBot.CarouselDeliverPos.CAROUSEL);
			}

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
