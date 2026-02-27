package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.configs.CompDriveConfig;
import org.firstinspires.ftc.teamcode.configs.CompShooterConfig;
import org.firstinspires.ftc.teamcode.configs.PIDController;

import java.util.List;

@TeleOp(name="Competition TeleOp", group="Competition")
public class CompTeleOp extends LinearOpMode {
	private ElapsedTime runtime = new ElapsedTime();

	CompBot robot;
	Limelight3A limelight;

	PIDController pid = new PIDController(CompShooterConfig.kP, CompShooterConfig.kI, CompShooterConfig.kD);

	@Override
	public void runOpMode() {
		telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

		robot = new CompBot(hardwareMap, telemetry);

		limelight = hardwareMap.get(Limelight3A.class, "limelight");
		limelight.setPollRateHz(100);
		limelight.pipelineSwitch(0);
		limelight.start();

		telemetry.addData("Status", "Initialized");
		telemetry.update();

		double driveSpeed = 1;
		int shooterSpeedIndex = 0;
		boolean spinShooter = false;
		double triggerStart = 0;

		double prevCameraTime = runtime.milliseconds();
		double curCameraTime;
		boolean foundTarget;
		double cameraAngle = 0;
		double cameraDistance = 0;

		pid.setSetpoint(0);
		pid.setOutputLimits(-1, 1);

		waitForStart();
		runtime.reset();

		while (opModeIsActive()) {
			pid.setPID(CompShooterConfig.kP, CompShooterConfig.kI, CompShooterConfig.kD);
			curCameraTime = runtime.milliseconds();
			foundTarget = false;

			//TODO: Reject positioning if we see obelisk tags
			LLResult result = limelight.getLatestResult();
			if (result != null && result.isValid()) {
				Pose3D pose = result.getBotpose();
				//For use with sensor fusion
//				Pose3D pose = result.getBotpose_MT2();

				List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
				for (LLResultTypes.FiducialResult fiducial : fiducials) {
					int id = fiducial.getFiducialId();
					cameraDistance = fiducial.getRobotPoseTargetSpace().getPosition().z;
					cameraAngle = fiducial.getTargetXDegrees();

					foundTarget = true;

					telemetry.addData("Fiducial " + id, -cameraDistance * 39.37007874015748031496  + " inches away");
					telemetry.addData("Fiducial " + id, cameraAngle + " degrees");
				}
			}

			//Drive
			if (gamepad1.left_bumper)
				driveSpeed = CompDriveConfig.slowSpeed;
			else if (gamepad1.right_bumper)
				driveSpeed = CompDriveConfig.superSlowSpeed;
			else
				driveSpeed = 1;

			if (gamepad1.left_trigger > 0.5 && foundTarget) {
				double turnAmount = -pid.calculate(cameraAngle, curCameraTime - prevCameraTime);
				robot.driveMecanum(-gamepad1.left_stick_y, gamepad1.left_stick_x, turnAmount, driveSpeed);
				telemetry.addData("Auto turn power", turnAmount);
			} else {
				robot.driveMecanum(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, driveSpeed);
			}

			prevCameraTime = curCameraTime;

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

//			robot.fireShooter(spinShooter, 0);
//			robot.fireShooter(spinShooter, shooterSpeedIndex);
			robot.fireShooterDistance(spinShooter, cameraDistance);

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
