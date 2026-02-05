package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Competition Auto", group = "Competition")
@Config
public class CompAuto extends LinearOpMode {
	private ElapsedTime runtime = new ElapsedTime();

	CompBot robot;

	public static int sleepTimer = 2500;

	@Override
	public void runOpMode() {
		telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

		robot = new CompBot(hardwareMap, telemetry);

		Robot.getRobotType(hardwareMap);
		telemetry.addData("Active configuration", Robot.configFile);

		telemetry.addData("Status", "Initialized");
		telemetry.update();

		waitForStart();
		runtime.reset();

		robot.fireShooter(true, 0);
		sleep(5000);

		for (int i = 0; i < 3; i++) {
			sleep(1000);
			robot.moveTrigger(true);
			sleep(1000);
			robot.moveTrigger(false);
			sleep(1000);
			robot.spinIntake(1);
			sleep(1000);
			robot.spinIntake(0);
		}

		robot.fireShooter(false, 0);

		robot.driveMecanum(-0.2, 0, 0);
		sleep(sleepTimer);
		robot.driveMecanum(0, 0, 0);
	}
}
