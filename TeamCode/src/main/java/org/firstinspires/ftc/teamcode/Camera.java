package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.configs.CameraConfig;
import org.firstinspires.ftc.teamcode.configs.CompShooterConfig;
import org.firstinspires.ftc.teamcode.configs.PIDController;

import java.util.List;

public class Camera {
	private HardwareMap hardwareMap;
	private Telemetry telemetry;
	Limelight3A limelight;
	public DigitalChannel cameraLED;

	ElapsedTime runtime;

	PIDController pid = new PIDController(CompShooterConfig.kP, CompShooterConfig.kI, CompShooterConfig.kD);

	double prevTime;
	double curTime;
	public boolean targetFound;
	double cameraAngle = 0;
	double cameraDistance = 0;

	final int BLUE_ID = 20;
	final int RED_ID = 24;
	int targetID = BLUE_ID;

	public Camera(HardwareMap hwMap, Telemetry telem, ElapsedTime rt) {
		hardwareMap = hwMap;
		telemetry = telem;
		runtime = rt;

		cameraLED = hardwareMap.get(DigitalChannel.class, CompShooterConfig.greenLEDString);
		cameraLED.setMode(DigitalChannel.Mode.OUTPUT);
        updateLED();

		limelight = hardwareMap.get(Limelight3A.class, "limelight");
		limelight.setPollRateHz(CameraConfig.pollRateHz);
		limelight.pipelineSwitch(CameraConfig.pipelineSwitch);
		limelight.start();

		prevTime = runtime.milliseconds();

		pid.setSetpoint(0);
		pid.setOutputLimits(-1, 1);
	}

	public void start() {
		limelight.start();
	}

	public void stop() {
		limelight.stop();
	}

	public void update() {
		pid.setPID(CompShooterConfig.kP, CompShooterConfig.kI, CompShooterConfig.kD);
		targetFound = false;

		LLResult result = limelight.getLatestResult();
		if (result != null && result.isValid()) {
			Pose3D pose = result.getBotpose();
			//For use with sensor fusion
//			Pose3D pose = result.getBotpose_MT2();

			List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
			for (LLResultTypes.FiducialResult fiducial : fiducials) {
				int id = fiducial.getFiducialId();

				//Ensure we are only finding the tag we want
				if (id == targetID) {
					cameraDistance = -fiducial.getRobotPoseTargetSpace().getPosition().z * 39.37007874015748031496;
					cameraAngle = fiducial.getTargetXDegrees();

					targetFound = true;

					telemetry.addData("Fiducial " + id, cameraDistance + " inches away");
					telemetry.addData("Fiducial " + id, cameraAngle + " degrees");
				}
			}
		}

		updateLED();
	}

	public void updateLED() {
		//The LED is active low, so set its value to false when we find a target
		cameraLED.setState(!targetFound);
	}

	public double getTurnPower() {
		curTime = runtime.milliseconds();

		double turnAmount = -pid.calculate(cameraAngle, curTime - prevTime);
		telemetry.addData("Auto turn power", turnAmount);

		prevTime = curTime;

		return turnAmount;
	}

	public double getCameraDistance() {
		return cameraDistance;
	}

	public Pose3D getRobotPose() {
		//TODO: Maybe average all our pose estimates. This could be mean or median value

		//Return null if we don't get a pose estimate
		return null;
	}
}
