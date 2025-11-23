package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

public class Camera {
	private final Size cameraResolution = new Size(1920, 1080);
	private final Position cameraPosition = new Position(DistanceUnit.INCH,
			0, 0, 0, 0);
	private final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
			0, -90, 0, 0);

	private AprilTagProcessor aprilTag;
	private HardwareMap hardwareMap;
	private VisionPortal visionPortal;
	private OpenCvWebcam openCvWebcam;

	public Camera(HardwareMap hardwareMap) {
		this.hardwareMap = hardwareMap;
	}

	public void init(boolean enableDashboardStream) {
		if (enableDashboardStream) {
			int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
			openCvWebcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
		}

		aprilTag = new AprilTagProcessor.Builder()
				// The following default settings are available to un-comment and edit as needed.
//				.setDrawAxes(false)
//				.setDrawCubeProjection(false)
				.setDrawTagOutline(true)
				.setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
				.setCameraPose(cameraPosition, cameraOrientation)
				.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
				.build();

		//TODO: Figure out what this value should be set to initially
		setDecimation(3);

		// Create the vision portal by using a builder.
		VisionPortal.Builder builder = new VisionPortal.Builder();
		builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
		builder.setCameraResolution(cameraResolution);

		// Set the stream format; MJPEG uses less bandwidth than default YUY2.
		builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);

		// Set and enable the processor.
		builder.addProcessor(aprilTag);

		// Build the Vision Portal, using the above settings.
		visionPortal = builder.build();
	}

	public void startCameraStream() {
		FtcDashboard.getInstance().startCameraStream(openCvWebcam, 0);
	}

	public void enableVisionProcessor(boolean enable) {
		visionPortal.setProcessorEnabled(aprilTag, enable);
	}

	public void setDecimation(int decimation) {
		// Adjust Image Decimation to trade-off detection-range for detection-rate.
		// eg: Some typical detection data using a Logitech C920 WebCam
		// Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
		// Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
		// Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
		// Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
		// Note: Decimation can be changed on-the-fly to adapt during a match.
		aprilTag.setDecimation(decimation);
	}

	//This function is mainly for use outside of the class
	public ArrayList<AprilTagDetection> getDetections() {
		return aprilTag.getDetections();
	}

	public Pose3D getRobotPose() {
		//TODO: Maybe average all our pose estimates. This could be mean or median value
		for (AprilTagDetection detection : getDetections()) {
			if (detection.metadata != null) {
				//Only use non-obelisk tags for localization
				if (!detection.metadata.name.contains("Obelisk")) {
					return detection.robotPose;
				}
			}
		}

		//Return null if we don't get a pose estimate
		return null;
	}

	public void stop() {
		visionPortal.close();
	}
}
