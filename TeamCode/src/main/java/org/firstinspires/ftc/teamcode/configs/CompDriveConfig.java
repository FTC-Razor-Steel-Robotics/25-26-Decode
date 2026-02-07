package org.firstinspires.ftc.teamcode.configs;

import com.acmerobotics.dashboard.config.Config;

@Config
public class CompDriveConfig extends DriveConfig {
	public static String frontLeftDriveString = "FL/LO";
	public static String backLeftDriveDriveString = "RL";
	public static String frontRightDriveString = "FR";
	public static String backRightDriveString = "RR";

	public String[] getDriveStrings() {
		return new String[] {
				frontLeftDriveString,
				backLeftDriveDriveString,
				frontRightDriveString,
				backRightDriveString
		};
	}

	public static boolean frontLeftDriveReverse = false;
	public static boolean backLeftDriveDriveReverse = true;
	public static boolean frontRightDriveReverse = true;
	public static boolean backRightDriveReverse = true;

	public boolean[] getDriveReversals() {
		return new boolean[] {
				frontLeftDriveReverse,
				backLeftDriveDriveReverse,
				frontRightDriveReverse,
				backRightDriveReverse
		};
	}

	public static double slowSpeed = 0.5;
}
