package frc.team449

import edu.wpi.first.wpilibj.RobotBase

object Constants {
    // --- OPERATIONAL MODES ---
    enum class Mode {
        REAL,
        SIM,
        REPLAY
    }

    val CURRENT_MODE: Mode = if (RobotBase.isReal()) Mode.REAL else Mode.SIM
    const val TUNING_MODE: Boolean = false

    // --- SYSTEM TIMING ---
    const val LOOP_TIME = 0.02

    // --- PHYSICAL SPECS ---
    const val ROBOT_MASS_KG = 59.8
    const val ROBOT_WIDTH_INCHES = 35.0 // including bumpers (front to rear)
    const val ROBOT_LENGTH_INCHES = 34.125 // including bumpers (left to right)

    object DriveConstants {
        const val WHEEL_DIAMETER_METERS = 0.1
        const val MAX_LINEAR_SPEED = 5.0 // meters per sec
        const val MAX_ROT_SPEED = 2 * Math.PI // rads per sec
    }
}
