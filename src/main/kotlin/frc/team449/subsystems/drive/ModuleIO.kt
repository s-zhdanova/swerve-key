package frc.team449.subsystems.drive

import com.ctre.phoenix6.swerve.jni.SwerveJNI
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModuleState
import org.littletonrobotics.junction.AutoLog

interface ModuleIO {
    @AutoLog
    open class ModuleIOInputs {
        @JvmField var turningVelocity = 0.0

        @JvmField var turningVoltage = 0.0

        @JvmField var turningAngle = Rotation2d()

        @JvmField var drivingVelocity = 0.0

        @JvmField var drivingVoltage = 0.0
    }

    fun updateInputs(inputs: ModuleIOInputs) {}

    fun setVoltageTurn(voltage: Double) {}

    fun setVoltageDrive(voltage: Double) {}
}
