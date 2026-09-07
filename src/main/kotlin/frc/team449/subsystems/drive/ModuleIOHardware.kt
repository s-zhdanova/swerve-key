package frc.team449.subsystems.drive

import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Units

open class ModuleIOHardware(turnID: Int, driveID: Int) : ModuleIO {
    val turnMotor = TalonFX(turnID)
    val driveMotor = TalonFX(driveID)

    override fun updateInputs(inputs: ModuleIO.ModuleIOInputs) {
        inputs.turningVoltage = turnMotor.supplyVoltage.value.`in`(Units.Volts)
        inputs.turningVelocity = turnMotor.velocity.value.`in`(Units.RadiansPerSecond)
        inputs.turningAngle = Rotation2d(turnMotor.position.value.`in`(Units.Radians))

        inputs.drivingVoltage = driveMotor.supplyVoltage.value.`in`(Units.Volts)
        inputs.drivingVelocity = driveMotor.velocity.value.`in`(Units.RadiansPerSecond)
    }

    override fun setVoltageTurn(voltage: Double) {
        turnMotor.setVoltage(voltage)
    }

    override fun setVoltageDrive(voltage: Double) {
        driveMotor.setVoltage(voltage)
    }
}
