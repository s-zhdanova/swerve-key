package frc.team449.subsystems.drive

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.wpilibj.simulation.DCMotorSim

open class ModuleIOSim : ModuleIO {
    val turnMotorSim: DCMotorSim =
        DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(1),
                0.005, // placeholder
                3.0, // placeholder
            ),
            DCMotor.getKrakenX60(1)
        )

    val driveMotorSim: DCMotorSim =
        DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(1),
                0.005, // placeholder
                3.0, // placeholder
            ),
            DCMotor.getKrakenX60(1)
        )

    override fun updateInputs(inputs: ModuleIO.ModuleIOInputs) {
        inputs.turningVoltage = turnMotorSim.inputVoltage
        inputs.turningVelocity = turnMotorSim.angularVelocityRadPerSec
        inputs.turningAngle = Rotation2d(turnMotorSim.angularPositionRad)
        turnMotorSim.update(0.02)

        inputs.drivingVoltage = driveMotorSim.inputVoltage
        inputs.drivingVelocity = driveMotorSim.angularVelocityRadPerSec
        driveMotorSim.update(0.02)
    }

    override fun setVoltageTurn(voltage: Double) {
        turnMotorSim.inputVoltage = voltage
    }

    override fun setVoltageDrive(voltage: Double) {
        driveMotorSim.inputVoltage = voltage
    }
}
