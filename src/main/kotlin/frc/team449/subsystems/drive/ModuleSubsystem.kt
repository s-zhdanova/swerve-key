package frc.team449.subsystems.drive

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase

class ModuleSubsystem(private val io: ModuleIO) : SubsystemBase() {
    private val inputs: ModuleIOInputsAutoLogged = ModuleIOInputsAutoLogged()

    private val turnController = PIDController(0.5, 0.0, 0.05)
    private val driveController = PIDController(0.5, 0.0, 0.05)

    override fun periodic() {
        io.updateInputs(inputs)
        io.setVoltageTurn(turnController.calculate(inputs.turningAngle))
        io.setVoltageDrive(driveController.calculate(inputs.drivingVelocity))
    }

    fun setAngle(rads: Double): Command =
        runOnce {
            turnController.setpoint = rads
        }

    fun setTurnVoltage(voltage: Double): Command =
        runOnce {
            io.setVoltageTurn(voltage)
        }

    fun stopTurn(): Command =
        runOnce {
            io.setVoltageTurn(0.0)
        }

    fun setDriveVelocity(radsPerSecond: Double): Command =
        runOnce {
            driveController.setpoint = radsPerSecond
        }
}
