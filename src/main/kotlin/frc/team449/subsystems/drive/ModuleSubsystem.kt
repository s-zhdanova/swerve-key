package frc.team449.subsystems.drive

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase

class ModuleSubsystem(private val io: ModuleIO) : SubsystemBase() {
    private val inputs: ModuleIOInputsAutoLogged = ModuleIoInputsAutoLogged()

    override fun periodic() {
        io.updateInputs(inputs)
    }

    fun setTurnVoltage(voltage: Double): Command =
        runOnce { io.setVoltageTurn(voltage) }

    fun stopTurn(): Command =
        runOnce { io.setVoltageTurn(0.0) }
}
