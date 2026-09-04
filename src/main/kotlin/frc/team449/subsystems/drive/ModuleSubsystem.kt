package frc.team449.subsystems.drive

import edu.wpi.first.wpilibj2.command.SubsystemBase

class ModuleSubsystem(private val io: ModuleIO) : SubsystemBase() {
    private val inputs: ModuleIOInputsAutoLogged = ModuleIoInputsAutoLogged()

    override fun periodic() {
        io.updateInputs(inputs)
    }
}
