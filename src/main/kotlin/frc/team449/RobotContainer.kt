package frc.team449

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.team449.subsystems.drive.ModuleIOHardware
import frc.team449.subsystems.drive.ModuleIOSim
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser

object RobotContainer {
    // driver controller
    val driveController: CommandXboxController = CommandXboxController(0)
    val operatorController: CommandXboxController = CommandXboxController(1)
    var autonomousCommand: Command = PrintCommand("If you see this, you probably didn't run an auto.")

    val bindings = Bindings(this)

    val autoChooser = LoggedDashboardChooser<Command>("Auto Routines")
}
