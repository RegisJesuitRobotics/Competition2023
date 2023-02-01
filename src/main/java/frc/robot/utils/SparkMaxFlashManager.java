package frc.robot.utils;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Robot;

import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;

public class SparkMaxFlashManager {
    private static boolean shouldFlash;

    public static void init() {
        if (Robot.isSimulation()) {
            shouldFlash = true;
            return;
        }
        File shouldFlashFile = new File(Filesystem.getDeployDirectory(), "hasFlashed.txt");
        try (FileReader shouldFlashReader = new FileReader(shouldFlashFile)) {
            char[] date = new char[1];
            int read = shouldFlashReader.read(date);
            if (read == 1) {
                // T represents true, F is false
                shouldFlash = date[0] == 'F';
            } else {
                // If we don't successfully read the character, assume we should flash
                shouldFlash = true;
            }
        } catch (IOException e) {
            shouldFlash = true;
        }

        // Write so we know we flashed
        try (FileWriter shouldFlashWriter = new FileWriter(shouldFlashFile)) {
            shouldFlashWriter.write('T');
        } catch (IOException e) {
            DriverStation.reportError(e.getMessage(), true);
        }

        if (!shouldFlash) {
            DataLogManager.log("Not flashing SparkMaxes as this is not a new deploy");
        }
    }

    /**
     * Due to the limited lifetime of SparkMax flash memory, flashing should only happen when necessary. This will
     * return true if it is the first time running after a new code deploy, which it knows happened as with every deploy
     * it will overwrite teh hasFlashed file.
     *
     * @return if the SparkMax should flash
     */
    public static boolean shouldFlash() {
        return shouldFlash;
    }
}
