package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.hardware.Gamepad;

public class Utils {

    public static int GetDesiredTagId(Alliance alliance, ScoringElementLocation selectedSide){
        if (alliance == Alliance.UNKNOWN || selectedSide == ScoringElementLocation.UNKNOWN){
            return -1;
        }
        if (alliance == Alliance.BLUE) {
            if (selectedSide == ScoringElementLocation.LEFT) {
                return 1;
            } else if (selectedSide == ScoringElementLocation.CENTER) {
                return 2;
            } else if (selectedSide == ScoringElementLocation.RIGHT) {
                return 3;
            }
        }
        if (alliance == Alliance.RED){
            if (selectedSide == ScoringElementLocation.LEFT){
                return 4;
            } else if (selectedSide == ScoringElementLocation.CENTER) {
                return 5;
            } else if (selectedSide == ScoringElementLocation.RIGHT){
                return 6;
            }
        }

        return -1;
    }

    public static Alliance GetAllianceFromKeyEntry(Gamepad gamepad1){
        // red button
        if(gamepad1.b){
            return Alliance.RED;
        }
        if(gamepad1.x) {
            // blue button
            return Alliance.BLUE;
        }

        return Alliance.UNKNOWN;
    }


    public static FieldPosition GetFieldPositionFromKeyEntry(Gamepad gamepad1){
        // green button
        if(gamepad1.a){
            return FieldPosition.NEAR;
        }
        if(gamepad1.y){
            return FieldPosition.FAR;
        }
        return FieldPosition.UNKNOWN;
    }

}
