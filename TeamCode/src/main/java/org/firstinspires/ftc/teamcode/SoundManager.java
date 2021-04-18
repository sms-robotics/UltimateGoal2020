package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.HashMap;

public class SoundManager {
    private final File soundsDirectory = AppUtil.ROBOT_DATA_DIR;
    private final HashMap<Sound, File> soundToFileMap = new HashMap<Sound, File>();
    private final SoundPlayer soundPlayer;
    private final HardwareMap hardwareMap;

    public enum Sound {
        OK("ok.wav"),
        BAD("bad.wav"),
        CONFIRM("confirm.wav"),
        COIN("coin.wav"),
        UNLOCKED("unlocked.wav"),
        WAKEUP("sweep.wav");

        private final String soundFileName;

        /**
         * @param soundFileName
         */
        Sound(final String soundFileName) {
            this.soundFileName = soundFileName;
        }

        /* (non-Javadoc)
         * @see java.lang.Enum#toString()
         */
        @Override
        public String toString() {
            return soundFileName;
        }
    };

    public SoundManager(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        soundPlayer = SoundPlayer.getInstance();
    }

    public void initialize() {
        soundPlayer.setMasterVolume(1.0f);

        for (Sound s : Sound.values()) {
            loadSound(s, soundToFileMap);
        }
//
//        loadSound("wakeup", "sweep.wav", soundToFileMap);
//        loadSound(Sound.OK, , soundToFileMap);
//        loadSound("confirm", "confirm.wav", soundToFileMap);
//        loadSound("bad", "lose-life.wav", soundToFileMap);
//        loadSound("coin", "coin.wav", soundToFileMap);
//        loadSound("unlocked", "unlocked.wav", soundToFileMap);

        // Preload all sounds in our map
//        for (Map.Entry<String,File> entry : soundNameToFileMap.entrySet()) {
//            soundPlayer.preload(this.hardwareMap.appContext, entry.getValue());
//        }

        play(Sound.WAKEUP);
    }

    public void play(Sound sound) {
        File soundFile = soundToFileMap.get(sound);

        if (soundFile != null) {
            soundPlayer.startPlaying(this.hardwareMap.appContext, soundFile);
        }
    }

    /**
     * Attempts to load a sound from the sounds directory, but checks
     * to make sure it exists first. If it does, the sound goes into the
     * soundNameToFileMap structure.
     */
    private boolean loadSound(Sound sound, HashMap<Sound, File> soundNameToFileMap) {
        File soundFile = new File(soundsDirectory, sound.soundFileName);

        if (soundFile.exists() && soundFile.canRead()) {
            soundNameToFileMap.put(sound, soundFile);

            // Pre-load it to make playing faster
            soundPlayer.preload(this.hardwareMap.appContext, soundFile);

            return true;
        }

        return false;
    }
}
