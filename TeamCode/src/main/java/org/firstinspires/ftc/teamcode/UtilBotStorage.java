package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;

/**
 * This is our settings class for storing saving things across
 * sessions.
 * <p>
 * It's a "singleton" meaning that there's only one of them. You can
 * get at it using UtilBotStorage.sharedInstance() and then following that
 * with the function you want.
 * <p>
 * Usage Example::
 * <p>
 * UtilBotStorage.sharedInstance().saveItem("foo", "bar)
 */
public class UtilBotStorage {
    private static final String TAG = "UtilBotStorage";
    private final File dataDirectory = AppUtil.ROBOT_DATA_DIR;
    private final String dataFileName = "data.json";
    private JSONObject jsonObject;

    public final static String LAST_GYRO_ANGLE = "lastGyroAngle";

    class Item {
        public final String name;
        public final Object value;
        public final long dateSet;

        Item(String name, Object value, long dateSet) {
            this.name = name;
            this.value = value;
            this.dateSet = dateSet;
        }

        public boolean isValid() {
            return this.value != null;
        }

        public double secondsSinceSaved() {
            long msSince = (System.currentTimeMillis() - dateSet);

            return msSince / 1000;
        }
    }

    public Item getItem(String name) {
        Object extractedValue = null;
        long extractedDateSet = 0;

        extractedValue = this.jsonObject.opt(name);
        extractedDateSet = this.jsonObject.optLong(getKeyNameForDateSet(name), 0L);

        return new Item(name, extractedValue, extractedDateSet);
    }

    private String getKeyNameForDateSet(String name) {
        return name + "-SaveDate";
    }

    public void setItem(String name, Object value) {
        try {
            this.jsonObject.put(name, value);
            String dateName = getKeyNameForDateSet(name);
            this.jsonObject.put(dateName, System.currentTimeMillis());
            store();
        } catch (JSONException e) {
            e.printStackTrace();
        }
    }

    //---------------------------------------------------------------------------------------------
    // Inner workings! You probably don't need to edit any of this.
    //---------------------------------------------------------------------------------------------
    private static class BotDataSingletonContainer {
        static final UtilBotStorage SHARED_INSTANCE = UtilBotStorage.factory();
    }

    static public UtilBotStorage sharedInstance() {
        return UtilBotStorage.BotDataSingletonContainer.SHARED_INSTANCE;
    }

    private static UtilBotStorage factory() {
        UtilBotStorage botStorage = new UtilBotStorage();

        botStorage.load();

        return botStorage;
    }

    private boolean load() {
        File dataFile = new File(dataDirectory, dataFileName);
        RobotLog.d("Attempting to load data from %s", dataFile);

        if (!(dataFile.exists() && dataFile.canRead())) {
            RobotLog.e("Either the data file %s does not exist or could not be read.", dataFile);
            this.jsonObject = new JSONObject();
            return false;
        }

        RobotLog.d("Loading data from %s", dataFile);
        FileReader fileReader = null;
        try {
            fileReader = new FileReader(dataFile);
            BufferedReader bufferedReader = new BufferedReader(fileReader);
            StringBuilder stringBuilder = new StringBuilder();
            String line = bufferedReader.readLine();
            while (line != null) {
                stringBuilder.append(line).append("\n");
                line = bufferedReader.readLine();
            }
            bufferedReader.close();

            JSONObject jsonObject = new JSONObject(stringBuilder.toString());

            this.jsonObject = jsonObject;

            return true;
        } catch (FileNotFoundException e) {
            RobotLog.ee(TAG, e, "Our settings file wasn't found, which is odd because we should have explicitly checked for it.");
            e.printStackTrace();
        } catch (IOException e) {
            RobotLog.ee(TAG, e, "An error occurred when reading our settings file. Perhaps it is corrupt?");
            e.printStackTrace();
        } catch (JSONException e) {
            RobotLog.ee(TAG, e, "An error occurred when reading our settings file because it is not valid .json. Check the file contents!");
            e.printStackTrace();
        } finally {
            if (fileReader != null) {
                try {
                    fileReader.close();
                } catch (IOException e) {
                    RobotLog.ee(TAG, e, "An error occurred when trying to close our settings .json.");
                    e.printStackTrace();
                }
            }
        }

        return false;
    }

    private boolean store() {
        File dataFile = new File(dataDirectory, dataFileName);
        RobotLog.d("Attempting to save data to %s", dataFile);

//        if (!dataFile.canWrite()) {
//            RobotLog.e("Data file cannot be written.", dataFile);
//            return false;
//        }
//
        RobotLog.d("Saving data to %s", dataFile);
        FileWriter fileWriter = null;
        try {
            fileWriter = new FileWriter(dataFile);
            BufferedWriter bufferedWriter = new BufferedWriter(fileWriter);

            String s = this.jsonObject.toString();
            bufferedWriter.write(s);

            bufferedWriter.close();

            return true;
        } catch (FileNotFoundException e) {
            RobotLog.ee(TAG, e, "Our settings file wasn't found, which is odd because we should have explicitly checked for it.");
            e.printStackTrace();
        } catch (IOException e) {
            RobotLog.ee(TAG, e, "An error occurred when reading our settings file. Perhaps it is corrupt?");
            e.printStackTrace();
        } finally {
            if (fileWriter != null) {
                try {
                    fileWriter.close();
                } catch (IOException e) {
                    RobotLog.ee(TAG, e, "An error occurred when trying to close our data .json.");
                    e.printStackTrace();
                }
            }
        }

        return false;
    }
}
