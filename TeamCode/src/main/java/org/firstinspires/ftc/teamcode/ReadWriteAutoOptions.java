package org.firstinspires.ftc.teamcode;

import android.content.Context;

import java.io.FileInputStream;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;

public class ReadWriteAutoOptions {
    private final String fileName = "autosettings.txt";
    private Context context;

    public ReadWriteAutoOptions(Context context) {
        this.context = context;
    }

    public void storeObject(AutonomousOptions autonomousOptions) {
        try {
            ObjectOutputStream objectOutputStream = new ObjectOutputStream(context.openFileOutput(fileName, context.MODE_PRIVATE));
            objectOutputStream.writeObject(autonomousOptions);
            objectOutputStream.flush();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public AutonomousOptions getObject() {
        ObjectInputStream objectInputStream;
        AutonomousOptions autonomousOptions = null;
        try {
            FileInputStream fileInputStream = context.openFileInput(fileName);
            objectInputStream = new ObjectInputStream(fileInputStream);
            autonomousOptions = (AutonomousOptions) objectInputStream.readObject();
            System.out.println(autonomousOptions);
        } catch (Exception e) {
            e.printStackTrace();
        }
        return autonomousOptions;
    }
}
