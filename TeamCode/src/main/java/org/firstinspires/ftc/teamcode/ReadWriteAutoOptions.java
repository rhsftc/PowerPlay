package org.firstinspires.ftc.teamcode;

import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.InputStream;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.OutputStream;

public class ReadWriteAutoOptions {
    public void storeObject(AutonomousOptions autonomousOptions) {
        OutputStream ops = null;
        ObjectOutputStream objOps = null;
        try {
            ops = new FileOutputStream("autonomous.txt");
            objOps = new ObjectOutputStream(ops);
            objOps.writeObject(autonomousOptions);
            objOps.flush();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public Object displayObjects() {
        InputStream fileIs = null;
        ObjectInputStream objIs = null;
        AutonomousOptions autonomousOptions = null;
        try {
            fileIs = new FileInputStream("autonomous.txt");
            objIs = new ObjectInputStream(fileIs);
            autonomousOptions = (AutonomousOptions) objIs.readObject();
            System.out.println(autonomousOptions);
        } catch (Exception e) {
            e.printStackTrace();
        }
        return autonomousOptions;
    }
}
