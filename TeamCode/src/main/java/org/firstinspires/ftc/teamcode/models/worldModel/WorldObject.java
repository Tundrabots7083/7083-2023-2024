package org.firstinspires.ftc.teamcode.models.worldModel;


import org.firstinspires.ftc.teamcode.models.Position;

public class WorldObject {
    public String name;
    public String key;
    public Position position;
    public WorldObjectSize size;

    public WorldObject(String name, String key, Position position, WorldObjectSize size) {
        this.name = name;
        this.key = key;
        this.position = position;
        this.size = size;
    }

    @Override
    public String toString() {
        return "\nWorldObject [key = " + key + ", position =" + position + ", size  =" + size + "]";
    }
}
