package frc.robot;
public enum PositionEnum{
    CargoLow(1), CargoMid(2), CargoHigh(3), HatchetLow(4), HatchetMid(5), HatchetHigh(6);
    private final int value;
    PositionEnum(int newValue){
        value = newValue;
    }
    public int getValue(){
        return value;
    }
}