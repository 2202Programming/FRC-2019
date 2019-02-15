package frc.robot;
public enum PositionEnum{
    CargoLow(1), CargoMid(2), CargoHigh(3), HatchLow(4), HatchMid(5), HatchHigh(6);
    private final double value;
    PositionEnum(double newValue){
        value = newValue;
    }
    public double getValue(){
        return value;
    }
}