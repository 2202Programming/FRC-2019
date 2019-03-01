package frc.robot.commands.util;


public class ExpoShaper {
     // expo only works on normalize inputs
    double kExpo;          // 0.0 to 1.0 for flatness 0 -> straight curve
    double kCexpo;         // complement to expo 

    ExpoShaper(double kExpo) {
        setExpo(kExpo);
    }

    public void setExpo(double a) {
        if (a > 1.0) a = 1.0;
        if (a < 0.0) a = 0.0;
        kExpo = a;
        kCexpo = 1.0 - a;
    }

    // use Gord W's expo function
     public double expo(double x) {
        return x*x*x*kExpo + kCexpo*x;   
    }
}

        
    