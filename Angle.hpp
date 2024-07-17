
#ifndef MTRN3100_H
#define MTRN3100_H

namespace mtrn3100 {
    class Angle {
    public:
        Angle() : x(0), y(0), z(0) {}
        
        double getX() const { return x; }
        double getY() const { return y; }
        double getZ() const { return z; }
        
        void setX(double val) { x = val; }
        void setY(double val) { y = val; }
        void setZ(double val) { z = val; }
        
    private:
        double x, y, z;
    };
}

#endif // MTRN3100_H