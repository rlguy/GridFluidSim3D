#pragma once

struct Triangle {
    int tri[3];     // indices to a vertex

    Triangle() {
        tri[0] = 0;
        tri[1] = 0;
        tri[2] = 0;
    }

    Triangle(int p1, int p2, int p3) {
        tri[0] = p1;
        tri[1] = p2;
        tri[2] = p3;
    }
};