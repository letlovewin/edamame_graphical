#include <bits/stdc++.h>
#include "gravity-modified.h"
#include <SFML/Graphics.hpp>

int main()
{
    float r,a,b,dt;
    int n;
    cout << "Input the radius of your simulation: ";
    cin >> r;
    cout << "Now input the start time, end time, and step size of your simulation: ";
    cin >> a >> b >> dt;
    Universe universe(a,b,dt,r);
    cout << "Input the number of bodies in your simulation: ";
    cin >> n;
    for(int i = 0; i < n; i++) {
        float px,py,vx,vy,m;
        cout << "Input the x position, y position, x velocity, y velocity, and mass of your particle\n";
        cin >> px >> py >> vx >> vy >> m;
        Vector2 p(px,py);
        Vector2 v(vx,vy);
        Particle particle(p,v,m);
        universe.addChild(particle);
    }

    universe.start();

    return 0;
}

/*
2.50e+11
0 365000 84600
5
1.4960e+11  0.0000e+00  0.0000e+00  2.9800e+04  5.9740e+24
2.2790e+11  0.0000e+00  0.0000e+00  2.4100e+04  6.4190e+23
5.7900e+10  0.0000e+00  0.0000e+00  4.7900e+04  3.3020e+23
0.0000e+00  0.0000e+00  0.0000e+00  0.0000e+00  1.9890e+30
1.0820e+11  0.0000e+00  0.0000e+00  3.5000e+04  4.8690e+24

*/