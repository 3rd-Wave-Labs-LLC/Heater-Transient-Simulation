/**
 * @file main
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "pid.h"
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include <chrono>
using namespace std;
using namespace std::chrono;

/**********************
 *      VARIABLES
 **********************/
const double V = 240.0;				//heater voltage [V]
const double h_n = 20;				//natural convection heat transfer coeffecient [W/m^2*K]
const double h_f = 3000;  			//forced cooling convection heat transfer coeffieient [W/m^2*K]
double h = h_f;
const double A = 80.0E-3 * 120.0E-3; 	//surface area for convection heat transfer [m^2] (per heater zone)
const double Cp = 951.8; 				//heater material heat capacity [J/kg*K]
const double mass = .015;			//mass of the heater and coupled mass [kg](per heater zone)

const double mCP = mass * Cp;			//heat capacity C [J/K], save some computation time 

const double T_amb = 21.0;			//ambinet temperature for convection [C]

double Input = 21.0;
double Output = 0.0;
double Setpoint = 50.0;

double Kp = 5.0;
double Ki = 0.5;
double Kd = 0.0;

PID pid(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

/**********************
 *      FUNCTIONS
 **********************/

int main(int argc, char **argv) {
    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(1);
    pid.SetOutputLimits(0, 4095);

    h = h_n; // natural convection

    cout << "Input(C), Output(counts), Setpoint(C), Time(ms)" << endl;
    while (pid.GetTimeCounterMS() < 5000) {

        int emulation_dt = 1;
        double P_loss = h * A * (Input - T_amb);							//convection loss here only
        double R_heater = 3.3;
        double P_heater = V * V / R_heater;
        double random_noise = 0; //(double(rand() % 5) - 2.5) / 10;

        
        Input += (((Output / 4095.0 * P_heater) - P_loss) / mCP * emulation_dt / 1000) + random_noise; //convert from ms to seconds for calculation
        pid.SetSampleTime(emulation_dt);
        pid.Compute(); //compute and increment time counter by emulation dt

        

        // cout with 3 decimal points
        cout << fixed << setprecision(3);
        cout << (Input) << ", " << Output << ", " << Setpoint <<  ", " << pid.GetTimeCounterMS() << "\n";
    }

    return 0; // SUCCESS
}